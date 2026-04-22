// SPDX-License-Identifier: GPL-2.0
/*
 * AD5940 AFE Linux SPI driver – platform driver + IIO integration
 *
 * This file contains the SPI driver probe/remove, interrupt handler,
 * IIO device registration with triggered buffer, and trigger ops.
 * Low-level register access lives in ad5940_core.c.
 *
 * Phase 1: BIA (Body Impedance) measurement, single-frequency 50kHz,
 * 4-wire, DFT=8192, FIFO threshold interrupt, WUPT-driven periodic
 * measurement via AD5940 sequencer.
 *
 * DTS:
 *	&spi1 {
 *	pinctrl-0 = <&spi1m1_cs0 &spi1m1_pins>;
 *	pinctrl-1 = <&spi1m1_cs0_hs &spi1m1_pins_hs>;
 *	pinctrl-names = "default", "high_speed";
 *	cs-gpios = <&gpio3 RK_PA1 GPIO_ACTIVE_LOW>;
 *	status = "okay";
 *	ad5940@0 {
 *		compatible = "adi,ad5940";
 *		reg = <0>;
 *		spi-max-frequency = <4000000>;
 *		interrupt-parent = <&gpio3>;
 *		interrupts = <8 IRQ_TYPE_EDGE_FALLING>;
 *		reset-gpios = <&gpio1 RK_PB1 GPIO_ACTIVE_LOW>;
 *	};
 *};
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "ad5940_core.h"

/* ------------------------------------------------------------------ */
/*  IIO channel definitions – DFT impedance mode (BIA 4-wire)        */
/* ------------------------------------------------------------------ */

/*
 * AD5940 FIFO in DFT (FIFOSRC_DFT) mode outputs 4 words per measurement
 * cycle.  The FIFO word order is determined by the Sequencer program's
 * ADC MUX switching sequence.
 *
 * BIA 4-wire measurement (BodyImpedance.c):
 *   Step 1: ADC MUX = HSTIA_P/HSTIA_N → DFT of current through RTIA
 *   Step 2: ADC MUX = AIN3/AIN2       → DFT of voltage across body
 *
 *   FIFO word order:
 *     word0 = Current Real     (pDftCurr->Real)
 *     word1 = Current Imag     (pDftCurr->Image)
 *     word2 = Voltage Real     (pDftVolt->Real)
 *     word3 = Voltage Imag     (pDftVolt->Image)
 *
 *   Electrode configuration (BIA, 4-wire):
 *     Current path:  CE0 → body → AIN1 → HSTIA (RTIA)
 *     Voltage sense: AIN3(+) / AIN2(-) — independent high-Z detection
 *
 * DFT FIFO word format: 18-bit signed two's complement in bits[17:0]
 *   Bit17 is the sign bit.
 *   scan_type = s18/32>>0 — raw 32-bit FIFO words pushed directly;
 *   sign-extension handled by user-space libiio based on _type attr.
 */

#define AD5940_DFT_CURR_REAL	0
#define AD5940_DFT_CURR_IMAG	1
#define AD5940_DFT_VOLT_REAL	2
#define AD5940_DFT_VOLT_IMAG	3
#define AD5940_DFT_TIMESTAMP	4

static const struct iio_chan_spec ad5940_dft_channels[] = {
	[AD5940_DFT_CURR_REAL] = {
		.type			= IIO_CURRENT,
		.indexed		= 1,
		.channel		= 0,
		.scan_index		= AD5940_DFT_CURR_REAL,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 18,	/* 18-bit signed DFT result in bits[17:0] */
			.storagebits	= 32,	/* each FIFO word is 32 bits wide */
			.shift		= 0,	/* data is right-aligned, no shift needed */
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_DFT_CURR_IMAG] = {
		.type			= IIO_CURRENT,
		.indexed		= 1,
		.channel		= 1,
		.scan_index		= AD5940_DFT_CURR_IMAG,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 18,
			.storagebits	= 32,
			.shift		= 0,
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_DFT_VOLT_REAL] = {
		.type			= IIO_VOLTAGE,
		.indexed		= 1,
		.channel		= 0,
		.scan_index		= AD5940_DFT_VOLT_REAL,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 18,
			.storagebits	= 32,
			.shift		= 0,
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_DFT_VOLT_IMAG] = {
		.type			= IIO_VOLTAGE,
		.indexed		= 1,
		.channel		= 1,
		.scan_index		= AD5940_DFT_VOLT_IMAG,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 18,
			.storagebits	= 32,
			.shift		= 0,
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_DFT_TIMESTAMP] = IIO_CHAN_SOFT_TIMESTAMP(AD5940_DFT_TIMESTAMP),
};

/* ------------------------------------------------------------------ */
/*  IIO trigger ops                                                   */
/* ------------------------------------------------------------------ */

/*
 * ad5940_trigger_set_state - Enable/disable the FIFO threshold interrupt
 *
 * Called by IIO core when buffer is enabled/disabled.
 * When buffer is enabled (state=true): start WUPT for periodic measurement.
 * When buffer is disabled (state=false): stop WUPT.
 */
static int ad5940_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct ad5940_priv *priv = iio_trigger_get_drvdata(trig);
	struct device *dev = &priv->spi->dev;

	dev_info(dev, "trigger_set_state: %s\n", state ? "ENABLE" : "DISABLE");

	if (state)
		return ad5940_bia_start(priv);
	else
		return ad5940_bia_stop(priv);
}

/*
 * ad5940_trigger_reenable - Re-enable IRQ after trigger handler completes
 *
 * Called by IIO core via try_reenable after iio_trigger_notify_done().
 * Re-enables the Linux IRQ that was disabled in ad5940_irq_handler().
 */
static int ad5940_trigger_reenable(struct iio_trigger *trig)
{
	struct ad5940_priv *priv = iio_trigger_get_drvdata(trig);

	enable_irq(priv->irq);
	return 0;
}

static const struct iio_trigger_ops ad5940_trigger_ops = {
	.set_trigger_state = ad5940_trigger_set_state,
	.try_reenable	= ad5940_trigger_reenable,
};

/* ------------------------------------------------------------------ */
/*  IIO triggered buffer handler                                      */
/* ------------------------------------------------------------------ */

/*
 * ad5940_trigger_handler - Read FIFO DFT data and push to IIO buffer
 *
 * Called from the IIO trigger poll thread after GPIO interrupt.
 * Reads 4 FIFO words (current real/imag + voltage real/imag) and
 * pushes them into the IIO buffer with a timestamp.
 *
 * The FIFO source is FIFOSRC_DFT. Each DFT result consists of
 * a real and imaginary part, both 18-bit signed in a 32-bit word.
 */
static irqreturn_t ad5940_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad5940_priv *priv = iio_priv(indio_dev);

	/*
	 * Buffer layout for iio_push_to_buffers_with_timestamp():
	 *   4 × u32 channel data  +  1 × s64 timestamp
	 * The timestamp is appended after channel data by the framework,
	 * so we must reserve space for it to avoid stack overflow.
	 */
	u32 buf[AD5940_DFT_CHANNELS + sizeof(s64) / sizeof(u32)];
	// u32 raw16[16];  /* for reading all 16 FIFO words when threshold=16 */
	int ret, fifo_cnt;

	memset(buf, 0, sizeof(buf));
	// memset(raw16, 0, sizeof(raw16));

	/* Wake up AFE (it may be in hibernate between measurements) */
	ret = ad5940_wakeup(priv);
	if (ret) {
		dev_err(&priv->spi->dev, "FIFO read: wakeup failed: %d\n",
			ret);
		goto out;
	}

	/* Lock sleep key during FIFO read (prevent hibernate mid-read).
	 * ADI: AD5940_SleepKeyCtrlS(SLPKEY_LOCK) - use 0 (any wrong value locks)
	 */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_LOCK);

	/* ---- Debug: read key registers after measurement cycle ---- */
	{
		static int dbg_cnt;
		int adccon, dsw, psw, nsw, tsw, swcon;

		if (dbg_cnt < 16) {
			adccon = ad5940_spi_read(priv, AD5940_REG_ADCCON);
			dsw = ad5940_spi_read(priv, AD5940_REG_DSWFULLCON);
			psw = ad5940_spi_read(priv, AD5940_REG_PSWFULLCON);
			nsw = ad5940_spi_read(priv, AD5940_REG_NSWFULLCON);
			tsw = ad5940_spi_read(priv, AD5940_REG_TSWFULLCON);
			swcon = ad5940_spi_read(priv, AD5940_REG_SWCON);
			dev_info(&priv->spi->dev,
				 "DBG[%d] ADCCON=0x%x MUXP=%d MUXN=%d PGA=%d | "
				 "SW D=0x%x P=0x%x N=0x%x T=0x%x SWCON=0x%x\n",
				 dbg_cnt, adccon,
				 adccon & 0x3F, (adccon >> 8) & 0x1F,
				 (adccon >> 16) & 0x7,
				 dsw, psw, nsw, tsw, swcon);
			dbg_cnt++;
		}
	}

	/* Read FIFO count to check how many words are available */
	ret = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "FIFO count read failed: %d\n",
			ret);
		goto out;
	}
	/* FIFOCNTSTA.DATAFIFOCNTSTA is in bits[26:16], per AD5940 datasheet */
	fifo_cnt = ret >> 16;

	/* DEBUG: print FIFOCNT raw value and extracted count */
	{
		static int _dbg_cnt;
		if (_dbg_cnt < 16) {
			dev_info(&priv->spi->dev,
				 "FIFO: raw=0x%x cnt=%d\n",
				 ret, fifo_cnt);
			_dbg_cnt++;
		}
	}

	/*
	 * Read exactly one DFT frame (4 words).
	 *
	 * TODO: Handle fifo_cnt > 4 (multiple measurement cycles in FIFO).
	 * For now, if there are more than 4 words, we read 4 and the
	 * remaining words will be picked up on the next interrupt.
	 * If fifo_cnt < 4, something is wrong — read what's available.
	 */
	if (fifo_cnt < AD5940_DFT_CHANNELS) {
		dev_warn(&priv->spi->dev,
			 "FIFO underrun: %d words (expected %d)\n",
			 fifo_cnt, AD5940_DFT_CHANNELS);
		if (fifo_cnt == 0)
			goto out;
	}

	ret = ad5940_fifo_read(priv, buf,
			       min(fifo_cnt, AD5940_DFT_CHANNELS));
	if (ret) {
		dev_err(&priv->spi->dev, "FIFO read failed: %d\n", ret);
		goto out;
	}

	/* DEBUG: print raw FIFO words */
	{
		static int _raw_cnt;
		if (_raw_cnt < 16) {
			dev_info(&priv->spi->dev,
					"RAW1: %08x %08x %08x %08x\n",
					buf[0], buf[1], buf[2], buf[3]);
			_raw_cnt++;
		}
	}



	/* Clear FIFO threshold interrupt flag in AD5940 INTC */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR,
			 AD5940_AFEINTSRC_DATAFIFOTHRESH);

	/* Unlock sleep key (allow AFE to hibernate after next measurement) */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK,
			 AD5940_SLPKEY_UNLOCK);

	/* Push data to IIO buffer (only if we have a full frame) */
	if (fifo_cnt >= AD5940_DFT_CHANNELS)
		iio_push_to_buffers_with_timestamp(indio_dev, buf,
						   iio_get_time_ns(indio_dev));

out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  IIO info callbacks                                                */
/* ------------------------------------------------------------------ */

/*
 * No sysfs read_raw / write_raw – data is available exclusively
 * through the triggered buffer.  The iio_info struct is minimal.
 */
static const struct iio_info ad5940_iio_info = {
};

/* ------------------------------------------------------------------ */
/*  IRQ handler                                                       */
/* ------------------------------------------------------------------ */

/*
 * ad5940_irq_handler - Hard IRQ handler for AD5940 GPIO interrupt
 *
 * Called when AD5940 GP0 pin goes low (FIFO threshold reached).
 * Disables IRQ to prevent re-entry, then schedules the IIO trigger
 * poll which will run ad5940_trigger_handler in thread context.
 */
static irqreturn_t ad5940_irq_handler(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad5940_priv *priv = iio_priv(indio_dev);


	// dev_info(&priv->spi->dev, "Enter ad5940_irq_handler\n");

	/*
	 * Disable IRQ to prevent re-entry until the trigger handler
	 * finishes and calls iio_trigger_notify_done() → reenable().
	 */
	disable_irq_nosync(priv->irq);

	iio_trigger_poll(priv->trig);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  IIO device setup helpers                                          */
/* ------------------------------------------------------------------ */

static int ad5940_setup_trigger(struct iio_dev *indio_dev)
{
	struct ad5940_priv *priv = iio_priv(indio_dev);
	struct device *dev = &priv->spi->dev;
	int ret;

	priv->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					     indio_dev->name,
					     indio_dev->id);

	if (!priv->trig)
		return -ENOMEM;

	priv->trig->dev.parent = dev;
	priv->trig->ops = &ad5940_trigger_ops;
	iio_trigger_set_drvdata(priv->trig, priv);

	ret = devm_iio_trigger_register(dev, priv->trig);
	if (ret) {
		dev_err(dev, "failed to register IIO trigger: %d\n", ret);
		return ret;
	}

	/*
	 * Associate our trigger as the default for this IIO device.
	 *
	 * We MUST NOT use iio_trigger_get() here because it calls
	 * __module_get(THIS_MODULE), creating a self-reference that
	 * makes the module refcount permanently > 0 and prevents
	 * rmmod from ever succeeding.  Instead, we assign directly
	 * and only take a device reference (get_device) to keep the
	 * trigger's struct device alive — matching the put_device()
	 * in ad5940_remove().
	 */
	indio_dev->trig = priv->trig;
	get_device(&priv->trig->dev);

	return 0;
}

static int ad5940_setup_buffer(struct iio_dev *indio_dev)
{
	struct device *dev = &((struct ad5940_priv *)iio_priv(indio_dev))->spi->dev;

	return devm_iio_triggered_buffer_setup(dev, indio_dev,
					       &iio_pollfunc_store_time,
					       &ad5940_trigger_handler,
					       NULL);
}

/* ------------------------------------------------------------------ */
/*  probe / remove                                                    */
/* ------------------------------------------------------------------ */

static int ad5940_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5940_priv *priv;
	int ret, adiid, chipid;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->spi = spi;
	priv->iio_dev = indio_dev;

	/* Setup SPI: CPOL=0, CPHA=0 (SPI mode 0) */
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 4000000;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(dev, "SPI setup failed: %d\n", ret);
		return ret;
	}

	/* Reset GPIO: initialize to inactive (reset deasserted) */
	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		ret = PTR_ERR(priv->reset_gpio);
		dev_err(dev, "failed to get reset GPIO: %d\n", ret);
		return ret;
	}

	/* ---- Hardware reset ---- */
	ad5940_reset(priv);

	/* ---- Apply post-reset init register sequence ---- */
	ret = ad5940_init(priv);
	if (ret) {
		dev_err(dev, "AFE initialization failed\n");
		return ret;
	}

	/* ---- Read chip identification ---- */
	adiid = ad5940_spi_read(priv, AD5940_REG_ADIID);
	if (adiid < 0) {
		dev_err(dev, "failed to read ADIID: %d\n", adiid);
		return adiid;
	}

	chipid = ad5940_spi_read(priv, AD5940_REG_CHIPID);
	if (chipid < 0) {
		dev_err(dev, "failed to read CHIPID: %d\n", chipid);
		return chipid;
	}

	dev_info(dev, "AD5940 detected: ADIID=0x%04x, CHIPID=0x%04x\n",
		 adiid, chipid);

	if (adiid != AD5940_ADIID_VALUE)
		dev_warn(dev, "unexpected ADIID: 0x%04x (expected 0x%04x)\n",
			 adiid, AD5940_ADIID_VALUE);

	if (chipid != AD5940_CHIPID_VALUE)
		dev_warn(dev, "unexpected CHIPID: 0x%04x (expected 0x%04x)\n",
			 chipid, AD5940_CHIPID_VALUE);

	/* ---- BIA initialization (Phase 1) ---- */
	/*
	 * Initialize AD5940 for BIA measurement:
	 * - Configure all AFE registers (reference, HSLoop, LPLoop, DSP)
	 * - Program init sequence (SEQID_1) and measure sequence (SEQID_0)
	 *   into sequencer SRAM
	 * - Configure FIFO (DFT source, threshold=4)
	 * - Configure interrupt controller (FIFO threshold → GP0)
	 * - Execute init sequence once
	 *
	 * Measurement is NOT started here — it starts when the IIO buffer
	 * is enabled, which calls ad5940_trigger_set_state(true),
	 * which calls ad5940_bia_start() to configure the Wakeup Timer.
	 */
	ret = ad5940_bia_init(priv);
	if (ret) {
		dev_err(dev, "BIA initialization failed: %d\n", ret);
		return ret;
	}

	/* ---- IIO device configuration ---- */
	indio_dev->name		= "ad5940";
	indio_dev->info		= &ad5940_iio_info;
	indio_dev->modes	= INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->channels	= ad5940_dft_channels;
	indio_dev->num_channels	= ARRAY_SIZE(ad5940_dft_channels);

	/* Setup IIO triggered buffer */
	ret = ad5940_setup_buffer(indio_dev);
	if (ret) {
		dev_err(dev, "IIO buffer setup failed: %d\n", ret);
		return ret;
	}

	/* Setup IIO trigger (driven by AD5940 GPIO interrupt) */
	ret = ad5940_setup_trigger(indio_dev);
	if (ret) {
		dev_err(dev, "IIO trigger setup failed: %d\n", ret);
		return ret;
	}

	/* Register IIO device */
	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "IIO device register failed: %d\n", ret);
		return ret;
	}

	/* Request IRQ after IIO is fully set up */
	priv->irq = spi->irq;
	if (priv->irq > 0) {
		ret = devm_request_threaded_irq(dev, priv->irq,
						ad5940_irq_handler, NULL,
						IRQF_TRIGGER_FALLING,
						"ad5940", indio_dev);
		if (ret)
			dev_warn(dev, "IRQ request failed (%d), continuing\n",
				 ret);
		else
			dev_info(dev, "IRQ %d registered\n", priv->irq);
	}

	spi_set_drvdata(spi, indio_dev);

	return 0;
}

static int ad5940_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad5940_priv *priv = iio_priv(indio_dev);

	/* Stop BIA measurement (disable WUPT) */
	ad5940_bia_stop(priv);

	/*
	 * Clear the trigger association before devm cleanup runs.
	 * This prevents iio_device_unregister_trigger_consumer()
	 * (called from iio_dev_release) from calling iio_trigger_put(),
	 * which would do an unpaired module_put() since we never called
	 * iio_trigger_get() in setup — we only did get_device().
	 */
	if (indio_dev->trig) {
		put_device(&indio_dev->trig->dev);
		indio_dev->trig = NULL;
	}

	/* Assert reset (active) to put AD5940 in low-power state */
	gpiod_set_value(priv->reset_gpio, 1);

	dev_info(&spi->dev, "ad5940_driver removed.\n");

	return 0;
}

/* ------------------------------------------------------------------ */
/*  Driver registration                                               */
/* ------------------------------------------------------------------ */

static const struct of_device_id ad5940_of_match[] = {
	{ .compatible = "adi,ad5940" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5940_of_match);

static struct spi_driver ad5940_driver = {
	.driver = {
		.name = "ad5940",
		.of_match_table = ad5940_of_match,
	},
	.probe  = ad5940_probe,
	.remove = ad5940_remove,
};
module_spi_driver(ad5940_driver);

MODULE_AUTHOR("Mason Wang");
MODULE_DESCRIPTION("AD5940 AFE SPI driver with IIO BIA support");
MODULE_LICENSE("GPL");
