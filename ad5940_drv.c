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
/*  Module parameters for frequency sweep configuration                */
/* ------------------------------------------------------------------ */

static bool sweep_en;
module_param(sweep_en, bool, 0644);
MODULE_PARM_DESC(sweep_en, "Enable frequency sweep (default: 0)");

static uint sweep_start_hz = 10000;
module_param(sweep_start_hz, uint, 0644);
MODULE_PARM_DESC(sweep_start_hz, "Sweep start frequency in Hz (default: 10000)");

static uint sweep_stop_hz = 150000;
module_param(sweep_stop_hz, uint, 0644);
MODULE_PARM_DESC(sweep_stop_hz, "Sweep stop frequency in Hz (default: 150000)");

static uint sweep_points = 100;
module_param(sweep_points, uint, 0644);
MODULE_PARM_DESC(sweep_points, "Number of sweep frequency points (default: 100)");

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
#define AD5940_DFT_FREQ		4
#define AD5940_DFT_TIMESTAMP	5

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
	[AD5940_DFT_FREQ] = {
		.type			= IIO_ALTVOLTAGE,
		.indexed		= 1,
		.channel		= 0,
		.scan_index		= AD5940_DFT_FREQ,
		.scan_type		= {
			.sign		= 'u',
			.realbits	= 32,
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

	priv->irq_disabled = false;
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
 * Reads all available DFT frames from FIFO (aligned to 4-word boundaries,
 * matching ADI's FifoCnt = (FIFOGetCnt()/4)*4 pattern) and pushes each
 * frame into the IIO buffer with a timestamp.
 *
 * Also implements ADI's STOPSYNC pattern: if stop_required is set,
 * disables WUPT after safely reading FIFO data (AppBIARegModify).
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
	 *   4 × u32 DFT data + 1 × u32 frequency  +  1 × s64 timestamp
	 * The timestamp is appended after channel data by the framework,
	 * so we must reserve space for it to avoid stack overflow.
	 */
	u32 buf[AD5940_DFT_CHANNELS + sizeof(s64) / sizeof(u32)];
	int ret, fifo_cnt, read_cnt, frames, frame;

	memset(buf, 0, sizeof(buf));

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

	// /* ---- Debug: verify clock, WUPT, sequencer & AFE state ---- */
	// {
	// 	static int clk_dbg_cnt;
	// 	static ktime_t last_ts;
	// 	ktime_t now = ktime_get();
	// 	s64 dt_us = last_ts ? ktime_us_delta(now, last_ts) : 0;

	// 	if (clk_dbg_cnt < 30) {
	// 		int osccon, hposccon, clkcon0, clksel;
	// 		int wuptcon, tmrcon;
	// 		int seq0wupl, seq0wuph, seq0sleepl, seq0sleeph;
	// 		u32 wakeup_time, sleep_time, wupt_period;
	// 		u32 sysclk_div, adcclk_div, sysclk_src, adcclk_src;
	// 		bool hfosc_16mhz;
	// 		int afecon, seqcon, seqcnt, seq0info;
	// 		int intcflag0, intcflag1;

	// 		/* Clock & WUPT registers */
	// 		osccon   = ad5940_spi_read(priv, AD5940_REG_ALLON_OSCCON);
	// 		hposccon = ad5940_spi_read(priv, AD5940_REG_HPOSCCON);
	// 		clkcon0  = ad5940_spi_read(priv, AD5940_REG_CLKCON0);
	// 		clksel   = ad5940_spi_read(priv, AD5940_REG_CLKSEL);
	// 		wuptcon  = ad5940_spi_read(priv, AD5940_REG_WUPTCON);
	// 		tmrcon   = ad5940_spi_read(priv, AD5940_REG_ALLON_TMRCON);
	// 		seq0wupl  = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0WUPL);
	// 		seq0wuph  = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0WUPH);
	// 		seq0sleepl = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0SLEEPL);
	// 		seq0sleeph = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0SLEEPH);

	// 		/* Sequencer & AFE state */
	// 		afecon   = ad5940_spi_read(priv, AD5940_REG_AFECON);
	// 		seqcon   = ad5940_spi_read(priv, AD5940_REG_SEQCON);
	// 		seqcnt   = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
	// 		seq0info = ad5940_spi_read(priv, AD5940_REG_SEQ0INFO);
	// 		intcflag0 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG0);
	// 		intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);

	// 		/* Decode clock config */
	// 		sysclk_div  = (clkcon0 >> AD5940_CLKCON0_SYSCLKDIV_SHIFT) & 0x3F;
	// 		adcclk_div  = (clkcon0 >> AD5940_CLKCON0_ADCCLKDIV_SHIFT) & 0xF;
	// 		sysclk_src  = (clksel >> AD5940_CLKSEL_SYSCLKSEL_SHIFT) & 0x3;
	// 		adcclk_src  = (clksel >> AD5940_CLKSEL_ADCCLKSEL_SHIFT) & 0x3;
	// 		hfosc_16mhz = !!(hposccon & AD5940_HPOSCCON_CLK32MHZEN);

	// 		/* Decode WUPT timing */
	// 		wakeup_time = (u32)seq0wupl | ((u32)seq0wuph << 16);
	// 		sleep_time  = (u32)seq0sleepl | ((u32)seq0sleeph << 16);
	// 		wupt_period = (wakeup_time + 1) + (sleep_time + 1);

	// 		/* Decode AFECON */
	// 		dev_info(&priv->spi->dev,
	// 			 "CLK[%d] dt=%lldus | OSCCON=0x%x HFOSC=%s LFOSC=%s | "
	// 			 "HPOSCCON=0x%x(%s) CLKCON0=0x%x SysDiv=%u AdcDiv=%u | "
	// 			 "CLKSEL=0x%x SysSrc=%u AdcSrc=%u\n",
	// 			 clk_dbg_cnt, dt_us,
	// 			 osccon,
	// 			 (osccon & AD5940_OSCCON_HFOSCOK) ? "OK" : "NOT_RDY",
	// 			 (osccon & AD5940_OSCCON_LFOSCOK) ? "OK" : "NOT_RDY",
	// 			 hposccon, hfosc_16mhz ? "16MHz" : "32MHz",
	// 			 clkcon0, sysclk_div, adcclk_div,
	// 			 clksel, sysclk_src, adcclk_src);

	// 		dev_info(&priv->spi->dev,
	// 			 "  WUPT: CON=0x%x EN=%d ENDSEQ=%u CLKSEL=%u | "
	// 			 "TMRCON=0x%x TMRINTEN=%d | "
	// 			 "WkpT=%u SlpT=%u Period=%u => ODR=%uHz\n",
	// 			 wuptcon, wuptcon & 1,
	// 			 (wuptcon >> 1) & 0x7, (wuptcon >> 4) & 0x3,
	// 			 tmrcon, tmrcon & 1,
	// 			 wakeup_time, sleep_time, wupt_period,
	// 			 wupt_period ? AD5940_BIA_WUPT_CLK_FREQ / wupt_period : 0);

	// 		dev_info(&priv->spi->dev,
	// 			 "  SEQ: CON=0x%x EN=%d HALT=%d | CNT=%u | "
	// 			 "SEQ0INFO=0x%x Addr=%u Len=%u\n",
	// 			 seqcon, seqcon & 1, (seqcon >> 1) & 1,
	// 			 seqcnt & 0xFFFF,
	// 			 seq0info, seq0info & 0xFFFF, (seq0info >> 16) & 0xFFFF);

	// 		dev_info(&priv->spi->dev,
	// 			 "  AFE: AFECON=0x%x WG=%d ADCPWR=%d ADCCNV=%d DFT=%d "
	// 			 "SINC2NOTCH=%d HPREFDIS=%d | "
	// 			 "INTC0=0x%x INTC1=0x%x\n",
	// 			 afecon,
	// 			 !!(afecon & AD5940_AFECON_WG),
	// 			 !!(afecon & AD5940_AFECON_ADCPWR),
	// 			 !!(afecon & AD5940_AFECON_ADCCNV),
	// 			 !!(afecon & AD5940_AFECON_DFT),
	// 			 !!(afecon & AD5940_AFECON_SINC2NOTCH),
	// 			 !!(afecon & AD5940_AFECON_HPREFDIS),
	// 			 intcflag0, intcflag1);

	// 		last_ts = now;
	// 		clk_dbg_cnt++;
	// 	}
	// }

	/* Read FIFO count: FIFOCNTSTA.DATAFIFOCNTSTA is bits[26:16] (11-bit field) */
	ret = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "FIFO count read failed: %d\n",
			ret);
		goto out;
	}
	fifo_cnt = (ret >> 16) & 0x7FF;  /* Mask 11-bit DATAFIFOCNTSTA field */

	/*
	 * Align to 4-word frames (ADI: FifoCnt = (FIFOGetCnt()/4)*4).
	 * This ensures we always read complete DFT frames and leaves
	 * any partial frame for the next interrupt.
	 */
	read_cnt = (fifo_cnt / AD5940_FIFO_WORDS_PER_FRAME) *
		   AD5940_FIFO_WORDS_PER_FRAME;

	// /* DEBUG: print FIFOCNT raw value and extracted count */
	// {
	// 	static int _dbg_cnt;
	// 	if (_dbg_cnt < 16) {
	// 		dev_info(&priv->spi->dev,
	// 			 "FIFO: raw=0x%x cnt=%d read_cnt=%d\n",
	// 			 ret, fifo_cnt, read_cnt);
	// 		_dbg_cnt++;
	// 	}
	// }

	if (read_cnt == 0) {
		dev_warn(&priv->spi->dev,
			 "FIFO underrun: %d words (no complete frame)\n",
			 fifo_cnt);
		goto out_clear;
	}

	/*
	 * Read all available frames from FIFO and push each one
	 * to the IIO buffer separately (with its own timestamp).
	 * ADI reads all frames in one FIFORd call then processes;
	 * we read frame-by-frame since IIO needs per-frame timestamps.
	 */
	frames = read_cnt / AD5940_FIFO_WORDS_PER_FRAME;
	for (frame = 0; frame < frames; frame++) {
		ret = ad5940_fifo_read(priv, buf, AD5940_FIFO_WORDS_PER_FRAME);
		if (ret) {
			dev_err(&priv->spi->dev,
				"FIFO read failed at frame %d/%d: %d\n",
				frame, frames, ret);
			goto out_clear;
		}

		/* Fill frequency channel with current measurement frequency */
		buf[AD5940_DFT_FREQ] = priv->freq_of_data_hz;

		iio_push_to_buffers_with_timestamp(indio_dev, buf,
						   iio_get_time_ns(indio_dev));

		/* Advance sweep state and update WGFCW for next cycle */
		if (priv->sweep_en) {
			ret = ad5940_bia_sweep_step(priv);
			if (ret) {
				dev_err(&priv->spi->dev,
					"sweep step failed: %d\n", ret);
				goto out_clear;
			}
		}
	}

out_clear:
	/* Clear FIFO threshold interrupt flag in AD5940 INTC */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR,
			 AD5940_AFEINTSRC_DATAFIFOTHRESH);

	/* Unlock sleep key (allow AFE to hibernate after next measurement) */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK,
			 AD5940_SLPKEY_UNLOCK);

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

	/*
	 * Disable IRQ to prevent re-entry until the trigger handler
	 * finishes and calls iio_trigger_notify_done() → reenable().
	 * Track the disabled state so ad5940_bia_stop() can re-enable
	 * the IRQ if the trigger handler never runs (e.g., buffer
	 * disabled between interrupt and handler execution).
	 */
	disable_irq_nosync(priv->irq);
	priv->irq_disabled = true;

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

	/* Configure sweep from module parameters */
	priv->sweep_en = sweep_en;
	priv->sweep_type = AD5940_SWEEP_LINEAR;
	priv->sweep_start_hz = sweep_start_hz;
	priv->sweep_stop_hz = sweep_stop_hz;
	priv->sweep_points = clamp_val(sweep_points, 1,
				       AD5940_MAX_SWEEP_POINTS);
	if (priv->sweep_en) {
		dev_info(dev, "Sweep: %uHz - %uHz, %u points, linear\n",
			 priv->sweep_start_hz, priv->sweep_stop_hz,
			 priv->sweep_points);
	}

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
