// SPDX-License-Identifier: GPL-2.0
/*
 * AD5940 AFE Linux SPI driver – platform driver + IIO integration
 *
 * This file contains the SPI driver probe/remove, interrupt handler,
 * IIO device registration with triggered buffer, and trigger ops.
 * Low-level register access lives in ad5940_core.c.
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
/*  IIO channel definitions – DFT impedance mode                     */
/* ------------------------------------------------------------------ */

/*
 * AD5940 FIFO in DFT (FIFOSRC_DFT) mode outputs 4 words per measurement
 * cycle.  The FIFO word order is determined by the Sequencer program and
 * ADC MUX switching sequence — it is NOT fixed by hardware.
 *
 * Current channel mapping follows the BIA (BodyImpedance) example:
 *   BodyImpedance.c measures in 2 ADC MUX steps per cycle:
 *     Step 1: ADC MUX = HSTIA_P/HSTIA_N → DFT of current through RTIA
 *     Step 2: ADC MUX = AIN3/AIN2       → DFT of voltage across body
 *   Since Step 1 runs first, current DFT enters FIFO before voltage DFT.
 *
 *   FIFO word order (BIA, 4-wire):
 *     word0 = Current Real     (iImpCar_Type: pDftCurr->Real)
 *     word1 = Current Imag     (iImpCar_Type: pDftCurr->Image)
 *     word2 = Voltage Real     (iImpCar_Type: pDftVolt->Real)
 *     word3 = Voltage Imag     (iImpCar_Type: pDftVolt->Image)
 *
 *   Electrode configuration (BIA, 4-wire):
 *     Current path:  CE0 → body → AIN1 → HSTIA (RTIA)
 *     Voltage sense: AIN3(+) / AIN2(-) — independent high-Z detection
 *
 *   Reference: ad5940_example/AD5940_BIA/BodyImpedance.c
 *     - AppBIADataProcess(): pDftCurr = pSrcData++, pDftVolt = pSrcData++
 *     - Measure sequence: ADC MUX HSTIA first, then AIN3/AIN2
 *
 * DFT FIFO word format (differs from standard 16-bit data):
 *   - Standard ADC data: 16-bit, mask = 0xFFFF
 *   - DFT result:        18-bit signed two's complement in bits[17:0]
 *     Bit17 is the sign bit.  ADI examples sign-extend with:
 *       pData[i] &= 0x3FFFF;
 *       if (pData[i] & (1L << 17)) pData[i] |= 0xFFFC0000;
 *
 * scan_type is set to s18/32>>0 to describe the raw FIFO format.
 * The driver pushes raw 32-bit FIFO words directly; sign-extension
 * is handled by user-space libiio based on the _type attribute
 * ("le:s18/32>>0").  This also facilitates future SPI DMA transfers
 * where raw FIFO data can be memcpy'd into the IIO buffer without
 * any per-sample CPU processing.
 *
 * No info_mask is set: data is available exclusively via triggered buffer.
 * Sysfs read_raw / write_raw are not provided for these channels.
 *
 * NOTE: If switching to a different measurement configuration (e.g.
 * Impedance, BIOZ-2Wire, BIA_HiZ), the FIFO word order may change.
 * Update the scan_index assignments and this comment accordingly.
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
 * The trigger is driven by the AD5940 GPIO interrupt (FIFO threshold
 * or data-ready).  set_trigger_state enables/disables the interrupt
 * source inside the AFE.
 */
static int ad5940_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct ad5940_priv *priv = iio_trigger_get_drvdata(trig);
	int ret = 0;

	if (state) {
		/*
		 * TODO: Enable FIFO threshold interrupt in AD5940
		 * interrupt controller.  Placeholder: enable AFE.
		 */
		dev_info(&priv->spi->dev, "ad5940_trigger_set_state 1\n");

		// ret = ad5940_spi_write(priv, AD5940_REG_AFECON,
		// 		       AD5940_AFECON_FIFOEN);
	} else {
		/* TODO: Disable FIFO threshold interrupt */

		dev_info(&priv->spi->dev, "ad5940_trigger_set_state 0\n");

		// ret = ad5940_spi_write(priv, AD5940_REG_AFECON, 0);
	}

	return ret;
}

static int ad5940_trigger_reenable(struct iio_trigger *trig)
{
	struct ad5940_priv *priv = iio_trigger_get_drvdata(trig);

	dev_info(&priv->spi->dev, "ad5940_trigger_reenable\n");

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
 * Called from the IRQ thread when the AD5940 asserts data-ready.
 * Reads FIFO samples and pushes them into the IIO buffer.
 *
 * In BIA 4-wire DFT mode the FIFO outputs 4 words per measurement cycle:
 *   word0 = Current Real,  word1 = Current Imag,
 *   word2 = Voltage Real,  word3 = Voltage Imag.
 *
 * This order matches the Sequencer program's ADC MUX switching:
 *   Step 1: HSTIA_P/HSTIA_N (current) → DFT → FIFO word0,1
 *   Step 2: AIN3/AIN2       (voltage) → DFT → FIFO word2,3
 *
 * Each FIFO word contains an 18-bit signed DFT result in bits[17:0]
 * (bit17 is the sign bit).  We push the raw 32-bit FIFO words directly
 * into the IIO buffer without sign-extension; user-space libiio handles
 * the 18-bit sign extension based on the channel's _type attribute
 * ("le:s18/32>>0").  This approach also enables future SPI DMA support
 * where raw FIFO data can be memcpy'd without per-sample CPU processing.
 */
static irqreturn_t ad5940_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad5940_priv *priv = iio_priv(indio_dev);

	dev_info(&priv->spi->dev, "ad5940_trigger_handler\n");

	// /*
	//  * Buffer layout for iio_push_to_buffers_with_timestamp():
	//  *   4 × u32 channel data  +  1 × s64 timestamp
	//  * The timestamp is appended after channel data by the framework,
	//  * so we must reserve space for it to avoid stack overflow.
	//  */
	// u32 buf[AD5940_DFT_CHANNELS + sizeof(s64) / sizeof(u32)];
	// int ret, fifo_cnt;

	// memset(buf, 0, sizeof(buf));

	// ret = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	// if (ret < 0) {
	// 	dev_err(&priv->spi->dev, "FIFO count read failed: %d\n",
	// 		ret);
	// 	goto out;
	// }
	// /* FIFOCNTSTA.DATAFIFOCNTSTA is in bits[26:16], per AD5940 datasheet */
	// fifo_cnt = ret >> 16;

	// /*
	//  * TODO: Handle fifo_cnt > 4 (multiple measurement cycles in FIFO).
	//  * For now, read exactly one DFT frame (4 words).
	//  *
	//  * Use ad5940_fifo_read() which sets the FIFO address once then
	//  * reads words sequentially — matching ADI's recommended method
	//  * for small read counts (< 3 words per CS toggle).
	//  * It returns 0 on success (data via output param), avoiding the
	//  * ambiguity of ad5940_spi_read() where 32-bit FIFO data with
	//  * bit31 set would be indistinguishable from a negative errno.
	//  */
	// ret = ad5940_fifo_read(priv, buf, AD5940_DFT_CHANNELS);
	// if (ret) {
	// 	dev_err(&priv->spi->dev, "FIFO read failed: %d\n", ret);
	// 	goto out;
	// }

	// iio_push_to_buffers_with_timestamp(indio_dev, buf,
	// 				   iio_get_time_ns(indio_dev));
// out:
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

static irqreturn_t ad5940_irq_handler(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad5940_priv *priv = iio_priv(indio_dev);

	dev_info(&priv->spi->dev, "ad5940_irq_handler\n");

	/*
	 * Disable IRQ to prevent re-entry until the trigger handler
	 * finishes and calls iio_trigger_notify_done() → reenable().
	 */
	disable_irq_nosync(priv->irq);

	/*
	 * TODO: Read INTCFLAG to determine interrupt source (FIFO
	 * threshold, sequencer done, etc.) and act accordingly.
	 */

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

	dev_info(dev, "iio trig name: %s-dev%d\n", indio_dev->name, indio_dev->id);

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
	 *
	 * Setting indio_dev->trig = NULL in remove() prevents
	 * iio_device_unregister_trigger_consumer() from calling
	 * iio_trigger_put(), which would otherwise do an unpaired
	 * module_put().
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

	/* Setup SPI: CPOL=0, CPHA=0 (SPI mode 0)
	 * Per AD5940 datasheet, CS asserts before the first SCLK rising
	 * edge, implying SCLK idles LOW. ADI examples confirm CPOL=0.
	 */
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

	/* Hardware reset */
	ad5940_reset(priv);

	/*
	 * Apply post-reset initialization register sequence.
	 * AD5940 is in active state right after hardware reset, so we can
	 * write registers directly.
	 */
	ret = ad5940_init(priv);
	if (ret) {
		dev_err(dev, "AFE initialization failed\n");
		return ret;
	}

	/* Read chip identification */
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

	/* ---- IIO device configuration ---- */
	indio_dev->name		= "ad5940";
	indio_dev->info		= &ad5940_iio_info;
	/* INDIO_DIRECT_MODE: sysfs 单次读取（当前未提供 read_raw）
	 * INDIO_BUFFER_TRIGGERED: 通过 trigger + buffer 连续采集
	 * INDIO_BUFFER_HARDWARE 不需要——那是给硬件 FIFO 直推的（如 DMA buffer），4.19 用的是 kfifo 后端
	 */
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
MODULE_DESCRIPTION("AD5940 AFE SPI driver with IIO support");
MODULE_LICENSE("GPL");
