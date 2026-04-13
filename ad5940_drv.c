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
/*  IIO channel definitions                                           */
/* ------------------------------------------------------------------ */

/*
 * Two ADC data channels (voltage & current) plus a timestamp.
 * .scan_index / .scan_type describe the layout of each sample in
 * the IIO triggered buffer.
 */
#define AD5940_VOLTAGE_CH	0
#define AD5940_CURRENT_CH	1
#define AD5940_TIMESTAMP_CH	2

static const struct iio_chan_spec ad5940_channels[] = {
	[AD5940_VOLTAGE_CH] = {
		.type			= IIO_VOLTAGE,
		.indexed		= 1,
		.channel		= 0,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index		= AD5940_VOLTAGE_CH,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 32,
			.storagebits	= 32,
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_CURRENT_CH] = {
		.type			= IIO_CURRENT,
		.indexed		= 1,
		.channel		= 0,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index		= AD5940_CURRENT_CH,
		.scan_type		= {
			.sign		= 's',
			.realbits	= 32,
			.storagebits	= 32,
			.endianness	= IIO_CPU,
		},
	},
	[AD5940_TIMESTAMP_CH] = IIO_CHAN_SOFT_TIMESTAMP(AD5940_TIMESTAMP_CH),
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
	int ret;

	if (state) {
		/*
		 * TODO: Enable FIFO threshold interrupt in AD5940
		 * interrupt controller.  Placeholder: enable AFE.
		 */
		ret = ad5940_spi_write(priv, AD5940_REG_AFECON,
				       AD5940_AFECON_FIFOEN);
	} else {
		/* TODO: Disable FIFO threshold interrupt */
		ret = ad5940_spi_write(priv, AD5940_REG_AFECON, 0);
	}

	return ret;
}

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
 * Called from the IRQ thread when the AD5940 asserts data-ready.
 * Reads FIFO samples and pushes them into the IIO buffer.
 */
static irqreturn_t ad5940_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad5940_priv *priv = iio_priv(indio_dev);
	s32 buf[AD5940_IIO_CHANNELS + 1]; /* 2 data ch + padding for ts */
	int ret, fifo_cnt, i;

	memset(buf, 0, sizeof(buf));

	/*
	 * TODO: Read FIFO count register to know how many samples are
	 * available.  For now, read a single sample pair as skeleton.
	 */
	fifo_cnt = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	if (fifo_cnt < 0) {
		dev_err(&priv->spi->dev, "FIFO count read failed: %d\n",
			fifo_cnt);
		goto out;
	}

	/*
	 * TODO: Implement proper FIFO burst read via AD5940_SPI_CMD_READFIFO
	 * and parse ADC data into voltage / current channels.
	 * For now, read two 32-bit FIFO words as placeholder data.
	 */
	for (i = 0; i < 2; i++) {
		ret = ad5940_spi_read(priv, AD5940_REG_FIFODATA);
		if (ret < 0) {
			dev_err(&priv->spi->dev,
				"FIFO read failed: %d\n", ret);
			goto out;
		}
		buf[i] = ret;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf,
					   iio_get_time_ns(indio_dev));
out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  IIO info callbacks                                                */
/* ------------------------------------------------------------------ */

static int ad5940_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad5940_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * TODO: Read the appropriate ADC result register based on
		 * chan->channel.  For now return a placeholder FIFO read.
		 */
		ret = ad5940_spi_read(priv, AD5940_REG_FIFODATA);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		/* TODO: Return actual sample frequency from AFE config */
		*val = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ad5940_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* TODO: Configure AFE sample rate */
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5940_iio_info = {
	.read_raw	= ad5940_read_raw,
	.write_raw	= ad5940_write_raw,
};

/* ------------------------------------------------------------------ */
/*  IRQ handler                                                       */
/* ------------------------------------------------------------------ */

static irqreturn_t ad5940_irq_handler(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad5940_priv *priv = iio_priv(indio_dev);

	/*
	 * Disable IRQ to prevent re-entry until the trigger handler
	 * finishes and calls iio_trigger_notify_done() → reenable().
	 */
	disable_irq_nosync(priv->irq);

	/*
	 * TODO: Read INTCFLAG to determine interrupt source (FIFO
	 * threshold, sequencer done, etc.) and act accordingly.
	 */

	iio_trigger_poll_chained(priv->trig);

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

	indio_dev->trig = iio_trigger_get(priv->trig);

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
	indio_dev->modes	= INDIO_DIRECT_MODE;
	indio_dev->channels	= ad5940_channels;
	indio_dev->num_channels	= ARRAY_SIZE(ad5940_channels);

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
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
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

	/* Assert reset (active) to put AD5940 in low-power state */
	gpiod_set_value(priv->reset_gpio, 1);

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
