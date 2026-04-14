/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AD5940 AFE core register access and initialization
 *
 * Low-level SPI register R/W, hardware reset/wakeup, and post-reset
 * initialization sequence.  Used by the platform driver and IIO layers.
 */
#ifndef _AD5940_CORE_H_
#define _AD5940_CORE_H_

#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

/* AD5940 SPI command opcodes */
#define AD5940_SPI_CMD_SETADDR	0x20
#define AD5940_SPI_CMD_WRITEREG	0x2d
#define AD5940_SPI_CMD_READREG	0x6d
#define AD5940_SPI_CMD_READFIFO	0x5f

/* AD5940 key register addresses (verified against ad5940lib/ad5940.h) */
#define AD5940_REG_ADIID	0x0400	/* REG_AFECON_ADIID, 16-bit */
#define AD5940_REG_CHIPID	0x0404	/* REG_AFECON_CHIPID, 16-bit */
#define AD5940_REG_INTCPOL	0x3000	/* REG_INTC_INTCPOL */
#define AD5940_REG_INTCFLAG0	0x3010	/* REG_INTC_INTCFLAG0 */
#define AD5940_REG_INTCFLAG1	0x3014	/* REG_INTC_INTCFLAG1 */
#define AD5940_REG_AFECON	0x2000	/* REG_AFE_AFECON, 32-bit */
#define AD5940_REG_FIFOCON	0x2008	/* REG_AFE_FIFOCON, 32-bit */
#define AD5940_REG_FIFOCNT	0x2200	/* REG_AFE_FIFOCNTSTA, 32-bit */
#define AD5940_REG_FIFODATA	0x206C	/* REG_AFE_DATAFIFORD, 32-bit */

/* AFECON bits */
#define AD5940_AFECON_FIFOEN	BIT(0)

/* Expected chip identification values */
#define AD5940_ADIID_VALUE	0x4144
#define AD5940_CHIPID_VALUE	0x5502

/* Timing */
#define AD5940_RESET_PULSE_MS	10
#define AD5940_RESET_WAIT_MS	10
#define AD5940_WAKEUP_RETRIES	5

/* FIFO threshold for interrupt generation */
#define AD5940_FIFO_THRESHOLD	4

/* Number of DFT data channels (voltage real/imag + current real/imag) */
#define AD5940_DFT_CHANNELS	4

/*
 * AD5940 post-reset initialization register table (AD5940 variant, not ADuCM355).
 * Sourced from AD5940_Initialize() in ad5940lib/ad5940.c.
 * Must be applied after every hardware/software reset to put the AFE in a
 * known state: enable CS-wakeup, configure FIFO, gate unused clocks, etc.
 */
static const struct {
	u16 addr;
	u32 val;
} ad5940_init_regs[] = {
	{ 0x0908, 0x02c9 },
	{ 0x0c08, 0x206C },	/* REG_ALLON_SRAMCON: CS wakeup & SRAM retention */
	{ 0x21F0, 0x0010 },
	{ 0x0410, 0x02c9 },	/* AD5940 variant (not ADuCM355) */
	{ 0x0A28, 0x0009 },	/* REG_ALLON_EI2CON: external interrupt config 2 */
	{ 0x238c, 0x0104 },
	{ 0x0a04, 0x4859 },	/* REG_ALLON_PWRKEY: unlock PWRMOD key 1 */
	{ 0x0a04, 0xF27B },	/* REG_ALLON_PWRKEY: unlock PWRMOD key 2 */
	{ 0x0a00, 0x8009 },	/* REG_ALLON_PWRMOD: set power mode */
	{ 0x22F0, 0x0000 },
	{ 0x2230, 0xDE87A5AF },	/* Sleep key: unlock */
	{ 0x2250, 0x103F },
	{ 0x22B0, 0x203C },
	{ 0x2230, 0xDE87A5A0 },	/* Sleep key: lock */
};

/**
 * struct ad5940_priv - AD5940 driver private data
 *
 * @spi:        SPI device handle
 * @reset_gpio: GPIO descriptor for hardware reset (active-low)
 * @irq:        Linux IRQ number from DT
 * @trig:       IIO trigger (data-ready / FIFO threshold)
 * @iio_dev:    Pointer back to the IIO device (for use in trigger ops)
 */
struct ad5940_priv {
	struct spi_device	*spi;
	struct gpio_desc	*reset_gpio;
	int			irq;

	/* IIO trigger (FIFO threshold / data-ready) */
	struct iio_trigger	*trig;

	/* Pointer back to the IIO device (for use in trigger ops) */
	struct iio_dev		*iio_dev;
};

/* ---- Core register access API ---- */

int ad5940_spi_xfer(struct ad5940_priv *priv,
		    const u8 *tx, u8 *rx, size_t len);
int ad5940_spi_write(struct ad5940_priv *priv, u16 reg, u32 val);
int ad5940_spi_read(struct ad5940_priv *priv, u16 reg);
int ad5940_fifo_read(struct ad5940_priv *priv, u32 *buf, int count);
void ad5940_reset(struct ad5940_priv *priv);
int ad5940_wakeup(struct ad5940_priv *priv);
int ad5940_init(struct ad5940_priv *priv);

#endif /* _AD5940_CORE_H_ */
