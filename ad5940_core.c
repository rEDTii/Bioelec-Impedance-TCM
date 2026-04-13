// SPDX-License-Identifier: GPL-2.0
/*
 * AD5940 AFE core register access and initialization
 *
 * Low-level SPI register R/W, hardware reset/wakeup, and post-reset
 * initialization sequence extracted from the platform driver so they
 * can be shared with the IIO layer and future measurement modes.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "ad5940_core.h"

/* ------------------------------------------------------------------ */
/*  Low-level SPI xfer helper (full-duplex, single CS-gated transfer) */
/* ------------------------------------------------------------------ */

int ad5940_spi_xfer(struct ad5940_priv *priv,
		    const u8 *tx, u8 *rx, size_t len)
{
	struct spi_message msg;
	struct spi_transfer xfer;

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx;
	xfer.rx_buf = rx;
	xfer.len = len;
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(priv->spi, &msg);
}
EXPORT_SYMBOL_GPL(ad5940_spi_xfer);

/* ------------------------------------------------------------------ */
/*  AD5940 two-transaction register access protocol                    */
/* ------------------------------------------------------------------ */

/**
 * ad5940_spi_write - Write a 16/32-bit register
 * @priv: driver private data
 * @reg: register address
 * @val: value to write
 *
 * Transaction 1: CS↓ [CMD_SETADDR] [reg_addr_hi] [reg_addr_lo] CS↑
 * Transaction 2: CS↓ [CMD_WRITEREG] [data...] CS↑
 *   (16-bit or 32-bit depending on reg range)
 */
int ad5940_spi_write(struct ad5940_priv *priv, u16 reg, u32 val)
{
	u8 tx[5], rx[5];
	int ret;

	/* Phase 1: set address */
	tx[0] = AD5940_SPI_CMD_SETADDR;
	tx[1] = (reg >> 8) & 0xff;
	tx[2] = reg & 0xff;
	ret = ad5940_spi_xfer(priv, tx, rx, 3);
	if (ret)
		return ret;

	/* Phase 2: write data */
	tx[0] = AD5940_SPI_CMD_WRITEREG;
	if (reg >= 0x1000 && reg <= 0x3014) {
		tx[1] = (val >> 24) & 0xff;
		tx[2] = (val >> 16) & 0xff;
		tx[3] = (val >> 8)  & 0xff;
		tx[4] = val & 0xff;
		return ad5940_spi_xfer(priv, tx, rx, 5);
	} else {
		tx[1] = (val >> 8) & 0xff;
		tx[2] = val & 0xff;
		return ad5940_spi_xfer(priv, tx, rx, 3);
	}
}
EXPORT_SYMBOL_GPL(ad5940_spi_write);

/**
 * ad5940_spi_read - Read a 16/32-bit register
 * @priv: driver private data
 * @reg: register address
 *
 * Transaction 1: [CMD_SETADDR] [reg_addr_hi] [reg_addr_lo]
 * Transaction 2: [CMD_READREG] [dummy] [data_hi] ... [data_lo]
 *
 * Return: register value on success, negative errno on failure
 */
int ad5940_spi_read(struct ad5940_priv *priv, u16 reg)
{
	u8 tx[6], rx[6];
	int ret, xfer_len;

	/* Phase 1: set address */
	tx[0] = AD5940_SPI_CMD_SETADDR;
	tx[1] = (reg >> 8) & 0xff;
	tx[2] = reg & 0xff;
	ret = ad5940_spi_xfer(priv, tx, rx, 3);
	if (ret)
		return ret;

	/* Phase 2: full-duplex read (CMD + dummy + N data bytes) */
	if (reg >= 0x1000 && reg <= 0x3014) {
		/* 32-bit register: CMD + 1 dummy + 4 data = 6 bytes */
		memset(tx, 0, 6);
		tx[0] = AD5940_SPI_CMD_READREG;
		xfer_len = 6;
	} else {
		/* 16-bit register: CMD + 1 dummy + 2 data = 4 bytes */
		memset(tx, 0, 4);
		tx[0] = AD5940_SPI_CMD_READREG;
		xfer_len = 4;
	}

	ret = ad5940_spi_xfer(priv, tx, rx, xfer_len);
	if (ret)
		return ret;

	if (reg >= 0x1000 && reg <= 0x3014)
		return (rx[2] << 24) | (rx[3] << 16) |
		       (rx[4] << 8)  | rx[5];
	else
		return (rx[2] << 8) | rx[3];
}
EXPORT_SYMBOL_GPL(ad5940_spi_read);

/* ------------------------------------------------------------------ */
/*  Hardware reset & wake-up                                          */
/* ------------------------------------------------------------------ */

/**
 * ad5940_reset - Hardware reset sequence
 *
 * reset-gpios is active-low in DT. gpiod semantics:
 *   gpiod_set_value(gpio, 1) = active  = physical LOW  (assert reset)
 *   gpiod_set_value(gpio, 0) = inactive = physical HIGH (deassert reset)
 */
void ad5940_reset(struct ad5940_priv *priv)
{
	/* Assert reset (physical LOW) */
	gpiod_set_value(priv->reset_gpio, 1);
	msleep(AD5940_RESET_PULSE_MS);
	/* Deassert reset (physical HIGH) */
	gpiod_set_value(priv->reset_gpio, 0);
	msleep(AD5940_RESET_WAIT_MS);
}
EXPORT_SYMBOL_GPL(ad5940_reset);

/**
 * ad5940_wakeup - Wake AD5940 from hibernate by dummy SPI read
 *
 * After AD5940 enters hibernate (via sequencer or software), any SPI
 * access (CS assertion) wakes it up, but the first read returns garbage.
 * This function retries reading ADIID until the chip responds correctly.
 * Note: Not needed right after hardware reset — chip is already active.
 */
int ad5940_wakeup(struct ad5940_priv *priv)
{
	int i, val;

	for (i = 0; i < AD5940_WAKEUP_RETRIES; i++) {
		val = ad5940_spi_read(priv, AD5940_REG_ADIID);
		if (val == AD5940_ADIID_VALUE)
			return 0;
		msleep(1);
	}

	return -EIO;
}
EXPORT_SYMBOL_GPL(ad5940_wakeup);

/**
 * ad5940_init - Apply post-reset initialization register sequence
 *
 * Must be called after every hardware/software reset. These registers
 * set up critical AFE functions: CS-wakeup, FIFO read, clock gating,
 * SRAM retention, and sequencer hibernate control.
 */
int ad5940_init(struct ad5940_priv *priv)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ad5940_init_regs); i++) {
		ret = ad5940_spi_write(priv, ad5940_init_regs[i].addr,
				       ad5940_init_regs[i].val);
		if (ret) {
			dev_err(&priv->spi->dev,
				"init reg 0x%04x write failed: %d\n",
				ad5940_init_regs[i].addr, ret);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_init);

MODULE_AUTHOR("Mason Wang");
MODULE_DESCRIPTION("AD5940 AFE core register access");
MODULE_LICENSE("GPL");
