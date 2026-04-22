// SPDX-License-Identifier: GPL-2.0
/*
 * AD5940 AFE core register access and initialization
 *
 * Low-level SPI register R/W, hardware reset/wakeup, and post-reset
 * initialization sequence extracted from the platform driver so they
 * can be shared with the IIO layer and future measurement modes.
 *
 * Phase 1 additions: BIA (Body Impedance) initialization, sequencer
 * SRAM programming, and Wakeup Timer control for periodic measurement.
 *
 * IMPORTANT: The BIA init/start flow is faithfully replicated from
 * ADI's BodyImpedance.c example (AppBIAInit + AppBIACtrl). Do NOT
 * add/remove steps without cross-referencing the ADI source.
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
		return (int)((rx[2] << 24) | (rx[3] << 16) |
		       (rx[4] << 8)  | rx[5]);
	else
		return (int)((rx[2] << 8) | rx[3]);
}
EXPORT_SYMBOL_GPL(ad5940_spi_read);

/**
 * ad5940_fifo_read - Read one or more words from the data FIFO
 * @priv: driver private data
 * @buf: output buffer (must have room for @count u32 words)
 * @count: number of FIFO words to read
 *
 * Uses ADI's recommended "small readcount" method: set address to
 * REG_AFE_DATAFIFORD once, then issue repeated READREG commands.
 * Each read automatically pops the next word from the FIFO.
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_fifo_read(struct ad5940_priv *priv, u32 *buf, int count)
{
	u8 tx[6], rx[6];
	int ret, i;

	/* Phase 1: set address to FIFO data register */
	tx[0] = AD5940_SPI_CMD_SETADDR;
	tx[1] = (AD5940_REG_FIFODATA >> 8) & 0xff;
	tx[2] = AD5940_REG_FIFODATA & 0xff;
	ret = ad5940_spi_xfer(priv, tx, rx, 3);
	if (ret)
		return ret;

	/* Phase 2: read 'count' words from FIFO (32-bit register) */
	for (i = 0; i < count; i++) {
		memset(tx, 0, 6);
		tx[0] = AD5940_SPI_CMD_READREG;
		ret = ad5940_spi_xfer(priv, tx, rx, 6);
		if (ret)
			return ret;
		buf[i] = (rx[2] << 24) | (rx[3] << 16) |
			  (rx[4] << 8)  | rx[5];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_fifo_read);

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
 *
 * Faithfully replicates AD5940_WakeUp(10): up to 10 retries.
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

/* ------------------------------------------------------------------ */
/*  Sequencer SRAM programming                                        */
/* ------------------------------------------------------------------ */

/**
 * ad5940_seq_cmd_write - Write sequencer commands to AD5940 SRAM
 * @priv: driver private data
 * @start_addr: SRAM start address (word index)
 * @cmd: array of 32-bit sequencer commands
 * @count: number of commands
 *
 * Each command is written via two register accesses:
 *   1. Write address to REG_AFE_CMDFIFOWADDR
 *   2. Write data to REG_AFE_CMDFIFOWRITE
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_seq_cmd_write(struct ad5940_priv *priv,
			 u32 start_addr, const u32 *cmd, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = ad5940_spi_write(priv, AD5940_REG_CMDFIFOWADDR,
				       start_addr + i);
		if (ret) {
			dev_err(&priv->spi->dev,
				"SEQ addr write failed at %u: %d\n",
				start_addr + i, ret);
			return ret;
		}
		ret = ad5940_spi_write(priv, AD5940_REG_CMDFIFOWRITE,
				       cmd[i]);
		if (ret) {
			dev_err(&priv->spi->dev,
				"SEQ data write failed at %u: %d\n",
				start_addr + i, ret);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_seq_cmd_write);

/* ------------------------------------------------------------------ */
/*  BIA (Body Impedance) measurement setup                            */
/*                                                                     */
/*  Faithfully replicates ADI's BodyImpedance.c flow:                  */
/*    AppBIAInit()  → ad5940_bia_init()                                */
/*    AppBIACtrl(BIACTRL_START) → ad5940_bia_start()                   */
/*                                                                     */
/*  Do NOT add/remove/reorder steps without cross-referencing the      */
/*  ADI source code.                                                   */
/* ------------------------------------------------------------------ */

/**
 * ad5940_bia_init - Initialize AD5940 for BIA measurement
 * @priv: driver private data
 *
 * This function faithfully replicates the FULL ADI BIA initialization
 * flow from AD5940Main.c + BodyImpedance.c, in EXACT order:
 *
 *   AD5940PlatformCfg():
 *     1.  AD5940_CLKCfg() - full clock config with OSCCON enable/wait
 *     2.  AD5940_FIFOCfg(disable) - first FIFO config, FIFOEn=FALSE
 *     3.  AD5940_FIFOCfg(enable)  - second FIFO config, FIFOEn=TRUE
 *     4.  AD5940_INTCCfg(AFEINTC_1, ALLINT, TRUE)
 *     5.  AD5940_INTCCfg(AFEINTC_0, DATAFIFOTHRESH, TRUE)
 *     6.  AD5940_INTCClrFlag(ALLINT)
 *     7.  AD5940_AGPIOCfg()
 *     8.  AD5940_SleepKeyCtrlS(UNLOCK)
 *
 *   AppBIAInit():
 *     9.  WakeUp AFE
 *     10. SEQCfg(SeqMemSize=2KB, SeqEnable=FALSE, SeqCntCRCClr=TRUE)
 *     11. (RTIA calibration skipped - needs MCU interaction)
 *     12. FIFOCtrlS(DFT, FALSE)
 *     13. FIFOCfg(enable, FIFO, 4KB, DFT, thresh=4)
 *     14. INTCClrFlag(ALLINT)
 *     15. Write init + measure sequences to SRAM
 *     16. SEQInfoCfg(InitSeqInfo, WriteSRAM=FALSE) → SEQ1INFO
 *     17. SEQCfg(Enable=TRUE)
 *     18. SEQMmrTrig(SEQID_1)
 *     19. Wait for ENDSEQ
 *     20. SEQInfoCfg(MeasureSeqInfo, WriteSRAM=FALSE) → SEQ0INFO
 *     21. SEQCfg(Enable=TRUE)
 *     22. ClrMCUIntFlag
 *     23. AFEPwrBW(AFEPWR_LP, AFEBW_250K)
 *     24. WriteReg(REG_AFE_SWMUX, 1<<3)
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_init(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	static const u32 init_seq[] = { SEQ_STOP() };
	u32 meas_seq_addr = ARRAY_SIZE(init_seq);
	int ret, i, rd;
	u32 val, reg_osccon, fifocon_saved, tempreg;

	/* ================================================================
	 * AD5940PlatformCfg() - EXACT order from AD5940Main.c
	 * ================================================================ */

	/* ---- PlatformCfg Step 1: AD5940_CLKCfg ---- */
	/*
	 * ADI CLKCfg with BIA parameters:
	 *   HFXTALEn = bFALSE, HFOSCEn = bTRUE, LFOSCEn = bTRUE
	 *   HfOSC32MHzMode = bFALSE (16MHz)
	 *   SysClkDiv = 1, ADCClkDiv = 1
	 *   SysClkSrc = HFOSC(0), ADCCLkSrc = HFOSC(0)
	 *
	 * Full sequence from ADI AD5940_CLKCfg():
	 *   1) Read OSCCON
	 *   2) HFXTALEn=FALSE → skip
	 *   3) HFOSCEn=TRUE → OSCKEY + OSCCON |= HFOSCEN + busy-wait HFOSCOK
	 *   4) HFOSC32MHzCtrl(bFALSE) → 16MHz mode
	 *   5) LFOSCEn=TRUE → OSCKEY + OSCCON |= LFOSCEN + busy-wait LFOSCOK
	 *   6) Write CLKCON0 + Delay10us(10)
	 *   7) Write CLKSEL
	 *   8) Disable unused clocks (HFXTAL) in OSCCON
	 */

	/* 1a: Read current OSCCON */
	rd = ad5940_spi_read(priv, AD5940_REG_ALLON_OSCCON);
	if (rd < 0) {
		dev_err(dev, "BIA: OSCCON read failed: %d\n", rd);
		return rd;
	}
	reg_osccon = rd;

	/* 1b: Enable HFOSC (HFOSCEn = bTRUE) */
	reg_osccon |= AD5940_OSCCON_HFOSCEN;
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCCON, reg_osccon);
	/* Busy-wait for HFOSC ready */
	for (i = 0; i < 100; i++) {
		val = ad5940_spi_read(priv, AD5940_REG_ALLON_OSCCON);
		if (val & AD5940_OSCCON_HFOSCOK)
			break;
		usleep_range(10, 20);
	}
	if (i >= 100)
		dev_warn(dev, "BIA: HFOSC not ready after 1ms\n");

	/* 1c: HFOSC32MHzCtrl(bFALSE) - select 16MHz output */
	/*
	 * ADI does:
	 *   1) Read CLKEN1, fix silicon bug (bit8/bit9 swap on readback)
	 *   2) Write CLKEN1 | ACLKDIS (disable ACLK during clock change)
	 *   3) Read HPOSCCON
	 *   4) Write HPOSCCON | CLK32MHZEN (bit=1 means 16MHz mode)
	 *   5) Busy-wait for HFOSCOK
	 *   6) Write CLKEN1 & ~ACLKDIS (re-enable ACLK)
	 */
	{
		u32 rd_clken1, rd_hposccon, bit8, bit9;

		rd_clken1 = ad5940_spi_read(priv, AD5940_REG_CLKEN1);
		/* Fix silicon bug: bit8 and bit9 are swapped on readback */
		bit8 = (rd_clken1 >> 9) & 0x01;
		bit9 = (rd_clken1 >> 8) & 0x01;
		rd_clken1 = rd_clken1 & 0xff;
		rd_clken1 |= (bit8 << 8) | (bit9 << 9);
		ad5940_spi_write(priv, AD5940_REG_CLKEN1,
				 rd_clken1 | AD5940_CLKEN1_ACLKDIS);/* Disable ACLK during clock changing */

		rd_hposccon = ad5940_spi_read(priv, AD5940_REG_HPOSCCON);
		/* bFALSE → 16MHz → set CLK32MHZEN bit (1=16MHz, 0=32MHz) */
		ad5940_spi_write(priv, AD5940_REG_HPOSCCON,
				 rd_hposccon | AD5940_HPOSCCON_CLK32MHZEN);
		/* Busy-wait for HFOSCOK */
		for (i = 0; i < 100; i++) {
			val = ad5940_spi_read(priv, AD5940_REG_ALLON_OSCCON);
			if (val & AD5940_OSCCON_HFOSCOK)
				break;
			usleep_range(10, 20);
		}

		ad5940_spi_write(priv, AD5940_REG_CLKEN1,
				 rd_clken1 & ~AD5940_CLKEN1_ACLKDIS); /* Enable ACLK */
	}

	/* 1d: Enable LFOSC (LFOSCEn = bTRUE) */
	reg_osccon |= AD5940_OSCCON_LFOSCEN;
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCCON, reg_osccon);
	/* Busy-wait for LFOSC ready */
	for (i = 0; i < 100; i++) {
		val = ad5940_spi_read(priv, AD5940_REG_ALLON_OSCCON);
		if (val & AD5940_OSCCON_LFOSCOK)
			break;
		usleep_range(10, 20);
	}
	if (i >= 100)
		dev_warn(dev, "BIA: LFOSC not ready after 1ms\n");

	/* 1e: Write CLKCON0 (clock dividers) + 100us delay */
	/*
	 * ADI: tempreg = (SysClkDiv & 0x3f);
	 *      tempreg |= (SysClkDiv & 0x3f) << BITP_SYSCLKDIV; // redundant
	 *      tempreg |= (ADCClkDiv & 0xf) << BITP_ADCCLKDIV;
	 * With SysClkDiv=1, ADCClkDiv=1: tempreg = 0x01 | 0x01 | 0x40 = 0x41
	 */
	tempreg = (1 & 0x3f);  /* SysClkDiv = 1 */
	tempreg |= (1 & 0x3f) << AD5940_CLKCON0_SYSCLKDIV_SHIFT;  /* per ADI */
	tempreg |= (1 & 0xf) << AD5940_CLKCON0_ADCCLKDIV_SHIFT;   /* ADCClkDiv = 1 */
	ad5940_spi_write(priv, AD5940_REG_CLKCON0, tempreg);
	usleep_range(100, 200);  /* AD5940_Delay10us(10) = 100us */

	/* 1f: Write CLKSEL (clock sources) */
	/* SysClkSrc=HFOSC(0), ADCCLkSrc=HFOSC(0) */
	ad5940_spi_write(priv, AD5940_REG_CLKSEL,
			 (AD5940_SYSCLKSRC_HFOSC << AD5940_CLKSEL_SYSCLKSEL_SHIFT) |
			 (AD5940_ADCCLKSRC_HFOSC << AD5940_CLKSEL_ADCCLKSEL_SHIFT));

	/* 1g: Disable unused clocks in OSCCON (HFXTALEn=bFALSE) */
	reg_osccon &= ~AD5940_OSCCON_HFXTALEN;
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
	ad5940_spi_write(priv, AD5940_REG_ALLON_OSCCON, reg_osccon);

	/* ---- PlatformCfg Step 2: AD5940_FIFOCfg(disable) ---- */
	/*
	 * ADI configures FIFO TWICE in PlatformCfg:
	 *   First:  FIFOEn=FALSE (disable to reset FIFO)
	 *   Second: FIFOEn=TRUE  (enable with proper config)
	 *
	 * First FIFOCfg call (disable):
	 *   FIFOMode=FIFOMODE_FIFO, FIFOSize=FIFOSIZE_4KB,
	 *   FIFOSrc=FIFOSRC_DFT, FIFOThresh=4, FIFOEn=FALSE
	 *
	 * ADI FIFOCfg() does:
	 *   1) Write FIFOCON = 0 (disable FIFO)
	 *   2) Read-modify-write CMDDATACON: keep CMD part, set DATA part
	 *   3) Write DATAFIFOTHRES
	 *   4) Write FIFOCON (no DATAFIFOEN since FIFOEn=FALSE)
	 */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);  /* Step 1: disable */
	rd = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if (rd < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", rd);
		return rd;
	}
	val = rd;
	/* Keep CMD_MEM_SEL + CMDMEMMDE bits, set DATA part */
	val &= AD5940_BITM_CMDDATACON_CMD_MEM_SEL | AD5940_BITM_CMDDATACON_CMDMEMMDE;
	val |= AD5940_FIFOMODE_FIFO << AD5940_BITP_CMDDATACON_DATAMEMMDE;
	val |= AD5940_FIFOSIZE_4KB << AD5940_BITP_CMDDATACON_DATA_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);
	ad5940_spi_write(priv, AD5940_REG_DATAFIFOTHRES,
			 AD5940_FIFO_THRESHOLD << AD5940_DATAFIFOTHRES_HIGHTHRES_SHIFT);
	/* FIFOEn=FALSE: only write FIFOSrc, no DATAFIFOEN */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT);  /* FIFOSRC_DFT */

	/* ---- PlatformCfg Step 3: AD5940_FIFOCfg(enable) ---- */
	/*
	 * Second FIFOCfg call (enable) - identical to first except FIFOEn=TRUE
	 */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);  /* Step 1: disable again */
	rd = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if (rd < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", rd);
		return rd;
	}
	val = rd;
	val &= AD5940_BITM_CMDDATACON_CMD_MEM_SEL | AD5940_BITM_CMDDATACON_CMDMEMMDE;
	val |= AD5940_FIFOMODE_FIFO << AD5940_BITP_CMDDATACON_DATAMEMMDE;
	val |= AD5940_FIFOSIZE_4KB << AD5940_BITP_CMDDATACON_DATA_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);
	ad5940_spi_write(priv, AD5940_REG_DATAFIFOTHRES,
			 AD5940_FIFO_THRESHOLD << AD5940_DATAFIFOTHRES_HIGHTHRES_SHIFT);
	/* FIFOEn=TRUE: DATAFIFOEN + FIFOSrc */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 AD5940_FIFOCON_DATAFIFOEN |
			 (2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT));

	/* ---- PlatformCfg Step 4: AD5940_INTCCfg(AFEINTC_1, ALLINT, TRUE) ---- */
	val = ad5940_spi_read(priv, AD5940_REG_INTCSEL1);
	if (val >= 0)
		ad5940_spi_write(priv, AD5940_REG_INTCSEL1, val | AD5940_AFEINTSRC_ALLINT);
	else
		ad5940_spi_write(priv, AD5940_REG_INTCSEL1, AD5940_AFEINTSRC_ALLINT);

	/* ---- PlatformCfg Step 5: AD5940_INTCCfg(AFEINTC_0, DATAFIFOTHRESH, TRUE) ---- */
	val = ad5940_spi_read(priv, AD5940_REG_INTCSEL0);
	if (val >= 0)
		ad5940_spi_write(priv, AD5940_REG_INTCSEL0,
				 val | AD5940_AFEINTSRC_DATAFIFOTHRESH);
	else
		ad5940_spi_write(priv, AD5940_REG_INTCSEL0,
				 AD5940_AFEINTSRC_DATAFIFOTHRESH);

	/* ---- PlatformCfg Step 6: AD5940_INTCClrFlag(ALLINT) ---- */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* ---- PlatformCfg Step 7: AD5940_AGPIOCfg ---- */
	ad5940_spi_write(priv, AD5940_REG_AGPIO_GP0CON,
			 AD5940_GP6_SYNC | AD5940_GP5_SYNC | AD5940_GP4_SYNC |
			 AD5940_GP2_TRIG | AD5940_GP1_SYNC | AD5940_GP0_INT);
	ad5940_spi_write(priv, AD5940_REG_AGPIO_GP0OEN,
			 AD5940_AGPIO_Pin0 | AD5940_AGPIO_Pin1 |
			 AD5940_AGPIO_Pin4 | AD5940_AGPIO_Pin5 |
			 AD5940_AGPIO_Pin6);
	ad5940_spi_write(priv, AD5940_REG_AGPIO_GP0IEN, AD5940_AGPIO_Pin2);
	ad5940_spi_write(priv, AD5940_REG_AGPIO_GP0PE, 0);
	ad5940_spi_write(priv, AD5940_REG_AGPIO_GP0OUT, 0);

	/* ---- PlatformCfg Step 8: AD5940_SleepKeyCtrlS(UNLOCK) ---- */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_UNLOCK);

	/* ================================================================
	 * AppBIAInit() - EXACT order from BodyImpedance.c
	 * ================================================================ */

	/* ---- AppBIAInit Step 1: WakeUp AFE ---- */
	ret = ad5940_wakeup(priv);
	if (ret) {
		dev_err(dev, "BIA: wakeup failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 2: SEQCfg(SeqMemSize=2KB, SeqEnable=FALSE, SeqCntCRCClr=TRUE) ---- */
	/*
	 * ADI SEQCfg() does:
	 *   a) Save FIFOCON
	 *   b) Write FIFOCON = 0
	 *   c) Read-modify-write CMDDATACON: clear CMDMEMMDE+CMD_MEM_SEL,
	 *      set CMDMEMMDE=1 (memory mode), CMD_MEM_SEL=SeqMemSize
	 *   d) SeqCntCRCClr=TRUE: write SEQCON=0, write SEQCNT=0
	 *   e) Write SEQCON (SeqEnable=FALSE → 0)
	 *   f) Restore FIFOCON
	 */
	rd = ad5940_spi_read(priv, AD5940_REG_FIFOCON);
	if (rd < 0) {
		dev_err(dev, "BIA: FIFOCON read failed: %d\n", rd);
		return rd;
	}
	fifocon_saved = rd;
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);

	/* CMDDATACON RMW: clear CMDMEMMDE+CMD_MEM_SEL, set memory mode + 2KB */
	val = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if ((int)val < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", (int)val);
		return (int)val;
	}
	val &= ~(AD5940_BITM_CMDDATACON_CMDMEMMDE | AD5940_BITM_CMDDATACON_CMD_MEM_SEL);
	val |= (1UL) << AD5940_BITP_CMDDATACON_CMDMEMMDE;   /* memory mode */
	val |= AD5940_SEQMEMSIZE_2KB << AD5940_BITP_CMDDATACON_CMD_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);

	/* SeqCntCRCClr: disable sequencer + clear counter */
	ad5940_spi_write(priv, AD5940_REG_SEQCON, 0);
	ad5940_spi_write(priv, AD5940_REG_SEQCNT, 0);

	/* SeqEnable=FALSE → SEQCON = 0 (already done above) */

	/* Restore FIFOCON */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, fifocon_saved);

	/* ---- AppBIAInit Step 3: RTIA calibration (SKIPPED) ---- */
	/*
	 * ADI does AppBIARtiaCal() here, which requires MCU-driven
	 * measurement and data processing. We skip this and rely on
	 * default RTIA value from the register table.
	 */

	/* ---- AppBIAInit Step 4: AD5940_FIFOCtrlS(DFT, FALSE) ---- */
	/* ADI: disable FIFO before reconfiguring */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);

	/* ---- AppBIAInit Step 5: AD5940_FIFOCfg(enable, FIFO, 4KB, DFT, thresh=4) ---- */
	/*
	 * Third FIFO configuration (in AppBIAInit), same as PlatformCfg Step 3
	 * but this time FIFO is re-enabled after FIFOCtrlS disabled it.
	 */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);  /* FIFOCfg always starts with this */
	rd = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if (rd < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", rd);
		return rd;
	}
	val = rd;
	val &= AD5940_BITM_CMDDATACON_CMD_MEM_SEL | AD5940_BITM_CMDDATACON_CMDMEMMDE;
	val |= AD5940_FIFOMODE_FIFO << AD5940_BITP_CMDDATACON_DATAMEMMDE;
	val |= AD5940_FIFOSIZE_4KB << AD5940_BITP_CMDDATACON_DATA_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);
	ad5940_spi_write(priv, AD5940_REG_DATAFIFOTHRES,
			 AD5940_FIFO_THRESHOLD << AD5940_DATAFIFOTHRES_HIGHTHRES_SHIFT);
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 AD5940_FIFOCON_DATAFIFOEN |
			 (2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT));

	/* ---- AppBIAInit Step 6: AD5940_INTCClrFlag(ALLINT) ---- */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* ---- AppBIAInit Step 7: Write BIA init register table ---- */
	/*
	 * Corresponds to AppBIASeqCfgGen() which configures:
	 * REFCfgS, HSLoopCfgS, LPLoopCfgS, DSPCfgS, AFECtrlS, SEQGpioCtrlS.
	 * All register writes done directly (not via sequencer).
	 */
	for (i = 0; i < ARRAY_SIZE(ad5940_bia_init_regs); i++) {
		u16 addr = ad5940_bia_init_regs[i].addr;
		u32 new_val = ad5940_bia_init_regs[i].val;

		if (addr == AD5940_REG_AFECON) {
			rd = ad5940_spi_read(priv, AD5940_REG_AFECON);
			if (rd < 0) {
				dev_err(dev, "BIA: AFECON read failed: %d\n", rd);
				return rd;
			}
			val = (rd & ~AD5940_AFECON_HPREFDIS) | new_val;
			ret = ad5940_spi_write(priv, addr, val);
		} else if (addr == AD5940_REG_BUFSENCON) {
			rd = ad5940_spi_read(priv, AD5940_REG_BUFSENCON);
			if (rd < 0)
				return rd;
			val = rd | new_val;
			ret = ad5940_spi_write(priv, addr, val);
		} else {
			ret = ad5940_spi_write(priv, addr, new_val);
		}

		if (ret) {
			dev_err(dev, "BIA: reg 0x%04x write failed: %d\n",
				addr, ret);
			return ret;
		}
	}

	/* ---- AppBIAInit Step 8: Program init sequence (SEQID_1) into SRAM ---- */
	ret = ad5940_seq_cmd_write(priv, 0, init_seq, ARRAY_SIZE(init_seq));
	if (ret) {
		dev_err(dev, "BIA: init seq SRAM write failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 9: Program measure sequence (SEQID_0) into SRAM ---- */
	ret = ad5940_seq_cmd_write(priv, meas_seq_addr,
				   ad5940_bia_measure_seq,
				   ARRAY_SIZE(ad5940_bia_measure_seq));
	if (ret) {
		dev_err(dev, "BIA: measure seq SRAM write failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 10: SEQInfoCfg(InitSeqInfo, WriteSRAM=FALSE) ---- */
	ret = ad5940_spi_write(priv, AD5940_REG_SEQ1INFO,
			       (ARRAY_SIZE(init_seq) << 16) | 0);
	if (ret) {
		dev_err(dev, "BIA: SEQ1INFO write failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 11: SEQCfg(SeqEnable=TRUE) ---- */
	/*
	 * ADI SEQCfg(Enable=TRUE) does:
	 *   a) Save FIFOCON
	 *   b) Write FIFOCON = 0
	 *   c) Read-modify-write CMDDATACON (same as before)
	 *   d) Write SEQCON = SEQEN (SeqEnable=TRUE)
	 *   e) Restore FIFOCON
	 */
	rd = ad5940_spi_read(priv, AD5940_REG_FIFOCON);
	if (rd < 0) {
		dev_err(dev, "BIA: FIFOCON read failed: %d\n", rd);
		return rd;
	}
	fifocon_saved = rd;
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);

	val = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if ((int)val < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", (int)val);
		return (int)val;
	}
	/* ADI SEQCfg clears CMDMEMMDE + CMD_MEM_SEL then re-sets them */
	val &= ~(AD5940_BITM_CMDDATACON_CMDMEMMDE | AD5940_BITM_CMDDATACON_CMD_MEM_SEL);
	val |= (1UL) << AD5940_BITP_CMDDATACON_CMDMEMMDE;
	val |= AD5940_SEQMEMSIZE_2KB << AD5940_BITP_CMDDATACON_CMD_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);

	/* SeqEnable=TRUE */
	ad5940_spi_write(priv, AD5940_REG_SEQCON, BIT(0));

	/* Restore FIFOCON */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, fifocon_saved);

	/* ---- AppBIAInit Step 12: SEQMmrTrig(SEQID_1) ---- */
	ad5940_spi_write(priv, AD5940_REG_TRIGSEQ, BIT(1));

	/* ---- AppBIAInit Step 13: Wait for ENDSEQ ---- */
	for (i = 0; i < 100; i++) {
		rd = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
		if (rd < 0)
			return rd;
		val = rd;
		if (val & AD5940_AFEINTSRC_ENDSEQ)
			break;
		usleep_range(100, 200);
	}
	if (i >= 100)
		dev_warn(dev, "BIA: init sequence ENDSEQ timeout\n");

	/* ---- AppBIAInit Step 14: SEQInfoCfg(MeasureSeqInfo, WriteSRAM=FALSE) ---- */
	ret = ad5940_spi_write(priv, AD5940_REG_SEQ0INFO,
			       (ARRAY_SIZE(ad5940_bia_measure_seq) << 16) |
			       meas_seq_addr);
	if (ret) {
		dev_err(dev, "BIA: SEQ0INFO write failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 15: SEQCfg(SeqEnable=TRUE) again ---- */
	/*
	 * ADI explicitly re-enables the sequencer here, same sequence as Step 11.
	 */
	rd = ad5940_spi_read(priv, AD5940_REG_FIFOCON);
	if (rd < 0) {
		dev_err(dev, "BIA: FIFOCON read failed: %d\n", rd);
		return rd;
	}
	fifocon_saved = rd;
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);

	val = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	if ((int)val < 0) {
		dev_err(dev, "BIA: CMDDATACON read failed: %d\n", (int)val);
		return (int)val;
	}
	val &= ~(AD5940_BITM_CMDDATACON_CMDMEMMDE | AD5940_BITM_CMDDATACON_CMD_MEM_SEL);
	val |= (1UL) << AD5940_BITP_CMDDATACON_CMDMEMMDE;
	val |= AD5940_SEQMEMSIZE_2KB << AD5940_BITP_CMDDATACON_CMD_MEM_SEL;
	ad5940_spi_write(priv, AD5940_REG_CMDDATACON, val);

	ad5940_spi_write(priv, AD5940_REG_SEQCON, BIT(0));

	ad5940_spi_write(priv, AD5940_REG_FIFOCON, fifocon_saved);

	/* ---- AppBIAInit Step 16: ClrMCUIntFlag ---- */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* ---- AppBIAInit Step 17: AFEPwrBW(AFEPWR_LP, AFEBW_250K) ---- */
	ret = ad5940_spi_write(priv, AD5940_REG_PMBW,
			       (3 << AD5940_PMBW_SYSBW_SHIFT));
	if (ret)
		return ret;

	/* ---- AppBIAInit Step 18: WriteReg(REG_AFE_SWMUX, 1<<3) ---- */
	ret = ad5940_spi_write(priv, AD5940_REG_SWMUX, BIT(3));
	if (ret)
		return ret;

	dev_info(dev, "BIA: initialization complete (ADI flow replicated)\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_init);

/**
 * ad5940_bia_start - Start periodic BIA measurement via Wakeup Timer
 * @priv: driver private data
 *
 * Faithfully replicates AppBIACtrl(BIACTRL_START) from BodyImpedance.c,
 * with additional FIFO/INTC cleanup for reliable restart.
 *
 * ADI's original flow is just WakeUp + WUPTCfg, but that assumes the
 * FIFO and interrupt flags are clean. After a stop→start cycle, stale
 * FIFO data and interrupt flags can prevent new DFT results from being
 * written. We therefore:
 *   1. WakeUp AFE
 *   2. Reset FIFO (disable → re-enable) and clear all interrupt flags
 *   3. Configure WUPT (WakeupTimer) and enable it
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_start(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	int ret;
	u32 sleep_time;

	/* ---- Step 1: Wake up AFE (AD5940_WakeUp) ---- */
	ret = ad5940_wakeup(priv);
	if (ret) {
		dev_err(dev, "BIA start: wakeup failed: %d\n", ret);
		return ret;
	}

	/* ---- Step 2: Reset FIFO and clear interrupt flags ---- */
	/*
	 * After bia_stop(), the FIFO may contain stale data and INTC flags
	 * may be set. This prevents new DFT results from triggering the
	 * FIFO threshold interrupt, causing the kernel to never read data.
	 *
	 * Reset sequence (mirrors FIFOCfg pattern from bia_init):
	 *   a) Disable FIFO
	 *   b) Clear all interrupt flags
	 *   c) Re-enable FIFO with DFT source + threshold
	 */
	/* 2a: Disable FIFO */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);

	/* 2b: Clear all interrupt flags (both INTC0 and INTC1) */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* 2c: Re-enable FIFO: FIFOSRC=DFT(2), DATAFIFOEN, threshold=4 */
	ad5940_spi_write(priv, AD5940_REG_DATAFIFOTHRES,
			 AD5940_FIFO_THRESHOLD << AD5940_DATAFIFOTHRES_HIGHTHRES_SHIFT);
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 AD5940_FIFOCON_DATAFIFOEN |
			 (2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT));

	/* ---- Step 3: Configure WUPT (AD5940_WUPTCfg) ---- */
	/*
	 * From ADI's AppBIACtrl(BIACTRL_START):
	 *   wupt_cfg.WuptEn = bTRUE;
	 *   wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
	 *   wupt_cfg.WuptOrder[0] = SEQID_0;
	 *   wupt_cfg.SeqxSleepTime[SEQID_0] =
	 *       (uint32_t)(WuptClkFreq / BiaODR) - 2 - 1;
	 *   wupt_cfg.SeqxWakeupTime[SEQID_0] = 1;
	 *   AD5940_WUPTCfg(&wupt_cfg);
	 *
	 * AD5940_WUPTCfg() does:
	 *   a) Write SEQ0 WAKEUP time (low + high)
	 *   b) Write SEQ0 SLEEP time (low + high)
	 *   c) Write SEQ1-3 WAKEUP/SLEEP time (all zeros - unused)
	 *   d) Write TMRCON: enable WUPT to wake up AFE
	 *   e) Write SEQORDER: position A = SEQID_0
	 *   f) Write WUPTCON: EN=1, ENDSEQ=A
	 */

	sleep_time = (u32)(AD5940_BIA_WUPT_CLK_FREQ / AD5940_BIA_ODR) - 2 - 1;

	/* 3a: SEQ0 wakeup time = 1 (minimum, per ADI) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0WUPL, 1);
	if (ret)
		return ret;
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0WUPH, 0);
	if (ret)
		return ret;

	/* 3b: SEQ0 sleep time */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0SLEEPL,
			       sleep_time & 0xFFFF);
	if (ret)
		return ret;
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0SLEEPH,
			       (sleep_time >> 16) & 0xF);
	if (ret)
		return ret;

	/* 3c: SEQ1-3 wakeup/sleep time = 0 (unused) - ADI writes all of them */
	ad5940_spi_write(priv, 0x0818, 0);  /* SEQ1WUPL */
	ad5940_spi_write(priv, 0x081C, 0);  /* SEQ1WUPH */
	ad5940_spi_write(priv, 0x0820, 0);  /* SEQ1SLEEPL */
	ad5940_spi_write(priv, 0x0824, 0);  /* SEQ1SLEEPH */
	ad5940_spi_write(priv, 0x0828, 0);  /* SEQ2WUPL */
	ad5940_spi_write(priv, 0x082C, 0);  /* SEQ2WUPH */
	ad5940_spi_write(priv, 0x0830, 0);  /* SEQ2SLEEPL */
	ad5940_spi_write(priv, 0x0834, 0);  /* SEQ2SLEEPH */
	ad5940_spi_write(priv, 0x0838, 0);  /* SEQ3WUPL */
	ad5940_spi_write(priv, 0x083C, 0);  /* SEQ3WUPH */
	ad5940_spi_write(priv, 0x0840, 0);  /* SEQ3SLEEPL */
	ad5940_spi_write(priv, 0x0844, 0);  /* SEQ3SLEEPH */

	/* 3d: TMRCON - allow WUPT to wake up AFE (per ADI) */
	ret = ad5940_spi_write(priv, AD5940_REG_ALLON_TMRCON, BIT(0));
	if (ret)
		return ret;

	/* 3e: SEQORDER - position A = SEQID_0 (=0) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQORDER, 0x00);
	if (ret)
		return ret;

	/* 3f: WUPTCON - enable WUPT, ENDSEQ = position A (0) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTCON,
			       AD5940_WUPTCON_EN |
			       (0 << AD5940_WUPTCON_ENDSEQ_SHIFT));
	if (ret)
		return ret;

	/* ---- Debug: read status registers after WUPT start ---- */
	{
		u32 seqcon, fifocon, fifocnt_raw, fifocnt, intcflag0, intcflag1;
		u32 afecon, wuptcon, cmddatacon, dftcon, adcfiltercon;
		u32 seq0info, seq1info, intcsel0, intcsel1, fifothres;
		u32 seqcnt;

		/* Quick snapshot at 10ms to see if sequence starts */
		msleep(10);
		ad5940_wakeup(priv);
		afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
		seqcon = ad5940_spi_read(priv, AD5940_REG_SEQCON);
		seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
		fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
		fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
			  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
		intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
		dev_info(dev, "BIA debug @10ms: AFECON=0x%x SEQCON=0x%x SEQCNT=%u FIFOCNT=%u INTF1=0x%x\n",
			 afecon, seqcon, seqcnt & 0xFFFF, fifocnt, intcflag1);

		/* Snapshot at 100ms (first DFT should be complete) */
		msleep(90);
		ad5940_wakeup(priv);
		afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
		seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
		fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
		fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
			  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
		intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
		dev_info(dev, "BIA debug @100ms: AFECON=0x%x SEQCNT=%u FIFOCNT=%u INTF1=0x%x\n",
			 afecon, seqcnt & 0xFFFF, fifocnt, intcflag1);

		/* Full register dump at 300ms (at least one full measure cycle done) */
		msleep(200);
		ad5940_wakeup(priv);

		seqcon = ad5940_spi_read(priv, AD5940_REG_SEQCON);
		seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
		afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
		fifocon = ad5940_spi_read(priv, AD5940_REG_FIFOCON);
		fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
		fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
			  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
		intcflag0 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG0);
		intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
		wuptcon = ad5940_spi_read(priv, AD5940_REG_WUPTCON);
		cmddatacon = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
		dftcon = ad5940_spi_read(priv, AD5940_REG_DFTCON);
		adcfiltercon = ad5940_spi_read(priv, AD5940_REG_ADCFILTERCON);
		seq0info = ad5940_spi_read(priv, AD5940_REG_SEQ0INFO);
		seq1info = ad5940_spi_read(priv, AD5940_REG_SEQ1INFO);
		intcsel0 = ad5940_spi_read(priv, AD5940_REG_INTCSEL0);
		intcsel1 = ad5940_spi_read(priv, AD5940_REG_INTCSEL1);
		fifothres = ad5940_spi_read(priv, AD5940_REG_DATAFIFOTHRES);

		dev_info(dev, "BIA debug @300ms:\n");
		dev_info(dev, "  SEQCON=0x%x SEQCNT=%u AFECON=0x%x FIFOCON=0x%x\n",
			 seqcon, seqcnt & 0xFFFF, afecon, fifocon);
		dev_info(dev, "  FIFOCNT=0x%x(raw) FIFOCNT=%u INTCFLAG0=0x%x INTCFLAG1=0x%x\n",
			 fifocnt_raw, fifocnt, intcflag0, intcflag1);
		dev_info(dev, "  WUPTCON=0x%x CMDDATACON=0x%x\n",
			 wuptcon, cmddatacon);
		dev_info(dev, "  DFTCON=0x%x ADCFILTERCON=0x%x\n",
			 dftcon, adcfiltercon);
		dev_info(dev, "  SEQ0INFO=0x%x SEQ1INFO=0x%x\n",
			 seq0info, seq1info);
		dev_info(dev, "  INTCSEL0=0x%x INTCSEL1=0x%x FIFOTHRES=0x%x\n",
			 intcsel0, intcsel1, fifothres);

		/* Also do a second read after another 500ms (multiple WUPT cycles) */
		msleep(500);
		ad5940_wakeup(priv);
		fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
		fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
			  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
		intcflag0 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG0);
		intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
		afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
		dev_info(dev, "BIA debug @800ms: FIFOCNT=%u INTF0=0x%x INTF1=0x%x AFECON=0x%x\n",
			 fifocnt, intcflag0, intcflag1, afecon);
	}

	dev_info(dev, "BIA: measurement started (ODR=%dHz, ADI flow replicated)\n",
		 AD5940_BIA_ODR);
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_start);

/**
 * ad5940_bia_stop - Stop BIA measurement by disabling Wakeup Timer
 * @priv: driver private data
 *
 * Faithfully replicates AppBIACtrl(BIACTRL_STOPNOW) from BodyImpedance.c.
 * ADI disables WUPT, then calls WUPTCtrl(FALSE) again as a safety measure.
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_stop(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	int ret;

	/* Disable WUPT (AD5940_WUPTCtrl(bFALSE)) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTCON, 0);
	if (ret)
		return ret;

	/* ADI calls WUPTCtrl(FALSE) again for safety (race condition with
	 * sequencer potentially putting AFE back to hibernate).
	 */
	ad5940_spi_write(priv, AD5940_REG_WUPTCON, 0);

	dev_info(dev, "BIA: measurement stopped\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_stop);

MODULE_AUTHOR("Mason Wang");
MODULE_DESCRIPTION("AD5940 AFE core register access and BIA measurement");
MODULE_LICENSE("GPL");
