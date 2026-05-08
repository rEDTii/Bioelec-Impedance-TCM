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
#include <linux/log2.h>
#include <asm/unaligned.h>
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
 * ad5940_spi_read32 - Read a 32-bit register into a u32 output parameter
 * @priv: driver private data
 * @reg: register address
 * @val: output register value
 *
 * Unlike ad5940_spi_read() which returns int, this function stores the
 * register value through a pointer and returns 0 on success / negative
 * errno on failure. This avoids the ambiguity where a 32-bit register
 * value with bit31 set (e.g. 0xFFFFFFFF) is indistinguishable from a
 * negative errno when cast to int.
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_spi_read32(struct ad5940_priv *priv, u16 reg, u32 *val)
{
	u8 tx[6], rx[6];
	int ret;

	if (reg < 0x1000 || reg > 0x3014)
		return -EINVAL;

	/* Phase 1: set address */
	tx[0] = AD5940_SPI_CMD_SETADDR;
	tx[1] = (reg >> 8) & 0xff;
	tx[2] = reg & 0xff;
	ret = ad5940_spi_xfer(priv, tx, rx, 3);
	if (ret)
		return ret;

	/* Phase 2: full-duplex read (CMD + 1 dummy + 4 data = 6 bytes) */
	memset(tx, 0, 6);
	tx[0] = AD5940_SPI_CMD_READREG;
	ret = ad5940_spi_xfer(priv, tx, rx, 6);
	if (ret)
		return ret;

	*val = (rx[2] << 24) | (rx[3] << 16) | (rx[4] << 8) | rx[5];
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_spi_read32);

/**
 * ad5940_fifo_read - Read one or more words from the data FIFO
 * @priv: driver private data
 * @buf: output buffer (must have room for @count u32 words)
 * @count: number of FIFO words to read
 *
 * Uses AD5940's SPICMD_READFIFO (0x5F) for continuous burst read.
 * This is the protocol used by ADI's FIFORd() when count >= 3:
 *
 *   CS↓ [READFIFO] [6×dummy] [data×(count-2)] [data+0x44444444×2] CS↑
 *
 * The first 2 words after the 6 dummy bytes are invalid (discarded).
 * The last 2 words require a non-zero write offset (0x44444444) to
 * advance the FIFO read pointer correctly.
 *
 * When count < 3, falls back to the SETADDR + per-word READREG
 * method (which does not require the dummy/offset protocol).
 * When count > AD5940_FIFO_MAX_WORDS, count is clamped to that limit.
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_fifo_read(struct ad5940_priv *priv, u32 *buf, int count)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret, i, total_len, last_off;
	u8 *tx_buf, *rx_buf;
	const u8 *data;

	if (count < 3) {
		/*
		 * Small readcount path: SETADDR + per-word READREG.
		 * This matches ADI's FIFORd() for count < 3.
		 */
		u8 tx[6], rx[6];

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

	if (count > AD5940_FIFO_MAX_WORDS)
		count = AD5940_FIFO_MAX_WORDS;

	/*
	 * READFIFO burst read path (3 <= count <= AD5940_FIFO_MAX_WORDS):
	 *
	 * SPI frame layout:
	 *   Byte 0:     READFIFO command (0x5F)
	 *   Bytes 1-6:  dummy (6 bytes, first 2 FIFO words are invalid)
	 *   Then count words of data, each 4 bytes, big-endian.
	 *   Last 2 words: TX must send 0x44444444 offset to advance
	 *                 the FIFO read pointer.
	 *
	 * Total transfer length: 1 + 6 + count * 4
	 */
	total_len = 1 + 6 + count * 4;

	tx_buf = kzalloc(total_len, GFP_KERNEL);
	rx_buf = kzalloc(total_len, GFP_KERNEL);
	if (!tx_buf || !rx_buf) {
		kfree(tx_buf);
		kfree(rx_buf);
		return -ENOMEM;
	}

	tx_buf[0] = AD5940_SPI_CMD_READFIFO;
	last_off = 1 + 6 + (count - 2) * 4;
	// 非对齐内存 + 大端序写入 避免非对齐访问崩溃
	put_unaligned_be32(0x44444444, &tx_buf[last_off]);
	put_unaligned_be32(0x44444444, &tx_buf[last_off + 4]);

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.len = total_len;
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(priv->spi, &msg);

	if (!ret) {
		/*
		 * Parse received data: skip 7-byte header (1 CMD + 6 dummy),
		 * then extract count big-endian u32 words.
		 */
		data = rx_buf;
		for (i = 0; i < count; i++) {
			int off = 7 + i * 4;
			buf[i] = (data[off] << 24)     | (data[off + 1] << 16) |
				  (data[off + 2] << 8) | data[off + 3];
		}
	}

	kfree(tx_buf);
	kfree(rx_buf);

	return ret;
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

/* ================================================================== */
/*  ADI-style sequence generator for BIA measure sequence             */
/*                                                                     */
/*  This replicates the EXACT flow of ADI's AppBIASeqMeasureGen()     */
/*  which uses high-level API functions (AFECtrlS, ADCMuxCfgS,        */
/*  SWMatrixCfgS, SEQGpioCtrlS, EnterSleepS) that internally do       */
/*  read-modify-write on AFECON/ADCCON registers.                     */
/*                                                                     */
/*  The key insight: ADI's sequence generator intercepts WriteReg()   */
/*  to generate SEQ_WR() commands instead of SPI writes. AFECtrlS()   */
/*  does ReadReg(AFECON)→modify→WriteReg(AFECON), tracking state.    */
/*  We replicate this with shadow registers.                          */
/* ================================================================== */


/*
 * seq_gen_buf - Sequence generator buffer and state
 *
 * @cmd:       Buffer for generated SEQ_WR/SEQ_WAIT commands
 * @len:       Number of commands generated so far
 * @max_len:   Buffer capacity
 * @shadow:    Shadow register state (shared across init/measure generation)
 */
struct seq_gen_buf {
	u32			*cmd;
	int			len;
	int			max_len;
	struct seq_shadow_regs	shadow;
};

/* Emit a single sequencer command into the buffer */
static void seq_emit(struct seq_gen_buf *sg, u32 command)
{
	if (sg->len < sg->max_len)
		sg->cmd[sg->len++] = command;
}

/* ---- Equivalent of AD5940_SEQGenInsert(SEQ_WAIT(clks)) ---- */
static void seq_wait(struct seq_gen_buf *sg, u32 clks)
{
	seq_emit(sg, SEQ_WAIT(clks));
}

/* ---- Equivalent of AD5940_WriteReg() when engine is running ---- */
static void seq_write_reg(struct seq_gen_buf *sg, u16 addr, u32 data)
{
	if (addr > 0x21ff) {
		/* Address out of sequencer range - skip */
		return;
	}
	seq_emit(sg, SEQ_WR(addr, data));
}

/*
 * seq_afe_ctrl - Equivalent of AD5940_AFECtrlS(AfeCtrlSet, State)
 *
 * ADI's AFECtrlS() has special handling for inverted bits:
 *   AFECTRL_HPREFPWR (bit5): enable → clear HPREFDIS, disable → set HPREFDIS
 *   AFECTRL_ALDOLIMIT (bit19): enable → clear ALDOILIMITEN, disable → set ALDOILIMITEN
 *
 * All other bits: enable → set, disable → clear
 *
 * Then generate SEQ_WR(AFECON, new_shadow_value)
 */
static void seq_afe_ctrl(struct seq_gen_buf *sg, u32 ctrl_set, bool enable)
{
	u32 special = ctrl_set;

	if (enable) {
		/* Inverted bits: enable means CLEAR the disable bit */
		if (ctrl_set & AD5940_AFECTRL_HPREFPWR) {
			sg->shadow.shadow_afecon &= ~AD5940_AFECON_HPREFDIS;
			special &= ~AD5940_AFECTRL_HPREFPWR;
		}
		if (ctrl_set & AD5940_AFECTRL_ALDOLIMIT) {
			sg->shadow.shadow_afecon &= ~AD5940_AFECON_ALDOILIMITEN;
			special &= ~AD5940_AFECTRL_ALDOLIMIT;
		}
		sg->shadow.shadow_afecon |= special;
	} else {
		/* Inverted bits: disable means SET the disable bit */
		if (ctrl_set & AD5940_AFECTRL_HPREFPWR) {
			sg->shadow.shadow_afecon |= AD5940_AFECON_HPREFDIS;
			special &= ~AD5940_AFECTRL_HPREFPWR;
		}
		if (ctrl_set & AD5940_AFECTRL_ALDOLIMIT) {
			sg->shadow.shadow_afecon |= AD5940_AFECON_ALDOILIMITEN;
			special &= ~AD5940_AFECTRL_ALDOLIMIT;
		}
		sg->shadow.shadow_afecon &= ~special;
	}

	seq_write_reg(sg, AD5940_REG_AFECON, sg->shadow.shadow_afecon);
}

/*
 * seq_adc_mux_cfg - Equivalent of AD5940_ADCMuxCfgS(MuxP, MuxN)
 *
 * Clear MUXSELP and MUXSELN fields, then set new values.
 * Generate SEQ_WR(ADCCON, new_shadow_value)
 */
static void seq_adc_mux_cfg(struct seq_gen_buf *sg, u32 muxp, u32 muxn)
{
	sg->shadow.shadow_adccon &= ~(AD5940_ADCCON_MUXSELP_MASK |
			       AD5940_ADCCON_MUXSELN_MASK);
	sg->shadow.shadow_adccon |= (muxp << AD5940_ADCCON_MUXSELP_SHIFT);
	sg->shadow.shadow_adccon |= (muxn << AD5940_ADCCON_MUXSELN_SHIFT);

	seq_write_reg(sg, AD5940_REG_ADCCON, sg->shadow.shadow_adccon);
}

/*
 * seq_sw_matrix_cfg - Equivalent of AD5940_SWMatrixCfgS(&sw_cfg)
 *
 * Write DSWFULLCON, PSWFULLCON, NSWFULLCON, TSWFULLCON, SWCON(commit)
 */
static void seq_sw_matrix_cfg(struct seq_gen_buf *sg,
			       u32 dsw, u32 psw, u32 nsw, u32 tsw)
{
	seq_write_reg(sg, AD5940_REG_DSWFULLCON, dsw);
	seq_write_reg(sg, AD5940_REG_PSWFULLCON, psw);
	seq_write_reg(sg, AD5940_REG_NSWFULLCON, nsw);
	seq_write_reg(sg, AD5940_REG_TSWFULLCON, tsw);
	seq_write_reg(sg, AD5940_REG_SWCON, AD5940_SWCON_SWSOURCESEL);
}

/*
 * seq_gpio_ctrl - Equivalent of AD5940_SEQGpioCtrlS(Gpio)
 *
 * Writes SYNCEXTDEVICE register via sequencer.
 */
static void seq_gpio_ctrl(struct seq_gen_buf *sg, u32 gpio)
{
	seq_write_reg(sg, AD5940_REG_SYNCEXTDEVICE, gpio);
}

/*
 * seq_enter_sleep - Equivalent of AD5940_EnterSleepS()
 *
 * Write SEQTRGSLP=0 then SEQTRGSLP=1 (trigger hibernate)
 */
static void seq_enter_sleep(struct seq_gen_buf *sg)
{
	seq_write_reg(sg, AD5940_REG_SEQTRGSLP, 0x00);
	seq_write_reg(sg, AD5940_REG_SEQTRGSLP, 0x01);
}

/* ================================================================== */
/*  BIA Init sequence generator                                        */
/*                                                                     */
/*  Faithfully replicates ADI's AppBIASeqCfgGen() which generates      */
/*  SEQ_WR commands for all init register writes. The sequence is      */
/*  executed by the AD5940 sequencer (SEQID_1), matching ADI's flow.   */
/*                                                                     */
/*  Call order (from BodyImpedance.c AppBIASeqCfgGen):                 */
/*    1. REFCfgS  → AFECON(RMW), BUFSENCON, LPREFBUFCON               */
/*    2. HSLoopCfgS → HSDacCfgS, HSTIACfgS, SWMatrixCfgS, WGCfgS      */
/*    3. LPLoopCfgS → LPDACCfgS, LPAMPCfgS                            */
/*    4. DSPCfgS  → ADCBaseCfgS, ADCFilterCfgS+AFECtrlS,              */
/*                   ADCDigCompCfgS, DFTCfgS, StatisticCfgS            */
/*    5. AFECtrlS (enable modules)                                     */
/*    6. SEQGpioCtrlS(0)                                              */
/*    7. SEQ_STOP()                                                   */
/* ================================================================== */

/**
 * ad5940_bia_gen_init_seq - Generate BIA init sequence (ADI AppBIASeqCfgGen flow)
 * @priv: driver private data
 * @buf:  output buffer for generated sequencer commands
 * @buf_size: buffer capacity (in u32 words)
 *
 * Generates the init sequence that is executed once by SEQID_1.
 * This configures all analog blocks (REF, HSLoop, LPLoop, DSP) and
 * enables the AFE modules, exactly as ADI's AppBIASeqCfgGen() does.
 *
 * Return: number of commands generated, or negative errno on failure
 */
static int ad5940_bia_gen_init_seq(struct ad5940_priv *priv,
				   u32 *buf, int buf_size,
				   struct seq_shadow_regs *shadow_out,
				   u32 init_freq_hz,
				   const struct ad5940_freq_params *freq_params)
{
	struct seq_gen_buf sg;
	int afecon_rd;

	sg.cmd = buf;
	sg.len = 0;
	sg.max_len = buf_size;

	/*
	 * Initialize shadow registers from hardware, matching ADI's
	 * sequence generator behavior where first ReadReg() reads
	 * the actual hardware value.
	 */
	afecon_rd = ad5940_spi_read(priv, AD5940_REG_AFECON);
	if (afecon_rd < 0)
		return afecon_rd;
	sg.shadow.shadow_afecon = (u32)afecon_rd;
	sg.shadow.shadow_adccon = (u32)ad5940_spi_read(priv, AD5940_REG_ADCCON);
	sg.shadow.shadow_bufsencon = (u32)ad5940_spi_read(priv, AD5940_REG_BUFSENCON);
	sg.shadow.shadow_adcfiltcon = (u32)ad5940_spi_read(priv, AD5940_REG_ADCFILTERCON);

	/* ================================================================
	 * Step 1: AD5940_REFCfgS - HP/LP reference configuration
	 * ================================================================ */

	/* REFCfgS: AFECON - clear HPREFDIS to enable HP bandgap */
	sg.shadow.shadow_afecon &= ~AD5940_AFECON_HPREFDIS;
	seq_write_reg(&sg, AD5940_REG_AFECON, sg.shadow.shadow_afecon);

	/* REFCfgS: BUFSENCON - read-modify-write, matching ADI's
	 * AD5940_ReadReg(REG_AFE_BUFSENCON) then OR-set bits.
	 * Hp1V8BuffEn=bTRUE  → V1P8HPADCEN
	 * Hp1V1BuffEn=bTRUE  → V1P1HPADCEN
	 * All other fields remain unchanged from hardware default.
	 */
	sg.shadow.shadow_bufsencon |= AD5940_BUFSENCON_V1P8HPADCEN;
	sg.shadow.shadow_bufsencon |= AD5940_BUFSENCON_V1P1HPADCEN;
	seq_write_reg(&sg, AD5940_REG_BUFSENCON, sg.shadow.shadow_bufsencon);

	/* REFCfgS: LPREFBUFCON
	 * LpBandgapEn=bTRUE → LPREFDIS=0 (clear bit)
	 * LpRefBufEn=bTRUE  → LPBUF2P5DIS=0 (clear bit)
	 * LpRefBoostEn=bFALSE → BOOSTCURRENT=0
	 * All zero → LP bandgap ON, LP buf ON
	 */
	seq_write_reg(&sg, AD5940_REG_LPREFBUFCON, 0x00);

	/* ================================================================
	 * Step 2: AD5940_HSLoopCfgS
	 * ================================================================ */

	/* 2a: AD5940_HSDacCfgS - HSDACCON
	 * ExcitBufGain=2, HsDacGain=1, HsDacUpdateRate=7
	 * RATE=7<<1=0x0E, INAMPGNMDE=0, ATTENEN=0
	 */
	seq_write_reg(&sg, AD5940_REG_HSDACCON,
		      7 << AD5940_HSDACCON_RATE_SHIFT);

	/* 2b: AD5940_HSTIACfgS - HSTIACON, HSRTIACON, DE0RESCON
	 * HstiaBias = HSTIABIAS_1P1 (=0), DiodeClose=0
	 * HSTIACON = 0x00
	 */
	seq_write_reg(&sg, AD5940_REG_HSTIACON, 0x00);

	/* HSRTIACON: Ctia=AD5940_BIA_RTIA_CTIA, RtiaSel=AD5940_BIA_RTIA_SEL
	 * Configurable via AD5940_BIA_RTIA_SEL / AD5940_BIA_RTIA_CTIA macros.
	 */
	seq_write_reg(&sg, AD5940_REG_HSRTIACON,
		      (AD5940_BIA_RTIA_CTIA << AD5940_HSRTIACON_CTIACON_SHIFT) |
		      AD5940_BIA_RTIA_SEL);

	/* DE0RESCON: DeRtia=OPEN(0x1F<<3=0xF8), DeRload=OPEN(=5) → 0xFD */
	seq_write_reg(&sg, AD5940_REG_DE0RESCON,
		      (AD5940_HSTIADERTIA_OPEN << 3) |
		      AD5940_HSTIADERLOAD_OPEN);

	/* 2c: AD5940_SWMatrixCfgS - Init switches (PL/NL closed, D open)
	 * D=SWD_OPEN=0, P=SWP_PL|SWP_PL2, N=SWN_NL|SWN_NL2, T=SWT_TRTIA
	 */
	seq_write_reg(&sg, AD5940_REG_DSWFULLCON, AD5940_SWD_OPEN);
	seq_write_reg(&sg, AD5940_REG_PSWFULLCON,
		      AD5940_SWP_PL | AD5940_SWP_PL2);
	seq_write_reg(&sg, AD5940_REG_NSWFULLCON,
		      AD5940_SWN_NL | AD5940_SWN_NL2);
	seq_write_reg(&sg, AD5940_REG_TSWFULLCON, AD5940_SWT_TRTIA);
	seq_write_reg(&sg, AD5940_REG_SWCON, AD5940_SWCON_SWSOURCESEL);

	/* 2d: AD5940_WGCfgS - Waveform generator (SIN mode)
	 * SinFreqWord calculated from init_freq_hz @ 16MHz SysClk
	 * SinAmplitudeWord = 0x7FF (800mVpp, max for ExcitBufGain=2/HsDacGain=1)
	 * SinPhaseWord = 0, SinOffsetWord = 0
	 * WGCON: WGTYPE_SIN=2, GainCalEn=1, OffsetCalEn=1
	 * DACGAINCAL(BIT5) and DACOFFSETCAL(BIT4) enabled to match ADI's
	 * Impedance_Adjustable example (GainCalEn=bTRUE, OffsetCalEn=bTRUE).
	 * This auto-calibrates WG DAC gain/offset for more accurate sine output,
	 * especially important at low frequencies where waveform quality directly
	 * impacts DFT precision.
	 */
	seq_write_reg(&sg, AD5940_REG_WGFCW,
		      ad5940_wg_freq_word_cal(init_freq_hz, 16000000));
	seq_write_reg(&sg, AD5940_REG_WGAMPLITUDE, 0x7FF);
	seq_write_reg(&sg, AD5940_REG_WGOFFSET, 0x00);
	seq_write_reg(&sg, AD5940_REG_WGPHASE, 0x00);
	seq_write_reg(&sg, AD5940_REG_WGCON,
		      (AD5940_WGTYPE_SIN << AD5940_WGCON_TYPESEL_SHIFT) |
		      AD5940_WGCON_DACGAINCAL | AD5940_WGCON_DACOFFSETCAL);

	/* ================================================================
	 * Step 3: AD5940_LPLoopCfgS
	 * ================================================================ */

	/* 3a: AD5940_LPDACCfgS - LPDACCON0, LPDACDAT0, LPDACSW0
	 * LPDACCON0: Src=MMR, VzeroMux=6BIT, VbiasMux=12BIT, Ref=2P5,
	 *            DataRst=FALSE(RSTEN=1), PowerEn=TRUE(PWDEN=0) → 0x01
	 */
	seq_write_reg(&sg, AD5940_REG_LPDACCON0, 0x01);

	/* LPDACDAT0: DacData6Bit=31, DacData12Bit=1675(0x68B)
	 * (1100-200)/2200.0*4095 = 1675.227 → 1675
	 * → (31<<12)|0x68B = 0xF68B
	 */
	seq_write_reg(&sg, AD5940_REG_LPDACDAT0, 0xF68B);

	/* LPDACSW0: VBIAS2LPPA|VBIAS2PIN|VZERO2LPTIA|VZERO2PIN|LPMODEDIS
	 * = 0x10|0x08|0x04|0x02|0x20 = 0x3E
	 */
	seq_write_reg(&sg, AD5940_REG_LPDACSW0,
		      AD5940_LPDACSW_VBIAS2LPPA | AD5940_LPDACSW_VBIAS2PIN |
		      AD5940_LPDACSW_VZERO2LPTIA | AD5940_LPDACSW_VZERO2PIN |
		      AD5940_LPDACSW_LPMODEDIS);

	/* 3b: AD5940_LPAMPCfgS - LPTIACON0, LPTIASW0
	 * LpTiaRf=LPTIARF_20K(=2), LpTiaRload=SHORT(=0), LpTiaRtia=OPEN(=0)
	 * LPTIACON0: 2<<13=0x4000, PA/PWR/TIA bits
	 */
	seq_write_reg(&sg, AD5940_REG_LPTIACON0,
		      AD5940_LPTIARF_20K << 13);

	/* LPTIASW0: SW(5)|SW(6)|SW(7)|SW(8)|SW(9)|SW(12)|SW(13) = 0x33E0 */
	seq_write_reg(&sg, AD5940_REG_LPTIASW0,
		      AD5940_LPTIASW(5) | AD5940_LPTIASW(6) |
		      AD5940_LPTIASW(7) | AD5940_LPTIASW(8) |
		      AD5940_LPTIASW(9) | AD5940_LPTIASW(12) |
		      AD5940_LPTIASW(13));

	/* ================================================================
	 * Step 4: AD5940_DSPCfgS
	 * ================================================================ */

	/* 4a: AD5940_ADCBaseCfgS - ADCCON
	 * Matching ADI's ADCBaseCfgS: full-write from zero (not RMW).
	 * ADCMuxP=ADCMUXP_HSTIA_P(=1), ADCMuxN=ADCMUXN_HSTIA_N(=1),
	 * ADCPga=ADCPGA_1P5(=1) — matching all ADI examples' default.
	 * tempreg = MuxP | (MuxN<<8) | (Pga<<16) = 0x1 | (0x1<<8) | (0x1<<16) = 0x10101
	 */
	sg.shadow.shadow_adccon =
		(AD5940_ADCMUXP_HSTIA_P << AD5940_ADCCON_MUXSELP_SHIFT) |
		(AD5940_ADCMUXN_HSTIA_N << AD5940_ADCCON_MUXSELN_SHIFT) |
		(AD5940_ADCPGA_1P5 << AD5940_ADCCON_GNPGA_SHIFT);
	seq_write_reg(&sg, AD5940_REG_ADCCON, sg.shadow.shadow_adccon);

	/* 4b: AD5940_ADCFilterCfgS - ADCFILTERCON
	 * Matching ADI's RMW: ReadReg → keep AVRGEN → set new fields.
	 * Uses per-band parameters from freq_params.
	 */
	sg.shadow.shadow_adcfiltcon &= AD5940_ADCFILTERCON_AVRGEN;
	sg.shadow.shadow_adcfiltcon |= AD5940_ADCRATE_800KHZ;
	sg.shadow.shadow_adcfiltcon |=
		(AD5940_ADCAVGNUM_16 << AD5940_ADCFILTERCON_AVRGNUM_SHIFT) |  /* Don't care: AVG disabled */
		(freq_params->sinc3osr << AD5940_ADCFILTERCON_SINC3OSR_SHIFT) |
		(freq_params->sinc2osr << AD5940_ADCFILTERCON_SINC2OSR_SHIFT);
	if (freq_params->dft_src == AD5940_DFTSRC_SINC2NOTCH)
		sg.shadow.shadow_adcfiltcon |= AD5940_ADCFILTERCON_LPFBYPEN;
	seq_write_reg(&sg, AD5940_REG_ADCFILTERCON, sg.shadow.shadow_adcfiltcon);

	/* ADCFilterCfgS calls AFECtrlS(SINC2NOTCH, bTRUE/bFALSE) → write AFECON
	 * Conditionally enable SINC2NOTCH based on DFT source.
	 * Band 1-2 (SINC2NOTCH path): enable; Band 3 (SINC3 path): disable.
	 */
	seq_afe_ctrl(&sg, AD5940_AFECTRL_SINC2NOTCH,
		     freq_params->dft_src == AD5940_DFTSRC_SINC2NOTCH);

	/* 4c: AD5940_ADCDigCompCfgS - ADCMIN, ADCMINSM, ADCMAX, ADCMAXSMEN
	 * memset to 0 in ADI's code → all zeros
	 */
	seq_write_reg(&sg, AD5940_REG_ADCMIN, 0x00);
	seq_write_reg(&sg, AD5940_REG_ADCMINSM, 0x00);
	seq_write_reg(&sg, AD5940_REG_ADCMAX, 0x00);
	seq_write_reg(&sg, AD5940_REG_ADCMAXSMEN, 0x00);

	/* 4d: AD5940_DFTCfgS - ADCFILTERCON (clear AVRGEN), DFTCON
	 * Matching ADI's RMW: ReadReg → &= ~AVRGEN → WriteReg
	 * DftSrc≠AVG for both SINC2NOTCH and SINC3, so clear AVRGEN.
	 * Uses per-band DFTNUM and DFTSRC from freq_params.
	 */
	sg.shadow.shadow_adcfiltcon &= ~AD5940_ADCFILTERCON_AVRGEN;
	seq_write_reg(&sg, AD5940_REG_ADCFILTERCON, sg.shadow.shadow_adcfiltcon);

	seq_write_reg(&sg, AD5940_REG_DFTCON,
		      BIT(0) |  /* HANNINGEN */
		      (freq_params->dft_num << AD5940_DFTCON_DFTNUM_SHIFT) |
		      (freq_params->dft_src << AD5940_DFTCON_DFTINSEL_SHIFT));

	/* 4e: AD5940_StatisticCfgS - STATSCON = 0 (disabled) */
	seq_write_reg(&sg, AD5940_REG_STATSCON, 0x00);

	/* ================================================================
	 * Step 5: AD5940_AFECtrlS - Enable AFE modules
	 *
	 * ADI enables: HPREFPWR|HSTIAPWR|INAMPPWR|EXTBUFPWR|
	 *              WG|DACREFPWR|HSDACPWR|SINC2NOTCH
	 *
	 * SINC2NOTCH is conditionally included based on DFT source:
	 * Band 1-2 (SINC2NOTCH) → enable; Band 3 (SINC3) → skip.
	 * ================================================================ */
	{
		u32 afe_en = AD5940_AFECTRL_HPREFPWR | AD5940_AFECTRL_HSTIAPWR |
			     AD5940_AFECTRL_INAMPPWR | AD5940_AFECTRL_EXTBUFPWR |
			     AD5940_AFECTRL_WG | AD5940_AFECTRL_DACREFPWR |
			     AD5940_AFECTRL_HSDACPWR;
		if (freq_params->dft_src == AD5940_DFTSRC_SINC2NOTCH)
			afe_en |= AD5940_AFECTRL_SINC2NOTCH;
		seq_afe_ctrl(&sg, afe_en, true);
	}

	/* ================================================================
	 * Step 6: AD5940_SEQGpioCtrlS(0) - No GPIO pins during init
	 * ================================================================ */
	seq_gpio_ctrl(&sg, 0);

	/* ================================================================
	 * Step 7: SEQ_STOP() - End of init sequence
	 * ================================================================ */
	seq_emit(&sg, SEQ_STOP());

	/* Output final shadow register state for measure sequence generation */
	if (shadow_out)
		*shadow_out = sg.shadow;

	return sg.len;
}

/**
 * ad5940_bia_gen_measure_seq - Generate BIA measure sequence (ADI flow)
 * @priv: driver private data
 * @buf:  output buffer for generated sequencer commands
 * @buf_size: buffer capacity (in u32 words)
 * @shadow_in: shadow register state from init sequence generation
 *
 * Generates the measurement sequence, including DSP reconfiguration
 * for band changes (matching ADI's Impedance_Adjustable approach).
 *
 * Sequence structure:
 *
 *   SEQGpioCtrlS(AGPIO_Pin6)
 *   SEQGenInsert(SEQ_WAIT(16*250))
 *   SWMatrixCfgS(D=CE0, P=CE0, N=AIN1, T=AIN1|TRTIA)
 *   --- DSP reconfiguration (ADCFILTERCON, DFTCON, SINC2NOTCH) ---
 *   ADCMuxCfgS(HSTIA_P, HSTIA_N)
 *   AFECtrlS(WG|ADCPWR, bTRUE)
 *   SEQGenInsert(SEQ_WAIT(16*50))
 *   AFECtrlS(ADCCNV|DFT, bTRUE)
 *   SEQGenInsert(SEQ_WAIT(WaitClks))
 *   AFECtrlS(ADCCNV|DFT|WG|ADCPWR, bFALSE)
 *   ADCMuxCfgS(AIN3, AIN2)
 *   AFECtrlS(WG|ADCPWR, bTRUE)
 *   SEQGenInsert(SEQ_WAIT(16*50))
 *   AFECtrlS(ADCCNV|DFT, bTRUE)
 *   SEQGenInsert(SEQ_WAIT(WaitClks))
 *   AFECtrlS(ADCCNV|DFT|WG|ADCPWR, bFALSE)
 *   SWMatrixCfgS(D=OPEN, P=PL|PL2, N=NL|NL2, T=TRTIA)
 *   SEQGpioCtrlS(0)
 *   EnterSleepS()
 *
 * Shadow register state is passed from ad5940_bia_gen_init_seq(),
 * matching ADI's SEQGen database that persists across init and
 * measure sequence generation calls.
 *
 * Return: number of commands generated, or negative errno on failure
 */
static int ad5940_bia_gen_measure_seq(struct ad5940_priv *priv,
				       u32 *buf, int buf_size,
				       const struct seq_shadow_regs *shadow_in,
				       const struct ad5940_freq_params *freq_params)
{
	struct seq_gen_buf sg;

	sg.cmd = buf;
	sg.len = 0;
	sg.max_len = buf_size;

	/*
	 * Initialize shadow registers from the init sequence's final state.
	 * This matches ADI's SEQGen behavior where the RegInfo database
	 * persists between AppBIASeqCfgGen() and AppBIASeqMeasureGen().
	 *
	 * We cannot read these from hardware because the init sequence
	 * hasn't executed yet at this point (only written to SRAM).
	 * Passing them from the init sequence generator ensures correctness
	 * without fragile manual computation of expected register values.
	 */
	if (shadow_in)
		sg.shadow = *shadow_in;
	else
		memset(&sg.shadow, 0, sizeof(sg.shadow));

	/* ---- Begin: exact AppBIASeqMeasureGen() call order ---- */

	/* AD5940_SEQGpioCtrlS(AGPIO_Pin6) */
	seq_gpio_ctrl(&sg, AD5940_AGPIO_Pin6);

	/* AD5940_SEQGenInsert(SEQ_WAIT(16*250))  - wait 250us */
	seq_wait(&sg, 16 * 250);

	/* AD5940_SWMatrixCfgS: D=CE0, P=CE0, N=AIN1, T=AIN1|TRTIA */
	seq_sw_matrix_cfg(&sg,
			  AD5940_SWD_CE0,
			  AD5940_SWP_CE0,
			  AD5940_SWN_AIN1,
			  AD5940_SWT_AIN1 | AD5940_SWT_TRTIA);

	/*
	 * Write ADCFILTERCON and DFTCON in the measure sequence so they
	 * take effect during WUPT wakeup.  Direct SPI writes between
	 * measurements may be lost if the AFE enters hibernate and
	 * re-initializes registers on wakeup.  ADI's Impedance_Adjustable
	 * example does the same: AppIMPCheckFreq() reconfigures the DSP
	 * via the measurement sequence on each frequency change.
	 */
	{
		u32 filtcon = AD5940_ADCRATE_800KHZ |
			      (AD5940_ADCAVGNUM_16 <<
			       AD5940_ADCFILTERCON_AVRGNUM_SHIFT) |
			      (freq_params->sinc3osr <<
			       AD5940_ADCFILTERCON_SINC3OSR_SHIFT) |
			      (freq_params->sinc2osr <<
			       AD5940_ADCFILTERCON_SINC2OSR_SHIFT);
		if (freq_params->dft_src == AD5940_DFTSRC_SINC2NOTCH)
			filtcon |= AD5940_ADCFILTERCON_LPFBYPEN;
		seq_write_reg(&sg, AD5940_REG_ADCFILTERCON, filtcon);
		/* Clear AVRGEN (DFT source, not average) */
		filtcon &= ~AD5940_ADCFILTERCON_AVRGEN;
		seq_write_reg(&sg, AD5940_REG_ADCFILTERCON, filtcon);
	}

	/* DFTCON */
	seq_write_reg(&sg, AD5940_REG_DFTCON,
		      BIT(AD5940_DFTCON_HANNINGEN_SHIFT) |
		      (freq_params->dft_num <<
		       AD5940_DFTCON_DFTNUM_SHIFT) |
		      (freq_params->dft_src <<
		       AD5940_DFTCON_DFTINSEL_SHIFT));

	/* SINC2NOTCH enable must match DFT source */
	if (freq_params->dft_src == AD5940_DFTSRC_SINC2NOTCH)
		sg.shadow.shadow_afecon |= AD5940_AFECTRL_SINC2NOTCH;
	else
		sg.shadow.shadow_afecon &= ~AD5940_AFECTRL_SINC2NOTCH;
	seq_write_reg(&sg, AD5940_REG_AFECON, sg.shadow.shadow_afecon);

	/* ---- Step 1: Measure Current (HSTIA_P/N) ---- */
	/* AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N) */
	seq_adc_mux_cfg(&sg, AD5940_ADCMUXP_HSTIA_P, AD5940_ADCMUXN_HSTIA_N);

	/* AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE) */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR,
		     true);

	/* AD5940_SEQGenInsert(SEQ_WAIT(16*50))  - wait 50us */
	seq_wait(&sg, 16 * 50);

	/* AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE) */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT,
		     true);

	/* AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)) */
	seq_wait(&sg, freq_params->wait_clks);

	/*
	 * AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE)
	 * Stop ADC, DFT, WG, and ADC power — matching all three ADI examples
	 * (BIA BodyImpedance.c, BIOZ-2Wire.c, Impedance_Adjustable.c).
	 * The WG must be stopped before switching MUX so that the ADC input
	 * is released; otherwise the second DFT still reads the old channel.
	 */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
		     AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR,
		     false);

	/* ---- Step 2: Measure Voltage (AIN3/AIN2) ---- */
	/* AD5940_ADCMuxCfgS(ADCMUXP_AIN3, ADCMUXN_AIN2) */
	seq_adc_mux_cfg(&sg, AD5940_ADCMUXP_AIN3, AD5940_ADCMUXN_AIN2);

	/*
	 * AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE)
	 * Re-enable WG and ADC power after MUX switch, matching ADI flow.
	 * WG restarts from phase 0 — this is the same behavior as all
	 * three ADI examples.  The phase relationship between the two
	 * DFT results is handled in the impedance calculation (not here).
	 */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR,
		     true);

	/* AD5940_SEQGenInsert(SEQ_WAIT(16*50)) - wait 50us for settling */
	seq_wait(&sg, 16 * 50);

	/* AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE) */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT,
		     true);

	/* AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)) */
	seq_wait(&sg, freq_params->wait_clks);

	/* AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE) */
	seq_afe_ctrl(&sg,
		     AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
		     AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR,
		     false);

	/* AD5940_SWMatrixCfgS: D=OPEN, P=PL|PL2, N=NL|NL2, T=TRTIA */
	seq_sw_matrix_cfg(&sg,
			  AD5940_SWD_OPEN,
			  AD5940_SWP_PL | AD5940_SWP_PL2,
			  AD5940_SWN_NL | AD5940_SWN_NL2,
			  AD5940_SWT_TRTIA);

	/* AD5940_SEQGpioCtrlS(0) */
	seq_gpio_ctrl(&sg, 0);

	/* AD5940_EnterSleepS() */
	seq_enter_sleep(&sg);

	return sg.len;
}

/* ================================================================== */
/*  Three-band DSP parameter lookup                                   */
/* ================================================================== */

/*
 * OSR lookup tables indexed by register value.
 * Used by ad5940_clks_calculate() to convert register values to actual OSR.
 */
static const u32 sinc2osr_table[] = {
	22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333
};
static const u32 sinc3osr_table[] = {
	5, 4, 2
};

/**
 * ad5940_clks_calculate - Calculate sequencer wait clocks for DFT
 * @dft_num:   DFTNUM_xxx register value
 * @dft_src:   DFTSRC_xxx register value
 * @sinc3osr:  ADCSINC3OSR_xxx register value
 * @sinc2osr:  ADCSINC2OSR_xxx register value
 *
 * Simplified port of ADI's AD5940_ClksCalculate() for the BIA use case.
 * Always uses RatioSys2AdcClk=1 (SysClk=16MHz, AdcClk=16MHz) and
 * BpNotch=TRUE (bypass notch filter).
 *
 * Return: wait clock count (in 16MHz clock cycles)
 */
static u32 ad5940_clks_calculate(u32 dft_num, u32 dft_src,
				 u32 sinc3osr, u32 sinc2osr)
{
	u32 data_count, temp;

	if (sinc3osr >= ARRAY_SIZE(sinc3osr_table) ||
	    sinc2osr >= ARRAY_SIZE(sinc2osr_table))
		return 0;

	data_count = 1UL << (dft_num + 2);

	if (dft_src == AD5940_DFTSRC_SINC3) {
		/* SINC3 path: DATATYPE_DFT → DATATYPE_SINC3 */
		temp = ((data_count + 2) * sinc3osr_table[sinc3osr] + 1) * 20;
		temp += 25;
		return temp;
	}

	/* SINC2NOTCH path with BpNotch=TRUE:
	 * DATATYPE_DFT → DATATYPE_SINC2 → DATATYPE_SINC3 → back
	 */
	temp = (data_count + 1) * sinc2osr_table[sinc2osr] + 1;
	temp = ((temp + 2) * sinc3osr_table[sinc3osr] + 1) * 20;
	temp += 15 + 25;
	return temp;
}

/**
 * ad5940_get_freq_params - Get DSP parameters for a frequency (3-band lookup)
 * @freq_hz: target excitation frequency in Hz
 *
 * Simple 3-band lookup table. Each band uses proven, static parameters:
 *
 *   Band 1 (freq <= 100Hz):
 *     SINC2NOTCH, S2OSR=667, S3OSR=4, DFTNUM=8192, low power
 *     ADC rate ~ 300Hz, 3x Nyquist up to 100Hz (planC-proven)
 *
 *   Band 2 (100Hz < freq <= 2kHz):
 *     SINC2NOTCH, S2OSR=22, S3OSR=4, DFTNUM=8192, low power
 *     ADC rate ~ 9091Hz, 3x Nyquist up to 3kHz
 *
 *   Band 3 (freq > 2kHz):
 *     SINC3, S3OSR=2, DFTNUM=8192, high power (ADI BIA example)
 *     ADC rate = 400kHz, supports up to ~200kHz
 *
 * Return: struct ad5940_freq_params with static settings
 */
struct ad5940_freq_params ad5940_get_freq_params(u32 freq_hz)
{
	struct ad5940_freq_params params = {0};

	if (freq_hz <= 100) {
		/* Band 1: Low frequency (planC-proven) */
		params.dft_src  = AD5940_DFTSRC_SINC2NOTCH;
		params.sinc2osr = AD5940_ADCSINC2OSR_667;
		params.sinc3osr = AD5940_ADCSINC3OSR_4;
		params.dft_num  = AD5940_DFTNUM_8192;
		params.high_pwr = false;
	} else if (freq_hz <= 2000) {
		/* Band 2: Mid frequency */
		params.dft_src  = AD5940_DFTSRC_SINC2NOTCH;
		params.sinc2osr = AD5940_ADCSINC2OSR_22;
		params.sinc3osr = AD5940_ADCSINC3OSR_4;
		params.dft_num  = AD5940_DFTNUM_8192;
		params.high_pwr = false;
	} else {
		/* Band 3: High frequency (ADI BIA example) */
		params.dft_src  = AD5940_DFTSRC_SINC3;
		params.sinc2osr = AD5940_ADCSINC2OSR_22; /* don't care in SINC3 */
		params.sinc3osr = AD5940_ADCSINC3OSR_2;
		params.dft_num  = AD5940_DFTNUM_8192;
		params.high_pwr = true;
	}

	/* Calculate wait_clks — no ×2 margin, matching ADI's AD5940_ClksCalculate()
	 * which only adds a 25-clock margin.  The ×2 margin caused Band1's
	 * wait_clks to be ~874M clocks (54s/step), making total measurement
	 * time ~108s which exceeded practical WUPT timing limits.
	 */
	params.wait_clks = ad5940_clks_calculate(
		params.dft_num, params.dft_src,
		params.sinc3osr, params.sinc2osr);

	return params;
}
EXPORT_SYMBOL_GPL(ad5940_get_freq_params);

/**
 * ad5940_freq_band_id - Map a frequency to its band ID
 * @freq_hz: frequency in Hz
 *
 * Band 0: freq <= 100Hz   (SINC2NOTCH, S2OSR=667, S3OSR=4)
 * Band 1: 100Hz < freq <= 2kHz (SINC2NOTCH, S2OSR=22, S3OSR=4)
 * Band 2: freq > 2kHz    (SINC3, S3OSR=2, high power)
 *
 * Return: band ID (0, 1, or 2)
 */
static int ad5940_freq_band_id(u32 freq_hz)
{
	if (freq_hz <= 100)
		return 0;
	else if (freq_hz <= 2000)
		return 1;
	else
		return 2;
}

/**
 * ad5940_band_representative_freq - Get representative calibration frequency for a band
 * @band_id: band ID (0, 1, or 2)
 *
 * Returns a frequency near the high end of the band for best DFT SNR,
 * since wait_clks is constant within a band (determined by DSP params).
 *
 * Return: representative frequency in Hz
 */
static u32 ad5940_band_representative_freq(int band_id)
{
	switch (band_id) {
	case 0:  return 80;		/* Band 0: ≤100Hz, use 80Hz */
	case 1:  return 1000;		/* Band 1: 100Hz~2kHz, use 1kHz */
	case 2:  return 50000;		/* Band 2: >2kHz, use 50kHz (ADI default) */
	default: return 50000;
	}
}

/* ================================================================== */
/*  Frequency sweep helpers                                            */
/* ================================================================== */

/**
 * ad5940_wg_freq_word_cal - Calculate WGFCW register value for a frequency
 * @freq_hz: target frequency in Hz
 * @sysclk_hz: system clock frequency in Hz (typically 16 MHz)
 *
 * Replicates ADI's AD5940_WGFreqWordCal() for S1 silicon (26-bit).
 * Formula: WGFCW = round(freq * 2^26 / sysclk), clamped to 20-bit max.
 *
 * Return: WGFCW register value
 */
u32 ad5940_wg_freq_word_cal(u32 freq_hz, u32 sysclk_hz)
{
	u32 freq_word;

	if (freq_hz == 0 || sysclk_hz == 0)
		return 0;

	/* S1 silicon: 26-bit frequency word, max 20-bit register value */
	freq_word = (u32)div_u64(((u64)freq_hz << 26) + sysclk_hz / 2,
				 sysclk_hz);
	if (freq_word > 0xFFFFF)
		freq_word = 0xFFFFF;

	return freq_word;
}
EXPORT_SYMBOL_GPL(ad5940_wg_freq_word_cal);

/**
 * ad5940_sweep_calc_freq - Calculate frequency for a sweep index
 * @start_hz: sweep start frequency in Hz
 * @stop_hz: sweep stop frequency in Hz
 * @points: total number of frequency points
 * @type: sweep type (AD5940_SWEEP_LINEAR, etc.)
 * @index: frequency index (0 .. points-1)
 *
 * Return: frequency in Hz for the given index
 */
u32 ad5940_sweep_calc_freq(u32 start_hz, u32 stop_hz, u32 points,
			    enum ad5940_sweep_type type, u32 index)
{
	if (points <= 1)
		return start_hz;

	switch (type) {
	case AD5940_SWEEP_LOG:
		/* Logarithmic: freq[i] = start * (stop/start)^(i/(points-1))
		 * Compute via incremental multiplication in Q16 fixed-point.
		 * step = (stop/start)^(1/(N-1)), freq[i] = start * step^i
		 */
		if (start_hz == 0 || stop_hz == 0)
			return start_hz;
		if (index == 0)
			return start_hz;
		if (index >= points - 1)
			return stop_hz;
		{
			/* Compute step ratio using Newton-Raphson in Q16:
			 * Find r such that r^(N-1) = stop/start
			 * i.e. r^(N-1) = ratio_q16
			 */
			u32 n = points - 1;
			u64 ratio_q16 = div64_u64((u64)stop_hz << 16,
						   start_hz);
			u64 r;   /* step ratio in Q16 */
			u64 freq_q16;
			int j;

			/* Initial guess: r ≈ 2^(log2(ratio)/n) */
			{
				int log2_r = fls64(ratio_q16) - 1 - 16;

				if (log2_r < 0)
					log2_r = 0;
				r = 1ULL << (16 + log2_r / (int)n);
			}

			/* Newton-Raphson: r = r - (r^n - R) / (n * r^(n-1)) */
			for (j = 0; j < 8; j++) {
				u64 rn_n = (1ULL << 16);   /* Q16: 1.0 */
				u64 rn_n1 = (1ULL << 16);  /* Q16: 1.0 */
				s64 diff;
				int k;

				for (k = 0; k < (int)n; k++)
					rn_n = (rn_n * r) >> 16;
				for (k = 0; k < (int)n - 1; k++)
					rn_n1 = (rn_n1 * r) >> 16;

				diff = (s64)rn_n - (s64)ratio_q16;
				if (diff == 0)
					break;
				/* delta = diff / (n * r^(n-1)), scaled by 2^16 */
				r = r - div64_s64(diff << 16,
						   (s64)n * (s64)rn_n1);
			}

			/* Compute freq = start * r^index */
			freq_q16 = (u64)start_hz << 16;
			for (j = 0; j < (int)index; j++)
				freq_q16 = (freq_q16 * r) >> 16;

			return freq_q16 >> 16;
		}
	case AD5940_SWEEP_LINEAR:
	default:
		/* Linear: freq[i] = start + i * (stop - start) / (points - 1) */
		if (stop_hz >= start_hz)
			return start_hz + (u32)div_u64(
				(u64)index * (stop_hz - start_hz), points - 1);
		else
			return start_hz - (u32)div_u64(
				(u64)index * (start_hz - stop_hz), points - 1);
	}
}
EXPORT_SYMBOL_GPL(ad5940_sweep_calc_freq);

/**
 * ad5940_bia_sweep_step - Advance sweep state and update WGFCW
 * @priv: driver private data
 *
 * Called from the trigger handler after reading FIFO data and
 * pushing it to the IIO buffer.  Prepares the AFE for the next
 * measurement cycle:
 *   1. Update sweep_curr_freq_hz to the next measurement frequency
 *   2. Tag freq_of_data_hz for the next data push
 *   3. Advance sweep_index and calculate the following frequency
 *   4. If frequency band changed, regenerate measure sequence + update PMBW
 *   5. Write WGFCW for the upcoming measurement
 *   6. Update RTIA calibration for the upcoming measurement
 *
 * Return: 0 on success, negative errno on SPI write failure
 */
int ad5940_bia_sweep_step(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	u32 freq_word;
	int ret;
	struct ad5940_freq_params next_params;

	if (!priv->sweep_en)
		return 0;

	/* The next measurement will use sweep_next_freq_hz */
	priv->sweep_curr_freq_hz = priv->sweep_next_freq_hz;

	/* Tag the NEXT data push with the upcoming measurement frequency */
	priv->freq_of_data_hz = priv->sweep_curr_freq_hz;

	/* Advance to next frequency point */
	priv->sweep_index++;
	if (priv->sweep_index >= priv->sweep_points)
		priv->sweep_index = 0;

	/* Calculate the frequency for the measurement AFTER next */
	priv->sweep_next_freq_hz = ad5940_sweep_calc_freq(
		priv->sweep_start_hz, priv->sweep_stop_hz,
		priv->sweep_points, priv->sweep_type,
		(priv->sweep_index + 1) % priv->sweep_points);

	/* Check if the next frequency falls in a different band */
	next_params = ad5940_get_freq_params(priv->sweep_curr_freq_hz);

	if (next_params.sinc2osr != priv->curr_freq_params.sinc2osr ||
	    next_params.sinc3osr != priv->curr_freq_params.sinc3osr ||
	    next_params.dft_num != priv->curr_freq_params.dft_num ||
	    next_params.dft_src != priv->curr_freq_params.dft_src ||
	    next_params.high_pwr != priv->curr_freq_params.high_pwr) {
		/*
		 * Band change: regenerate measure sequence with new DSP
		 * parameters and write it to SRAM. Also update PMBW if
		 * power mode changed.
		 */
		u32 meas_buf[128];
		int meas_len;

		dev_info(dev, "Band change at %uHz: S2OSR %u→%u, DFTSRC %u→%u, HP %u→%u\n",
			 priv->sweep_curr_freq_hz,
			 priv->curr_freq_params.sinc2osr, next_params.sinc2osr,
			 priv->curr_freq_params.dft_src, next_params.dft_src,
			 priv->curr_freq_params.high_pwr, next_params.high_pwr);

		/*
		 * Regenerate measure sequence with new wait_clks.
		 * Use saved shadow registers from initial sequence generation
		 * so that seq_afe_ctrl() produces correct AFECON values
		 * (instead of all-zero shadow which would clear critical bits).
		 */
		meas_len = ad5940_bia_gen_measure_seq(priv, meas_buf,
						      ARRAY_SIZE(meas_buf),
						      &priv->meas_shadow,
						      &next_params);
		if (meas_len > 0) {
			ret = ad5940_seq_cmd_write(priv,
						   priv->meas_seq_addr,
						   meas_buf, meas_len);
			if (ret) {
				dev_err(dev, "Band change: meas seq write failed: %d\n",
					ret);
				return ret;
			}

			/* Update SEQ0INFO if command count changed */
			if (meas_len != priv->meas_seq_len) {
				ad5940_spi_write(priv, AD5940_REG_SEQ0INFO,
						 (meas_len << 16) |
						 priv->meas_seq_addr);
			}

			priv->meas_seq_len = meas_len;

			dev_info(dev, "Regenerated meas seq: %d cmds at SRAM[%u]\n",
				 meas_len, priv->meas_seq_addr);
		}

		/* Update PMBW if power mode changed */
		if (next_params.high_pwr != priv->curr_freq_params.high_pwr) {
			u32 pmbw_val = (3 << AD5940_PMBW_SYSBW_SHIFT);
			if (next_params.high_pwr)
				pmbw_val |= AD5940_PMBW_SYSHP;
			ad5940_spi_write(priv, AD5940_REG_PMBW, pmbw_val);
		}

		/*
		 * DSP register updates (ADCFILTERCON, DFTCON, SINC2NOTCH)
		 * are now handled by the measurement sequence itself
		 * (seq_write_reg commands), ensuring they take effect
		 * reliably after WUPT wakeup.  No direct SPI writes needed.
		 */

		priv->curr_freq_params = next_params;
	}

	/* Write WGFCW for the upcoming measurement (at sweep_curr_freq_hz) */
	freq_word = ad5940_wg_freq_word_cal(priv->sweep_curr_freq_hz,
					    16000000);
	ret = ad5940_spi_write(priv, AD5940_REG_WGFCW, freq_word);
	if (ret)
		return ret;

	/* Update RTIA calibration for the upcoming measurement frequency */
	priv->rtia_cal = priv->band_cal_table[priv->sweep_band_map[priv->sweep_index]];

	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_sweep_step);

/* ================================================================== */
/*  RTIA Calibration - replicates ADI's AD5940_HSRtiaCal()            */
/* ================================================================== */

/*
 * rtia_cal_afe_ctrl - Direct SPI AFE control (matching ADI's AFECtrlS)
 * @priv: driver private data
 * @ctrl_set: bits to modify (AFECTRL_* constants)
 * @enable: true to enable, false to disable
 *
 * Exactly replicates AD5940_AFECtrlS() register behavior including
 * the inverted semantics of HPREFPWR(bit5) and ALDOLIMIT(bit19).
 */
static int rtia_cal_afe_ctrl(struct ad5940_priv *priv, u32 ctrl_set,
			      bool enable)
{
	u32 val, special = ctrl_set;
	int ret;

	ret = ad5940_spi_read32(priv, AD5940_REG_AFECON, &val);
	if (ret)
		return ret;

	if (enable) {
		if (ctrl_set & AD5940_AFECTRL_HPREFPWR) {
			val &= ~AD5940_AFECON_HPREFDIS;
			special &= ~AD5940_AFECTRL_HPREFPWR;
		}
		if (ctrl_set & AD5940_AFECTRL_ALDOLIMIT) {
			val &= ~AD5940_AFECON_ALDOILIMITEN;
			special &= ~AD5940_AFECTRL_ALDOLIMIT;
		}
		val |= special;
	} else {
		if (ctrl_set & AD5940_AFECTRL_HPREFPWR) {
			val |= AD5940_AFECON_HPREFDIS;
			special &= ~AD5940_AFECTRL_HPREFPWR;
		}
		if (ctrl_set & AD5940_AFECTRL_ALDOLIMIT) {
			val |= AD5940_AFECON_ALDOILIMITEN;
			special &= ~AD5940_AFECTRL_ALDOLIMIT;
		}
		val &= ~special;
	}

	return ad5940_spi_write(priv, AD5940_REG_AFECON, val);
}

/*
 * rtia_cal_adc_mux_cfg - Direct SPI ADC MUX config (matching ADI's ADCMuxCfgS)
 * @priv: driver private data
 * @muxp: positive MUX selection
 * @muxn: negative MUX selection
 *
 * RMW on ADCCON: preserve PGA, update MUXSELP and MUXSELN.
 */
static int rtia_cal_adc_mux_cfg(struct ad5940_priv *priv, u32 muxp, u32 muxn)
{
	u32 val;
	int ret;

	ret = ad5940_spi_read32(priv, AD5940_REG_ADCCON, &val);
	if (ret)
		return ret;
	val &= ~(AD5940_ADCCON_MUXSELP_MASK | AD5940_ADCCON_MUXSELN_MASK);
	val |= muxp << AD5940_ADCCON_MUXSELP_SHIFT;
	val |= muxn << AD5940_ADCCON_MUXSELN_SHIFT;

	return ad5940_spi_write(priv, AD5940_REG_ADCCON, val);
}

/*
 * rtia_cal_wait_dft_ready - Poll INTCFLAG1 for DFTRDY via SPI
 * @priv: driver private data
 * @timeout_ms: maximum wait time in milliseconds
 *
 * Matches ADI's: while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);
 * Uses SPI polling with ~100us intervals.
 *
 * Return: 0 on success, -ETIMEDOUT on timeout
 */
static int rtia_cal_wait_dft_ready(struct ad5940_priv *priv, int timeout_ms)
{
	int i, ret;

	/* Poll at ~100us intervals, timeout_ms * 10 = number of polls */
	for (i = 0; i < timeout_ms * 10; i++) {
		u32 flag;

		ret = ad5940_spi_read32(priv, AD5940_REG_INTCFLAG1, &flag);
		if (ret)
			 return ret;
		if (flag & AD5940_AFEINTSRC_DFTRDY)
			return 0;
		usleep_range(80, 120);
	}
	return -ETIMEDOUT;
}

/*
 * rtia_cal_atan2_mdeg - Integer atan2 in millidegrees using CORDIC
 * @y: imaginary component
 * @x: real component
 *
 * Returns angle in millidegrees, range [-180000, 180000].
 * Uses CORDIC vectoring mode with 16 iterations for ~0.01° accuracy.
 */
static s32 rtia_cal_atan2_mdeg(s64 y, s64 x)
{
	static const s32 atan_table[16] = {
		45000, 26565, 14036, 7125, 3576, 1789, 895, 448,
		224, 112, 56, 28, 14, 7, 4, 2
	};
	s32 angle = 0;
	int i;

	if (x == 0 && y == 0)
		return 0;

	/* Normalize to fit 32-bit CORDIC range while preserving ratio */
	while (x > 0x3FFFFFFF || x < -0x3FFFFFFF ||
	       y > 0x3FFFFFFF || y < -0x3FFFFFFF) {
		x >>= 1;
		y >>= 1;
	}

	/* Handle 2nd/3rd quadrants via recursion */
	if (x < 0) {
		if (y >= 0)
			return 180000 - rtia_cal_atan2_mdeg(y, -x);
		else
			return -180000 + rtia_cal_atan2_mdeg(-y, -x);
	}

	/* CORDIC vectoring mode: rotate vector towards x-axis */
	for (i = 0; i < 16; i++) {
		s64 x_new, y_new;

		if (y >= 0) {
			x_new = x + (y >> i);
			y_new = y - (x >> i);
			angle += atan_table[i];
		} else {
			x_new = x - (y >> i);
			y_new = y + (x >> i);
			angle -= atan_table[i];
		}
		x = x_new;
		y = y_new;
	}

	return angle;
}

/**
 * ad5940_bia_rtia_cal - Perform RTIA calibration (per-band)
 * @priv: driver private data
 *
 * Faithfully replicates ADI's AppBIARtiaCal() → AD5940_HSRtiaCal() flow,
 * but optimizes calibration from per-frequency to per-band:
 *
 *   - RTIA is a physical resistor whose value doesn't change with frequency.
 *   - Frequency-dependent variation (from CTIA parasitics, PCB traces) is
 *     dominated by the DSP filter configuration, which is constant within
 *     each band.
 *   - Per-band calibration captures band-to-band differences while
 *     dramatically reducing startup time for wide-band sweeps:
 *       e.g. 100-point 10Hz-200kHz sweep: ~10min → ~56s
 *
 * Band definitions match ad5940_get_freq_params():
 *   Band 0: freq ≤ 100Hz   (SINC2NOTCH, S2OSR=667, S3OSR=4)
 *   Band 1: 100Hz < freq ≤ 2kHz (SINC2NOTCH, S2OSR=22, S3OSR=4)
 *   Band 2: freq > 2kHz    (SINC3, S3OSR=2, high power)
 *
 * Representative calibration frequencies (chosen near band high end
 * for best DFT SNR; wait_clks is constant within a band):
 *   Band 0: 80Hz, Band 1: 1000Hz, Band 2: 50000Hz
 *
 * When sweep is disabled: single-frequency calibration at 50kHz (Band 2).
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_rtia_cal(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	int ret, i, band_id;
	u32 val, intc1_saved;
	s32 dft_rcal_real, dft_rcal_imag;
	s32 dft_rtia_real, dft_rtia_imag;
	u32 excit_buf_gain, hs_dac_gain;
	u32 excit_volt_mv; /* Excitation voltage in mVpp */
	u32 wg_amp_word;
	u32 freq_word;
	u32 full_range_mv;

	/* ---- RTIA configuration (from ad5940_core.h macros) ---- */
	const u32 rcal_ohm = AD5940_BIA_RCAL_OHM;	/* RCAL on PCB */
	const u32 rtia_sel = AD5940_BIA_RTIA_SEL;
	const u32 rtia_val = AD5940_BIA_RTIA_OHM;
	const u32 ctia_val = AD5940_BIA_RTIA_CTIA;
	const u32 sysclk_hz = 16000000; /* SysClkFreq = 16MHz */

	/* ---- Phase 0: Build band map and identify active bands ---- */
	bool band_active[AD5940_MAX_FREQ_BANDS] = { false };
	int active_bands[AD5940_MAX_FREQ_BANDS];
	int num_active_bands = 0;
	int first_active_band = -1;

	if (priv->sweep_en) {
		for (i = 0; i < priv->sweep_points; i++) {
			u32 freq = ad5940_sweep_calc_freq(
				priv->sweep_start_hz, priv->sweep_stop_hz,
				priv->sweep_points, priv->sweep_type, i);
			int bid = ad5940_freq_band_id(freq);

			priv->sweep_band_map[i] = bid;
			if (!band_active[bid]) {
				band_active[bid] = true;
				active_bands[num_active_bands++] = bid;
			}
		}
	} else {
		/* Single-frequency: Band 2 at 50kHz */
		int bid = ad5940_freq_band_id(50000);

		priv->sweep_band_map[0] = bid;
		band_active[bid] = true;
		active_bands[0] = bid;
		num_active_bands = 1;
	}

	first_active_band = active_bands[0];

	dev_info(dev, "RTIA cal: %d active band(s) out of %d\n",
		 num_active_bands, AD5940_MAX_FREQ_BANDS);

	/* ---- Phase 1: Parameter calculation ---- */

	/* ExcitVolt = 1800 * 0.8 * Rcal / Rtia  (mVpp) */
	excit_volt_mv = (u32)div_u64(1440ULL * rcal_ohm, rtia_val);

	/* Gain/amplitude selection - matching AD5940_HSRtiaCal lines 3460-3490 */
	if (excit_volt_mv <= 40) {		/* <= 800*0.05 */
		excit_buf_gain = 1;	/* EXCITBUFGAIN_0P25 */
		hs_dac_gain = 1;	/* HSDACGAIN_0P2 */
		full_range_mv = 40;
	} else if (excit_volt_mv <= 200) {	/* <= 800*0.25 */
		excit_buf_gain = 1;
		hs_dac_gain = 0;	/* HSDACGAIN_1 */
		full_range_mv = 200;
	} else if (excit_volt_mv <= 320) {	/* <= 800*0.4 */
		excit_buf_gain = 0;	/* EXCITBUFGAIN_2 */
		hs_dac_gain = 1;
		full_range_mv = 320;
	} else {
		excit_buf_gain = 0;
		hs_dac_gain = 0;
		full_range_mv = 1600;
	}

	/* WgAmpWord = ((ExcitVolt/FullRange * 2047 * 2) + 1) >> 1 */
	wg_amp_word = (u32)div_u64((u64)excit_volt_mv * 2047 * 2,
				   full_range_mv);
	wg_amp_word = (wg_amp_word + 1) >> 1;
	if (wg_amp_word > 0x7FF)
		wg_amp_word = 0x7FF;

	dev_info(dev, "RTIA cal: ExcitVolt=%umVpp, WgAmp=0x%03x, bands=%d\n",
		 excit_volt_mv, wg_amp_word, num_active_bands);

	/* ---- Phase 2: AFE hardware configuration (done once) ---- */

	/* 2a: INTC configuration - save INTCSEL1, enable DFTRDY in INTC1 */
	ad5940_spi_read32(priv, AD5940_REG_INTCSEL1, &intc1_saved);
	ret = ad5940_spi_read32(priv, AD5940_REG_INTCSEL1, &val);
	if (ret)
		return ret;
	ad5940_spi_write(priv, AD5940_REG_INTCSEL1,
			 val | AD5940_AFEINTSRC_DFTRDY);

	/* 2b: AD5940_AFECtrlS(AFECTRL_ALL, bFALSE) - disable all AFE modules */
	ret = rtia_cal_afe_ctrl(priv, AD5940_AFECTRL_ALL, false);
	if (ret)
		return ret;

	/* 2c: REFCfgS - Configure reference system */
	ret = ad5940_spi_read32(priv, AD5940_REG_AFECON, &val);
	if (ret)
		return ret;
	val &= ~AD5940_AFECON_HPREFDIS;
	ad5940_spi_write(priv, AD5940_REG_AFECON, val);

	ret = ad5940_spi_read32(priv, AD5940_REG_BUFSENCON, &val);
	if (ret)
		return ret;
	val |= AD5940_BUFSENCON_V1P8HPADCEN | AD5940_BUFSENCON_V1P1HPADCEN;
	ad5940_spi_write(priv, AD5940_REG_BUFSENCON, val);

	ad5940_spi_write(priv, AD5940_REG_LPREFBUFCON,
			 AD5940_LPREFBUFCON_LPREFDIS |
			 AD5940_LPREFBUFCON_LPBUF2P5DIS);

	/* 2d: HSLoopCfgS - Configure HP loop */
	val = 0;
	if (excit_buf_gain == 1)
		val |= AD5940_HSDACCON_INAMPGNMDE;
	if (hs_dac_gain == 1)
		val |= AD5940_HSDACCON_ATTENEN;
	val |= (7 & 0xFF) << AD5940_HSDACCON_RATE_SHIFT;
	ad5940_spi_write(priv, AD5940_REG_HSDACCON, val);

	ad5940_spi_write(priv, AD5940_REG_HSTIACON, AD5940_HSTIABIAS_1P1);

	val = (ctia_val << AD5940_HSRTIACON_CTIACON_SHIFT) | rtia_sel;
	ad5940_spi_write(priv, AD5940_REG_HSRTIACON, val);

	ad5940_spi_write(priv, AD5940_REG_DE0RESCON,
			 (AD5940_HSTIADERTIA_TODE << 3) | AD5940_HSTIADERLOAD_OPEN);

	/* Switch matrix: D=RCAL0, P=RCAL0, N=RCAL1, T=RCAL1|TRTIA|AIN1 */
	ad5940_spi_write(priv, AD5940_REG_DSWFULLCON, AD5940_SWD_RCAL0);
	ad5940_spi_write(priv, AD5940_REG_PSWFULLCON, AD5940_SWP_RCAL0);
	ad5940_spi_write(priv, AD5940_REG_NSWFULLCON, AD5940_SWN_RCAL1);
	ad5940_spi_write(priv, AD5940_REG_TSWFULLCON,
			 AD5940_SWT_RCAL1 | AD5940_SWT_TRTIA | AD5940_SWT_AIN1);
	ad5940_spi_write(priv, AD5940_REG_SWCON, AD5940_SWCON_SWSOURCESEL);

	/* WG: sine wave, GainCal+OffsetCal, amplitude/offset/phase */
	ad5940_spi_write(priv, AD5940_REG_WGAMPLITUDE, wg_amp_word);
	ad5940_spi_write(priv, AD5940_REG_WGOFFSET, 0);
	ad5940_spi_write(priv, AD5940_REG_WGPHASE, 0);
	/* WGCON: SIN(2) << TYPESEL | DACGAINCAL | DACOFFSETCAL */
	val = (AD5940_WGTYPE_SIN << AD5940_WGCON_TYPESEL_SHIFT) |
	      AD5940_WGCON_DACGAINCAL | AD5940_WGCON_DACOFFSETCAL;
	ad5940_spi_write(priv, AD5940_REG_WGCON, val);

	/* 2e: ADCCON - MUX to P_NODE/N_NODE for RCAL measurement, PGA=1.5x */
	ad5940_spi_write(priv, AD5940_REG_ADCCON,
			 AD5940_ADCMUXP_P_NODE |
			 (AD5940_ADCMUXN_N_NODE << AD5940_ADCCON_MUXSELN_SHIFT) |
			 (AD5940_ADCPGA_1P5 << AD5940_ADCCON_GNPGA_SHIFT));

	ad5940_spi_write(priv, AD5940_REG_STATSCON, 0);

	/* 2f: Enable power modules (not WG/ADCCNV/DFT yet) */
	ret = rtia_cal_afe_ctrl(priv,
		AD5940_AFECTRL_HSTIAPWR | AD5940_AFECTRL_INAMPPWR |
		AD5940_AFECTRL_EXTBUFPWR | AD5940_AFECTRL_DACREFPWR |
		AD5940_AFECTRL_HSDACPWR,
		true);
	if (ret)
		return ret;

	/* ---- Phase 3: Per-band calibration loop ---- */
	priv->num_active_bands = num_active_bands;

	for (i = 0; i < num_active_bands; i++) {
		struct ad5940_freq_params cal_params;
		u32 freq_hz;
		s64 denom, real_num, imag_num;
		s64 rtia_real_mohm, rtia_imag_mohm;

		band_id = active_bands[i];
		freq_hz = ad5940_band_representative_freq(band_id);
		cal_params = ad5940_get_freq_params(freq_hz);

		dev_info(dev, "RTIA cal band %d [%d/%d]: rep_freq=%uHz\n",
			 band_id, i + 1, num_active_bands, freq_hz);

		/* 3a: Configure DSP for this band
		 *
		 * CRITICAL: SINC2NOTCH and DFT must be disabled before changing
		 * ADCFILTERCON or DFTCON.  Changing filter parameters while the
		 * DSP block is actively running can leave the hardware in an
		 * inconsistent state, causing DFT to never complete (DFTRDY
		 * timeout).  This matches ADI's practice of disabling all AFE
		 * modules (AFECTRL_ALL = bFALSE) before calling DSPCfgS().
		 */
		{
			u32 pmbw_val;

			/* Disable SINC2NOTCH and DFT before reconfiguring filters */
			rtia_cal_afe_ctrl(priv,
				AD5940_AFECTRL_SINC2NOTCH | AD5940_AFECTRL_DFT,
				false);

			/* Update PMBW for power mode */
			pmbw_val = (3 << AD5940_PMBW_SYSBW_SHIFT);

			if (cal_params.high_pwr)
				pmbw_val |= AD5940_PMBW_SYSHP;
			ad5940_spi_write(priv, AD5940_REG_PMBW, pmbw_val);

			/* ADCFILTERCON: S2OSR, S3OSR, ADCRATE, LPFBYPEN
			 * Build from scratch (like ADI's ADCFilterCfgS):
			 *   - Keep AVRGEN off (don't temporarily enable it)
			 *   - Set ADCRATE, S2OSR, S3OSR, LPFBYPEN
			 */
			val = AD5940_ADCRATE_800KHZ;
			val |= (cal_params.sinc2osr <<
				AD5940_ADCFILTERCON_SINC2OSR_SHIFT);
			val |= (cal_params.sinc3osr <<
				AD5940_ADCFILTERCON_SINC3OSR_SHIFT);
			if (cal_params.dft_src == AD5940_DFTSRC_SINC2NOTCH)
				val |= AD5940_ADCFILTERCON_LPFBYPEN;
			ad5940_spi_write(priv, AD5940_REG_ADCFILTERCON, val);

			ad5940_spi_write(priv, AD5940_REG_ADCMIN, 0);
			ad5940_spi_write(priv, AD5940_REG_ADCMINSM, 0);
			ad5940_spi_write(priv, AD5940_REG_ADCMAX, 0);
			ad5940_spi_write(priv, AD5940_REG_ADCMAXSMEN, 0);

			/* DFTCON: Hanning window, DFTNUM, DFTSRC */
			val = BIT(AD5940_DFTCON_HANNINGEN_SHIFT) |
			      (cal_params.dft_num <<
			       AD5940_DFTCON_DFTNUM_SHIFT) |
			      (cal_params.dft_src <<
			       AD5940_DFTCON_DFTINSEL_SHIFT);
			ad5940_spi_write(priv, AD5940_REG_DFTCON, val);

			/* Re-enable SINC2NOTCH for bands that use it
			 * (must be after ADCFILTERCON write)
			 */
			if (cal_params.dft_src == AD5940_DFTSRC_SINC2NOTCH) {
				ret = rtia_cal_afe_ctrl(priv,
					AD5940_AFECTRL_SINC2NOTCH, true);
				if (ret)
					goto restore_intc;
			}
		}

		/* 3b: Set WGFCW for representative frequency */
		freq_word = ad5940_wg_freq_word_cal(freq_hz, sysclk_hz);
		ad5940_spi_write(priv, AD5940_REG_WGFCW, freq_word);

		/* ---- 3c: Measure V_Rcal ---- */
		/* Ensure ADC MUX is on P_NODE/N_NODE for RCAL path */
		ret = rtia_cal_adc_mux_cfg(priv,
			AD5940_ADCMUXP_P_NODE, AD5940_ADCMUXN_N_NODE);
		if (ret)
			goto restore_intc;

		ret = rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);
		if (ret)
			goto restore_intc;

		usleep_range(250, 300);

		/* Clear stale DFTRDY before starting new conversion */
		ad5940_spi_write(priv, AD5940_REG_INTCCLR,
				 AD5940_AFEINTSRC_DFTRDY);

		ret = rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);
		if (ret)
			goto restore_intc;

		ret = rtia_cal_wait_dft_ready(priv, 35000);
		if (ret) {
			dev_err(dev, "RTIA cal: V_Rcal DFTRDY timeout at %uHz (band %d)\n",
				freq_hz, band_id);
			goto restore_intc;
		}

		rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
			AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, false);

		ad5940_spi_write(priv, AD5940_REG_INTCCLR,
				 AD5940_AFEINTSRC_DFTRDY);

		{
			u32 tmp_r, tmp_i;

			ad5940_spi_read32(priv, AD5940_REG_DFTREAL, &tmp_r);
			ad5940_spi_read32(priv, AD5940_REG_DFTIMAG, &tmp_i);
			dft_rcal_real = (s32)tmp_r;
			dft_rcal_imag = (s32)tmp_i;
		}

		/* ---- 3d: Measure V_Rtia ---- */
		ret = rtia_cal_adc_mux_cfg(priv,
			AD5940_ADCMUXP_HSTIA_P, AD5940_ADCMUXN_HSTIA_N);
		if (ret)
			goto restore_intc;

		ret = rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);
		if (ret)
			goto restore_intc;

		usleep_range(250, 300);

		/* Clear stale DFTRDY before starting new conversion */
		ad5940_spi_write(priv, AD5940_REG_INTCCLR,
				 AD5940_AFEINTSRC_DFTRDY);

		ret = rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);
		if (ret)
			goto restore_intc;

		ret = rtia_cal_wait_dft_ready(priv, 35000);
		if (ret) {
			dev_err(dev, "RTIA cal: V_Rtia DFTRDY timeout at %uHz (band %d)\n",
				freq_hz, band_id);
			goto restore_intc;
		}

		rtia_cal_afe_ctrl(priv,
			AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
			AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, false);

		ad5940_spi_write(priv, AD5940_REG_INTCCLR,
				 AD5940_AFEINTSRC_DFTRDY);

		{
			u32 tmp_r, tmp_i;

			ad5940_spi_read32(priv, AD5940_REG_DFTREAL, &tmp_r);
			ad5940_spi_read32(priv, AD5940_REG_DFTIMAG, &tmp_i);
			dft_rtia_real = (s32)tmp_r;
			dft_rtia_imag = (s32)tmp_i;
		}

		/* ---- 3e: Data post-processing ---- */
		if (dft_rcal_real & BIT(17))
			dft_rcal_real |= 0xFFFC0000;
		if (dft_rcal_imag & BIT(17))
			dft_rcal_imag |= 0xFFFC0000;
		if (dft_rtia_real & BIT(17))
			dft_rtia_real |= 0xFFFC0000;
		if (dft_rtia_imag & BIT(17))
			dft_rtia_imag |= 0xFFFC0000;

		dft_rtia_imag = -dft_rtia_imag;
		dft_rtia_real = -dft_rtia_real;
		dft_rtia_imag = -dft_rtia_imag;
		dft_rcal_imag = -dft_rcal_imag;

		/* ---- 3f: Compute RTIA = (V_Rtia / V_Rcal) * Rcal ---- */
		denom = (s64)dft_rcal_real * dft_rcal_real +
			(s64)dft_rcal_imag * dft_rcal_imag;
		if (denom == 0) {
			dev_err(dev, "RTIA cal: Rcal DFT denominator is zero at %uHz (band %d)\n",
				freq_hz, band_id);
			ret = -EINVAL;
			goto restore_intc;
		}

		real_num = (s64)dft_rtia_real * dft_rcal_real +
			   (s64)dft_rtia_imag * dft_rcal_imag;
		imag_num = (s64)dft_rtia_imag * dft_rcal_real -
			   (s64)dft_rtia_real * dft_rcal_imag;

		rtia_real_mohm = div64_s64(real_num * (s64)rcal_ohm * 1000,
					   denom);
		rtia_imag_mohm = div64_s64(imag_num * (s64)rcal_ohm * 1000,
					   denom);

		/* Store in per-band calibration table */
		priv->band_cal_table[band_id].real_mohm = rtia_real_mohm;
		priv->band_cal_table[band_id].imag_mohm = rtia_imag_mohm;

		{
			u64 r2, i2, sum_sq;

			r2 = rtia_real_mohm < 0 ?
				(u64)(-rtia_real_mohm) * (-rtia_real_mohm) :
				(u64)rtia_real_mohm * rtia_real_mohm;
			i2 = rtia_imag_mohm < 0 ?
				(u64)(-rtia_imag_mohm) * (-rtia_imag_mohm) :
				(u64)rtia_imag_mohm * rtia_imag_mohm;
			sum_sq = r2 + i2;
			priv->band_cal_table[band_id].magnitude_mohm =
				int_sqrt64(sum_sq);
		}

		priv->band_cal_table[band_id].phase_mdeg =
			rtia_cal_atan2_mdeg(rtia_imag_mohm, rtia_real_mohm);

		{
			s64 mag_mohm = priv->band_cal_table[band_id].magnitude_mohm;
			s32 ph_mdeg = priv->band_cal_table[band_id].phase_mdeg;
			s64 mag_frac = mag_mohm >= 0 ? mag_mohm % 1000 :
						  -(mag_mohm % 1000);

			dev_info(dev,
				 "RTIA cal band %d @%uHz: Mag=%lld.%03lld Ohm, Phase=%d.%03d deg\n",
				 band_id, freq_hz,
				 div_s64(mag_mohm, 1000), mag_frac,
				 ph_mdeg / 1000,
				 ph_mdeg >= 0 ? ph_mdeg % 1000 :
						-(ph_mdeg % 1000));
		}
	}

	/* ---- Phase 4: Set initial active calibration ---- */
	priv->rtia_cal = priv->band_cal_table[first_active_band];
	if (priv->sweep_en)
		priv->sweep_index = 0;

restore_intc:
	/* Restore INTC1 DFTRDY configuration */
	if (!(intc1_saved & AD5940_AFEINTSRC_DFTRDY)) {
		/* DFTRDY was not previously enabled, disable it */
		u32 intc1_cur;

		if (ad5940_spi_read32(priv, AD5940_REG_INTCSEL1,
					&intc1_cur) == 0)
			ad5940_spi_write(priv, AD5940_REG_INTCSEL1,
					 intc1_cur & ~AD5940_AFEINTSRC_DFTRDY);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ad5940_bia_rtia_cal);

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
 *     11. RTIA calibration (AppBIARtiaCal → AD5940_HSRtiaCal)
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
	int init_seq_len = 0;
	u32 meas_seq_addr = 0;
	int meas_seq_len = 0;
	int ret, i, rd;
	u32 val, reg_osccon, fifocon_saved, tempreg;
	struct seq_shadow_regs init_shadow;

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
	ad5940_spi_write(priv, AD5940_REG_SEQCON, 0);
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, fifocon_saved);

	/* ---- AppBIAInit Step 3: RTIA calibration ---- */
	/*
	 * ADI does AppBIARtiaCal() → AD5940_HSRtiaCal() here.
	 * This performs a single-frequency (50kHz) calibration of the
	 * HSTIA RTIA resistor by measuring voltages across RCAL and RTIA,
	 * then computing Rtia = (V_Rtia / V_Rcal) * Rcal.
	 * The result is stored in priv->rtia_cal for use in impedance
	 * calculation.
	 */
	ret = ad5940_bia_rtia_cal(priv);
	if (ret) {
		dev_err(dev, "BIA: RTIA calibration failed: %d\n", ret);
		return ret;
	}

	/* ---- AppBIAInit Step 4: AD5940_FIFOCtrlS(DFT, FALSE) ---- */
	/*
	 * ADI FIFOCtrlS(FIFOSRC_DFT, bFALSE):
	 *   tempreg = 0;  (no DATAFIFOEN since FifoEn=FALSE)
	 *   tempreg |= FIFOSRC_DFT << BITP_AFE_FIFOCON_DATAFIFOSRCSEL;
	 *   WriteReg(REG_AFE_FIFOCON, tempreg);
	 * → FIFOCON = 2 << 13 = 0x4000
	 */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT);

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

	/* ---- AppBIAInit Step 7+8: Generate and write init sequence (SEQID_1) ---- */
	/*
	 * ADI's AppBIASeqCfgGen() uses the sequence generator to produce
	 * SEQ_WR commands for all init register writes (REFCfgS, HSLoopCfgS,
	 * LPLoopCfgS, DSPCfgS, AFECtrlS, SEQGpioCtrlS). The generated
	 * commands are written to SRAM and executed by SEQID_1.
	 *
	 * We replicate this by calling ad5940_bia_gen_init_seq() which
	 * produces the exact same SEQ_WR commands as ADI's generator.
	 */
	{
		u32 init_buf[128];  /* 128 words = plenty for init sequence */
		int init_len;
		struct ad5940_freq_params init_freq_params;
		u32 init_freq_hz = priv->sweep_en ?
				   priv->sweep_start_hz : 50000;

		init_freq_params = ad5940_get_freq_params(init_freq_hz);
		priv->curr_freq_params = init_freq_params;

		init_len = ad5940_bia_gen_init_seq(priv, init_buf,
						   ARRAY_SIZE(init_buf),
						   &init_shadow,
						   init_freq_hz,
						   &init_freq_params);
		if (init_len < 0) {
			dev_err(dev, "BIA: init sequence generation failed: %d\n",
				init_len);
			return init_len;
		}
		if (init_len == 0) {
			dev_err(dev, "BIA: init sequence is empty!\n");
			return -EINVAL;
		}

		dev_info(dev, "BIA: generated init sequence: %d commands\n",
			 init_len);

		/* Debug: print generated init sequence */
		{
			int k;
			for (k = 0; k < init_len; k++)
				dev_info(dev, "  init[%d] = 0x%08x\n", k,
					 init_buf[k]);
		}

		ret = ad5940_seq_cmd_write(priv, 0, init_buf, init_len);
		if (ret) {
			dev_err(dev, "BIA: init seq SRAM write failed: %d\n",
				ret);
			return ret;
		}

		/* Measure sequence starts right after init sequence */
		meas_seq_addr = init_len;
		init_seq_len = init_len;
	}

	/* ---- AppBIAInit Step 9: Program measure sequence (SEQID_0) into SRAM ---- */
	/*
	 * Use ADI-style sequence generator to produce the exact same
	 * sequence as AppBIASeqMeasureGen() in BodyImpedance.c.
	 * Shadow register state is passed from init sequence generation,
	 * matching ADI's SEQGen database that persists between calls.
	 */
	{
		u32 meas_buf[128];  /* 128 words = plenty for BIA sequence */
		int meas_len;

		meas_len = ad5940_bia_gen_measure_seq(priv, meas_buf,
						      ARRAY_SIZE(meas_buf),
						      &init_shadow,
						      &priv->curr_freq_params);
		if (meas_len < 0) {
			dev_err(dev, "BIA: measure sequence generation failed: %d\n",
				meas_len);
			return meas_len;
		}
		if (meas_len == 0) {
			dev_err(dev, "BIA: measure sequence is empty!\n");
			return -EINVAL;
		}

		/* Save shadow and SRAM info for band-change sequence regeneration */
		priv->meas_shadow = init_shadow;
		priv->meas_seq_addr = meas_seq_addr;
		priv->meas_seq_len = meas_len;

		dev_info(dev, "BIA: generated measure sequence: %d commands\n",
			 meas_len);

		/* Debug: print generated sequence (enable via dynamic_debug) */
		{
			int k;
			for (k = 0; k < meas_len; k++)
				dev_dbg(dev, "  seq[%d] = 0x%08x\n", k,
					 meas_buf[k]);
		}

		ret = ad5940_seq_cmd_write(priv, meas_seq_addr,
					   meas_buf, meas_len);
		if (ret) {
			dev_err(dev, "BIA: measure seq SRAM write failed: %d\n",
				ret);
			return ret;
		}

		/* Clear SRAM after measure sequence */
		{
			int clear_start = meas_seq_addr + meas_len;
			int clear_end = 512; // 2KB SRAM 容量 / 每 command 4字节 = 512 个 word
			int j;

			for (j = clear_start; j < clear_end; j++) {
				ad5940_spi_write(priv, AD5940_REG_CMDFIFOWADDR,
						 j);
				ad5940_spi_write(priv, AD5940_REG_CMDFIFOWRITE,
						 0);
			}
			dev_info(dev, "BIA: cleared SRAM [%d..%d] with NOPs\n",
				 clear_start, clear_end - 1);
		}

		/* Store meas_len for SEQ0INFO below */
		meas_seq_len = meas_len;
	}

	/* ---- AppBIAInit Step 10: SEQInfoCfg(InitSeqInfo, WriteSRAM=FALSE) ---- */
	/*
	 * SEQ1INFO format: [31:16] = SeqLen (command count),
	 *                   [9:0]   = SeqRamAddr (SRAM start address)
	 * Init sequence was written starting at SRAM address 0.
	 */
	ret = ad5940_spi_write(priv, AD5940_REG_SEQ1INFO,
			       (init_seq_len << 16) | 0);
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

	/* SeqCntCRCClr: disable sequencer + clear counter */
	ad5940_spi_write(priv, AD5940_REG_SEQCON, 0);
	ad5940_spi_write(priv, AD5940_REG_SEQCNT, 0);

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
			       (meas_seq_len << 16) |
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

	/* SeqCntCRCClr: disable sequencer + clear counter */
	ad5940_spi_write(priv, AD5940_REG_SEQCON, 0);
	ad5940_spi_write(priv, AD5940_REG_SEQCNT, 0);

	ad5940_spi_write(priv, AD5940_REG_SEQCON, BIT(0));

	ad5940_spi_write(priv, AD5940_REG_FIFOCON, fifocon_saved);

	/* ---- AppBIAInit Step 16: ClrMCUIntFlag ---- */
	/* ADI's ClrMCUIntFlag() clears the MCU-side GPIO interrupt flag,
	 * not AD5940's AFE interrupt flags. In Linux, the kernel IRQ
	 * subsystem handles this automatically. No AD5940 register write needed.
	 */

	/* ---- AppBIAInit Step 17: AFEPwrBW ---- */
	{
		u32 pmbw_val = (3 << AD5940_PMBW_SYSBW_SHIFT);
		if (priv->curr_freq_params.high_pwr)
			pmbw_val |= AD5940_PMBW_SYSHP;
		ret = ad5940_spi_write(priv, AD5940_REG_PMBW, pmbw_val);
	}
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
 * with additional FIFO/INTC cleanup to ensure clean state after a
 * previous stop/restart cycle.
 *
 * ADI's exact flow is:
 *   1. WakeUp AFE
 *   2. Configure WUPT and enable it
 *
 * We add:
 *   - FIFO reset (disable/re-enable) to flush stale data
 *   - INTC flag clearing to ensure GP0 is high before WUPT starts
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_start(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	int ret;
	u32 sleep_time;

	/* ---- Step 1: Wake up AFE (AD5940_WakeUp(10)) ---- */
	ret = ad5940_wakeup(priv);
	if (ret) {
		dev_err(dev, "BIA start: wakeup failed: %d\n", ret);
		return ret;
	}

	/* ---- Step 1b: Clean up FIFO and INTC state ---- */
	/*
	 * After a previous stop, the FIFO may contain stale data and
	 * the INTC DATAFIFOTHRESH flag may be set (keeping GP0 low).
	 * Without cleanup, no falling edge is generated when new data
	 * arrives, so the Linux IRQ (edge-triggered) never fires.
	 *
	 * Lock sleep key to prevent hibernate during register access.
	 */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_LOCK);

	/* Reset FIFO: disable then re-enable (clears FIFO pointers/count) */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 AD5940_FIFOCON_DATAFIFOEN |
			 (2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT));

	/* Clear all AFE interrupt flags (ensures GP0 goes high) */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* Unlock sleep key (allow AFE to hibernate after each measurement) */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_UNLOCK);

	priv->running = true;

	/* Initialize sweep measurement state */
	if (priv->sweep_en) {
		priv->sweep_index = 0;
		priv->sweep_curr_freq_hz = priv->sweep_start_hz;
		priv->sweep_next_freq_hz = ad5940_sweep_calc_freq(
			priv->sweep_start_hz, priv->sweep_stop_hz,
			priv->sweep_points, priv->sweep_type, 1);
		priv->freq_of_data_hz = priv->sweep_start_hz;
	} else {
		priv->freq_of_data_hz = 50000;
	}

	/* ---- Step 2: Configure WUPT (AD5940_WUPTCfg) ---- */
	/*
	 * From ADI's AppBIACtrl(BIACTRL_START):
	 *   wupt_cfg.WuptEn = bTRUE;
	 *   wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
	 *   wupt_cfg.WuptOrder[0] = SEQID_0;
	 *   wupt_cfg.SeqxSleepTime[SEQID_0] =
	 *       (uint32_t)(WuptClkFreq/BiaODR)-2-1;
	 *   wupt_cfg.SeqxWakeupTime[SEQID_0] = 1;
	 *   AD5940_WUPTCfg(&wupt_cfg);
	 *   FifoDataCount = 0;  // restart
	 *
	 * AD5940_WUPTCfg() does:
	 *   a) Write SEQ0-3 WAKEUP/SLEEP time
	 *   b) Write TMRCON: enable WUPT to wake up AFE
	 *   c) Write SEQORDER: position A = SEQID_0
	 *   d) Write WUPTCON: EN=1, ENDSEQ=A
	 */

	sleep_time = (u32)(AD5940_BIA_WUPT_CLK_FREQ / AD5940_BIA_ODR) - 2 - 1;

	/* 2a: SEQ0 wakeup time = 1 (minimum, per ADI) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0WUPL, 1);
	if (ret)
		return ret;
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0WUPH, 0);
	if (ret)
		return ret;

	/* 2b: SEQ0 sleep time */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0SLEEPL,
			       sleep_time & 0xFFFF);
	if (ret)
		return ret;
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQ0SLEEPH,
			       (sleep_time >> 16) & 0xF);
	if (ret)
		return ret;

	/* 2c: SEQ1-3 wakeup/sleep time = 0 (unused) - ADI writes all of them */
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

	/* 2d: TMRCON - allow WUPT to wake up AFE (per ADI) */
	ret = ad5940_spi_write(priv, AD5940_REG_ALLON_TMRCON, BIT(0));
	if (ret)
		return ret;

	/* 2e: SEQORDER - position A = SEQID_0 (=0) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTSEQORDER, 0x00);
	if (ret)
		return ret;

	/* 2f: WUPTCON - enable WUPT, ENDSEQ = position A (0) */
	ret = ad5940_spi_write(priv, AD5940_REG_WUPTCON,
			       AD5940_WUPTCON_EN |
			       (0 << AD5940_WUPTCON_ENDSEQ_SHIFT));
	if (ret)
		return ret;

	// /* ---- Debug: read status registers after WUPT start ---- */
	// {
	// 	u32 seqcon, fifocon, fifocnt_raw, fifocnt, intcflag0, intcflag1;
	// 	u32 afecon, wuptcon, cmddatacon, dftcon, adcfiltercon;
	// 	u32 seq0info, seq1info, intcsel0, intcsel1, fifothres;
	// 	u32 seqcnt;

	// 	/* Quick snapshot at 10ms to see if sequence starts */
	// 	msleep(10);
	// 	ad5940_wakeup(priv);
	// 	afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
	// 	seqcon = ad5940_spi_read(priv, AD5940_REG_SEQCON);
	// 	seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
	// 	fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	// 	fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
	// 		  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
	// 	intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
	// 	dev_info(dev, "BIA debug @10ms: AFECON=0x%x SEQCON=0x%x SEQCNT=%u FIFOCNT=%u INTF1=0x%x\n",
	// 		 afecon, seqcon, seqcnt & 0xFFFF, fifocnt, intcflag1);

	// 	/* Snapshot at 100ms (first DFT should be complete) */
	// 	msleep(90);
	// 	ad5940_wakeup(priv);
	// 	afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
	// 	seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
	// 	fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	// 	fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
	// 		  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
	// 	intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
	// 	dev_info(dev, "BIA debug @100ms: AFECON=0x%x SEQCNT=%u FIFOCNT=%u INTF1=0x%x\n",
	// 		 afecon, seqcnt & 0xFFFF, fifocnt, intcflag1);

	// 	/* Full register dump at 300ms (at least one full measure cycle done) */
	// 	msleep(200);
	// 	ad5940_wakeup(priv);

	// 	seqcon = ad5940_spi_read(priv, AD5940_REG_SEQCON);
	// 	seqcnt = ad5940_spi_read(priv, AD5940_REG_SEQCNT);
	// 	afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
	// 	fifocon = ad5940_spi_read(priv, AD5940_REG_FIFOCON);
	// 	fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	// 	fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
	// 		  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
	// 	intcflag0 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG0);
	// 	intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
	// 	wuptcon = ad5940_spi_read(priv, AD5940_REG_WUPTCON);
	// 	cmddatacon = ad5940_spi_read(priv, AD5940_REG_CMDDATACON);
	// 	dftcon = ad5940_spi_read(priv, AD5940_REG_DFTCON);
	// 	adcfiltercon = ad5940_spi_read(priv, AD5940_REG_ADCFILTERCON);
	// 	seq0info = ad5940_spi_read(priv, AD5940_REG_SEQ0INFO);
	// 	seq1info = ad5940_spi_read(priv, AD5940_REG_SEQ1INFO);
	// 	intcsel0 = ad5940_spi_read(priv, AD5940_REG_INTCSEL0);
	// 	intcsel1 = ad5940_spi_read(priv, AD5940_REG_INTCSEL1);
	// 	fifothres = ad5940_spi_read(priv, AD5940_REG_DATAFIFOTHRES);

	// 	dev_info(dev, "BIA debug @300ms:\n");
	// 	dev_info(dev, "  SEQCON=0x%x SEQCNT=%u AFECON=0x%x FIFOCON=0x%x\n",
	// 		 seqcon, seqcnt & 0xFFFF, afecon, fifocon);
	// 	dev_info(dev, "  FIFOCNT=0x%x(raw) FIFOCNT=%u INTCFLAG0=0x%x INTCFLAG1=0x%x\n",
	// 		 fifocnt_raw, fifocnt, intcflag0, intcflag1);
	// 	dev_info(dev, "  WUPTCON=0x%x CMDDATACON=0x%x\n",
	// 		 wuptcon, cmddatacon);
	// 	dev_info(dev, "  DFTCON=0x%x ADCFILTERCON=0x%x\n",
	// 		 dftcon, adcfiltercon);
	// 	dev_info(dev, "  SEQ0INFO=0x%x SEQ1INFO=0x%x\n",
	// 		 seq0info, seq1info);
	// 	dev_info(dev, "  INTCSEL0=0x%x INTCSEL1=0x%x FIFOTHRES=0x%x\n",
	// 		 intcsel0, intcsel1, fifothres);

	// 	/* Also do a second read after another 500ms (multiple WUPT cycles) */
	// 	msleep(500);
	// 	ad5940_wakeup(priv);
	// 	fifocnt_raw = ad5940_spi_read(priv, AD5940_REG_FIFOCNT);
	// 	fifocnt = (fifocnt_raw & AD5940_FIFOCNT_DATAFIFOCNT_MASK)
	// 		  >> AD5940_FIFOCNT_DATAFIFOCNT_SHIFT;
	// 	intcflag0 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG0);
	// 	intcflag1 = ad5940_spi_read(priv, AD5940_REG_INTCFLAG1);
	// 	afecon = ad5940_spi_read(priv, AD5940_REG_AFECON);
	// 	dev_info(dev, "BIA debug @800ms: FIFOCNT=%u INTF0=0x%x INTF1=0x%x AFECON=0x%x\n",
	// 		 fifocnt, intcflag0, intcflag1, afecon);
	// }

	// /* ---- Debug: read back WUPT registers to verify ---- */
	// {
	// 	int wupt_wupl, wupt_wuph, wupt_sleepl, wupt_sleeph;
	// 	int wupt_seqorder, wuptcon, tmrcon;

	// 	wupt_wupl = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0WUPL);
	// 	wupt_wuph = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0WUPH);
	// 	wupt_sleepl = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0SLEEPL);
	// 	wupt_sleeph = ad5940_spi_read(priv, AD5940_REG_WUPTSEQ0SLEEPH);
	// 	wupt_seqorder = ad5940_spi_read(priv, AD5940_REG_WUPTSEQORDER);
	// 	wuptcon = ad5940_spi_read(priv, AD5940_REG_WUPTCON);
	// 	tmrcon = ad5940_spi_read(priv, AD5940_REG_ALLON_TMRCON);

	// 	dev_info(dev, "WUPT readback: WUPL=%d WUPH=%d SLEEPL=%d SLEEPH=%d "
	// 		 "SEQORDER=0x%x WUPTCON=0x%x TMRCON=0x%x sleep_time=%d\n",
	// 		 wupt_wupl, wupt_wuph, wupt_sleepl, wupt_sleeph,
	// 		 wupt_seqorder, wuptcon, tmrcon, sleep_time);
	// }

	dev_info(dev, "BIA: measurement started (ODR=%dHz, ADI flow replicated)\n",
		 AD5940_BIA_ODR);
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_start);

/**
 * ad5940_bia_stop - Stop BIA measurement (STOPNOW with cleanup)
 * @priv: driver private data
 *
 * Stops the WUPT timer immediately and cleans up FIFO/INTC state.
 *
 * IMPORTANT: We cannot use ADI's STOPSYNC pattern in the Linux kernel
 * because it depends on the trigger handler (ISR bottom half) to
 * disable WUPT after reading FIFO data.  When the IIO buffer is
 * disabled, the trigger handler is not scheduled, so the stop never
 * completes.  This leaves WUPT running, which causes FIFO interrupts
 * that disable the Linux IRQ via disable_irq_nosync() without a
 * matching enable_irq(), permanently blocking all future interrupts.
 *
 * Instead, we use STOPNOW (wake AFE + disable WUPT immediately),
 * protected by the sleep key to prevent the race condition where
 * the sequencer puts AFE back to hibernate between our wake-up
 * and WUPT disable.
 *
 * After stopping WUPT, we also reset the FIFO and clear INTC flags
 * so that GP0 returns high, ensuring the next start will produce a
 * clean falling edge when the FIFO threshold is reached.
 *
 * Return: 0 on success, negative errno on failure
 */
int ad5940_bia_stop(struct ad5940_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	int ret;

	if (!priv->running)
		return 0;  /* Already stopped */

	/* ---- Step 1: Wake up AFE ---- */
	ret = ad5940_wakeup(priv);
	if (ret) {
		dev_err(dev, "BIA stop: wakeup failed: %d\n", ret);
		return ret;
	}

	/* ---- Step 2: Lock sleep key to prevent hibernate ---- */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_LOCK);

	/* ---- Step 3: Disable WUPT (write twice per ADI safety) ---- */
	ad5940_spi_write(priv, AD5940_REG_WUPTCON, 0);
	ad5940_spi_write(priv, AD5940_REG_WUPTCON, 0);

	/* ---- Step 4: Reset FIFO (flush stale data) ---- */
	ad5940_spi_write(priv, AD5940_REG_FIFOCON, 0);
	ad5940_spi_write(priv, AD5940_REG_FIFOCON,
			 AD5940_FIFOCON_DATAFIFOEN |
			 (2 << AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT));

	/* ---- Step 5: Clear all AFE interrupt flags ---- */
	ad5940_spi_write(priv, AD5940_REG_INTCCLR, AD5940_AFEINTSRC_ALLINT);

	/* ---- Step 6: Unlock sleep key ---- */
	ad5940_spi_write(priv, AD5940_REG_SEQSLPLOCK, AD5940_SLPKEY_UNLOCK);

	priv->running = false;

	/*
	 * Step 7: Re-enable the Linux IRQ if it was disabled by our
	 * hard IRQ handler but never re-enabled by the trigger handler.
	 * This can happen if a GPIO interrupt fired after the IIO buffer
	 * was disabled (but before WUPT was stopped), causing
	 * disable_irq_nosync() without a matching enable_irq().
	 */
	if (priv->irq_disabled) {
		enable_irq(priv->irq);
		priv->irq_disabled = false;
	}

	dev_info(dev, "BIA: measurement stopped\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ad5940_bia_stop);

MODULE_AUTHOR("Mason Wang");
MODULE_DESCRIPTION("AD5940 AFE core register access and BIA measurement");
MODULE_LICENSE("GPL");
