/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AD5940 AFE core register access and initialization
 *
 * Low-level SPI register R/W, hardware reset/wakeup, and post-reset
 * initialization sequence.  Used by the platform driver and IIO layers.
 *
 * All bit positions and register values are derived from the ADI
 * ad5940lib/ad5940.h BITP_/BITM_/ENUM_ definitions.  Do NOT invent
 * offsets - always cross-reference against the ADI header.
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

/* ================================================================== */
/*  Register addresses - from ad5940lib/ad5940.h                      */
/* ================================================================== */

/* AFECON - main AFE control register */
#define AD5940_REG_AFECON		0x2000

/* Sequence / FIFO / Switch / DAC / WG / DSP registers */
#define AD5940_REG_SEQCON		0x2004
#define AD5940_REG_FIFOCON		0x2008
#define AD5940_REG_SWCON		0x200C
#define AD5940_REG_HSDACCON		0x2010
#define AD5940_REG_WGCON		0x2014
#define AD5940_REG_WGFCW		0x2030
#define AD5940_REG_WGPHASE		0x2034
#define AD5940_REG_WGOFFSET		0x2038
#define AD5940_REG_WGAMPLITUDE		0x203C
#define AD5940_REG_LPREFBUFCON		0x2050
#define AD5940_REG_SYNCEXTDEVICE	0x2054	/* SEQ GPIO control */
#define AD5940_REG_FIFODATA		0x206C	/* REG_AFE_DATAFIFORD */
#define AD5940_REG_LPTIASW0		0x20E4	/* LPTIA Switch Config Ch0 */
#define AD5940_REG_HSRTIACON		0x20F0
#define AD5940_REG_DE0RESCON		0x20F8
#define AD5940_REG_HSTIACON		0x20FC
#define AD5940_REG_DFTCON		0x20D0
#define AD5940_REG_LPTIACON0		0x20EC
#define AD5940_REG_BUFSENCON		0x2180
#define AD5940_REG_DFTREAL		0x2078  /* DFT Result, Real Part */
#define AD5940_REG_DFTIMAG		0x207C  /* DFT Result, Imaginary Part */
#define AD5940_REG_LPREFBUFCON		0x2050  /* LP Reference Buffer Config */
#define AD5940_REG_DE0RESCON		0x20F8  /* DE0 HSTIA Resistors Config */
#define AD5940_REG_ADCCON		0x21A8
#define AD5940_REG_ADCFILTERCON		0x2044
#define AD5940_REG_ADCMIN		0x20A8
#define AD5940_REG_ADCMINSM		0x20AC
#define AD5940_REG_ADCMAX		0x20B0
#define AD5940_REG_ADCMAXSMEN		0x20B4
#define AD5940_REG_DSWFULLCON		0x2150
#define AD5940_REG_NSWFULLCON		0x2154
#define AD5940_REG_PSWFULLCON		0x2158
#define AD5940_REG_TSWFULLCON		0x215C
#define AD5940_REG_LPDACDAT0		0x2120
#define AD5940_REG_LPDACSW0		0x2124
#define AD5940_REG_LPDACCON0		0x2128
#define AD5940_REG_PMBW			0x22F0
#define AD5940_REG_DATAFIFOTHRES	0x21E0
#define AD5940_REG_SEQSLPLOCK		0x2118
#define AD5940_REG_SEQTRGSLP		0x211C
#define AD5940_REG_SEQ0INFO		0x21CC
#define AD5940_REG_SEQ1INFO		0x21E8
#define AD5940_REG_STATSCON		0x21C4
#define AD5940_REG_CMDFIFOWADDR		0x21D4
#define AD5940_REG_CMDFIFOWRITE		0x2070
#define AD5940_REG_CMDDATACON		0x21D8
#define AD5940_REG_TRIGSEQ		0x0430	/* REG_AFECON_TRIGSEQ */
#define AD5940_REG_SWMUX		0x235C

/* INTC registers */
#define AD5940_REG_ADIID		0x0400
#define AD5940_REG_CHIPID		0x0404
#define AD5940_REG_INTCPOL		0x3000
#define AD5940_REG_INTCSEL0		0x3008
#define AD5940_REG_INTCSEL1		0x300C
#define AD5940_REG_INTCCLR		0x3004
#define AD5940_REG_INTCFLAG0		0x3010
#define AD5940_REG_INTCFLAG1		0x3014

/* WUPT (Wakeup Timer) registers */
#define AD5940_REG_WUPTCON		0x0800
#define AD5940_REG_WUPTSEQORDER		0x0804
#define AD5940_REG_WUPTSEQ0WUPL		0x0808
#define AD5940_REG_WUPTSEQ0WUPH		0x080C
#define AD5940_REG_WUPTSEQ0SLEEPL	0x0810
#define AD5940_REG_WUPTSEQ0SLEEPH	0x0814

/* AllOn registers */
#define AD5940_REG_ALLON_TMRCON		0x0A1C
#define AD5940_REG_ALLON_OSCCON		0x0A10
#define AD5940_REG_ALLON_OSCKEY		0x0A0C

/* Clock configuration registers - Source: REG_AFECON_CLK* in ad5940.h */
#define AD5940_REG_CLKCON0		0x0408
#define AD5940_REG_CLKSEL		0x0414

/* CLKCON0 bit fields - Source: BITP_AFECON_CLKCON0_* in ad5940.h */
#define AD5940_CLKCON0_SYSCLKDIV_SHIFT	0
#define AD5940_CLKCON0_ADCCLKDIV_SHIFT	6

/* CLKSEL bit fields - Source: BITP_AFECON_CLKSEL_* in ad5940.h */
#define AD5940_CLKSEL_SYSCLKSEL_SHIFT	0
#define AD5940_CLKSEL_ADCCLKSEL_SHIFT	2

/* Clock source selections - Source: SYSCLKSRC_*, ADCCLKSRC_* in ad5940.h */
#define AD5940_SYSCLKSRC_HFOSC		0
#define AD5940_ADCCLKSRC_HFOSC		0

/* OSCCON bits - Source: BITM_ALLON_OSCCON_* in ad5940.h */
#define AD5940_OSCCON_HFXTALEN		BIT(2)	/* 0x04 */
#define AD5940_OSCCON_HFOSCEN		BIT(1)	/* 0x02 */
#define AD5940_OSCCON_LFOSCEN		BIT(0)	/* 0x01 */
#define AD5940_OSCCON_HFXTALOK		BIT(10)	/* 0x400 */
#define AD5940_OSCCON_HFOSCOK		BIT(9)	/* 0x200 */
#define AD5940_OSCCON_LFOSCOK		BIT(8)	/* 0x100 */
#define AD5940_KEY_OSCCON		0xCB14

/* HFOSC32MHzCtrl related registers */
#define AD5940_REG_CLKEN1		0x0410
#define AD5940_REG_HPOSCCON		0x20BC
#define AD5940_CLKEN1_ACLKDIS		BIT(5)	/* 0x20 */
#define AD5940_HPOSCCON_CLK32MHZEN	BIT(2)	/* 0=32MHz, 1=16MHz */

/* FIFOCNT */
#define AD5940_REG_FIFOCNT		0x2200

/* SEQCNT - writing while SEQ disabled clears counter & CRC */
#define AD5940_REG_SEQCNT		0x2064

/* FIFOCNT bit fields */
#define AD5940_FIFOCNT_DATAFIFOCNT_SHIFT	16
#define AD5940_FIFOCNT_DATAFIFOCNT_MASK		0x07FF0000

/* ================================================================== */
/*  Bit field constants - verified against ad5940lib/ad5940.h          */
/*  BITP_ = bit position, BITM_ = bitmask.  We use the same values    */
/*  as the ADI header to avoid translation errors.                     */
/* ================================================================== */

/* AFECON bit fields (AFECTRL_* in ADI lib, but these map 1:1 to AFECON
 * bits EXCEPT HPREFDIS which is inverted: set=disable, clear=enable)
 * Source: BITP_AFE_AFECON_* in ad5940.h */
#define AD5940_AFECON_HPREFDIS		BIT(5)	/* inverted! 0=enable */
#define AD5940_AFECON_HSDACPWR		BIT(6)	/* DACEN */
#define AD5940_AFECON_ADCPWR		BIT(7)	/* ADCEN */
#define AD5940_AFECON_ADCCNV		BIT(8)	/* ADCCONVEN */
#define AD5940_AFECON_EXTBUFPWR		BIT(9)	/* EXBUFEN */
#define AD5940_AFECON_INAMPPWR		BIT(10)	/* INAMPEN */
#define AD5940_AFECON_HSTIAPWR		BIT(11)	/* TIAEN */
#define AD5940_AFECON_ALDOILIMITEN	BIT(19)	/* inverted! 0=enable LDO limit */
#define AD5940_AFECON_WG		BIT(14)	/* WAVEGENEN */
#define AD5940_AFECON_DFT		BIT(15)	/* DFTEN */
#define AD5940_AFECON_SINC2NOTCH	BIT(16)	/* SINC2EN */
#define AD5940_AFECON_DACREFPWR		BIT(20)	/* DACREFEN */

/* BUFSENCON bits - Source: BITP_AFE_BUFSENCON_* in ad5940.h */
#define AD5940_BUFSENCON_V1P8HPADCEN		BIT(0)	/* HP 1.8V ref buffer */
#define AD5940_BUFSENCON_V1P8HPADCILIMITEN	BIT(1)	/* HP ADC input limit */
#define AD5940_BUFSENCON_V1P8HPADCCHGDIS	BIT(3)	/* cap discharge */
#define AD5940_BUFSENCON_V1P8LPADCEN		BIT(2)	/* LP 1.8V ref buffer */
#define AD5940_BUFSENCON_V1P1HPADCEN		BIT(4)	/* HP 1.1V CM buffer */
#define AD5940_BUFSENCON_V1P1LPADCCHGDIS	BIT(6)	/* cap discharge */
#define AD5940_BUFSENCON_V1P1LPADCEN		BIT(5)	/* LP 1.1V buffer */
#define AD5940_BUFSENCON_V1P8THERMSTEN		BIT(8)	/* therm output */

/* LPREFBUFCON bits - Source: BITP_AFE_LPREFBUFCON_* in ad5940.h
 * All are disable-inverted: set=disable */
#define AD5940_LPREFBUFCON_LPREFDIS		BIT(0)	/* 1=disable LP bandgap */
#define AD5940_LPREFBUFCON_LPBUF2P5DIS		BIT(1)	/* 1=disable LP 2.5V buf */
#define AD5940_LPREFBUFCON_BOOSTCURRENT	BIT(2)	/* 1=drive 2 DACs */

/* HSDACCON bits - Source: BITP_AFE_HSDACCON_* */
#define AD5940_HSDACCON_INAMPGNMDE		BIT(12)	/* ExcitBufGain=0P25 */
#define AD5940_HSDACCON_ATTENEN		BIT(0)	/* HsDacGain=0P2 */
#define AD5940_HSDACCON_RATE_SHIFT		1	/* HsDacUpdateRate [7:1] */

/* HSRTIACON bits - Source: BITP_AFE_HSRTIACON_* */
#define AD5940_HSRTIACON_CTIACON_SHIFT		5	/* bits[12:5] */
#define AD5940_HSRTIACON_TIASW6CON		BIT(4)	/* DiodeClose */

/* WGCON bits - Source: BITP_AFE_WGCON_*
 * WGTYPE: 0=MMR, 2=SIN, 3=TRAPZ */
#define AD5940_WGCON_TYPESEL_SHIFT		1	/* bits[2:1] */
#define AD5940_WGCON_DACGAINCAL			BIT(5)
#define AD5940_WGCON_DACOFFSETCAL		BIT(4)
#define AD5940_WGTYPE_SIN			2

/* SWCON bits */
#define AD5940_SWCON_SWSOURCESEL			BIT(16)

/* ADCCON bits - Source: BITP_AFE_ADCCON_*
 * MUXSELP is bits[5:0] (6-bit), MUXSELN is bits[12:8] (5-bit) */
#define AD5940_ADCCON_MUXSELP_SHIFT		0
#define AD5940_ADCCON_MUXSELN_SHIFT		8
#define AD5940_ADCCON_GNPGA_SHIFT		16

/* ADC MUX enum values - Source: ADCMUXP_xxx, ADCMUXN_xxx in ad5940.h */
#define AD5940_ADCMUXP_HSTIA_P		1
#define AD5940_ADCMUXP_P_NODE		0x24	/* Buffered excitation P node */
#define AD5940_ADCMUXP_AIN3		7
#define AD5940_ADCMUXN_HSTIA_N		1
#define AD5940_ADCMUXN_N_NODE		0x14	/* Buffered excitation N node */
#define AD5940_ADCMUXN_AIN2		6

/* ADC PGA gain values */
#define AD5940_ADCPGA_1P5		1

/* ADCFILTERCON bits - Source: BITP_AFE_ADCFILTERCON_* */
#define AD5940_ADCFILTERCON_ADCCLK_SHIFT	0	/* bit0: 0=1.6MHz, 1=800kHz */
#define AD5940_ADCFILTERCON_LPFBYPEN_SHIFT	4	/* bit4: bypass notch */
#define AD5940_ADCFILTERCON_SINC3BYP_SHIFT	6	/* bit6: bypass SINC3 */
#define AD5940_ADCFILTERCON_AVRGEN_SHIFT	7	/* bit7: avg enable */
#define AD5940_ADCFILTERCON_SINC2OSR_SHIFT	8	/* bits[11:8] */
#define AD5940_ADCFILTERCON_SINC3OSR_SHIFT	12	/* bits[13:12] */
#define AD5940_ADCFILTERCON_AVRGNUM_SHIFT	14	/* bits[15:14] */

/* ADCFILTERCON bit masks - Source: BITM_AFE_ADCFILTERCON_* */
#define AD5940_ADCFILTERCON_AVRGEN		BIT(7)	/* Average function enable */
#define AD5940_ADCFILTERCON_LPFBYPEN		BIT(4)	/* Bypass 50/60Hz LP filter */

/* ADC enum values */
#define AD5940_ADCRATE_800KHZ		1
#define AD5940_ADCSINC3OSR_2		2
#define AD5940_ADCSINC2OSR_22		0
#define AD5940_ADCAVGNUM_16		3

/* DFTCON bits - Source: BITP_AFE_DFTCON_* */
#define AD5940_DFTCON_HANNINGEN_SHIFT		0	/* bit0 */
#define AD5940_DFTCON_DFTNUM_SHIFT		4	/* bits[7:4] */
#define AD5940_DFTCON_DFTINSEL_SHIFT		20	/* bits[21:20] */

/* DFT enum values */
#define AD5940_DFTNUM_8192		11
#define AD5940_DFTSRC_SINC3		1

/* FIFOCON bits - Source: BITP_AFE_FIFOCON_* */
#define AD5940_FIFOCON_DATAFIFOEN		BIT(11)
#define AD5940_FIFOCON_DATAFIFOSRCSEL_SHIFT	13	/* bits[15:13] */

/* DATAFIFOTHRES bits */
#define AD5940_DATAFIFOTHRES_HIGHTHRES_SHIFT	16

/* PMBW bits */
#define AD5940_PMBW_SYSBW_SHIFT			2

/* INTC interrupt source flags */
#define AD5940_AFEINTSRC_DFTRDY		BIT(1)
#define AD5940_AFEINTSRC_DATAFIFOTHRESH		BIT(25)
#define AD5940_AFEINTSRC_ENDSEQ			BIT(15)
#define AD5940_AFEINTSRC_ALLINT			0xFFFFFFFF

/* WUPTCON bits */
#define AD5940_WUPTCON_EN			BIT(0)
#define AD5940_WUPTCON_ENDSEQ_SHIFT		1

/* Sleep key values */
#define AD5940_SLPKEY_UNLOCK			0x0A47E5
#define AD5940_SLPKEY_LOCK			0x0

/* Switch matrix bit definitions - Source: SWD_xxx, SWP_xxx, SWN_xxx, SWT_xxx */
/* D-switch */
#define AD5940_SWD_OPEN			0x00
#define AD5940_SWD_RCAL0		BIT(0)	/* 0x01 */
#define AD5940_SWD_AIN3		BIT(3)	/* 0x08 */
#define AD5940_SWD_CE0		BIT(4)	/* 0x10 */
#define AD5940_SWD_SE0		BIT(6)	/* 0x40 */
/* P-switch */
#define AD5940_SWP_OPEN		0x00
#define AD5940_SWP_RCAL0		BIT(0)	/* 0x01 */
#define AD5940_SWP_AIN1		BIT(1)	/* 0x02 */
#define AD5940_SWP_CE0		BIT(10)	/* 0x400 */
#define AD5940_SWP_PL		BIT(13)	/* 0x2000 */
#define AD5940_SWP_PL2		BIT(14)	/* 0x4000 */
/* N-switch */
#define AD5940_SWN_OPEN		0x00
#define AD5940_SWN_AIN1		BIT(1)	/* 0x02 */
#define AD5940_SWN_RCAL1		BIT(9)	/* 0x200 */
#define AD5940_SWN_NL		BIT(10)	/* 0x400 */
#define AD5940_SWN_NL2		BIT(11)	/* 0x800 */
/* T-switch */
#define AD5940_SWT_OPEN		0x00
#define AD5940_SWT_AIN1		BIT(1)	/* 0x02 */
#define AD5940_SWT_RCAL1		BIT(11)	/* 0x800 */
#define AD5940_SWT_TRTIA		BIT(8)	/* 0x100 */

/* HSTIARTIA enum values */
#define AD5940_HSTIARTIA_1K		1

/* HSTIADERLOAD enum values */
#define AD5940_HSTIADERLOAD_OPEN	5

/* HSTIADERTIA enum values (for DeRtia field, bits[7:3]) */
#define AD5940_HSTIADERTIA_TODE	10
#define AD5940_HSTIADERTIA_OPEN	0x1F

/* HSTIABIAS enum values */
#define AD5940_HSTIABIAS_1P1		0

/* HSRTIACON additional bit fields */
#define AD5940_HSRTIACON_TIASW6CON	BIT(4)	/* DiodeClose switch */
#define AD5940_HSRTIACON_RTIACON_MASK	0x0F

/* LPREFBUFCON bits - Source: BITM_AFE_LPREFBUFCON_* in ad5940.h */
#define AD5940_LPREFBUFCON_LPBUF2P5DIS	BIT(1)

/* AFECTRL_ALL - all AFE control signal bits */
#define AD5940_AFECTRL_ALL		0x39FFE0

/* LPDACSW switch enum values */
#define AD5940_LPDACSW_VZERO2HSTIA	0x01
#define AD5940_LPDACSW_VZERO2PIN	0x02
#define AD5940_LPDACSW_VZERO2LPTIA	0x04
#define AD5940_LPDACSW_VBIAS2PIN	0x08
#define AD5940_LPDACSW_VBIAS2LPPA	0x10
#define AD5940_LPDACSW_LPMODEDIS	0x20

/* LPTIASW macro */
#define AD5940_LPTIASW(n)		(1UL << (n))

/* LPTIACON0 enum values - Source: LPTIARF_*, LPTIARLOAD_*, etc. */
#define AD5940_LPTIARF_20K		2
#define AD5940_LPTIARLOAD_SHORT	0
#define AD5940_LPTIARTIA_OPEN		0

/* CMDDATACON bits - Source: BITP_AFE_CMDDATACON_*, BITM_AFE_CMDDATACON_* in ad5940.h */
#define AD5940_BITP_CMDDATACON_CMDMEMMDE	3
#define AD5940_BITM_CMDDATACON_CMDMEMMDE	0x38  /* bits[5:3] */
#define AD5940_BITP_CMDDATACON_CMD_MEM_SEL	0
#define AD5940_BITM_CMDDATACON_CMD_MEM_SEL	0x07  /* bits[2:0] */
#define AD5940_SEQMEMSIZE_2KB			1	/* CMD_MEM_SEL, BITP=0 */
#define AD5940_BITP_CMDDATACON_DATAMEMMDE	9
#define AD5940_BITM_CMDDATACON_DATAMEMMDE	0xE00  /* bits[11:9] */
#define AD5940_BITP_CMDDATACON_DATA_MEM_SEL	6
#define AD5940_BITM_CMDDATACON_DATA_MEM_SEL	0x1C0  /* bits[8:6] */
#define AD5940_FIFOMODE_FIFO		2	/* DATAMEMMDE, BITP=9 */
#define AD5940_FIFOSIZE_4KB		2	/* DATA_MEM_SEL, BITP=6 */

/* AGPIO register addresses - Source: REG_AGPIO_GP0* in ad5940.h */
#define AD5940_REG_AGPIO_GP0CON		0x0000
#define AD5940_REG_AGPIO_GP0OEN		0x0004
#define AD5940_REG_AGPIO_GP0PE		0x0008
#define AD5940_REG_AGPIO_GP0IEN		0x000C
#define AD5940_REG_AGPIO_GP0OUT		0x0014

/* AGPIO pin function values - Source: GPx_* in ad5940.h */
#define AD5940_GP0_INT			0	/* Interrupt Controller 0 output */
#define AD5940_GP2_TRIG		(1<<4)	/* Sequence1 trigger */
#define AD5940_GP4_SYNC		(2<<8)
#define AD5940_GP5_SYNC		(2<<10)
#define AD5940_GP6_SYNC		(2<<12)
#define AD5940_GP1_SYNC		(2<<2)

/* AGPIO pin bitmask */
#define AD5940_AGPIO_Pin0		BIT(0)
#define AD5940_AGPIO_Pin1		BIT(1)
#define AD5940_AGPIO_Pin2		BIT(2)
#define AD5940_AGPIO_Pin4		BIT(4)
#define AD5940_AGPIO_Pin5		BIT(5)
#define AD5940_AGPIO_Pin6		BIT(6)

/* ================================================================== */
/*  Sequencer command encoding - from ad5940lib/ad5940.h              */
/* ================================================================== */

/*
 * Sequencer commands are 32-bit words written to SRAM via
 * REG_AFE_CMDFIFOWADDR / REG_AFE_CMDFIFOWRITE.
 *
 * Encoding:
 *   WAIT: bits[31:30]=00, bits[29:0]=clock_count
 *   WR:   bit[31]=1, bits[30:24]=(addr>>2)&0x7F, bits[23:0]=24-bit data
 *   STOP: WR to SEQCON with value 0 (halts sequencer, generates ENDSEQ)
 *   SLP:  WR to SEQTRGSLP with value 1 (triggers sleep/hibernate)
 */
#define SEQ_WAIT(clks)		(0x00000000u | ((u32)(clks) & 0x3FFFFFFFu))
#define SEQ_NOP		0x00000000u  /* Same as SEQ_WAIT(0) */
#define SEQ_WR(addr, data)	(0x80000000u | ((((u32)(addr) >> 2) & 0x7Fu) << 24) \
				 | ((u32)(data) & 0x00FFFFFFu))
#define SEQ_STOP()		SEQ_WR(AD5940_REG_SEQCON, 0x00)
#define SEQ_SLP()		SEQ_WR(AD5940_REG_SEQTRGSLP, 0x00), \
				SEQ_WR(AD5940_REG_SEQTRGSLP, 0x01)

/* ================================================================== */
/*  Expected chip ID values                                           */
/* ================================================================== */

#define AD5940_ADIID_VALUE	0x4144
#define AD5940_CHIPID_VALUE	0x5502

/* ================================================================== */
/*  Timing / FIFO / channel constants                                 */
/* ================================================================== */

#define AD5940_RESET_PULSE_MS	10
#define AD5940_RESET_WAIT_MS	10
#define AD5940_WAKEUP_RETRIES	10
#define AD5940_FIFO_THRESHOLD	4
#define AD5940_DFT_CHANNELS	4

/*
 * BIA ODR and WUPT clock frequency.
 *
 * ADI sets BiaODR=20 in AD5940BIAStructInit, but AppBIASeqMeasureGen()
 * adjusts it: if BiaODR > MaxODR, then BiaODR = MaxODR.
 *
 * With DFTNUM_8192 + ADCSINC3OSR_2 + DFTSRC_SINC3:
 *   WaitClks = 1310845 per DFT, ~82ms
 *   Total measure cycle ≈ 164ms → MaxODR ≈ 6Hz
 *
 * So ADI's actual effective ODR is ~6Hz, not 20Hz.
 * We set AD5940_BIA_ODR to match this.
 */
#define AD5940_BIA_ODR			5
#define AD5940_BIA_WUPT_CLK_FREQ	32000

/* ================================================================== */
/*  AD5940 post-reset init table (from AD5940_Initialize)             */
/* ================================================================== */

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

/* ================================================================== */
/*  BIA (Body Impedance) register configuration tables                */
/*  Phase 1: single-frequency, 4-wire, DFT, FIFO threshold interrupt */
/*  Reference: ad5940_example/AD5940_BIA/BodyImpedance.c             */
/*                                                                     */
/*  All register values computed from BITP_/BITM_ definitions in      */
/*  ad5940lib/ad5940.h.  Do NOT change without cross-referencing.     */
/*                                                                     */
/*  IMPORTANT: This table faithfully mirrors AppBIASeqCfgGen() which  */
/*  generates the init sequence commands. PMBW and SWMUX are NOT      */
/*  included here because ADI sets them AFTER the init sequence       */
/*  executes (in AppBIAInit, after SEQMmrTrig + wait ENDSEQ).        */
/* ================================================================== */

/*
 * BIA Init register writes - corresponds to AppBIASeqCfgGen().
 * Default parameters (from AppBIACfg default values):
 *   SysClkFreq = 16MHz, SinFreq = 50kHz, DacVoltPP = 800mV
 *   ExcitBufGain = EXCITBUFGAIN_2, HsDacGain = HSDACGAIN_1
 *   HsDacUpdateRate = 7, HstiaRtiaSel = HSTIARTIA_1K (=1)
 *   CtiaSel = 16, ADCPgaGain = ADCPGA_1 (=0)
 *   ADCSinc3Osr = ADCSINC3OSR_2 (=2), ADCSinc2Osr = ADCSINC2OSR_22 (=0)
 *   DftNum = DFTNUM_8192 (=11), DftSrc = DFTSRC_SINC3 (=1), HanWinEn = true
 *   PwrMod = AFEPWR_LP (=0), AFEBW = 250kHz (=3)
 *
 * WG frequency word for 50kHz @ 16MHz SysClk (S1 silicon, 26-bit):
 *   FreqWord = round(50000 * 2^26 / 16000000) = round(209715.2) = 209715 = 0x33333
 *
 * WG amplitude word for 800mVpp:
 *   AmpWord = round(800/800 * 2047) = 2047 = 0x7FF
 *
 * NOTE: The init register table (ad5940_bia_init_regs[]) has been replaced
 * by ad5940_bia_gen_init_seq() which generates SEQ_WR commands matching
 * ADI's AppBIASeqCfgGen() flow. The init sequence is now executed by
 * the AD5940 sequencer (SEQID_1), not via direct SPI writes.
 */

/*
 * BIA Measure sequence - Sequencer commands for one measurement cycle.
 * Corresponds to AppBIASeqMeasureGen() in BodyImpedance.c.
 *
 * Each cycle performs 2 DFT measurements:
 *   Step 1: Current through RTIA (ADC MUX = HSTIA_P/N, SW = CE0/AIN1)
 *   Step 2: Voltage across body (ADC MUX = AIN3/AIN2)
 * Each DFT produces 2 FIFO words (Real + Imaginary), total = 4 words.
 *
 * WaitClks calculation via AD5940_ClksCalculate():
 *   DataType = DATATYPE_DFT, DftSrc = DFTSRC_SINC3
 *   DataCount = 1L<<(DFTNUM_8192+2) = 2^15 = 32768
 *   ADCSinc3Osr = ADCSINC3OSR_2 → sinc3osr_table[2] = 2
 *   RatioSys2AdcClk = SysClkFreq/AdcClkFreq = 16MHz/16MHz = 1
 *   DATATYPE_SINC3: temp = ((32768+2)*2+1)*20*1 = 65541*20 = 1310820
 *   DATATYPE_DFT: temp += 25 → WaitClks = 1310845
 *
 * Measure sequence total cycle time (approx):
 *   2 * WaitClks + 16*250 + 2*16*50 + ~20 (write commands) ≈ 2627330
 *   AD5940_ClksCalculate with BIA params:
 *     DataType=DFT, DftSrc=SINC3, DftNum=8192, Sinc3Osr=2, Sinc2Osr=22
 *     = ((8192+2)*2+1)*20*1 + 25 = 327805
 *   At 16MHz: ~20.5ms per DFT → MaxODR ≈ 48.8 Hz
 */
#define AD5940_BIA_WAIT_CLKS	327805

/*
 * AFECTRL bit definitions - matching ADI's AFECTRL_* constants.
 * These are the AFECON register bits that AFECtrlS modifies.
 * Note: AFECTRL_HPREFPWR(bit5) and AFECTRL_ALDOLIMIT(bit19) have
 * inverted semantics in AFECON (set=disable).
 */
#define AD5940_AFECTRL_HPREFPWR		BIT(5)
#define AD5940_AFECTRL_HSDACPWR		BIT(6)
#define AD5940_AFECTRL_ADCPWR		BIT(7)
#define AD5940_AFECTRL_ADCCNV		BIT(8)
#define AD5940_AFECTRL_EXTBUFPWR		BIT(9)
#define AD5940_AFECTRL_INAMPPWR		BIT(10)
#define AD5940_AFECTRL_HSTIAPWR		BIT(11)
#define AD5940_AFECTRL_WG			BIT(14)
#define AD5940_AFECTRL_DFT			BIT(15)
#define AD5940_AFECTRL_SINC2NOTCH		BIT(16)
#define AD5940_AFECTRL_ALDOLIMIT		BIT(19)
#define AD5940_AFECTRL_DACREFPWR		BIT(20)

/* ADCCON bit masks for MUX fields */
#define AD5940_ADCCON_MUXSELP_MASK	0x0000003F
#define AD5940_ADCCON_MUXSELN_MASK	0x00001F00

/* AGPIO pin for sequence debug output */
#define AD5940_AGPIO_Pin6			BIT(6)

/*
 * Measure sequence is now generated dynamically using ADI's exact
 * high-level API flow (AppBIASeqMeasureGen). See ad5940_core.c.
 * The generated commands are written directly to SRAM.
 */

/* ================================================================== */
/*  struct ad5940_priv - driver private data                          */
/* ================================================================== */

/**
 * struct ad5940_rtia_cal_result - RTIA calibration result
 *
 * @magnitude_mohm:  RTIA magnitude in milliohms
 * @phase_mdeg:      RTIA phase in millidegrees
 * @real_mohm:       RTIA real part in milliohms
 * @imag_mohm:       RTIA imaginary part in milliohms
 */
struct ad5940_rtia_cal_result {
	s64	magnitude_mohm;
	s32	phase_mdeg;
	s64	real_mohm;
	s64	imag_mohm;
};

/**
 * struct ad5940_priv - AD5940 driver private data
 *
 * @spi:        SPI device handle
 * @reset_gpio: GPIO descriptor for hardware reset (active-low)
 * @irq:        Linux IRQ number from DT
 * @trig:       IIO trigger (data-ready / FIFO threshold)
 * @iio_dev:    Pointer back to the IIO device (for use in trigger ops)
 * @rtia_cal:   RTIA calibration result from init-time calibration
 */
struct ad5940_priv {
	struct spi_device		*spi;
	struct gpio_desc		*reset_gpio;
	int				irq;

	/* IIO trigger (FIFO threshold / data-ready) */
	struct iio_trigger		*trig;

	/* Pointer back to the IIO device (for use in trigger ops) */
	struct iio_dev			*iio_dev;

	/* RTIA calibration result */
	struct ad5940_rtia_cal_result	rtia_cal;

	/*
	 * Measurement state tracking.
	 * running: true when WUPT is active and measurements are in progress.
	 * irq_disabled: tracks whether our hard IRQ handler has called
	 *   disable_irq_nosync() without a matching enable_irq().  Needed
	 *   because the IIO trigger re-enable callback may not fire if the
	 *   buffer is disabled while a trigger handler is pending.
	 */
	bool			running;
	bool			irq_disabled;
};

/* ---- Core register access API ---- */

int ad5940_spi_xfer(struct ad5940_priv *priv,
		    const u8 *tx, u8 *rx, size_t len);
int ad5940_spi_write(struct ad5940_priv *priv, u16 reg, u32 val);
int ad5940_spi_read(struct ad5940_priv *priv, u16 reg);
int ad5940_spi_read32(struct ad5940_priv *priv, u16 reg, u32 *val);
int ad5940_fifo_read(struct ad5940_priv *priv, u32 *buf, int count);
void ad5940_reset(struct ad5940_priv *priv);
int ad5940_wakeup(struct ad5940_priv *priv);
int ad5940_init(struct ad5940_priv *priv);

/* ---- BIA measurement API ---- */
int ad5940_bia_init(struct ad5940_priv *priv);
int ad5940_bia_rtia_cal(struct ad5940_priv *priv);
int ad5940_bia_start(struct ad5940_priv *priv);
int ad5940_bia_stop(struct ad5940_priv *priv);
int ad5940_seq_cmd_write(struct ad5940_priv *priv,
			 u32 start_addr, const u32 *cmd, int count);

#endif /* _AD5940_CORE_H_ */
