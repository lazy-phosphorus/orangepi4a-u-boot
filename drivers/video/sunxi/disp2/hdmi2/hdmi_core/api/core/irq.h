/*
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef HALINTERRUPT_H_
#define HALINTERRUPT_H_

#include "../access.h"
#include "../log.h"
#include "ctrl_phy.h"
#include "../hdmitx_dev.h"

#define ALL_IRQ_MASK 0xff

typedef enum irq_sources {
	AUDIO_PACKETS = 1,
	OTHER_PACKETS,
	PACKETS_OVERFLOW,
	AUDIO_SAMPLER,
	PHY,
	I2C_DDC,
	CEC,
	VIDEO_PACKETIZER,
	I2C_PHY,
	AUDIO_DMA,
} irq_sources_t;

/*****************************************************************************
 *                                                                           *
 *                            Interrupt Registers                            *
 *                                                                           *
 *****************************************************************************/

/* Frame Composer Interrupt Status Register 0 (Packet Interrupts) */
#define IH_FC_STAT0  0x00000400
#define IH_FC_STAT0_NULL_MASK  0x00000001 /* Active after successful transmission of an Null packet */
#define IH_FC_STAT0_ACR_MASK  0x00000002 /* Active after successful transmission of an Audio Clock Regeneration (N/CTS transmission) packet */
#define IH_FC_STAT0_AUDS_MASK  0x00000004 /* Active after successful transmission of an Audio Sample packet */
#define IH_FC_STAT0_NVBI_MASK  0x00000008 /* Active after successful transmission of an NTSC VBI packet */
#define IH_FC_STAT0_MAS_MASK  0x00000010 /* Active after successful transmission of an MultiStream Audio packet */
#define IH_FC_STAT0_HBR_MASK  0x00000020 /* Active after successful transmission of an Audio HBR packet */
#define IH_FC_STAT0_ACP_MASK  0x00000040 /* Active after successful transmission of an Audio Content Protection packet */
#define IH_FC_STAT0_AUDI_MASK  0x00000080 /* Active after successful transmission of an Audio InfoFrame packet */

/* Frame Composer Interrupt Status Register 1 (Packet Interrupts) */
#define IH_FC_STAT1  0x00000404
#define IH_FC_STAT1_GCP_MASK  0x00000001 /* Active after successful transmission of an General Control Packet */
#define IH_FC_STAT1_AVI_MASK  0x00000002 /* Active after successful transmission of an AVI InfoFrame packet */
#define IH_FC_STAT1_AMP_MASK  0x00000004 /* Active after successful transmission of an Audio Metadata packet */
#define IH_FC_STAT1_SPD_MASK  0x00000008 /* Active after successful transmission of an Source Product Descriptor InfoFrame packet */
#define IH_FC_STAT1_VSD_MASK  0x00000010 /* Active after successful transmission of an Vendor Specific Data InfoFrame packet */
#define IH_FC_STAT1_ISCR2_MASK  0x00000020 /* Active after successful transmission of an International Standard Recording Code 2 packet */
#define IH_FC_STAT1_ISCR1_MASK  0x00000040 /* Active after successful transmission of an International Standard Recording Code 1 packet */
#define IH_FC_STAT1_GMD_MASK  0x00000080 /* Active after successful transmission of an Gamut metadata packet */

/* Frame Composer Interrupt Status Register 2 (Packet Queue Overflow Interrupts) */
#define IH_FC_STAT2  0x00000408
#define IH_FC_STAT2_HIGHPRIORITY_OVERFLOW_MASK  0x00000001 /* Frame Composer high priority packet queue descriptor overflow indication */
#define IH_FC_STAT2_LOWPRIORITY_OVERFLOW_MASK  0x00000002 /* Frame Composer low priority packet queue descriptor overflow indication */

/* Audio Sampler Interrupt Status Register (FIFO Threshold, Underflow and Overflow Interrupts) */
#define IH_AS_STAT0  0x0000040C
#define IH_AS_STAT0_AUD_FIFO_OVERFLOW_MASK  0x00000001 /* Audio Sampler audio FIFO full indication */
#define IH_AS_STAT0_AUD_FIFO_UNDERFLOW_MASK  0x00000002 /* Audio Sampler audio FIFO empty indication */
#define IH_AS_STAT0_AUD_FIFO_UNDERFLOW_THR_MASK  0x00000004 /* Audio Sampler audio FIFO empty threshold (four samples) indication for the legacy HBR audio interface */
#define IH_AS_STAT0_FIFO_OVERRUN_MASK  0x00000008 /* Indicates an overrun on the audio FIFO */

/* PHY Interface Interrupt Status Register (RXSENSE, PLL Lock and HPD Interrupts) */
#define IH_PHY_STAT0  0x00000410
#define IH_PHY_STAT0_HPD_MASK  0x00000001 /* HDMI Hot Plug Detect indication */
#define IH_PHY_STAT0_TX_PHY_LOCK_MASK  0x00000002 /* TX PHY PLL lock indication */
#define IH_PHY_STAT0_RX_SENSE_0_MASK  0x00000004 /* TX PHY RX_SENSE indication for driver 0 */
#define IH_PHY_STAT0_RX_SENSE_1_MASK  0x00000008 /* TX PHY RX_SENSE indication for driver 1 */
#define IH_PHY_STAT0_RX_SENSE_2_MASK  0x00000010 /* TX PHY RX_SENSE indication for driver 2 */
#define IH_PHY_STAT0_RX_SENSE_3_MASK  0x00000020 /* TX PHY RX_SENSE indication for driver 3 */

/* E-DDC I2C Master Interrupt Status Register (Done and Error Interrupts) */
#define IH_I2CM_STAT0  0x00000414
#define IH_I2CM_STAT0_I2CMASTERERROR_MASK  0x00000001 /* I2C Master error indication */
#define IH_I2CM_STAT0_I2CMASTERDONE_MASK  0x00000002 /* I2C Master done indication */
#define IH_I2CM_STAT0_SCDC_READREQ_MASK  0x00000004 /* I2C Master SCDC read request indication */

/* CEC Interrupt Status Register (Functional Operation Interrupts) */
#define IH_CEC_STAT0  0x00000418
#define IH_CEC_STAT0_DONE_MASK  0x00000001 /* CEC Done Indication */
#define IH_CEC_STAT0_EOM_MASK  0x00000002 /* CEC End of Message Indication */
#define IH_CEC_STAT0_NACK_MASK  0x00000004 /* CEC Not Acknowledge indication */
#define IH_CEC_STAT0_ARB_LOST_MASK  0x00000008 /* CEC Arbitration Lost indication */
#define IH_CEC_STAT0_ERROR_INITIATOR_MASK  0x00000010 /* CEC Error Initiator indication */
#define IH_CEC_STAT0_ERROR_FOLLOW_MASK  0x00000020 /* CEC Error Follow indication */
#define IH_CEC_STAT0_WAKEUP_MASK  0x00000040 /* CEC Wake-up indication */

/* Video Packetizer Interrupt Status Register (FIFO Full and Empty Interrupts) */
#define IH_VP_STAT0  0x0000041C
#define IH_VP_STAT0_FIFOEMPTYBYP_MASK  0x00000001 /* Video Packetizer 8 bit bypass FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLBYP_MASK  0x00000002 /* Video Packetizer 8 bit bypass FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYREMAP_MASK  0x00000004 /* Video Packetizer pixel YCC 422 re-mapper FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLREMAP_MASK  0x00000008 /* Video Packetizer pixel YCC 422 re-mapper FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYPP_MASK  0x00000010 /* Video Packetizer pixel packing FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLPP_MASK  0x00000020 /* Video Packetizer pixel packing FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYREPET_MASK  0x00000040 /* Video Packetizer pixel repeater FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLREPET_MASK  0x00000080 /* Video Packetizer pixel repeater FIFO full interrupt */

/* PHY GEN2 I2C Master Interrupt Status Register (Done and Error Interrupts) */
#define IH_I2CMPHY_STAT0  0x00000420
#define IH_I2CMPHY_STAT0_I2CMPHYERROR_MASK  0x00000001 /* I2C Master PHY error indication */
#define IH_I2CMPHY_STAT0_I2CMPHYDONE_MASK  0x00000002 /* I2C Master PHY done indication */

/* DMA - not supported in this build */
#define IH_AHBDMAAUD_STAT0  0x00000424

/* Interruption Handler Decode Assist Register */
#define IH_DECODE  0x000005C0
#define IH_DECODE_IH_AHBDMAAUD_STAT0_MASK  0x00000001 /* Interruption active at the ih_ahbdmaaud_stat0 register */
#define IH_DECODE_IH_CEC_STAT0_MASK  0x00000002 /* Interruption active at the ih_cec_stat0 register */
#define IH_DECODE_IH_I2CM_STAT0_MASK  0x00000004 /* Interruption active at the ih_i2cm_stat0 register */
#define IH_DECODE_IH_PHY_MASK  0x00000008 /* Interruption active at the ih_phy_stat0 or ih_i2cmphy_stat0 register */
#define IH_DECODE_IH_AS_STAT0_MASK  0x00000010 /* Interruption active at the ih_as_stat0 register */
#define IH_DECODE_IH_FC_STAT2_VP_MASK  0x00000020 /* Interruption active at the ih_fc_stat2 or ih_vp_stat0 register */
#define IH_DECODE_IH_FC_STAT1_MASK  0x00000040 /* Interruption active at the ih_fc_stat1 register */
#define IH_DECODE_IH_FC_STAT0_MASK  0x00000080 /* Interruption active at the ih_fc_stat0 register */

/* Frame Composer Interrupt Mute Control Register 0 */
#define IH_MUTE_FC_STAT0  0x00000600
#define IH_MUTE_FC_STAT0_NULL_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat0[0] */
#define IH_MUTE_FC_STAT0_ACR_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat0[1] */
#define IH_MUTE_FC_STAT0_AUDS_MASK  0x00000004 /* When set to 1, mutes ih_fc_stat0[2] */
#define IH_MUTE_FC_STAT0_NVBI_MASK  0x00000008 /* When set to 1, mutes ih_fc_stat0[3] */
#define IH_MUTE_FC_STAT0_MAS_MASK  0x00000010 /* When set to 1, mutes ih_fc_stat0[4] */
#define IH_MUTE_FC_STAT0_HBR_MASK  0x00000020 /* When set to 1, mutes ih_fc_stat0[5] */
#define IH_MUTE_FC_STAT0_ACP_MASK  0x00000040 /* When set to 1, mutes ih_fc_stat0[6] */
#define IH_MUTE_FC_STAT0_AUDI_MASK  0x00000080 /* When set to 1, mutes ih_fc_stat0[7] */

/* Frame Composer Interrupt Mute Control Register 1 */
#define IH_MUTE_FC_STAT1  0x00000604
#define IH_MUTE_FC_STAT1_GCP_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat1[0] */
#define IH_MUTE_FC_STAT1_AVI_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat1[1] */
#define IH_MUTE_FC_STAT1_AMP_MASK  0x00000004 /* When set to 1, mutes ih_fc_stat1[2] */
#define IH_MUTE_FC_STAT1_SPD_MASK  0x00000008 /* When set to 1, mutes ih_fc_stat1[3] */
#define IH_MUTE_FC_STAT1_VSD_MASK  0x00000010 /* When set to 1, mutes ih_fc_stat1[4] */
#define IH_MUTE_FC_STAT1_ISCR2_MASK  0x00000020 /* When set to 1, mutes ih_fc_stat1[5] */
#define IH_MUTE_FC_STAT1_ISCR1_MASK  0x00000040 /* When set to 1, mutes ih_fc_stat1[6] */
#define IH_MUTE_FC_STAT1_GMD_MASK  0x00000080 /* When set to 1, mutes ih_fc_stat1[7] */

/* Frame Composer Interrupt Mute Control Register 2 */
#define IH_MUTE_FC_STAT2  0x00000608
#define IH_MUTE_FC_STAT2_HIGHPRIORITY_OVERFLOW_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat2[0] */
#define IH_MUTE_FC_STAT2_LOWPRIORITY_OVERFLOW_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat2[1] */

/* Audio Sampler Interrupt Mute Control Register */
#define IH_MUTE_AS_STAT0  0x0000060C
#define IH_MUTE_AS_STAT0_AUD_FIFO_OVERFLOW_MASK  0x00000001 /* When set to 1, mutes ih_as_stat0[0] */
#define IH_MUTE_AS_STAT0_AUD_FIFO_UNDERFLOW_MASK  0x00000002 /* When set to 1, mutes ih_as_stat0[1] */
#define IH_MUTE_AS_STAT0_AUD_FIFO_UNDERFLOW_THR_MASK  0x00000004 /* When set to 1, mutes ih_as_stat0[2] */
#define IH_MUTE_AS_STAT0_FIFO_OVERRUN_MASK  0x00000008 /* When set to 1, mutes ih_as_stat0[3] */

/* PHY Interface Interrupt Mute Control Register */
#define IH_MUTE_PHY_STAT0  0x00000610
#define IH_MUTE_PHY_STAT0_HPD_MASK  0x00000001 /* When set to 1, mutes ih_phy_stat0[0] */
#define IH_MUTE_PHY_STAT0_TX_PHY_LOCK_MASK  0x00000002 /* When set to 1, mutes ih_phy_stat0[1] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_0_MASK  0x00000004 /* When set to 1, mutes ih_phy_stat0[2] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_1_MASK  0x00000008 /* When set to 1, mutes ih_phy_stat0[3] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_2_MASK  0x00000010 /* When set to 1, mutes ih_phy_stat0[4] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_3_MASK  0x00000020 /* When set to 1, mutes ih_phy_stat0[5] */

/* E-DDC I2C Master Interrupt Mute Control Register */
#define IH_MUTE_I2CM_STAT0  0x00000614
#define IH_MUTE_I2CM_STAT0_I2CMASTERERROR_MASK  0x00000001 /* When set to 1, mutes ih_i2cm_stat0[0] */
#define IH_MUTE_I2CM_STAT0_I2CMASTERDONE_MASK  0x00000002 /* When set to 1, mutes ih_i2cm_stat0[1] */
#define IH_MUTE_I2CM_STAT0_SCDC_READREQ_MASK  0x00000004 /* When set to 1, mutes ih_i2cm_stat0[2] */

/* CEC Interrupt Mute Control Register */
#define IH_MUTE_CEC_STAT0  0x00000618
#define IH_MUTE_CEC_STAT0_DONE_MASK  0x00000001 /* When set to 1, mutes ih_cec_stat0[0] */
#define IH_MUTE_CEC_STAT0_EOM_MASK  0x00000002 /* When set to 1, mutes ih_cec_stat0[1] */
#define IH_MUTE_CEC_STAT0_NACK_MASK  0x00000004 /* When set to 1, mutes ih_cec_stat0[2] */
#define IH_MUTE_CEC_STAT0_ARB_LOST_MASK  0x00000008 /* When set to 1, mutes ih_cec_stat0[3] */
#define IH_MUTE_CEC_STAT0_ERROR_INITIATOR_MASK  0x00000010 /* When set to 1, mutes ih_cec_stat0[4] */
#define IH_MUTE_CEC_STAT0_ERROR_FOLLOW_MASK  0x00000020 /* When set to 1, mutes ih_cec_stat0[5] */
#define IH_MUTE_CEC_STAT0_WAKEUP_MASK  0x00000040 /* When set to 1, mutes ih_cec_stat0[6] */

/* Video Packetizer Interrupt Mute Control Register */
#define IH_MUTE_VP_STAT0  0x0000061C
#define IH_MUTE_VP_STAT0_FIFOEMPTYBYP_MASK  0x00000001 /* When set to 1, mutes ih_vp_stat0[0] */
#define IH_MUTE_VP_STAT0_FIFOFULLBYP_MASK  0x00000002 /* When set to 1, mutes ih_vp_stat0[1] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYREMAP_MASK  0x00000004 /* When set to 1, mutes ih_vp_stat0[2] */
#define IH_MUTE_VP_STAT0_FIFOFULLREMAP_MASK  0x00000008 /* When set to 1, mutes ih_vp_stat0[3] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYPP_MASK  0x00000010 /* When set to 1, mutes ih_vp_stat0[4] */
#define IH_MUTE_VP_STAT0_FIFOFULLPP_MASK  0x00000020 /* When set to 1, mutes ih_vp_stat0[5] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYREPET_MASK  0x00000040 /* When set to 1, mutes ih_vp_stat0[6] */
#define IH_MUTE_VP_STAT0_FIFOFULLREPET_MASK  0x00000080 /* When set to 1, mutes ih_vp_stat0[7] */

/* PHY GEN2 I2C Master Interrupt Mute Control Register */
#define IH_MUTE_I2CMPHY_STAT0  0x00000620
#define IH_MUTE_I2CMPHY_STAT0_I2CMPHYERROR_MASK  0x00000001 /* When set to 1, mutes ih_i2cmphy_stat0[0] */
#define IH_MUTE_I2CMPHY_STAT0_I2CMPHYDONE_MASK  0x00000002 /* When set to 1, mutes ih_i2cmphy_stat0[1] */

/* AHB Audio DMA Interrupt Mute Control Register */
#define IH_MUTE_AHBDMAAUD_STAT0  0x00000624
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFEMPTY_MASK  0x00000001 /* When set to 1, mutes ih_ahbdmaaud_stat0[0] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFULL_MASK  0x00000002 /* en set to 1, mutes ih_ahbdmaaud_stat0[1] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTDONE_MASK  0x00000004 /* When set to 1, mutes ih_ahbdmaaud_stat0[2] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTINTERTRYSPLIT_MASK  0x00000008 /* When set to 1, mutes ih_ahbdmaaud_stat0[3] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTLOSTOWNERSHIP_MASK  0x00000010 /* When set to 1, mutes ih_ahbdmaaud_stat0[4] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTERROR_MASK  0x00000020 /* When set to 1, mutes ih_ahbdmaaud_stat0[5] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFOVERRUN_MASK  0x00000040 /* When set to 1, mutes ih_ahbdmaaud_stat0[6] */

/* Global Interrupt Mute Control Register */
#define IH_MUTE  0x000007FC
#define IH_MUTE_MUTE_ALL_INTERRUPT_MASK  0x00000001 /* When set to 1, mutes the main interrupt line (where all interrupts are ORed) */
#define IH_MUTE_MUTE_WAKEUP_INTERRUPT_MASK  0x00000002 /* When set to 1, mutes the main interrupt output port */

int irq_read_stat(hdmi_tx_dev_t *dev, irq_sources_t irq_source, u8 *stat);

int irq_clear_source(hdmi_tx_dev_t *dev, irq_sources_t irq_source);
int irq_clear_bit(hdmi_tx_dev_t *dev, irq_sources_t irq_source, u8 bit_mask);

int irq_mute_source(hdmi_tx_dev_t *dev, irq_sources_t irq_source);
int irq_unmute_source(hdmi_tx_dev_t *dev, irq_sources_t irq_source);

void irq_mute(hdmi_tx_dev_t *dev);
void irq_unmute(hdmi_tx_dev_t *dev);

void irq_clear_all(hdmi_tx_dev_t *dev);
void irq_mask_all(hdmi_tx_dev_t *dev);

int irq_mask_bit(hdmi_tx_dev_t *dev, irq_sources_t irq_source, u8 bit_mask);
int irq_unmask_bit(hdmi_tx_dev_t *dev, irq_sources_t irq_source, u8 bit_mask);


extern void irq_hpd_sense_enable(hdmi_tx_dev_t *dev, u8 enable);

void irq_scdc_read_request(hdmi_tx_dev_t *dev, int enable);

/**
 * Decode functions
 */
u32 read_interrupt_decode(hdmi_tx_dev_t *dev);

int decode_is_fc_stat0(u32 decode);

int decode_is_fc_stat1(u32 decode);

int decode_is_fc_stat2_vp(u32 decode);

int decode_is_as_stat0(u32 decode);

int decode_is_phy(u32 decode);

int decode_is_phy_lock(u32 decode);

int decode_is_phy_hpd(u32 decode);

int decode_is_i2c_stat0(u32 decode);

int decode_is_cec_stat0(u32 decode);


#endif	/* HALINTERRUPT_H_ */
