/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2008,2010 Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based (loosely) on the Linux code
 */

#ifndef _MMC_H_
#define _MMC_H_

#include <linux/list.h>
#include <linux/sizes.h>
#include <linux/compiler.h>
#include <part.h>
#include <config.h>
#include <clk/clk.h>
#include <linux/err.h>

#ifndef CONFIG_MACH_SUN20IW5
#define CONFIG_MMC_SUNXI_USE_DMA
#endif

#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)
#define MMC_SUPPORTS_TUNING
#endif
#if CONFIG_IS_ENABLED(MMC_UHS_SUPPORT)
#define MMC_SUPPORTS_TUNING
#endif

/* SD/MMC version bits; 8 flags, 8 major, 8 minor, 8 change */
#define SD_VERSION_SD	(1U << 31)
#define MMC_VERSION_MMC	(1U << 30)

#define MAKE_SDMMC_VERSION(a, b, c)	\
	((((u32)(a)) << 16) | ((u32)(b) << 8) | (u32)(c))
#define MAKE_SD_VERSION(a, b, c)	\
	(SD_VERSION_SD | MAKE_SDMMC_VERSION(a, b, c))
#define MAKE_MMC_VERSION(a, b, c)	\
	(MMC_VERSION_MMC | MAKE_SDMMC_VERSION(a, b, c))

#define EXTRACT_SDMMC_MAJOR_VERSION(x)	\
	(((u32)(x) >> 16) & 0xff)
#define EXTRACT_SDMMC_MINOR_VERSION(x)	\
	(((u32)(x) >> 8) & 0xff)
#define EXTRACT_SDMMC_CHANGE_VERSION(x)	\
	((u32)(x) & 0xff)

#define SD_VERSION_3		MAKE_SD_VERSION(3, 0, 0)
#define SD_VERSION_2		MAKE_SD_VERSION(2, 0, 0)
#define SD_VERSION_1_0		MAKE_SD_VERSION(1, 0, 0)
#define SD_VERSION_1_10		MAKE_SD_VERSION(1, 10, 0)

#define MMC_VERSION_UNKNOWN	MAKE_MMC_VERSION(0, 0, 0)
#define MMC_VERSION_1_2		MAKE_MMC_VERSION(1, 2, 0)
#define MMC_VERSION_1_4		MAKE_MMC_VERSION(1, 4, 0)
#define MMC_VERSION_2_2		MAKE_MMC_VERSION(2, 2, 0)
#define MMC_VERSION_3		MAKE_MMC_VERSION(3, 0, 0)
#define MMC_VERSION_4		MAKE_MMC_VERSION(4, 0, 0)
#define MMC_VERSION_4_1		MAKE_MMC_VERSION(4, 1, 0)
#define MMC_VERSION_4_2		MAKE_MMC_VERSION(4, 2, 0)
#define MMC_VERSION_4_3		MAKE_MMC_VERSION(4, 3, 0)
#define MMC_VERSION_4_4		MAKE_MMC_VERSION(4, 4, 0)
#define MMC_VERSION_4_41	MAKE_MMC_VERSION(4, 4, 1)
#define MMC_VERSION_4_5		MAKE_MMC_VERSION(4, 5, 0)
#define MMC_VERSION_5_0		MAKE_MMC_VERSION(5, 0, 0)
#define MMC_VERSION_5_1		MAKE_MMC_VERSION(5, 1, 0)

#define MMC_CAP(mode)		(1 << mode)
#define MMC_MODE_HS		(MMC_CAP(MMC_HS) | MMC_CAP(SD_HS))
#define MMC_MODE_HS_52MHz	MMC_CAP(MMC_HS_52)
#define MMC_MODE_DDR_52MHz	MMC_CAP(MMC_DDR_52)
#define MMC_MODE_HS200		MMC_CAP(MMC_HS_200)
#define MMC_MODE_HS400          MMC_CAP(MMC_HS_400)

#define MMC_MODE_8BIT		BIT(30)
#define MMC_MODE_4BIT		BIT(29)
#define MMC_MODE_1BIT		BIT(28)
#define MMC_MODE_SPI		BIT(27)


#define SD_DATA_4BIT	0x00040000

#define IS_SD(x)	((x)->version & SD_VERSION_SD)
#define IS_MMC(x)	((x)->version & MMC_VERSION_MMC)

#define MMC_CMD_MANUAL  1//add by sunxi.not sent stop when read/write multi block,and sent stop when sent cmd12

#define MMC_DATA_READ		1
#define MMC_DATA_WRITE		2
#define NO_CARD_ERR             -16 /* No SD/MMC card inserted */
#define UNUSABLE_ERR           -17 /* Unusable Card */
#define COMM_ERR                -18 /* Communications Error */
#define TIMEOUT                 -19
#define IN_PROGRESS             -20 /* operation is in progress */
#define SWITCH_ERR              -21 /* Card reports failure to switch mode */

#define MMC_CMD_GO_IDLE_STATE		0
#define MMC_CMD_SEND_OP_COND		1
#define MMC_CMD_ALL_SEND_CID		2
#define MMC_CMD_SET_RELATIVE_ADDR	3
#define MMC_CMD_SET_DSR			4
#define MMC_CMD_SWITCH			6
#define MMC_CMD_SELECT_CARD		7
#define MMC_CMD_SEND_EXT_CSD		8
#define MMC_CMD_SEND_CSD		9
#define MMC_CMD_SEND_CID		10
#define MMC_CMD_STOP_TRANSMISSION	12
#define MMC_CMD_SEND_STATUS		13
#define MMC_CMD_SET_BLOCKLEN		16
#define MMC_CMD_READ_SINGLE_BLOCK	17
#define MMC_CMD_READ_MULTIPLE_BLOCK	18
#define MMC_CMD_SEND_TUNING_BLOCK	19
#define MMC_CMD_SEND_TUNING_BLOCK_HS200	21
#define MMC_CMD_SET_BLOCK_COUNT         23
#define MMC_CMD_WRITE_SINGLE_BLOCK	24
#define MMC_CMD_WRITE_MULTIPLE_BLOCK	25
#define MMC_CMD_SET_WRITE_PROT          28
#define MMC_CMD_CLR_WRITE_PROT          29
#define MMC_CMD_SEND_WRITE_PROT         30
#define MMC_CMD_SEND_WRITE_PROT_TYPE    31
#define MMC_CMD_ERASE_GROUP_START	35
#define MMC_CMD_ERASE_GROUP_END		36
#define MMC_CMD_ERASE			38
#define MMC_CMD_APP_CMD			55
#define MMC_CMD_SPI_READ_OCR		58
#define MMC_CMD_SPI_CRC_ON_OFF		59
#define MMC_CMD_RES_MAN			62

#define MMC_CMD62_ARG1			0xefac62ec
#define MMC_CMD62_ARG2			0xcbaea7


#define SD_CMD_SEND_RELATIVE_ADDR	3
#define SD_CMD_SWITCH_FUNC		6
#define SD_CMD_SEND_IF_COND		8
#define SD_CMD_SWITCH_UHS18V		11

#define SD_CMD_APP_SET_BUS_WIDTH	6
#define SD_CMD_APP_SD_STATUS		13
#define SD_CMD_ERASE_WR_BLK_START	32
#define SD_CMD_ERASE_WR_BLK_END		33
#define SD_CMD_APP_SEND_OP_COND		41
#define SD_CMD_APP_SEND_SCR		51

static inline bool mmc_is_tuning_cmd(uint cmdidx)
{
	if ((cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200) ||
	    (cmdidx == MMC_CMD_SEND_TUNING_BLOCK))
		return true;
	return false;
}

/* SCR definitions in different words */
#define SD_HIGHSPEED_BUSY	0x00020000
#define SD_HIGHSPEED_SUPPORTED	0x00020000

#define UHS_SDR12_BUS_SPEED	0
#define HIGH_SPEED_BUS_SPEED	1
#define UHS_SDR25_BUS_SPEED	1
#define UHS_SDR50_BUS_SPEED	2
#define UHS_SDR104_BUS_SPEED	3
#define UHS_DDR50_BUS_SPEED	4

#define SD_MODE_UHS_SDR12	BIT(UHS_SDR12_BUS_SPEED)
#define SD_MODE_UHS_SDR25	BIT(UHS_SDR25_BUS_SPEED)
#define SD_MODE_UHS_SDR50	BIT(UHS_SDR50_BUS_SPEED)
#define SD_MODE_UHS_SDR104	BIT(UHS_SDR104_BUS_SPEED)
#define SD_MODE_UHS_DDR50	BIT(UHS_DDR50_BUS_SPEED)

#define OCR_BUSY		0x80000000
#define OCR_HCS			0x40000000
#define OCR_S18R		0x1000000
#define OCR_VOLTAGE_MASK	0x007FFF80
#define OCR_ACCESS_MODE		0x60000000

#define MMC_ERASE_ARG		0x00000000
#define MMC_SECURE_ERASE_ARG	0x80000000
#define MMC_TRIM_ARG		0x00000001
#define MMC_DISCARD_ARG		0x00000003
#define MMC_SECURE_TRIM1_ARG	0x80000001
#define MMC_SECURE_TRIM2_ARG	0x80008000

#define MMC_STATUS_MASK		(~0x0206BF7F)
#define MMC_STATUS_SWITCH_ERROR	(1 << 7)
#define MMC_STATUS_RDY_FOR_DATA (1 << 8)
#define MMC_STATUS_CURR_STATE	(0xf << 9)
#define MMC_STATUS_ERROR	(1 << 19)

#define MMC_STATE_PRG		(7 << 9)

#define MMC_VDD_165_195		0x00000080	/* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21		0x00000100	/* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22		0x00000200	/* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23		0x00000400	/* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24		0x00000800	/* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25		0x00001000	/* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26		0x00002000	/* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27		0x00004000	/* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28		0x00008000	/* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29		0x00010000	/* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30		0x00020000	/* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31		0x00040000	/* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32		0x00080000	/* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33		0x00100000	/* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34		0x00200000	/* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35		0x00400000	/* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36		0x00800000	/* VDD voltage 3.5 ~ 3.6 */

#define MMC_SWITCH_MODE_CMD_SET		0x00 /* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS	0x01 /* Set bits in EXT_CSD byte
						addressed by index which are
						1 in value field */
#define MMC_SWITCH_MODE_CLEAR_BITS	0x02 /* Clear bits in EXT_CSD byte
						addressed by index, which are
						1 in value field */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03 /* Set target byte to value */

#define SD_SWITCH_CHECK		0
#define SD_SWITCH_SWITCH	1

/*
 * EXT_CSD fields
 */
#define EXT_CSD_FFU_STATUS		26	/* RO */
#define EXT_CSD_MODE_CONFIG		30	/* R/W/E_P */
#define EXT_CSD_ENH_START_ADDR		136	/* R/W */
#define EXT_CSD_ENH_SIZE_MULT		140	/* R/W */
#define EXT_CSD_GP_SIZE_MULT		143	/* R/W */
#define EXT_CSD_PARTITION_SETTING	155	/* R/W */
#define EXT_CSD_PARTITIONS_ATTRIBUTE	156	/* R/W */
#define EXT_CSD_MAX_ENH_SIZE_MULT	157	/* R */
#define EXT_CSD_PARTITIONING_SUPPORT	160	/* RO */
#define EXT_CSD_RST_N_FUNCTION		162	/* R/W */
#define EXT_CSD_BKOPS_EN		163	/* R/W & R/W/E */
#define EXT_CSD_SANITIZE_START          165     /* W */
#define EXT_CSD_WR_REL_PARAM		166	/* R */
#define EXT_CSD_WR_REL_SET		167	/* R/W */
#define EXT_CSD_RPMB_MULT		168	/* RO */
#define EXT_CSD_FW_CONFIG		169	/* R/W */
#define EXT_CSD_USER_WP			171	/* R/W */
#define EXT_CSD_ERASE_GROUP_DEF		175	/* R/W */
#define EXT_CSD_BOOT_BUS_WIDTH		177
#define EXT_CSD_PART_CONF		179	/* R/W */
#define EXT_CSD_ERASED_MEM_CONT         181     /* RO */
#define EXT_CSD_BUS_WIDTH		183	/* R/W */
#define EXT_CSD_HS_TIMING		185	/* R/W */
#define EXT_CSD_REV			192	/* RO */
#define EXT_CSD_STRUCTURE		194     /* RO */
#define EXT_CSD_CARD_TYPE		196	/* RO */
#define EXT_CSD_SEC_CNT			212	/* RO, 4 bytes */
#define EXT_CSD_HC_WP_GRP_SIZE		221	/* RO */
#define EXT_CSD_ERASE_TIMEOUT_MULT      223     /* RO */
#define EXT_CSD_HC_ERASE_GRP_SIZE	224	/* RO */
#define EXT_CSD_BOOT_MULT		226	/* RO */
#define EXT_CSD_SEC_TRIM_MULT           229     /* RO */
#define EXT_CSD_SEC_ERASE_MULT          230     /* RO */
#define EXT_CSD_SEC_FEATURE_SUPPORT     231     /* RO */
#define EXT_CSD_TRIM_MULT               232    /* RO */
#define EXT_CSD_POWER_OFF_LONG_TIME     247     /* RO */
#define EXT_CSD_GENERIC_CMD6_TIME       248     /* RO */
#define EXT_CSD_SUPPORTED_MODES       	493     /* RO */
#define EXT_CSD_BKOPS_SUPPORT		502	/* RO */

/*
 * EXT_CSD field definitions
 */

#define EXT_CSD_CMD_SET_NORMAL		(1 << 0)
#define EXT_CSD_CMD_SET_SECURE		(1 << 1)
#define EXT_CSD_CMD_SET_CPSECURE	(1 << 2)

#define EXT_CSD_CARD_TYPE_26	(1 << 0)	/* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_52	(1 << 1)	/* Card can run at 52MHz */
#define EXT_CSD_CARD_TYPE_HS           (EXT_CSD_CARD_TYPE_26 | \
		+                                       EXT_CSD_CARD_TYPE_52)
#define EXT_CSD_CARD_TYPE_DDR_1_8V	(1 << 2)
#define EXT_CSD_CARD_TYPE_DDR_1_2V	(1 << 3)
#define EXT_CSD_CARD_TYPE_DDR_52	(EXT_CSD_CARD_TYPE_DDR_1_8V \
					| EXT_CSD_CARD_TYPE_DDR_1_2V)

#define EXT_CSD_CARD_TYPE_HS200_1_8V	BIT(4)	/* Card can run at 200MHz */
						/* SDR mode @1.8V I/O */
#define EXT_CSD_CARD_TYPE_HS200_1_2V	BIT(5)	/* Card can run at 200MHz */
						/* SDR mode @1.2V I/O */
#define EXT_CSD_CARD_TYPE_HS200		(EXT_CSD_CARD_TYPE_HS200_1_8V | \
					 EXT_CSD_CARD_TYPE_HS200_1_2V)
#define EXT_CSD_CARD_TYPE_HS400_1_8V    (1<<6)  /* Card can run at 200MHz DDR, 1.8V */
#define EXT_CSD_CARD_TYPE_HS400_1_2V    (1<<7)  /* Card can run at 200MHz DDR, 1.8V */
#define EXT_CSD_CARD_TYPE_HS400         (EXT_CSD_CARD_TYPE_HS400_1_8V | \
					EXT_CSD_CARD_TYPE_HS400_1_2V)

/* -- EXT_CSD[171] USER_WP */
#define MMC_SWITCH_USER_WP_PERM_PSWD_DIS    (0x1 << 7)
#define MMC_SWITCH_USER_WP_CD_PERM_WP_DIS   (0x1 << 6)
#define MMC_SWITCH_USER_WP_US_PERM_WP_DIS   (0x1 << 4)
#define MMC_SWITCH_USER_WP_US_PWR_WP_DIS    (0x1 << 3)
#define MMC_SWITCH_USER_WP_US_PERM_WP_EN    (0x1 << 2)
#define MMC_SWITCH_USER_WP_US_PWR_WP_EN     (0x1 << 0)

/* -- EXT_CSD[183] BUS_WIDTH */
#define EXT_CSD_BUS_WIDTH_1	0	/* Card is in 1 bit mode */
#define EXT_CSD_BUS_WIDTH_4	1	/* Card is in 4 bit mode */
#define EXT_CSD_BUS_WIDTH_8	2	/* Card is in 8 bit mode */
#define EXT_CSD_DDR_BUS_WIDTH_4	5	/* Card is in 4 bit DDR mode */
#define EXT_CSD_DDR_BUS_WIDTH_8	6	/* Card is in 8 bit DDR mode */
#define EXT_CSD_DDR_FLAG	BIT(2)	/* Flag for DDR mode */

/* -- EXT_CSD[185] HS_TIMING */
#define EXT_CSD_TIMING_LEGACY	0	/* no high speed */
#define EXT_CSD_TIMING_HS	1	/* HS */
#define EXT_CSD_TIMING_HS200	2	/* HS200 */
#define EXT_CSD_TIMING_HS400    3        /* HS400 */

/* -- EXT_CSD[231] SEC_FEATURE_SUPPORT */
#define EXT_CSD_SEC_ER_EN           (1U << 0)
#define EXT_CSD_SEC_BD_BLK_EN   (1U << 2)
#define EXT_CSD_SEC_GB_CL_EN    (1U << 4)
#define EXT_CSD_SEC_SANITIZE    (1U << 6)  /* v4.5 only */

#define EXT_CSD_BOOT_ACK_ENABLE			(1 << 6)
#define EXT_CSD_BOOT_PARTITION_ENABLE		(1 << 3)
#define EXT_CSD_PARTITION_ACCESS_ENABLE		(1 << 0)
#define EXT_CSD_PARTITION_ACCESS_DISABLE	(0 << 0)

#define EXT_CSD_BOOT_ACK(x)		(x << 6)
#define EXT_CSD_BOOT_PART_NUM(x)	(x << 3)
#define EXT_CSD_PARTITION_ACCESS(x)	(x << 0)

#define EXT_CSD_EXTRACT_BOOT_ACK(x)		(((x) >> 6) & 0x1)
#define EXT_CSD_EXTRACT_BOOT_PART(x)		(((x) >> 3) & 0x7)
#define EXT_CSD_EXTRACT_PARTITION_ACCESS(x)	((x) & 0x7)

#define EXT_CSD_BOOT_BUS_WIDTH_MODE(x)	(x << 3)
#define EXT_CSD_BOOT_BUS_WIDTH_RESET(x)	(x << 2)
#define EXT_CSD_BOOT_BUS_WIDTH_WIDTH(x)	(x)

#define EXT_CSD_PARTITION_SETTING_COMPLETED	(1 << 0)

#define EXT_CSD_ENH_USR		(1 << 0)	/* user data area is enhanced */
#define EXT_CSD_ENH_GP(x)	(1 << ((x)+1))	/* GP part (x+1) is enhanced */

#define EXT_CSD_HS_CTRL_REL	(1 << 0)	/* host controlled WR_REL_SET */

#define EXT_CSD_WR_DATA_REL_USR		(1 << 0)	/* user data area WR_REL */
#define EXT_CSD_WR_DATA_REL_GP(x)	(1 << ((x)+1))	/* GP part (x+1) WR_REL */

#define R1_ILLEGAL_COMMAND		(1 << 22)
#define R1_APP_CMD			(1 << 5)

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136	(1 << 1)		/* 136 bit response */
#define MMC_RSP_CRC	(1 << 2)		/* expect valid crc */
#define MMC_RSP_BUSY	(1 << 3)		/* card may send busy */
#define MMC_RSP_OPCODE	(1 << 4)		/* response contains opcode */

#define MMC_RSP_NONE	(0)
#define MMC_RSP_R1	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1b	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE| \
			MMC_RSP_BUSY)
#define MMC_RSP_R2	(MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3	(MMC_RSP_PRESENT)
#define MMC_RSP_R4	(MMC_RSP_PRESENT)
#define MMC_RSP_R5	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

#define MMCPART_NOAVAILABLE	(0xff)
#define PART_ACCESS_MASK	(0x7)
#define PART_SUPPORT		(0x1)
#define ENHNCD_SUPPORT		(0x2)
#define PART_ENH_ATTRIB		(0x1f)

#define MMC_QUIRK_RETRY_SEND_CID	BIT(0)
#define MMC_QUIRK_RETRY_SET_BLOCKLEN	BIT(1)

enum mmc_voltage {
	MMC_SIGNAL_VOLTAGE_000 = 0,
	MMC_SIGNAL_VOLTAGE_120 = 1,
	MMC_SIGNAL_VOLTAGE_180 = 2,
	MMC_SIGNAL_VOLTAGE_330 = 4,
};

#define MMC_ALL_SIGNAL_VOLTAGE (MMC_SIGNAL_VOLTAGE_120 |\
				MMC_SIGNAL_VOLTAGE_180 |\
				MMC_SIGNAL_VOLTAGE_330)

/* Maximum block size for MMC */
#define MMC_MAX_BLOCK_LEN	512

/* Maximum dts string for MMC */
#define MMC_MAX_DTS_STRING_LEN	50
/* Maximum dts string for MMC */
#define MMC_MAX_PLL_STRING_LEN	32

/* The number of MMC physical partitions.  These consist of:
 * boot partitions (2), general purpose partitions (4) in MMC v4.4.
 */
#define MMC_NUM_BOOT_PARTITION	2
#define MMC_PART_RPMB           3       /* RPMB partition number */

/* Driver model support */

/**
 * struct mmc_uclass_priv - Holds information about a device used by the uclass
 */
struct mmc_uclass_priv {
	struct mmc *mmc;
};

/**
 * mmc_get_mmc_dev() - get the MMC struct pointer for a device
 *
 * Provided that the device is already probed and ready for use, this value
 * will be available.
 *
 * @dev:	Device
 * @return associated mmc struct pointer if available, else NULL
 */
struct mmc *mmc_get_mmc_dev(struct udevice *dev);

/* End of driver model support */

struct mmc_cid {
	unsigned long psn;
	unsigned short oid;
	unsigned char mid;
	unsigned char prv;
	unsigned char mdt;
	char pnm[7];
};

struct mmc_cmd {
	ushort cmdidx;
	uint resp_type;
	uint cmdarg;
	uint response[4];
};

struct mmc_data {
	union {
		char *dest;
		const char *src; /* src buffers don't get written to */
	};
	uint flags;
	uint blocks;
	uint blocksize;
};

/* forward decl. */
struct mmc;

#if CONFIG_IS_ENABLED(DM_MMC)
struct dm_mmc_ops {
	/**
	 * send_cmd() - Send a command to the MMC device
	 *
	 * @dev:	Device to receive the command
	 * @cmd:	Command to send
	 * @data:	Additional data to send/receive
	 * @return 0 if OK, -ve on error
	 */
	int (*send_cmd)(struct udevice *dev, struct mmc_cmd *cmd,
			struct mmc_data *data);

	/**
	 * set_ios() - Set the I/O speed/width for an MMC device
	 *
	 * @dev:	Device to update
	 * @return 0 if OK, -ve on error
	 */
	int (*set_ios)(struct udevice *dev);

	/**
	 * send_init_stream() - send the initialization stream: 74 clock cycles
	 * This is used after power up before sending the first command
	 *
	 * @dev:	Device to update
	 */
	void (*send_init_stream)(struct udevice *dev);

	/**
	 * get_cd() - See whether a card is present
	 *
	 * @dev:	Device to check
	 * @return 0 if not present, 1 if present, -ve on error
	 */
	int (*get_cd)(struct udevice *dev);

	/**
	 * get_wp() - See whether a card has write-protect enabled
	 *
	 * @dev:	Device to check
	 * @return 0 if write-enabled, 1 if write-protected, -ve on error
	 */
	int (*get_wp)(struct udevice *dev);

#ifdef MMC_SUPPORTS_TUNING
	/**
	 * execute_tuning() - Start the tuning process
	 *
	 * @dev:	Device to start the tuning
	 * @opcode:	Command opcode to send
	 * @return 0 if OK, -ve on error
	 */
	int (*execute_tuning)(struct udevice *dev, uint opcode);
#endif

#if CONFIG_IS_ENABLED(MMC_UHS_SUPPORT)
	/**
	 * wait_dat0() - wait until dat0 is in the target state
	 *		(CLK must be running during the wait)
	 *
	 * @dev:	Device to check
	 * @state:	target state
	 * @timeout:	timeout in us
	 * @return 0 if dat0 is in the target state, -ve on error
	 */
	int (*wait_dat0)(struct udevice *dev, int state, int timeout);
#endif
};

#define mmc_get_ops(dev)        ((struct dm_mmc_ops *)(dev)->driver->ops)

int dm_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
		    struct mmc_data *data);
int dm_mmc_set_ios(struct udevice *dev);
void dm_mmc_send_init_stream(struct udevice *dev);
int dm_mmc_get_cd(struct udevice *dev);
int dm_mmc_get_wp(struct udevice *dev);
int dm_mmc_execute_tuning(struct udevice *dev, uint opcode);
int dm_mmc_wait_dat0(struct udevice *dev, int state, int timeout);

/* Transition functions for compatibility */
int mmc_set_ios(struct mmc *mmc);
void mmc_send_init_stream(struct mmc *mmc);
int mmc_getcd(struct mmc *mmc);
int mmc_getwp(struct mmc *mmc);
int mmc_execute_tuning(struct mmc *mmc, uint opcode);
int mmc_wait_dat0(struct mmc *mmc, int state, int timeout);

#else
struct mmc_ops {
	int (*send_cmd)(struct mmc *mmc,
			struct mmc_cmd *cmd, struct mmc_data *data);
	int (*set_ios)(struct mmc *mmc);
	int (*init)(struct mmc *mmc);
	int (*getcd)(struct mmc *mmc);
	int (*getwp)(struct mmc *mmc);
	/*
	   add these members to impliment sample point auto-adaption
	   */
	int (*decide_retry)(struct mmc *mmc, int err_no, uint reset_count);
	int (*update_sdly)(struct mmc *mmc, uint sdly);
	int (*get_detail_errno)(struct mmc *mmc);

	int (*update_phase)(struct mmc *mmc);
};
#endif
struct tune_sdly {
	u32 tm4_smx_fx[12];
};

struct boot_mmc_cfg {
	u8 boot0_para;
	u8 boot_odly_50M;
	u8 boot_sdly_50M;
	u8 boot_odly_50M_ddr;
	u8 boot_sdly_50M_ddr;
	u8 boot_hs_f_max;
	u8 res[2];
};

#define SDMMC_PRIV_INFO_ADDR_OFFSET (128)
struct boot_sdmmc_private_info_t {
	struct tune_sdly tune_sdly;
	struct boot_mmc_cfg boot_mmc_cfg;

#define CARD_TYPE_SD  0x8000001
#define CARD_TYPE_MMC 0x8000000
#define CARD_TYPE_NULL 0xffffffff
	u32 card_type;  /*0xffffffff: invalid; 0x8000000: mmc card; 0x8000001: sd card*/

#define EXT_PARA0_ID                  (0x55000000)
#define EXT_PARA0_TUNING_SUCCESS_FLAG (1U<<0)
	u32 ext_para0;

	/**GPIO 1.8V bias setting***/
#define  EXT_PARA1_1V8_GPIO_BIAS        0x1
#define BOOT0_SUP_HS			0x2
	u32 ext_para1;
	/* ext_para/2/3 reseved for future */
	u32 ext_para2;
	u32 ext_para3;
};


struct mmc_config {
	const char *name;
#if !CONFIG_IS_ENABLED(DM_MMC)
	const struct mmc_ops *ops;
#endif
	struct tune_sdly sdly;
#define MAX_EXT_FREQ_POINT_NUM (4)
	u32 tm4_tune_ext_freq[MAX_EXT_FREQ_POINT_NUM];
	uint host_no;
	uint host_caps;
	uint voltages;
	uint f_min;
	uint f_max;
	uint b_max;
	unsigned char part_type;
	/* boot0 burn positon */
#define DRV_PARA_NOT_BURN_USER_PART           (1U<<0)
#define DRV_PARA_BURN_EMMC_BOOT_PART          (1U<<1)
#define DRV_PARA_BURN_FORCE_FLUSH_CACHE       (1U<<3)
	uint drv_burn_boot_pos;

	/* struct mmc/drv_wipe_feature, define for driver secure wipe opeation */
#define DRV_PARA_DISABLE_SECURE_WIPE          (1U<<0)
#define DRV_PARA_DISABLE_EMMC_SANITIZE        (1U<<1)
#define DRV_PARA_DISABLE_EMMC_SECURE_PURGE    (1U<<2)
#define DRV_PARA_DISABLE_EMMC_TRIM            (1U<<3)
	uint drv_wipe_feature;

	/* struct mmc/drv_erase_feature, define for drvier erase operation */
#define DRV_PARA_DISABLE_EMMC_ERASE               (1U<<0)
#define DRV_PARA_ENABLE_EMMC_SANITIZE_WHEN_ERASE  (1U<<1)
	uint drv_erase_feature;

	/* struct mmc/drv_wp_feature */
#define DRV_PARA_ENABLE_EMMC_USER_PART_WP     (1U<<0)
	uint drv_wp_feature;

	/* struct mmc/drv_hce_feature */
#define DRV_PARA_ENABLE_EMMC_HC_CAP_UNIT     (1U<<0)
	uint drv_hc_cap_unit_feature;

#define AUTO_SAMPLE_MODE   (2)
#define MAUNAL_SAMPLE_MODE (1)
	uint sample_mode;

	uint io_is_1v8;

	uint cal_delay_unit;
#define BOOT0_PARA_USE_INTERNAL_DEFAULT_TIMING_PARA (1U<<0)
	/* 0: pass through struct sdly;  1: pass through boot_odly/sdly_* */
#define BOOT0_PARA_USE_EXTERNAL_INPUT_TIMING_PARA   (1U<<1)
	u8 boot0_para;

	u8 boot_odly_50M;
	u8 boot_sdly_50M;
	u8 boot_odly_50M_ddr;
	u8 boot_sdly_50M_ddr;
	u8 boot_hs_f_max;

	u8 *odly_spd_freq; //[40];
	u8 *sdly_spd_freq; //[40];

	u8 tm4_timing_window_th;
	u8 tm4_tune_r_cycle;
	u8 tm4_tune_hs200_max_freq;
	u8 tm4_tune_hs400_max_freq;
	u8 tune_limit_kernel_timing;
	u8 res[1];

	/* bit31: valid; bit23~16: speed mode; bit15~8: freq id; bit7:0 freq value */

#define DRV_PARA_DISABLE_MMC_MODE_HS        (1 << 0) /* can run at 26MHz -- DS26_SDR12*/
#define DRV_PARA_DISABLE_MMC_MODE_HS_52MHz     (1 << 1) /* can run at 52MHz with SDR mode -- HSSDR52_SDR25 */
#define DRV_PARA_DISABLE_MMC_MODE_4BIT         (1 << 2)
#define DRV_PARA_DISABLE_MMC_MODE_8BIT         (1 << 3)
#define DRV_PARA_DISABLE_MMC_MODE_SPI          (1 << 4)
#define DRV_PARA_DISABLE_MMC_MODE_HC        (1 << 5)
#define DRV_PARA_DISABLE_MMC_MODE_DDR_52MHz    (1 << 6) /* can run at 52Mhz with DDR mode -- HSDDR52_DDR50 */
#define DRV_PARA_DISABLE_MMC_MODE_HS200     (1 << 7) /* can run at 200/208MHz with SDR mode -- HS200_SDR104*/
#define DRV_PARA_DISABLE_MMC_MODE_HS400     (1 << 8) /* can run at 200MHz with DDR mode -- HS400 */
#define DRV_PARA_ENABLE_EMMC_HW_RST         (1 << 16) /* support to enable eMMC HW Rst when product */
	u32 host_caps_mask;
	u32 retune_caps_mask;

	u32 force_boot_tuning;

	/* enable the flow of field firmware update(FFU) flow */
	u32 enable_ffu;
	/* the byte length of emmc firmware, if it is 0, use the length get from toc0 header. if it is 0xffffffff, invalid len */
	u32 emmc_fw_byte_len;
	/* emmc_fw_ver0[31:0] = ext_csd[257] | ext_csd[256] | ext_csd[255] | ext_csd[254] */
	u32 emmc_fw_ver0;
	/* emmc_fw_ver1[31:0] = ext_csd[261] | ext_csd[260] | ext_csd[259] | ext_csd[258] */
	u32 emmc_fw_ver1;
	u32 emmc_ffu_mid;
	u32 emmc_ffu_spt_fw;    /*whether support many FWs*/
	char *emmc_ffu_fw;      /*original fw(32G),other FWs please add at the back*/
	char *emmc_ffu_fw1;
	uint emmc_set_block_count;
	/*ext_csd 261:254*/
	u64 ffu_src_fw_version;
	u64 ffu_dest_fw_version;

	/* 1: boot0 support HS400 or HS200; 0: not support*/
	u8 boot0_sup_1v8;
	struct clk *clk_mmc;
	char clk_mmc_name[MMC_MAX_PLL_STRING_LEN];
	char pll0[MMC_MAX_PLL_STRING_LEN];
	char pll1[MMC_MAX_PLL_STRING_LEN];
	char pll2[MMC_MAX_PLL_STRING_LEN];
};

struct sd_ssr {
	unsigned int au;		/* In sectors */
	unsigned int erase_timeout;	/* In milliseconds */
	unsigned int erase_offset;	/* In milliseconds */
};

enum bus_mode {
	MMC_LEGACY,
	SD_LEGACY,
	MMC_HS,
	SD_HS,
	MMC_HS_52,
	MMC_DDR_52,
	UHS_SDR12,
	UHS_SDR25,
	UHS_SDR50,
	UHS_DDR50,
	UHS_SDR104,
	MMC_HS_200,
	MMC_HS_400,
	MMC_MODES_END
};

const char *mmc_mode_name(enum bus_mode mode);
void mmc_dump_capabilities(const char *text, uint caps);

static inline bool mmc_is_mode_ddr(enum bus_mode mode)
{
	if ((mode == MMC_DDR_52) || (mode == MMC_HS_400))
		return true;
#if CONFIG_IS_ENABLED(MMC_UHS_SUPPORT)
	else if (mode == UHS_DDR50)
		return true;
#endif
	else
		return false;
}

#define UHS_CAPS (MMC_CAP(UHS_SDR12) | MMC_CAP(UHS_SDR25) | \
		  MMC_CAP(UHS_SDR50) | MMC_CAP(UHS_SDR104) | \
		  MMC_CAP(UHS_DDR50))

static inline bool supports_uhs(uint caps)
{
#if CONFIG_IS_ENABLED(MMC_UHS_SUPPORT)
	return (caps & UHS_CAPS) ? true : false;
#else
	return false;
#endif
}

/*
 * With CONFIG_DM_MMC enabled, struct mmc can be accessed from the MMC device
 * with mmc_get_mmc_dev().
 *
 * TODO struct mmc should be in mmc_private but it's hard to fix right now
 */
 #pragma pack(4)
struct mmc {
#if !CONFIG_IS_ENABLED(BLK)
	struct list_head link;
#endif
	const struct mmc_config *cfg;	/* provided configuration */
	uint version;
	void *priv;
	uint has_init;
	int high_capacity;
	bool clk_disable; /* true if the clock can be turned off */
	uint bus_width;
	uint clock;
	enum mmc_voltage signal_voltage;
	uint card_caps;
	uint host_caps;
	uint ocr;
	uint dsr;
	uint dsr_imp;
	uint scr[2];
	uint csd[4];
	uint cid[4];
	ushort rca;
	u8 part_support;
	u8 part_attr;
	u8 wr_rel_set;
	u8 part_config;
	uint tran_speed;
	uint legacy_speed; /* speed for the legacy mode provided by the card */
	uint read_bl_len;
#if CONFIG_IS_ENABLED(MMC_WRITE)
	uint write_bl_len;
	uint erase_grp_size;	/* in 512-byte sectors */
#endif
#if CONFIG_IS_ENABLED(MMC_HW_PARTITIONING)
	uint hc_wp_grp_size;	/* in 512-byte sectors */
#endif
#if CONFIG_IS_ENABLED(MMC_WRITE)
	struct sd_ssr	ssr;	/* SD status register */
#endif
	u64 capacity;
	u64 capacity_user;
	u64 capacity_boot;
	u64 capacity_rpmb;
	u64 capacity_gp[4];
#ifndef CONFIG_SPL_BUILD
	u64 enh_user_start;
	u64 enh_user_size;
#endif
#if !CONFIG_IS_ENABLED(BLK)
	struct blk_desc block_dev;
#endif
	char op_cond_pending;	/* 1 if we are waiting on an op_cond command */
	char init_in_progress;	/* 1 if we have done mmc_start_init() */
	char preinit;		/* start init as early as possible */
	int ddr_mode;
#if CONFIG_IS_ENABLED(DM_MMC)
	struct udevice *dev;	/* Device for this MMC controller */
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	struct udevice *vmmc_supply;	/* Main voltage regulator (Vcc)*/
	struct udevice *vqmmc_supply;	/* IO voltage regulator (Vccq)*/
#endif
#endif
	uint speed_mode;
	u8 *ext_csd;
	u32 cardtype;		/* cardtype read from the MMC */
	enum mmc_voltage current_voltage;
	enum bus_mode selected_mode; /* mode currently used */
	enum bus_mode best_mode; /* best mode is the supported mode with the
				  * highest bandwidth. It may not always be the
				  * operating mode due to limitations when
				  * accessing the boot partitions
				  */
	u32 quirks;
	u32 tuning_end;
	u32 do_tuning;
	u32 msglevel;
	int manual_stop_flag;

	uint erase_timeout; /*default erasetimeout or hc_erase_timeout*/
	uint trim_discard_timeout;
	uint secure_erase_timeout;
	uint secure_trim_timeout;

	//u64 wp_grp_size; /* write protect group size */
	//u64 csd_perm_wp;
	//u64 csd_wp_grp_size;
	uchar secure_feature; // extcsd[231]
};
#pragma pack()

struct mmc_hwpart_conf {
	struct {
		uint enh_start;	/* in 512-byte sectors */
		uint enh_size;	/* in 512-byte sectors, if 0 no enh area */
		unsigned wr_rel_change : 1;
		unsigned wr_rel_set : 1;
	} user;
	struct {
		uint size;	/* in 512-byte sectors */
		unsigned enhanced : 1;
		unsigned wr_rel_change : 1;
		unsigned wr_rel_set : 1;
	} gp_part[4];
};

enum mmc_hwpart_conf_mode {
	MMC_HWPART_CONF_CHECK,
	MMC_HWPART_CONF_SET,
	MMC_HWPART_CONF_COMPLETE,
};

struct mmc_ext_csd {
	u8			rev;
	u8			erase_group_def;
	u8			sec_feature_support;
	u8			rel_sectors;
	u8			rel_param;
	u8			part_config;
	u8			cache_ctrl;
	u8			rst_n_function;
	u8			max_packed_writes;
	u8			max_packed_reads;
	u8			packed_event_en;
	unsigned int		part_time;		/* Units: ms */
	unsigned int		sa_timeout;		/* Units: 100ns */
	unsigned int		generic_cmd6_time;	/* Units: 10ms */
	unsigned int            power_off_longtime;     /* Units: ms */
	u8			power_off_notification;	/* state */
	unsigned int		hs_max_dtr;
	unsigned int		hs200_max_dtr;
#define MMC_HIGH_26_MAX_DTR	26000000
#define MMC_HIGH_52_MAX_DTR	52000000
#define MMC_HIGH_DDR_MAX_DTR	52000000
#define MMC_HS200_MAX_DTR	200000000
	unsigned int		sectors;
	unsigned int		hc_erase_size;		/* In sectors */
	unsigned int		hc_erase_timeout;	/* In milliseconds */
	unsigned int		sec_trim_mult;	/* Secure trim multiplier  */
	unsigned int		sec_erase_mult;	/* Secure erase multiplier */
	unsigned int		trim_timeout;		/* In milliseconds */
	u8			enhanced_area_en;	/* enable bit */
	unsigned long long	enhanced_area_offset;	/* Units: Byte */
	unsigned int		enhanced_area_size;	/* Units: KB */
	unsigned int		cache_size;		/* Units: KB */
	u8			hpi_en;			/* HPI enablebit */
	u8			hpi;			/* HPI support bit */
	unsigned int		hpi_cmd;		/* cmd used as HPI */
	u8			bkops;		/* background support bit */
	u8			bkops_en;	/* background enable bit */
	unsigned int            data_sector_size;       /* 512 bytes or 4KB */
	unsigned int            data_tag_unit_size;     /* DATA TAG UNIT size */
	unsigned int		boot_ro_lock;		/* ro lock support */
	u8			boot_ro_lockable;
	u8			raw_exception_status;	/* 54 */
	u8			raw_partition_support;	/* 160 */
	u8			raw_rpmb_size_mult;	/* 168 */
	u8			raw_erased_mem_count;	/* 181 */
	u8			raw_ext_csd_structure;	/* 194 */
	u8			raw_card_type;		/* 196 */
	u8			out_of_int_time;	/* 198 */
	u8			raw_pwr_cl_52_195;	/* 200 */
	u8			raw_pwr_cl_26_195;	/* 201 */
	u8			raw_pwr_cl_52_360;	/* 202 */
	u8			raw_pwr_cl_26_360;	/* 203 */
	u8			raw_s_a_timeout;	/* 217 */
	u8			raw_hc_erase_gap_size;	/* 221 */
	u8			raw_erase_timeout_mult;	/* 223 */
	u8			raw_hc_erase_grp_size;	/* 224 */
	u8			raw_sec_trim_mult;	/* 229 */
	u8			raw_sec_erase_mult;	/* 230 */
	u8			raw_sec_feature_support;/* 231 */
	u8			raw_trim_mult;		/* 232 */
	u8			raw_pwr_cl_200_195;	/* 236 */
	u8			raw_pwr_cl_200_360;	/* 237 */
	u8			raw_pwr_cl_ddr_52_195;	/* 238 */
	u8			raw_pwr_cl_ddr_52_360;	/* 239 */
	u8			raw_pwr_cl_ddr_200_360;	/* 253 */
	u8			raw_bkops_status;	/* 246 */
	u8			raw_sectors[4];		/* 212 - 4 bytes */

	unsigned int            feature_support;
#define MMC_DISCARD_FEATURE	BIT(0)                  /* CMD38 feature */
};

struct mmc *mmc_create(const struct mmc_config *cfg, void *priv);

/**
 * mmc_bind() - Set up a new MMC device ready for probing
 *
 * A child block device is bound with the IF_TYPE_MMC interface type. This
 * allows the device to be used with CONFIG_BLK
 *
 * @dev:	MMC device to set up
 * @mmc:	MMC struct
 * @cfg:	MMC configuration
 * @return 0 if OK, -ve on error
 */
int mmc_bind(struct udevice *dev, struct mmc *mmc,
	     const struct mmc_config *cfg);
void mmc_destroy(struct mmc *mmc);

/**
 * mmc_unbind() - Unbind a MMC device's child block device
 *
 * @dev:	MMC device
 * @return 0 if OK, -ve on error
 */
int mmc_unbind(struct udevice *dev);
int mmc_initialize(bd_t *bis);
int mmc_init(struct mmc *mmc);
int mmc_send_tuning(struct mmc *mmc, u32 opcode, int *cmd_error);

/**
 * mmc_of_parse() - Parse the device tree to get the capabilities of the host
 *
 * @dev:	MMC device
 * @cfg:	MMC configuration
 * @return 0 if OK, -ve on error
 */
int mmc_of_parse(struct udevice *dev, struct mmc_config *cfg);

int mmc_read(struct mmc *mmc, u64 src, uchar *dst, int size);

/**
 * mmc_voltage_to_mv() - Convert a mmc_voltage in mV
 *
 * @voltage:	The mmc_voltage to convert
 * @return the value in mV if OK, -EINVAL on error (invalid mmc_voltage value)
 */
int mmc_voltage_to_mv(enum mmc_voltage voltage);

/**
 * mmc_set_clock() - change the bus clock
 * @mmc:	MMC struct
 * @clock:	bus frequency in Hz
 * @disable:	flag indicating if the clock must on or off
 * @return 0 if OK, -ve on error
 */
int mmc_set_clock(struct mmc *mmc, uint clock, bool disable);
int mmc_set_bus_width(struct mmc *mmc, uint width);

#define MMC_CLK_ENABLE		false
#define MMC_CLK_DISABLE		true

struct mmc *find_mmc_device(int dev_num);
int mmc_set_dev(int dev_num);
void print_mmc_devices(char separator);

/**
 * get_mmc_num() - get the total MMC device number
 *
 * @return 0 if there is no MMC device, else the number of devices
 */
int get_mmc_num(void);
int mmc_switch_part(struct mmc *mmc, unsigned int part_num);
int mmc_hwpart_config(struct mmc *mmc, const struct mmc_hwpart_conf *conf,
		      enum mmc_hwpart_conf_mode mode);

#if !CONFIG_IS_ENABLED(DM_MMC)
int mmc_getcd(struct mmc *mmc);
int board_mmc_getcd(struct mmc *mmc);
int mmc_getwp(struct mmc *mmc);
int board_mmc_getwp(struct mmc *mmc);
#endif

int mmc_set_dsr(struct mmc *mmc, u16 val);
/* Function to change the size of boot partition and rpmb partitions */
int mmc_boot_partition_size_change(struct mmc *mmc, unsigned long bootsize,
					unsigned long rpmbsize);
/* Function to modify the PARTITION_CONFIG field of EXT_CSD */
int mmc_set_part_conf(struct mmc *mmc, u8 ack, u8 part_num, u8 access);
/* Function to modify the BOOT_BUS_WIDTH field of EXT_CSD */
int mmc_set_boot_bus_width(struct mmc *mmc, u8 width, u8 reset, u8 mode);
/* Function to modify the RST_n_FUNCTION field of EXT_CSD */
int mmc_set_rst_n_function(struct mmc *mmc, u8 enable);
/* Functions to read / write the RPMB partition */
int mmc_rpmb_set_key(struct mmc *mmc, void *key);
int mmc_rpmb_get_counter(struct mmc *mmc, unsigned long *counter);
int mmc_rpmb_read(struct mmc *mmc, void *addr, unsigned short blk,
		  unsigned short cnt, unsigned char *key);
int mmc_rpmb_write(struct mmc *mmc, void *addr, unsigned short blk,
		   unsigned short cnt, unsigned char *key);
#ifdef CONFIG_CMD_BKOPS_ENABLE
int mmc_set_bkops_enable(struct mmc *mmc);
#endif

/**
 * Start device initialization and return immediately; it does not block on
 * polling OCR (operation condition register) status.  Then you should call
 * mmc_init, which would block on polling OCR status and complete the device
 * initializatin.
 *
 * @param mmc	Pointer to a MMC device struct
 * @return 0 on success, IN_PROGRESS on waiting for OCR status, <0 on error.
 */
int mmc_start_init(struct mmc *mmc);

/**
 * Set preinit flag of mmc device.
 *
 * This will cause the device to be pre-inited during mmc_initialize(),
 * which may save boot time if the device is not accessed until later.
 * Some eMMC devices take 200-300ms to init, but unfortunately they
 * must be sent a series of commands to even get them to start preparing
 * for operation.
 *
 * @param mmc		Pointer to a MMC device struct
 * @param preinit	preinit flag value
 */
void mmc_set_preinit(struct mmc *mmc, int preinit);

#ifdef CONFIG_MMC_SPI
#define mmc_host_is_spi(mmc)	((mmc)->cfg->host_caps & MMC_MODE_SPI)
#else
#define mmc_host_is_spi(mmc)	0
#endif
struct mmc *mmc_spi_init(uint bus, uint cs, uint speed, uint mode);

void board_mmc_power_init(void);
int board_mmc_init(bd_t *bis);
int cpu_mmc_init(bd_t *bis);
int mmc_get_env_addr(struct mmc *mmc, int copy, u32 *env_addr);
int mmc_get_env_dev(void);

/* Set block count limit because of 16 bit register limit on some hardware*/
#ifndef CONFIG_SYS_MMC_MAX_BLK_COUNT
#define CONFIG_SYS_MMC_MAX_BLK_COUNT 65535
#endif

/**
 * mmc_get_blk_desc() - Get the block descriptor for an MMC device
 *
 * @mmc:	MMC device
 * @return block device if found, else NULL
 */
struct blk_desc *mmc_get_blk_desc(struct mmc *mmc);

int mmc_send_manual_stop(struct mmc *mmc);
int mmc_send_ext_csd(struct mmc *mmc, u8 *ext_csd);
int sunxi_need_rty(struct mmc *mmc);
int sunxi_mmc_tuning_init(void);
int sunxi_write_tuning(struct mmc *mmc);
int sunxi_bus_tuning(struct mmc *mmc);
int sunxi_mmc_tuning_exit(void);
int sunxi_switch_to_best_bus(struct mmc *mmc);

int mmc_exit(void);
void mmc_update_config_for_dragonboard(int card_no);
void mmc_set_mmcblckx(const char *node_name);
void mmc_update_config_for_sdly(struct mmc *mmc);
unsigned int sunxi_select_freq(struct mmc *mmc, int speed_md,
			int freq_index);
#endif /* _MMC_H_ */
