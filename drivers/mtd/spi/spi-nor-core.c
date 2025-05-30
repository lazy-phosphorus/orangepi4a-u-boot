// SPDX-License-Identifier: GPL-2.0
/*
 * Based on m25p80.c, by Mike Lavender (mike@steroidmicros.com), with
 * influence from lart.c (Abraham Van Der Merwe) and mtd_dataflash.c
 *
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 *
 * Synced from Linux v4.19
 */

#include <common.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/sizes.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/spi-nor.h>
#include <spi-mem.h>
#include <spi.h>
#include <sunxi_board.h>

#include "sf_internal.h"
#include <fdt_support.h>
#include <private_boot0.h>
#include <boot_param.h>
#include "../../spi/spi-sunxi.h"
#include "../../spi/spif-sunxi.h"

/* Define max times to check status register before we give up. */

/*
 * For everything but full-chip erase; probably could be much smaller, but kept
 * around for safety for now
 */

#define HZ					CONFIG_SYS_HZ

#define DEFAULT_READY_WAIT_JIFFIES		(40UL * HZ)

#define BP_SZ_4K       (4 * 1024)
#define BP_SZ_32K      (32 * 1024)
#define BP_SZ_64K      (64 * 1024)
#define BP_SZ_128K     (128 * 1024)
#define BP_SZ_256K     (256 * 1024)
#define BP_SZ_512K     (512 * 1024)
#define BP_SZ_1M       (1 * 1024 * 1024)
#define BP_SZ_2M       (2 * 1024 * 1024)
#define BP_SZ_4M       (4 * 1024 * 1024)
#define BP_SZ_6M       (6 * 1024 * 1024)
#define BP_SZ_7M       (7 * 1024 * 1024)
#define BP_SZ_7680K	(7680 * 1024)
#define BP_SZ_7936K	(7936 * 1024)
#define BP_SZ_8064K	(8064 * 1024)
#define BP_SZ_8128K	(8128 * 1024)
#define BP_SZ_8M       (8 * 1024 * 1024)
#define BP_SZ_12M      (12 * 1024 * 1024)
#define BP_SZ_14M      (14 * 1024 * 1024)
#define BP_SZ_15M      (15 * 1024 * 1024)
#define BP_SZ_15872K   (15872 * 1024)
#define BP_SZ_16128K   (16128 * 1024)
#define BP_SZ_16M      (16 * 1024 * 1024)
#define BP_SZ_32M      (32 * 1024 * 1024)
#define BP_SZ_64M      (64 * 1024 * 1024)

#define NOR_LOCK_BLOCK_SIZE  (64 * 1024)
#define NOR_LOCK_SECTOR_SIZE (4 * 1024)

/* TB bit is OTP (one times program) */
struct nor_protection mxic_protection[] = {
    { .boundary = 0, .bp = 0, .flag = 0 },
    { .boundary = BP_SZ_64K, .bp = BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_128K, .bp = BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_256K, .bp = BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_512K, .bp = BIT(2), .flag = SET_TB },
    { .boundary = BP_SZ_1M, .bp = BIT(2) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_2M, .bp = BIT(2) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_4M, .bp = BIT(2) | BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_8M, .bp = BIT(3), .flag = SET_TB },
    { .boundary = BP_SZ_16M, .bp = BIT(3) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_32M, .bp = BIT(3) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_64M, .bp = BIT(3) | BIT(1) | BIT(0), .flag = SET_TB },
};

struct nor_protection gd_protection[] = {
    { .boundary = 0, .bp = 0, .flag = 0 },
    { .boundary = BP_SZ_256K, .bp = BIT(3) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_512K, .bp = BIT(3) | BIT(1), .flag = 0 },
    { .boundary = BP_SZ_1M, .bp = BIT(3) | BIT(1) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_2M, .bp = BIT(3) | BIT(2), .flag = 0 },
    { .boundary = BP_SZ_4M, .bp = BIT(3) | BIT(2) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_8M, .bp = BIT(3) | BIT(2) | BIT(1), .flag = 0 },
    { .boundary = BP_SZ_12M, .bp = BIT(2) | BIT(0), .flag = SET_CMP },
    { .boundary = BP_SZ_14M, .bp = BIT(2), .flag = SET_CMP },
    { .boundary = BP_SZ_15M, .bp = BIT(1) | BIT(0), .flag = SET_CMP },
    { .boundary = BP_SZ_15872K, .bp = BIT(1), .flag = SET_CMP },
    { .boundary = BP_SZ_16128K, .bp = BIT(0), .flag = SET_CMP },
    { .boundary = BP_SZ_16M, .bp = BIT(2) | BIT(1) | BIT(0), .flag = 0},
};

struct nor_protection esmt_protection[] = {
    { .boundary = 0, .bp = 0, .flag = 0 },
    { .boundary = BP_SZ_256K, .bp = BIT(3) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_512K, .bp = BIT(3) | BIT(1), .flag = 0 },
    { .boundary = BP_SZ_1M, .bp = BIT(3) | BIT(1) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_2M, .bp = BIT(3) | BIT(2), .flag = 0 },
    { .boundary = BP_SZ_4M, .bp = BIT(3) | BIT(2) | BIT(0), .flag = 0 },
    { .boundary = BP_SZ_8M, .bp = BIT(3) | BIT(2) | BIT(1), .flag = 0 },
#if 0 /* TB in otp zone */
    { .boundary = BP_SZ_12M, .bp = BIT(2) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_14M, .bp = BIT(2), .flag = SET_TB },
    { .boundary = BP_SZ_15M, .bp = BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_15872K, .bp = BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_16128K, .bp = BIT(0), .flag = SET_TB },
#endif
    { .boundary = BP_SZ_16M, .bp = BIT(3) | BIT(2) | BIT(1) | BIT(0), .flag = 0 },
};

struct nor_protection esmt_protection_8M[] = {
    { .boundary = 0, .bp = 0, .flag = 0 },
    { .boundary = BP_SZ_64K, .bp = BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_128K, .bp = BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_256K, .bp = BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_512K, .bp = BIT(2), .flag = SET_TB },
    { .boundary = BP_SZ_1M, .bp = BIT(2) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_2M, .bp = BIT(2) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_4M, .bp = BIT(2) | BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_6M, .bp = BIT(3) | BIT(2) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_7M, .bp = BIT(3) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_7680K, .bp = BIT(3) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_7936K, .bp = BIT(3) | BIT(1) | BIT(0), .flag = SET_TB },
    { .boundary = BP_SZ_8064K, .bp = BIT(3) | BIT(2), .flag = SET_TB },
    { .boundary = BP_SZ_8128K, .bp = BIT(3) | BIT(2) | BIT(1), .flag = SET_TB },
    { .boundary = BP_SZ_8M, .bp = BIT(3) | BIT(2) | BIT(1) | BIT(0), .flag = SET_TB },
};

static int spi_nor_read_write_reg(struct spi_nor *nor, struct spi_mem_op
		*op, void *buf)
{
	if (op->data.dir == SPI_MEM_DATA_IN)
		op->data.buf.in = buf;
	else
		op->data.buf.out = buf;
	return spi_mem_exec_op(nor->spi, op);
}

static int spi_nor_read_reg(struct spi_nor *nor, u8 code, u8 *val, int len)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(code, 1),
					  SPI_MEM_OP_NO_ADDR,
					  SPI_MEM_OP_NO_MODE,
					  SPI_MEM_OP_NO_DUMMY,
					  SPI_MEM_OP_DATA_IN(len, NULL, 1));
	int ret;

	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	op.dummy.buswidth = 1;
	op.data.buswidth = op.cmd.buswidth;

	if (op.cmd.buswidth == 8)
		op.dummy.cycle = 8;

	ret = spi_nor_read_write_reg(nor, &op, val);
	if (ret < 0)
		dev_dbg(&flash->spimem->spi->dev, "error %d reading %x\n", ret,
			code);

	return ret;
}

#ifdef CONFIG_SUNXI_SPIF
static int spif_nor_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 1),
				   SPI_MEM_OP_NO_ADDR,
				   SPI_MEM_OP_MODE(buf, 1),
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_NO_DATA);

	/* get transfer protocols. */
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->write_proto);
	op.mode.buswidth = op.cmd.buswidth;

	return spi_mem_exec_op(nor->spi, &op);
}

static int spif_nor_write_reg_2byte(struct spi_nor *nor, u8 opcode, u8 *buf)
{
	loff_t cmd_sr_cr = (opcode << 16) | (buf[0] << 8) | buf[1];
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_NO_CMD,
				   SPI_MEM_OP_ADDR(3, cmd_sr_cr, 1),
				   SPI_MEM_OP_NO_MODE,
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_NO_DATA);

	/* The command line width is the same as the address line width. */
	op.addr.buswidth = spi_nor_get_protocol_inst_nbits(nor->write_proto);

	return spi_mem_exec_op(nor->spi, &op);
}

static ssize_t spi_nor_io_read_data(struct spi_nor *nor, loff_t from, size_t len,
				 u_char *buf)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(nor->read_opcode, 1),
				   SPI_MEM_OP_ADDR(nor->addr_width, from, 1),
				   SPI_MEM_OP_MODE(&nor->mode, 1),
				   SPI_MEM_OP_DUMMY(nor->read_dummy, 1),
				   SPI_MEM_OP_DATA_IN(len, buf, 1));
	size_t remaining = len;
	int ret;

	/* get transfer protocols. */
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	op.addr.buswidth = spi_nor_get_protocol_addr_nbits(nor->read_proto);
	op.mode.buswidth = op.addr.buswidth;
	op.dummy.buswidth = op.addr.buswidth;
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->read_proto);

	while (remaining) {
		op.data.nbytes = remaining < UINT_MAX ? remaining : UINT_MAX;
		ret = spi_mem_adjust_op_size(nor->spi, &op);
		if (ret)
			return ret;

		ret = spi_mem_exec_op(nor->spi, &op);
		if (ret)
			return ret;

		op.addr.val += op.data.nbytes;
		remaining -= op.data.nbytes;
		op.data.buf.in += op.data.nbytes;
	}

	return len;
}
#else
static int spi_nor_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 1),
					  SPI_MEM_OP_NO_ADDR,
					  SPI_MEM_OP_NO_MODE,
					  SPI_MEM_OP_NO_DUMMY,
					  SPI_MEM_OP_DATA_OUT(len, NULL, 1));

	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->read_proto);

	return spi_nor_read_write_reg(nor, &op, buf);
}
#endif

static ssize_t spi_nor_read_data(struct spi_nor *nor, loff_t from, size_t len,
				 u_char *buf)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(nor->read_opcode, 1),
				   SPI_MEM_OP_ADDR(nor->addr_width, from, 1),
				   SPI_MEM_OP_NO_MODE,
				   SPI_MEM_OP_DUMMY(nor->read_dummy, 1),
				   SPI_MEM_OP_DATA_IN(len, buf, 1));
	size_t remaining = len;
	int ret;

	/* get transfer protocols. */
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	op.addr.buswidth = spi_nor_get_protocol_addr_nbits(nor->read_proto);
	op.dummy.buswidth = op.addr.buswidth;
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->read_proto);

	while (remaining) {
		op.data.nbytes = remaining < UINT_MAX ? remaining : UINT_MAX;
		ret = spi_mem_adjust_op_size(nor->spi, &op);
		if (ret)
			return ret;

		ret = spi_mem_exec_op(nor->spi, &op);
		if (ret)
			return ret;

		op.addr.val += op.data.nbytes;
		remaining -= op.data.nbytes;
		op.data.buf.in += op.data.nbytes;
	}

	return len;
}

static ssize_t spi_nor_write_data(struct spi_nor *nor, loff_t to, size_t len,
				  const u_char *buf)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(nor->program_opcode, 1),
				   SPI_MEM_OP_ADDR(nor->addr_width, to, 1),
				   SPI_MEM_OP_NO_MODE,
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_OUT(len, buf, 1));
	int ret;

	/* get transfer protocols. */
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->write_proto);
	op.addr.buswidth = spi_nor_get_protocol_addr_nbits(nor->write_proto);
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->write_proto);

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		op.addr.nbytes = 0;

	ret = spi_mem_adjust_op_size(nor->spi, &op);
	if (ret)
		return ret;
	op.data.nbytes = len < op.data.nbytes ? len : op.data.nbytes;

	ret = spi_mem_exec_op(nor->spi, &op);
	if (ret)
		return ret;

	return op.data.nbytes;
}

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR, &val, 1);
	if (ret < 0) {
		pr_debug("error %d reading SR\n", (int)ret);
		return ret;
	}

	return val;
}

static int read_sr2(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR2, &val, 1);
	if (ret < 0) {
		pr_debug("error %d reading SR\n", (int)ret);
		return ret;
	}

	return val;
}

static int read_sr3(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR3, &val, 1);
	if (ret < 0) {
		pr_debug("error %d reading SR\n", (int)ret);
		return ret;
	}

	return val;
}

/*
 * Read the flag status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_fsr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDFSR, &val, 1);
	if (ret < 0) {
		pr_debug("error %d reading FSR\n", ret);
		return ret;
	}

	return val;
}

/*
 * Read configuration register, returning its value in the
 * location. Return the configuration register value.
 * Returns negative if error occurred.
 */
#if defined(CONFIG_SPI_FLASH_SPANSION) || defined(CONFIG_SPI_FLASH_WINBOND)   \
    || defined(CONFIG_SPI_FLASH_PUYA) || defined(CONFIG_SPI_FLASH_GIGADEVICE)
static int read_cr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDCR, &val, 1);
	if (ret < 0) {
		dev_dbg(nor->dev, "error %d reading CR\n", ret);
		return ret;
	}

	return val;
}
#endif

/*
 * Read MXIC security register, returning its value in
 * the location. Return the configuration register value.
 * Returns negative if error occurred.
 */
static int read_mxic_sr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSCUR, &val, 1);
	if (ret < 0) {
		pr_debug("error %d reading MXIC scur\n", (int)ret);
		return ret;
	}

	return val;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static int write_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
}

/*
 * Send write disable instruction to the chip.
 */
static int write_disable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WRDI, NULL, 0);
}

static struct spi_nor *mtd_to_spi_nor(struct mtd_info *mtd)
{
	return mtd->priv;
}

#ifndef CONFIG_SPI_FLASH_BAR
static u8 spi_nor_convert_opcode(u8 opcode, const u8 table[][2], size_t size)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (table[i][0] == opcode)
			return table[i][1];

	/* No conversion found, keep input op code. */
	return opcode;
}

static u8 spi_nor_convert_3to4_read(u8 opcode)
{
	static const u8 spi_nor_3to4_read[][2] = {
		{ SPINOR_OP_READ,	SPINOR_OP_READ_4B },
		{ SPINOR_OP_READ_FAST,	SPINOR_OP_READ_FAST_4B },
		{ SPINOR_OP_READ_1_1_2,	SPINOR_OP_READ_1_1_2_4B },
		{ SPINOR_OP_READ_1_2_2,	SPINOR_OP_READ_1_2_2_4B },
		{ SPINOR_OP_READ_1_1_4,	SPINOR_OP_READ_1_1_4_4B },
		{ SPINOR_OP_READ_1_1_8,	SPINOR_OP_READ_1_1_8_4B },
		{ SPINOR_OP_READ_1_4_4,	SPINOR_OP_READ_1_4_4_4B },

		{ SPINOR_OP_READ_1_1_1_DTR,	SPINOR_OP_READ_1_1_1_DTR_4B },
		{ SPINOR_OP_READ_1_2_2_DTR,	SPINOR_OP_READ_1_2_2_DTR_4B },
		{ SPINOR_OP_READ_1_4_4_DTR,	SPINOR_OP_READ_1_4_4_DTR_4B },
	};

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_read,
				      ARRAY_SIZE(spi_nor_3to4_read));
}

static u8 spi_nor_convert_3to4_program(u8 opcode)
{
	static const u8 spi_nor_3to4_program[][2] = {
		{ SPINOR_OP_PP,		SPINOR_OP_PP_4B },
		{ SPINOR_OP_PP_1_1_4,	SPINOR_OP_PP_1_1_4_4B },
		{ SPINOR_OP_PP_1_1_8,	SPINOR_OP_PP_1_1_8_4B },
		{ SPINOR_OP_PP_1_4_4,	SPINOR_OP_PP_1_4_4_4B },
	};

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_program,
				      ARRAY_SIZE(spi_nor_3to4_program));
}

static u8 spi_nor_convert_3to4_erase(u8 opcode)
{
	static const u8 spi_nor_3to4_erase[][2] = {
		{ SPINOR_OP_BE_4K,	SPINOR_OP_BE_4K_4B },
		{ SPINOR_OP_BE_32K,	SPINOR_OP_BE_32K_4B },
		{ SPINOR_OP_SE,		SPINOR_OP_SE_4B },
	};

	return spi_nor_convert_opcode(opcode, spi_nor_3to4_erase,
				      ARRAY_SIZE(spi_nor_3to4_erase));
}

static void spi_nor_set_4byte_opcodes(struct spi_nor *nor,
				      const struct flash_info *info)
{
	/* Do some manufacturer fixups first */
	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_SPANSION:
		/* No small sector erase for 4-byte command set */
		nor->erase_opcode = SPINOR_OP_SE;
		nor->mtd.erasesize = info->sector_size;
		break;

	default:
		break;
	}

	nor->read_opcode = spi_nor_convert_3to4_read(nor->read_opcode);
	nor->program_opcode = spi_nor_convert_3to4_program(nor->program_opcode);
	nor->erase_opcode = spi_nor_convert_3to4_erase(nor->erase_opcode);
}
#endif /* !CONFIG_SPI_FLASH_BAR */
/* Reset enable(66h) and Reset Device(99h)*/
void spi_nor_reset_device(struct spi_nor *nor)
{
	nor->write_reg(nor, SPINOR_OP_RESTEN, NULL, 0);
	nor->write_reg(nor, SPINOR_OP_RESET, NULL, 0);
}

/* Enable/disable 4-byte addressing mode. */
static int set_4byte(struct spi_nor *nor, const struct flash_info *info,
		     int enable)
{
	int status;
	bool need_wren = false;
	u8 cmd;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_ST:
	case SNOR_MFR_MICRON:
		/* Some Micron need WREN command; all will accept it */
		need_wren = true;
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_WINBOND:
		if (need_wren)
			write_enable(nor);

		cmd = enable ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
		status = nor->write_reg(nor, cmd, NULL, 0);
		if (need_wren)
			write_disable(nor);

		if (!status && !enable &&
		    JEDEC_MFR(info) == SNOR_MFR_WINBOND) {
			/*
			 * On Winbond W25Q256FV, leaving 4byte mode causes
			 * the Extended Address Register to be set to 1, so all
			 * 3-byte-address reads come from the second 16M.
			 * We must clear the register to enable normal behavior.
			 */
			write_enable(nor);
			nor->cmd_buf[0] = 0;
			nor->write_reg(nor, SPINOR_OP_WREAR, nor->cmd_buf, 1);
			write_disable(nor);
		}

		return status;
	default:
		/* Spansion style */
		nor->cmd_buf[0] = enable << 7;
		return nor->write_reg(nor, SPINOR_OP_BRWR, nor->cmd_buf, 1);
	}
}

static int spi_nor_sr_ready(struct spi_nor *nor)
{
	int sr = read_sr(nor);

	if (sr < 0)
		return sr;

	if (nor->flags & SNOR_F_USE_CLSR && sr & (SR_E_ERR | SR_P_ERR)) {
		if (sr & SR_E_ERR)
			dev_dbg(nor->dev, "Erase Error occurred\n");
		else
			dev_dbg(nor->dev, "Programming Error occurred\n");

		nor->write_reg(nor, SPINOR_OP_CLSR, NULL, 0);
		return -EIO;
	}

	return !(sr & SR_WIP);
}

static int spi_nor_fsr_ready(struct spi_nor *nor)
{
	int fsr = read_fsr(nor);

	if (fsr < 0)
		return fsr;

	if (fsr & (FSR_E_ERR | FSR_P_ERR)) {
		if (fsr & FSR_E_ERR)
			dev_dbg(nor->dev, "Erase operation failed.\n");
		else
			dev_dbg(nor->dev, "Program operation failed.\n");

		if (fsr & FSR_PT_ERR)
			dev_dbg(nor->dev,
				"Attempted to modify a protected sector.\n");

		nor->write_reg(nor, SPINOR_OP_CLFSR, NULL, 0);
		return -EIO;
	}

	return fsr & FSR_READY;
}

static int spi_nor_ready(struct spi_nor *nor)
{
	int sr, fsr;

	sr = spi_nor_sr_ready(nor);
	if (sr < 0)
		return sr;
	fsr = nor->flags & SNOR_F_USE_FSR ? spi_nor_fsr_ready(nor) : 1;
	if (fsr < 0)
		return fsr;
	return sr && fsr;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int spi_nor_wait_till_ready_with_timeout(struct spi_nor *nor,
						unsigned long timeout)
{
	unsigned long timebase;
	int ret;

	timebase = get_timer(0);

	while (get_timer(timebase) < timeout) {
		ret = spi_nor_ready(nor);
		if (ret < 0)
			return ret;
		if (ret)
			return 0;
	}

	dev_err(nor->dev, "flash operation timed out\n");

	return -ETIMEDOUT;
}

static int spi_nor_wait_till_ready(struct spi_nor *nor)
{
	return spi_nor_wait_till_ready_with_timeout(nor,
						    DEFAULT_READY_WAIT_JIFFIES);
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct spi_nor *nor, u8 val)
{
	nor->cmd_buf[0] = val;
	return nor->write_reg(nor, SPINOR_OP_WRSR, nor->cmd_buf, 1);
}

static int write_sr2(struct spi_nor *nor, u8 val)
{
	int ret;

	nor->cmd_buf[0] = val;
	write_enable(nor);
	ret = nor->write_reg(nor, SPINOR_OP_WRSR2, nor->cmd_buf, 1);
	if (ret < 0) {
		dev_dbg(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret) {
		dev_dbg(nor->dev,
			"timeout while writing configuration register\n");
		return ret;
	}

	return 0;
}

static int write_sr3(struct spi_nor *nor, u8 val)
{
	int ret;

	nor->cmd_buf[0] = val;
	write_enable(nor);
	ret = nor->write_reg(nor, SPINOR_OP_WRSR3, nor->cmd_buf, 1);
	if (ret < 0) {
		dev_dbg(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret) {
		dev_dbg(nor->dev,
			"timeout while writing configuration register\n");
		return ret;
	}

	return 0;
}

/*
 * Write status Register and configuration register with 2 bytes
 * The first byte will be written to the status register, while the
 * second byte will be written to the configuration register.
 * Return negative if error occurred.
 */
static int write_sr_cr(struct spi_nor *nor, u8 *sr_cr)
{
	int ret;

	write_enable(nor);

#ifdef CONFIG_SUNXI_SPIF
	ret = spif_nor_write_reg_2byte(nor, SPINOR_OP_WRSR, sr_cr);
#else
	ret = nor->write_reg(nor, SPINOR_OP_WRSR, sr_cr, 2);
#endif
	if (ret < 0) {
		dev_dbg(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret) {
		dev_dbg(nor->dev,
			"timeout while writing configuration register\n");
		return ret;
	}

	return 0;
}

static int enter_otp(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_ENTER_OTP, NULL, 0);
}

static int exit_otp(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_EXIT_OTP, NULL, 0);
}

static int write_vsr_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WREN_VSR, NULL, 0);
}

static int write_otp_sr(struct spi_nor *nor, u8 val)
{
	int ret;
	ret = enter_otp(nor);
	if (ret)
		return ret;

	ret = write_vsr_enable(nor);
	if (ret)
		goto exit_otp;

	ret = write_sr(nor, val);
	if (ret)
		goto exit_otp;

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto exit_otp;
exit_otp:
	exit_otp(nor);
	return ret;
}

static int read_otp_sr(struct spi_nor *nor)
{
	int ret;
	int val;

	ret = enter_otp(nor);
	if (ret)
		return ret;

	ret = write_vsr_enable(nor);
	if (ret)
		return ret;

	val = read_sr(nor);
	if (val < 0) {
		exit_otp(nor);
		return val;
	}

	ret = exit_otp(nor);
	if (ret)
		return ret;

	return val | 0xff;
}

#ifdef CONFIG_SPI_FLASH_EON
static int read_cr_EON(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDCR_EON, &val, 1);
	if (ret < 0) {
		dev_dbg(nor->dev, "error %d reading CR\n", ret);
		return ret;
	}

	return val;
}

static int Eon_quad_enable(struct spi_nor *nor)
{
	int ret, val;
	printf("EON device ID:%x,enable quad\n", JEDEC_ID(nor->info));
	if ((JEDEC_ID(nor->info) == 0x7018) || (JEDEC_ID(nor->info) == 0x7017)) {
		/*
		 * EN25QH128A no QE bit,you should enter otp mode,
		 * than you can see WXDIS bit on status register.
		 * Set it and enable quad mode.
		 */
		enter_otp(nor);
		val = read_sr(nor);
		if (val < 0) {
			exit_otp(nor);
			return val;
		}
		if (val & SR_OTP_WXDIS_EN_EON) {
			exit_otp(nor);
			return 0;
		}
		write_enable(nor);
		write_sr(nor, val | SR_OTP_WXDIS_EN_EON);

		ret = spi_nor_wait_till_ready(nor);
		if (ret) {
			exit_otp(nor);
			return ret;
		}

		ret = read_sr(nor);
		if (!(ret > 0 && (ret & SR_OTP_WXDIS_EN_EON))) {
			exit_otp(nor);
			dev_err(nor->dev, "ESMT WXDIS bit not set\n");
			return -EINVAL;
		}

		exit_otp(nor);

	} else {
		/*
		 * other ESMT nor still enable quad mode by setting
		 * QE bit,use 31h to write control register.
		 *
		 */
		val = read_cr_EON(nor);
		if (val < 0)
			return val;
		if (val & CR_QUAD_EN_EON)
			return 0;

		write_enable(nor);
		write_sr2(nor, val | CR_QUAD_EN_EON);

		ret = read_cr_EON(nor);
		if (!(ret > 0 && (ret & CR_QUAD_EN_EON))) {
			dev_err(nor->dev, "EON Quad bit not set\n");
			return -EINVAL;
		}
	}
	return 0;

}
#endif

#ifdef CONFIG_SPI_FLASH_GIGADEVICE
static int write_sr_gd(struct spi_nor *nor, u8 val)
{
	nor->cmd_buf[0] = val;
	return nor->write_reg(nor, SPINOR_OP_WRSR2, nor->cmd_buf, 1);
}

static int gd_read_cr_quad_enable(struct spi_nor *nor)
{
	int ret, val;

	val = read_cr(nor);
	if (val < 0)
		return val;
	if (val & CR_QUAD_EN_GD)
		return 0;

	write_enable(nor);

	write_sr_gd(nor, val | CR_QUAD_EN_GD);

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;

	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_GD))) {
		dev_err(nor->dev, "Gd Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}
static int gigadevice_config_cmp(struct spi_nor *nor, int cmp)
{
	int ret = 0, sr2_old, sr2_new;
	sr2_old = read_sr2(nor);
	if (sr2_old < 0)
		return sr2_old;

	if (cmp)
		sr2_new = sr2_old | SR2_CMP_GD;
	else
		sr2_new = sr2_old & ~SR2_CMP_GD;

	if (sr2_old != sr2_new) {
		write_enable(nor);
		ret = write_sr2(nor, sr2_new & 0xff);
		spi_nor_wait_till_ready(nor);
	}
	dev_dbg(nor->dev, "sr2:0x%x --> 0x%x\n", sr2_old, sr2_new);

	return ret;
}
#endif

#ifdef CONFIG_SPI_FLASH_BAR
/*
 * This "clean_bar" is necessary in a situation when one was accessing
 * spi flash memory > 16 MiB by using Bank Address Register's BA24 bit.
 *
 * After it the BA24 bit shall be cleared to allow access to correct
 * memory region after SW reset (by calling "reset" command).
 *
 * Otherwise, the BA24 bit may be left set and then after reset, the
 * ROM would read/write/erase SPL from 16 MiB * bank_sel address.
 */
static int clean_bar(struct spi_nor *nor)
{
	u8 cmd, bank_sel = 0;

	if (nor->bank_curr == 0)
		return 0;
	cmd = nor->bank_write_cmd;
	nor->bank_curr = 0;
	write_enable(nor);

	return nor->write_reg(nor, cmd, &bank_sel, 1);
}

static int write_bar(struct spi_nor *nor, u32 offset)
{
	u8 cmd, bank_sel;
	int ret;

	bank_sel = offset / SZ_16M;
	if (bank_sel == nor->bank_curr)
		goto bar_end;

	cmd = nor->bank_write_cmd;
	write_enable(nor);
	ret = nor->write_reg(nor, cmd, &bank_sel, 1);
	if (ret < 0) {
		debug("SF: fail to write bank register\n");
		return ret;
	}

bar_end:
	nor->bank_curr = bank_sel;
	return nor->bank_curr;
}

static int read_bar(struct spi_nor *nor, const struct flash_info *info)
{
	u8 curr_bank = 0;
	int ret;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_SPANSION:
		nor->bank_read_cmd = SPINOR_OP_BRRD;
		nor->bank_write_cmd = SPINOR_OP_BRWR;
		break;
	default:
		nor->bank_read_cmd = SPINOR_OP_RDEAR;
		nor->bank_write_cmd = SPINOR_OP_WREAR;
	}

	ret = nor->read_reg(nor, nor->bank_read_cmd,
				    &curr_bank, 1);
	if (ret) {
		debug("SF: fail to read bank addr register\n");
		return ret;
	}
	nor->bank_curr = curr_bank;

	return 0;
}
#endif

static int sunxi_select_die(struct spi_nor *nor, u8 select_die)
{
	int ret;

	nor->cmd_buf[0] = select_die;

	if (select_die != nor->active_stack_die) {
		write_enable(nor);
		ret = nor->write_reg(nor, SPINOR_OP_SDS, nor->cmd_buf, 1);
		if (ret < 0) {
			dev_dbg(nor->dev,
				"error while writing configuration register\n");
			return -EINVAL;
		}

		ret = spi_nor_wait_till_ready(nor);
		if (ret) {
			dev_dbg(nor->dev,
				"timeout while writing configuration register\n");
			return ret;
		}

		write_disable(nor);
		nor->active_stack_die = select_die;
	}

	return 0;
}

/*
 * Initiate the erasure of a single sector
 */
static int spi_nor_erase_sector(struct spi_nor *nor, u32 addr)
{
	struct spi_mem_op op =
		SPI_MEM_OP(SPI_MEM_OP_CMD(nor->erase_opcode, 1),
			   SPI_MEM_OP_ADDR(nor->addr_width, addr, 1),
			   SPI_MEM_OP_NO_MODE,
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_NO_DATA);

	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->write_proto);
	op.addr.buswidth = op.cmd.buswidth;

	if (nor->erase)
		return nor->erase(nor, addr);

	/*
	 * Default implementation, if driver doesn't have a specialized HW
	 * control
	 */
	return spi_mem_exec_op(nor->spi, &op);
}

/*
 * Erase an address range on the nor chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int spi_nor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	u32 addr, len, rem;
	int ret  = 0;
	u32 single_die_size = 0;

	dev_dbg(nor->dev, "at 0x%llx, len %lld\n", (long long)instr->addr,
		(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	if (nor->info->flags & SPI_NOR_STACK_DIE) {
		u8 total_stack_die = 2; //FIXME
		u8 select_die;

		single_die_size = nor->info->sector_size * nor->info->n_sectors / total_stack_die;
		select_die = addr / single_die_size;

		ret = sunxi_select_die(nor, select_die);
		if (ret)
			goto erase_err;

		addr %= single_die_size;
	}

	while (len) {
#ifdef CONFIG_SPI_FLASH_BAR
		ret = write_bar(nor, addr);
		if (ret < 0)
			return ret;
#endif
		write_enable(nor);

		ret = spi_nor_erase_sector(nor, addr);
		if (ret)
			goto erase_err;

		addr += mtd->erasesize;
		len -= mtd->erasesize;

		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto erase_err;

		if ((nor->info->flags & SPI_NOR_STACK_DIE) && addr >= single_die_size && len) {
			u8 select_die = ++nor->active_stack_die;
			ret = sunxi_select_die(nor, select_die);
			if (ret)
				goto erase_err;
			addr = 0;
		}
	}

erase_err:
#ifdef CONFIG_SPI_FLASH_BAR
	ret = clean_bar(nor);
#endif
	write_disable(nor);
	return ret;
}

static int spi_nor_has_lock_erase(struct mtd_info *mtd,
		struct erase_info *instr)
{
	int ret;

	if (mtd->_unlock(mtd, instr->addr, instr->len))
		dev_err(mtd->dev, "erase unlock error\n");

	ret = spi_nor_erase(mtd, instr);

	mtd->_lock(mtd, 0, mtd->size);

	return ret;
}

/*
 *The Chip Erase instruction clears all bits in the device to be FFh,
 *Return an error is there is a problem erasing.
 */
static int spi_nor_force_erase(struct mtd_info *mtd)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;
	u32 timeout = (mtd->size / 1024 / 1024) * 20 * HZ;
	struct spi_mem_op op =
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_CHIP_ERASE, 1),
			   SPI_MEM_OP_NO_ADDR,
			   SPI_MEM_OP_NO_MODE,
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_NO_DATA);

	if (nor->info->flags & SPI_NOR_STACK_DIE) {
		u8 total_stack_die = 2; //FIXME
		s8 select_die = --total_stack_die;
		while (select_die >= 0) {
			ret = sunxi_select_die(nor, select_die);
			if (ret)
				goto force_erase_err;

			write_enable(nor);

			ret = spi_mem_exec_op(nor->spi, &op);
			if (ret)
				goto force_erase_err;

			ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
			if (ret)
				goto force_erase_err;

			select_die--;
		}
	} else {
		write_enable(nor);

		ret = spi_mem_exec_op(nor->spi, &op);
		if (ret)
			goto force_erase_err;

		ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
		if (ret)
			goto force_erase_err;
	}

force_erase_err:
	write_disable(nor);
	return ret;
}

#if defined(CONFIG_SPI_FLASH_STMICRO) || defined(CONFIG_SPI_FLASH_SST)
/* Write status register and ensure bits in mask match written values */
static int write_sr_and_check(struct spi_nor *nor, u8 status_new, u8 mask)
{
	int ret;

	write_enable(nor);
	ret = write_sr(nor, status_new);
	if (ret)
		return ret;

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;

	ret = read_sr(nor);
	if (ret < 0)
		return ret;

	return ((ret & mask) != (status_new & mask)) ? -EIO : 0;
}

static void stm_get_locked_range(struct spi_nor *nor, u8 sr, loff_t *ofs,
				 uint64_t *len)
{
	struct mtd_info *mtd = &nor->mtd;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	int shift = ffs(mask) - 1;
	int pow;

	if (!(sr & mask)) {
		/* No protection */
		*ofs = 0;
		*len = 0;
	} else {
		pow = ((sr & mask) ^ mask) >> shift;
		*len = mtd->size >> pow;
		if (nor->flags & SNOR_F_HAS_SR_TB && sr & SR_TB)
			*ofs = 0;
		else
			*ofs = mtd->size - *len;
	}
}

/*
 * Return 1 if the entire region is locked (if @locked is true) or unlocked (if
 * @locked is false); 0 otherwise
 */
static int stm_check_lock_status_sr(struct spi_nor *nor, loff_t ofs, u64 len,
				    u8 sr, bool locked)
{
	loff_t lock_offs;
	uint64_t lock_len;

	if (!len)
		return 1;

	stm_get_locked_range(nor, sr, &lock_offs, &lock_len);

	if (locked)
		/* Requested range is a sub-range of locked range */
		return (ofs + len <= lock_offs + lock_len) && (ofs >= lock_offs);
	else
		/* Requested range does not overlap with locked range */
		return (ofs >= lock_offs + lock_len) || (ofs + len <= lock_offs);
}

static int stm_is_locked_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
			    u8 sr)
{
	return stm_check_lock_status_sr(nor, ofs, len, sr, true);
}

static int stm_is_unlocked_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
			      u8 sr)
{
	return stm_check_lock_status_sr(nor, ofs, len, sr, false);
}

/*
 * Lock a region of the flash. Compatible with ST Micro and similar flash.
 * Supports the block protection bits BP{0,1,2} in the status register
 * (SR). Does not support these features found in newer SR bitfields:
 *   - SEC: sector/block protect - only handle SEC=0 (block protect)
 *   - CMP: complement protect - only support CMP=0 (range is not complemented)
 *
 * Support for the following is provided conditionally for some flash:
 *   - TB: top/bottom protect
 *
 * Sample table portion for 8MB flash (Winbond w25q64fw):
 *
 *   SEC  |  TB   |  BP2  |  BP1  |  BP0  |  Prot Length  | Protected Portion
 *  --------------------------------------------------------------------------
 *    X   |   X   |   0   |   0   |   0   |  NONE         | NONE
 *    0   |   0   |   0   |   0   |   1   |  128 KB       | Upper 1/64
 *    0   |   0   |   0   |   1   |   0   |  256 KB       | Upper 1/32
 *    0   |   0   |   0   |   1   |   1   |  512 KB       | Upper 1/16
 *    0   |   0   |   1   |   0   |   0   |  1 MB         | Upper 1/8
 *    0   |   0   |   1   |   0   |   1   |  2 MB         | Upper 1/4
 *    0   |   0   |   1   |   1   |   0   |  4 MB         | Upper 1/2
 *    X   |   X   |   1   |   1   |   1   |  8 MB         | ALL
 *  ------|-------|-------|-------|-------|---------------|-------------------
 *    0   |   1   |   0   |   0   |   1   |  128 KB       | Lower 1/64
 *    0   |   1   |   0   |   1   |   0   |  256 KB       | Lower 1/32
 *    0   |   1   |   0   |   1   |   1   |  512 KB       | Lower 1/16
 *    0   |   1   |   1   |   0   |   0   |  1 MB         | Lower 1/8
 *    0   |   1   |   1   |   0   |   1   |  2 MB         | Lower 1/4
 *    0   |   1   |   1   |   1   |   0   |  4 MB         | Lower 1/2
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_lock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	int status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;
	loff_t lock_len;
	bool can_be_top = true, can_be_bottom = nor->flags & SNOR_F_HAS_SR_TB;
	bool use_top;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	/* If nothing in our range is unlocked, we don't need to do anything */
	if (stm_is_locked_sr(nor, ofs, len, status_old))
		return 0;

	/* If anything below us is unlocked, we can't use 'bottom' protection */
	if (!stm_is_locked_sr(nor, 0, ofs, status_old))
		can_be_bottom = false;

	/* If anything above us is unlocked, we can't use 'top' protection */
	if (!stm_is_locked_sr(nor, ofs + len, mtd->size - (ofs + len),
			      status_old))
		can_be_top = false;

	if (!can_be_bottom && !can_be_top)
		return -EINVAL;

	/* Prefer top, if both are valid */
	use_top = can_be_top;

	/* lock_len: length of region that should end up locked */
	if (use_top)
		lock_len = mtd->size - ofs;
	else
		lock_len = ofs + len;

	/*
	 * Need smallest pow such that:
	 *
	 *   1 / (2^pow) <= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = ceil(log2(size / len)) = log2(size) - floor(log2(len))
	 */
	pow = ilog2(mtd->size) - ilog2(lock_len);
	val = mask - (pow << shift);
	if (val & ~mask)
		return -EINVAL;
	/* Don't "lock" with no region! */
	if (!(val & mask))
		return -EINVAL;

	status_new = (status_old & ~mask & ~SR_TB) | val;

	/* Disallow further writes if WP pin is asserted */
	status_new |= SR_SRWD;

	if (!use_top)
		status_new |= SR_TB;

	/* Don't bother if they're the same */
	if (status_new == status_old)
		return 0;

	/* Only modify protection if it will not unlock other areas */
	if ((status_new & mask) < (status_old & mask))
		return -EINVAL;

	return write_sr_and_check(nor, status_new, mask);
}

/*
 * Unlock a region of the flash. See stm_lock() for more info
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_unlock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	int status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;
	loff_t lock_len;
	bool can_be_top = true, can_be_bottom = nor->flags & SNOR_F_HAS_SR_TB;
	bool use_top;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	/* If nothing in our range is locked, we don't need to do anything */
	if (stm_is_unlocked_sr(nor, ofs, len, status_old))
		return 0;

	/* If anything below us is locked, we can't use 'top' protection */
	if (!stm_is_unlocked_sr(nor, 0, ofs, status_old))
		can_be_top = false;

	/* If anything above us is locked, we can't use 'bottom' protection */
	if (!stm_is_unlocked_sr(nor, ofs + len, mtd->size - (ofs + len),
				status_old))
		can_be_bottom = false;

	if (!can_be_bottom && !can_be_top)
		return -EINVAL;

	/* Prefer top, if both are valid */
	use_top = can_be_top;

	/* lock_len: length of region that should remain locked */
	if (use_top)
		lock_len = mtd->size - (ofs + len);
	else
		lock_len = ofs;

	/*
	 * Need largest pow such that:
	 *
	 *   1 / (2^pow) >= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = floor(log2(size / len)) = log2(size) - ceil(log2(len))
	 */
	pow = ilog2(mtd->size) - order_base_2(lock_len);
	if (lock_len == 0) {
		val = 0; /* fully unlocked */
	} else {
		val = mask - (pow << shift);
		/* Some power-of-two sizes are not supported */
		if (val & ~mask)
			return -EINVAL;
	}

	status_new = (status_old & ~mask & ~SR_TB) | val;

	/* Don't protect status register if we're fully unlocked */
	if (lock_len == 0)
		status_new &= ~SR_SRWD;

	if (!use_top)
		status_new |= SR_TB;

	/* Don't bother if they're the same */
	if (status_new == status_old)
		return 0;

	/* Only modify protection if it will not lock other areas */
	if ((status_new & mask) > (status_old & mask))
		return -EINVAL;

	return write_sr_and_check(nor, status_new, mask);
}

/*
 * Check if a region of the flash is (completely) locked. See stm_lock() for
 * more info.
 *
 * Returns 1 if entire region is locked, 0 if any portion is unlocked, and
 * negative on errors.
 */
static int stm_is_locked(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	int status;

	status = read_sr(nor);
	if (status < 0)
		return status;

	return stm_is_locked_sr(nor, ofs, len, status);
}
#endif /* CONFIG_SPI_FLASH_STMICRO */
#if 0
static int upper_lock_para_check(loff_t flash_size, loff_t ofs, uint64_t len)
{
	return (ofs <= flash_size) && (len <= flash_size) &&
		(ofs + len <= flash_size) ? 0 : -1;
}

static loff_t ofs_of_uplocked(struct spi_nor *nor, u8 sr_val, u8 cr_val)
{
	u8 dummy_mask;
	u8 bp_mask;
	int pow;
	u8 mask = SR_BP2 |SR_BP1 |SR_BP0;
	/* lock none */
	if ((cr_val & GENERAL_CMP && ((sr_val & mask) == mask))
			|| (!(cr_val & GENERAL_CMP) && ((sr_val & mask) == 0)))
		return nor->mtd.size;
	/* lock all */
	if ((cr_val & GENERAL_CMP && (sr_val & mask) == 0)
			|| (!(cr_val & GENERAL_CMP) && ((sr_val & mask) == mask)))
		return 0;
	/* dummy_mask composed of CMP SEC BP */
	dummy_mask = ((cr_val & GENERAL_CMP) << 1) | (sr_val & (SR_SEC |SR_TB));
	/* get BP[0:2] */
	bp_mask = sr_val & mask;
	/*
	 * According to the flash manual,
	 * there are four sections in the upper lock area,
	 * using dummy_ Mask to row,return locked offset;
	 */
	switch (dummy_mask) {
	case SR_CMP |SR_SEC |SR_TB:
		if (bp_mask > SR_BP2)
			bp_mask = SR_BP2;
		pow = (bp_mask - SR_BP0) >> ilog2(SR_BP0);
		return BP_SZ_4K << pow;
	case SR_CMP |SR_TB:
		pow = (bp_mask - SR_BP0) >> ilog2(SR_BP0);
		return nor->mtd.size >> (6 - pow);
	case 0:
		pow = (bp_mask - SR_BP0) >> ilog2(SR_BP0);
		return nor->mtd.size - (nor->mtd.size >> (6 - pow));
	case SR_SEC:
		if (bp_mask > SR_BP2)
			bp_mask = SR_BP2;
		pow = (bp_mask - SR_BP0) >> ilog2(SR_BP0);
		return nor->mtd.size - (BP_SZ_4K << pow);
	}
	return -EINVAL;
}

static int sunxi_upper_lock(struct spi_nor *nor, loff_t ofs, uint64_t len, int lock)
{
	loff_t target_addr;
	int status_old = 0, status_new = 0;
	int config = 0;
	int pow = 0, mask_new = 0;
	u8 mask = SR_BP2 |SR_BP1 |SR_BP0;
	u8 quarter_mask = SR_BP2 |SR_BP0;
	u8 block_mask = SR_BP2 |SR_BP1;
	u8 sector_mask = SR_BP2;
	uint64_t flash_size = nor->mtd.size - 1;
	int ret;
	loff_t upper_locked_ofs;

	ret = upper_lock_para_check(nor->mtd.size, ofs, len);
	if (ret < 0) {
		dev_err(nor->dev, "%s, invalid address!\n", __func__);
		return -EINVAL;
	}
	target_addr = lock ? ofs : ofs + len;

	config = read_cr(nor);
	if (config < 0)
		return config;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	upper_locked_ofs = ofs_of_uplocked(nor, status_old, config);
	if (upper_locked_ofs < 0) {
		dev_err(nor->dev, "Upper lock mode is not used\n");
		return upper_locked_ofs;
	}
	dev_err(nor->dev, "target_addr: %lld, locked_addr: %lld", target_addr, upper_locked_ofs);
	dev_err(nor->dev, "config_old:0x%x, status_old: 0x%x\n", config, status_old);

	switch (JEDEC_MFR(nor->info)) {

	case SNOR_MFR_GIGADEVICE:
		/* reserve */
		break;

	default:
		dev_err(nor->dev, "not support upper lock!\n");
		return 0;
	}

	if (lock) {
		/* It's locked, don't need to do anythings */
		if (upper_locked_ofs <= target_addr) {
			dev_dbg(nor->dev, "don't need to do anythings\n");
			return 0;
		}
		/* Segment the flash and process target_addr in the corresponding segment */
		if (target_addr < BP_SZ_4K) {
			config |= GENERAL_CMP;
			status_old |= SR_SEC |SR_TB;
			mask_new = 0;
		} else if (target_addr >= BP_SZ_4K
				&& target_addr <= (flash_size >> 6)) {
			pow = ilog2(BP_SZ_32K) - ilog2(target_addr);
			if (pow < 0)
				pow = 0;
			mask_new = sector_mask - (pow << ilog2(SR_BP0));
			config |= GENERAL_CMP;
			status_old |= SR_SEC |SR_TB;
		} else if ((target_addr > (flash_size >> 6))
					&& (target_addr <= (flash_size / 2))) {
			pow = ilog2(flash_size / 2) - ilog2(target_addr);
			mask_new = quarter_mask - (pow << ilog2(SR_BP0));
			config |= GENERAL_CMP;
			status_old = (status_old |SR_TB) & (~SR_SEC);
		} else if ((target_addr > (flash_size / 2))
					&& (target_addr <= (flash_size - BP_SZ_32K))) {
			pow = ilog2(flash_size / 2) - ilog2(flash_size - target_addr);
			if ((pow << ilog2(SR_BP0)) > block_mask)
				pow = block_mask >> ilog2(SR_BP0);
			mask_new = block_mask - (pow << ilog2(SR_BP0));
			config &= ~GENERAL_CMP;
			status_old &= ~(SR_SEC |SR_TB);
		} else {
			pow = ilog2(BP_SZ_32K) - (ilog2(flash_size - target_addr) + 1);
			if ((pow << ilog2(SR_BP0)) >= sector_mask)
				pow = sector_mask >> ilog2(SR_BP0);
			mask_new = sector_mask - (pow << ilog2(SR_BP0));
			config &= GENERAL_CMP;
			status_old = (status_old |SR_SEC) & ~SR_TB;
		}
	} else {
		/* It's locked, don't need to do anythings */
		if (upper_locked_ofs >= target_addr) {
			dev_dbg(nor->dev, "don't need to do anythings\n");
			return 0;
		}
		status_old &= ~(SR_SEC |SR_TB);
		config &= ~SR_CMP;
		pow = ilog2(flash_size) - ilog2(flash_size - target_addr);
		if (pow << ilog2(SR_BP0) >= block_mask)
			pow = block_mask >> ilog2(SR_BP0);
		mask_new = block_mask - (pow << ilog2(SR_BP0));
	}

	status_new = (status_old & ~mask) |mask_new;
	dev_dbg(nor->dev, "config_new: 0x%x, status_new:0x%x\n", config, status_new);

	write_sr2(nor, config);
	spi_nor_wait_till_ready(nor);

	write_enable(nor);
	write_sr(nor, status_new);
	spi_nor_wait_till_ready(nor);

	return 0;
}
#endif
static void sunxi_get_locked_range(struct spi_nor *nor, u8 sr, u8 cr,
		loff_t *ofs, uint64_t *len)
{
	u8 mask, shift;
	int bp_check, i, flash_protection_size = 0;
	struct nor_protection *flash_protection = NULL;

	if (JEDEC_MFR(nor->info) == SNOR_MFR_MACRONIX) {
		mask = SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
		flash_protection = mxic_protection;
		flash_protection_size = ARRAY_SIZE(mxic_protection);
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_EON) { /* esmt */
		mask = SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
		if (nor->size == BP_SZ_16M) {
			flash_protection = esmt_protection;
			flash_protection_size = ARRAY_SIZE(esmt_protection);
		} else {
			flash_protection = esmt_protection_8M;
			flash_protection_size = ARRAY_SIZE(esmt_protection_8M);
		}
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_GIGADEVICE) {
		mask = SR_BP4 | SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
		flash_protection = gd_protection;
		flash_protection_size = ARRAY_SIZE(gd_protection);
	} else {
		dev_err(nor->dev, "not support lock 0x%x\n",
				JEDEC_MFR(nor->info));
		*ofs = 0;
		*len = 0;
		return;
	}
	shift = ffs(mask) - 1;

	bp_check = (sr & mask) >> shift;

	if (!(sr & mask)) {
		/* No protection */
		*ofs = 0;
		*len = 0;
	} else {
		for (i = 0; i < flash_protection_size; i++) {
			if (bp_check == flash_protection[i].bp) {
				*len = flash_protection[i].boundary;
				break;
			}
		}

		if (JEDEC_MFR(nor->info) == SNOR_MFR_MACRONIX) {
			/* if (nor->flags & SNOR_F_HAS_CR_TB && cr & CR_TB_MX) */
				/* *ofs = 0; */
			/* else */
				/* *ofs = mtd->size - *len; */
			*ofs = 0;	/* now only support bottom in mxic_protection */
		} else if (JEDEC_MFR(nor->info) == SNOR_MFR_EON) {
			/* TODO: read TB */
			*ofs = 0;	/* now only support bottom in esmt_protection */
		} else if (JEDEC_MFR(nor->info) == SNOR_MFR_GIGADEVICE) {
			/* TODO: read TB */
			*ofs = 0;	/* now only support bottom in gd_protection */
		}
	}
	dev_dbg(nor->dev, "ofs:0x%llx,len=0x%llx flag:0x%x sr:0x%x cr:0x%x\n",
			*ofs, *len, nor->flags, sr, cr);
}


/*
 * Return 1 if the entire region is locked (if @locked is true) or unlocked (if
 * @locked is false); 0 otherwise
 */
static int sunxi_check_lock_status_sr_cr(struct spi_nor *nor, loff_t ofs,
		u64 len, u8 sr, u8 cr, bool locked)
{
	loff_t lock_offs;
	uint64_t lock_len = 0;

	if (!len)
		return 1;

	sunxi_get_locked_range(nor, sr, cr, &lock_offs, &lock_len);

	if (locked)
		/* Requested range is a sub-range of locked range */
		return (ofs + len <= lock_offs + lock_len) && (ofs >= lock_offs);
	else
		/* Requested range does not overlap with locked range */
		return (ofs >= lock_offs + lock_len) || (ofs + len <= lock_offs);
}

static int sunxi_is_locked_sr_cr(struct spi_nor *nor, loff_t ofs,
		uint64_t len, u8 sr, u8 cr)
{
	return sunxi_check_lock_status_sr_cr(nor, ofs, len, sr, cr, true);
}

static int sunxi_is_unlocked_sr_cr(struct spi_nor *nor, loff_t ofs,
		uint64_t len, u8 sr, u8 cr)
{
	return sunxi_check_lock_status_sr_cr(nor, ofs, len, sr, cr, false);
}

static int sunxi_handle_lock(struct spi_nor *nor, loff_t ofs,
		uint64_t len, int lock)
{
	struct mtd_info *mtd = &nor->mtd;
	int status_old = 0, status_new = 0;
	int config_old = 0, config_new = 0;
	int otp_status_old = 0, otp_status_new = 0;
	u8 mask, shift, val = 0;
	loff_t lock_len;
	/* hardcode now, sunxi only use bottom */
	bool can_be_top = false, can_be_bottom = true;
	bool use_bottom;
	int i, ret, protect_flag = 0, flash_protection_size = 0;
	struct nor_protection *flash_protection = NULL;

	if (JEDEC_MFR(nor->info) == SNOR_MFR_MACRONIX) {
		flash_protection = mxic_protection;
		flash_protection_size = ARRAY_SIZE(mxic_protection);
		mask = SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
		config_old = read_cr(nor);
		if (config_old < 0)
			return config_old;
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_EON) {
		if (JEDEC_ID(nor->info) == GM_64A_ID) {
			dev_err(nor->dev, "not support lock 0x%x\n",
					JEDEC_ID(nor->info));
			return 0;
		}
		if (mtd->size == BP_SZ_16M) {
			flash_protection = esmt_protection;
			flash_protection_size = ARRAY_SIZE(esmt_protection);
		} else {
			flash_protection = esmt_protection_8M;
			flash_protection_size = ARRAY_SIZE(esmt_protection_8M);
		}
		mask = SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
		otp_status_old = read_otp_sr(nor);
		if (otp_status_old < 0)
			return otp_status_old;
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_GIGADEVICE) {
		flash_protection = gd_protection;
		flash_protection_size = ARRAY_SIZE(gd_protection);
		mask = SR_BP4 | SR_BP3 | SR_BP2 | SR_BP1 | SR_BP0;
	} else {
		dev_dbg(nor->dev, "not support lock 0x%x\n",
				JEDEC_MFR(nor->info));
		return 0;
	}
	shift = ffs(mask) - 1;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	if (lock) {
		/* If nothing in our range is unlocked, we don't need to do anything */
		if (sunxi_is_locked_sr_cr(nor, ofs, len,
					status_old, config_old))
			return 0;
	} else if (ofs + len == mtd->size) {
		val = flash_protection[0].bp << shift;
		status_new = (status_old & ~mask) | (val & mask);
		status_new &= (~SR_SRWD);
		goto set_status;
	} else {
		/* If nothing in our range is locked, we don't need to do anything */
		if (sunxi_is_unlocked_sr_cr(nor, ofs, len,
					status_old, config_old))
			return 0;
	}

	if (!can_be_bottom && !can_be_top)
		return -EINVAL;

	/* for sunxi, prefer bottom */
	use_bottom = can_be_bottom;

	/* lock_len: length of region that should end up locked */
	if (use_bottom)
		lock_len = lock ? (ofs + len) : ofs;
	else
		lock_len = lock ? (mtd->size - ofs) : (mtd->size - (ofs + len));

	/* in case over the max, we set the max default */
	val = flash_protection[flash_protection_size - 1].bp << shift;
	protect_flag = flash_protection[flash_protection_size - 1].flag;
	for (i = 1; i < flash_protection_size; i++) {
		if (lock_len < flash_protection[i].boundary) {
			val = flash_protection[i - 1].bp << shift;
			protect_flag = flash_protection[i - 1].flag;
			break;
		}
	}

	dev_dbg(nor->dev, "sunxi_lock:%d val:0x%x, flag:0x%x\n",
			lock, val, protect_flag);
	status_new = (status_old & ~mask) | (val & mask);

#if 0 /* not suitable for gd */
	if (lock) {
		/* Only modify protection if it will not unlock other areas */
		if ((status_new & mask) < (status_old & mask))
			return -EINVAL;
	} else {
		/* Only modify protection if it will not lock other areas */
		if ((status_new & mask) > (status_old & mask))
			return -EINVAL;
	}
#endif

set_status:
	if (JEDEC_MFR(nor->info) == SNOR_MFR_MACRONIX) {
		if (protect_flag & SET_TB)
			config_new = config_old | CR_TB_MX;
		else
			config_new = config_old & ~CR_TB_MX;

		if ((status_old != status_new) || (config_old != config_new)) {
			u8 sr_cr[2] = {status_new, config_new};
			ret = write_sr_cr(nor, sr_cr);
			if (ret)
				return ret;
		}
		dev_dbg(nor->dev, "sr:0x%x --> 0x%x cr:0x%x --> 0x%x\n",
				status_old, status_new, config_old, config_new);
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_EON) {
		if (protect_flag & SET_TB) {
			otp_status_new = otp_status_old | OTP_SR_TB_EON;
		} else {
			otp_status_new = otp_status_old & ~OTP_SR_TB_EON;
		}

		if (otp_status_old != otp_status_new) {
			ret = write_otp_sr(nor, otp_status_new);
			if (ret)
				return ret;
		}

		if (status_old != status_new) {
			ret = write_sr_and_check(nor, status_new, mask);
			if (ret)
				return ret;
		}
		dev_dbg(nor->dev, "sr:0x%x --> 0x%x otp_sr:0x%x --> 0x%x\n",
				status_old, status_new, otp_status_old,
				otp_status_new);
	} else if (JEDEC_MFR(nor->info) == SNOR_MFR_GIGADEVICE) {
		if (protect_flag & SET_CMP)
			ret = gigadevice_config_cmp(nor, 1);
		else
			ret = gigadevice_config_cmp(nor, 0);
		if (ret)
			return ret;

		if (status_old != status_new) {
			ret = write_sr_and_check(nor, status_new, mask);
			if (ret)
				return ret;
		}
		dev_dbg(nor->dev, "sr:0x%x --> 0x%x\n", status_old, status_new);
	}

	return 0;
}

static int sunxi_lock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	return sunxi_handle_lock(nor, ofs, len, true);
}
static int sunxi_unlock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	return sunxi_handle_lock(nor, ofs, len, false);
}

/*
 * Check if a region of the flash is (completely) locked. See sunxi_lock() for
 * more info.
 *
 * Returns 1 if entire region is locked, 0 if any portion is unlocked, and
 * negative on errors.
 */
static int sunxi_is_locked(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	int status = 0;
	int config = 0;

	status = read_sr(nor);
	if (status < 0)
		return status;

	if (JEDEC_MFR(nor->info) == SNOR_MFR_MACRONIX) {
		config = read_cr(nor);
		if (config < 0)
			return config;
	}

	return sunxi_is_locked_sr_cr(nor, ofs, len, status, config);
}

static int sunxi_individual_lock_status(struct spi_nor *nor,
					loff_t ofs, uint64_t len, bool locked)
{
	u8 read_opcode, read_dummy;
	loff_t ofs_lock = ofs;
	loff_t ofs_lock_end = ofs + len;
	u8 lock_status;
	u8 addr_width;
	u8 lock_mask;

	if (ofs > nor->mtd.size) {
		dev_err(nor->dev, "The lock address exceeds the flash size\n");
		return -1;
	}
	if (ofs_lock_end > nor->mtd.size)
		ofs_lock_end = nor->mtd.size - ofs_lock;

	if (locked)
		lock_mask = 0xff;
	else
		lock_mask = 0;

	/* align down */
	if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
			(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
		ofs_lock &= ~(NOR_LOCK_SECTOR_SIZE - 1);
	else
		ofs_lock &= ~(NOR_LOCK_BLOCK_SIZE - 1);

	read_opcode = nor->read_opcode;
	read_dummy = nor->read_dummy;
	addr_width = nor->addr_width;
	nor->read_opcode = SPINOR_OP_RDBLK;
	nor->read_dummy = 0;

	write_enable(nor);
	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_MXIC:
		nor->read_opcode = SPINOR_OP_MXICRBLK;
		nor->addr_width = 4;

		while (ofs_lock < ofs_lock_end) {
			nor->read(nor, ofs_lock, 1, &lock_status);
			dev_dbg(nor->dev, "check %s to:%lld lock_status:%d\n",
					locked ? "lock" : "unlock",
					ofs_lock, lock_status);
			/* 0xff is locked sign */
			if (lock_status == lock_mask) {
				if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
					(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
					/* sector for first and last block */
					ofs_lock += NOR_LOCK_SECTOR_SIZE;
				else
					/* block for middle blocks */
					ofs_lock += NOR_LOCK_BLOCK_SIZE;
				continue;
			} else {
				nor->read_opcode = read_opcode;
				nor->read_dummy = read_dummy;
				nor->addr_width = addr_width;
				return -1;
			}
		}
		break;
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
	case SNOR_MFR_FM:
	case SNOR_MFR_XTX:
		while (ofs_lock < ofs_lock_end) {
			nor->read(nor, ofs_lock, 1, &lock_status);
			dev_dbg(nor->dev, "check %s to:%lld lock_status:%d\n",
					locked == 1 ? "lock" : "unlock",
					ofs_lock, lock_status);

			if (lock_status != locked) {
				nor->read_opcode = read_opcode;
				nor->read_dummy = read_dummy;
				return -1;
			}

			if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
				(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
				/* sector for first and last block */
				ofs_lock += NOR_LOCK_SECTOR_SIZE;
			else
				/* block for middle blocks */
				ofs_lock += NOR_LOCK_BLOCK_SIZE;
		}
		break;
	default:
		dev_err(nor->dev, "not support individual is lock nor 0x%x\n",
				JEDEC_MFR(nor->info));
	}

	nor->read_opcode = read_opcode;
	nor->read_dummy = read_dummy;
	nor->addr_width = addr_width;

	return 1;
}

static int sunxi_individual_is_lock(struct spi_nor *nor,
					loff_t ofs, uint64_t len)
{
	return sunxi_individual_lock_status(nor, ofs, len, true);
}

#ifdef WLOCK_CHECK
static int sunxi_individual_is_unlock(struct spi_nor *nor,
					loff_t ofs, uint64_t len)
{
	return sunxi_individual_lock_status(nor, ofs, len, false);
}
#endif
/* Write status register and ensure bits in mask match written values */
static int write_lock_and_check(struct spi_nor *nor, loff_t to, bool locked)
{
	int ret;
#ifdef WLOCK_CHECK
	int check_len;

	if ((to < NOR_LOCK_BLOCK_SIZE) ||
			(to >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
		check_len = NOR_LOCK_SECTOR_SIZE;
	else
		check_len = NOR_LOCK_BLOCK_SIZE;
#endif

	write_enable(nor);
	ret = nor->write(nor, to, 0, NULL);

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return -1;
#ifdef WLOCK_CHECK
	if (locked)
		ret = sunxi_individual_is_lock(nor, to, check_len);
	else
		ret = sunxi_individual_is_unlock(nor, to, check_len);
#endif
	return ret < 0 ? ret : 0;
}

/* Write DPB register lock block/sector */
static int write_mxic_lock(struct spi_nor *nor, loff_t ofs, uint64_t len, bool locked)
{
	u32 ret;
	u_char lock_com;
	loff_t ofs_lock = ofs;
	loff_t ofs_lock_end = ofs + len;

	if (ofs_lock_end > nor->mtd.size)
		ofs_lock_end = nor->mtd.size - ofs_lock;

	if (locked) {
		lock_com = 0xff;

		/* area is locked, no need to do antrhing */
		if (sunxi_individual_is_lock(nor, ofs_lock, len) > 0)
			return 0;
	} else {
		lock_com = 0;

		#ifdef WLOCK_CHECK
		/* area is locked, no need to do antrhing */
		if (sunxi_individual_is_unlock(nor, ofs_lock, len) > 0)
			return 0;
		#endif
	}

	while (ofs_lock < ofs_lock_end) {
		write_enable(nor);
		ret = nor->write(nor, ofs_lock, 1, &lock_com);

		if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
			(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
		/* sector for first and last block */
			ofs_lock += NOR_LOCK_SECTOR_SIZE;
		else
			/* block for middle blocks */
			ofs_lock += NOR_LOCK_BLOCK_SIZE;
	}

	return ret < 0 ? ret : 0;
}
static int sunxi_individual_lock_global(struct spi_nor *nor)
{
	write_enable(nor);

	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
	case SNOR_MFR_FM:
	case SNOR_MFR_XTX:
	case SNOR_MFR_MXIC:
		return nor->write_reg(nor, SPINOR_OP_GBLK, NULL, 0);
	default:
		dev_err(nor->dev,
			"not support individual global lock nor 0x%x\n",
			JEDEC_MFR(nor->info));
	}

	return 0;
}

static int sunxi_individual_unlock_global(struct spi_nor *nor)
{
	write_enable(nor);

	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
	case SNOR_MFR_FM:
	case SNOR_MFR_XTX:
		return nor->write_reg(nor, SPINOR_OP_UGBLK, NULL, 0);
	case SNOR_MFR_MXIC:
		nor->write_reg(nor, SPINOR_OP_USPB, NULL, 0);
		spi_nor_wait_till_ready(nor);
		write_enable(nor);
		nor->write_reg(nor, SPINOR_OP_UGBLK, NULL, 0);
		break;
	default:
		dev_err(nor->dev,
			"not support individual global unlock nor 0x%x\n",
			JEDEC_MFR(nor->info));
	}

	return 0;
}

static int sunxi_individual_handle_lock(struct spi_nor *nor,
					loff_t ofs, uint64_t len, bool locked)
{
	u32 ret;
	u8 program_opcode;
	u8 read_opcode;
	u8 read_dummy;
	u8 addr_width;
	loff_t ofs_lock = ofs;
	loff_t ofs_lock_end = ofs + len;

	if (ofs > nor->size) {
		dev_err(nor->dev, "The lock address exceeds the flash size\n");
		return -1;
	}
	if (ofs_lock_end > nor->mtd.size)
		ofs_lock_end = nor->mtd.size - ofs_lock;
	/* align down */
	if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
			(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
		ofs_lock &= ~(NOR_LOCK_SECTOR_SIZE - 1);
	else
		ofs_lock &= ~(NOR_LOCK_BLOCK_SIZE - 1);

	read_opcode = nor->read_opcode;
	read_dummy = nor->read_dummy;
	program_opcode = nor->program_opcode;
	addr_width = nor->addr_width;

	write_enable(nor);
	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_MXIC:
		nor->program_opcode = SPINOR_OP_MXICWBLK;
		nor->read_opcode = SPINOR_OP_MXICRBLK;
		nor->read_dummy = 0;
		nor->addr_width = 4;

		write_mxic_lock(nor, ofs_lock, len, locked);
		break;
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
	case SNOR_MFR_FM:
	case SNOR_MFR_XTX:
		if (locked)
			nor->program_opcode = SPINOR_OP_IBLK;
		else
			nor->program_opcode = SPINOR_OP_UIBLK;

		while (ofs_lock < ofs_lock_end) {
			dev_dbg(nor->dev, "%s handle to:%lld\n",
					locked ?
					"lock" : "unlock", ofs_lock);
			ret = write_lock_and_check(nor, ofs_lock, locked);
			if (ret)
				dev_err(nor->dev, "%s to:%lld err\n",
						locked ? "lock" : "unlock", ofs_lock);

			if ((ofs_lock < NOR_LOCK_BLOCK_SIZE) ||
				(ofs_lock >= nor->mtd.size - NOR_LOCK_BLOCK_SIZE))
				/* sector for first and last block */
				ofs_lock += NOR_LOCK_SECTOR_SIZE;
			else
				/* block for middle blocks */
				ofs_lock += NOR_LOCK_BLOCK_SIZE;
		}
		break;
	default:
		dev_err(nor->dev, "not support individual lock nor 0x%x...\n",
				JEDEC_MFR(nor->info));
	}

	nor->read_opcode = read_opcode;
	nor->read_dummy = read_dummy;
	nor->program_opcode = program_opcode;
	nor->addr_width = addr_width;

	return 0;
}

static int sunxi_individual_lock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	if (ofs == 0 && len == nor->size)
		return sunxi_individual_lock_global(nor);

	return sunxi_individual_handle_lock(nor, ofs, len, true);
}

static int sunxi_individual_unlock(struct spi_nor *nor,
					loff_t ofs, uint64_t len)
{
	if (ofs == 0 && len == nor->size)
		return sunxi_individual_unlock_global(nor);

	return sunxi_individual_handle_lock(nor, ofs, len, false);
}

static int sunxi_individual_lock_enable(struct spi_nor *nor)
{
	u8 status;

	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
		status = read_sr3(nor);
		write_sr3(nor, status | SR_WPS_EN_WINBOND);
		if (read_sr3(nor) | SR_WPS_EN_WINBOND)
			return 0;
		else
			return -1;
	case SNOR_MFR_FM:
		status = read_sr2(nor);
		write_sr2(nor, status | SR_WPS_EN_FM);
		if (read_sr2(nor) | SR_WPS_EN_FM)
			return 0;
		else
			return -1;
	case SNOR_MFR_XTX:
		status = read_sr2(nor);
		write_sr2(nor, status | SR_WPS_EN_XTX);
		if (read_sr2(nor) | SR_WPS_EN_XTX)
			return 0;
		else
			return -1;
	default:
		dev_err(nor->dev, "not support individual lock nor 0x%x\n",
				JEDEC_MFR(nor->info));
	}

	return -1;
}

static int sunxi_individual_lock_is_enable(struct spi_nor *nor)
{
	switch (JEDEC_MFR(nor->info)) {
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_PUYA:
		if (read_sr3(nor) & SR_WPS_EN_WINBOND)
			return 1;
		else
			return 0;
	case SNOR_MFR_FM:
		if (read_sr2(nor) & SR_WPS_EN_FM)
			return 1;
		else
			return 0;
	case SNOR_MFR_XTX:
		if (read_sr2(nor) & SR_WPS_EN_XTX)
			return 1;
		else
			return 0;
	case SNOR_MFR_MXIC:
		if (read_mxic_sr(nor) & SR_WPSEL)
			return 1;
		else
			return 0;
	default:
		return 0;
	}
}

static void sunxi_individual_lock_init(struct spi_nor *nor)
{
	int ret = 0, nodeoffset = 0;
	unsigned int rval = 0;


	if (nor->info->flags & SPI_NOR_INDIVIDUAL_LOCK) {
		nodeoffset =  fdt_path_offset(working_fdt, "spi0/spi_board0");
		if (nodeoffset < 0) {
			dev_err(nor->dev, "get spi0 para fail\n");
		} else {
			ret = fdt_getprop_u32(working_fdt, nodeoffset,
					"individual_lock", (uint32_t *)(&rval));
			if (ret < 0) {
				dev_err(nor->dev, "individual_lock fail %d\n",
						ret);
			} else {
				if (rval) {
					nor->flags |= SNOR_F_INDIVIDUAL_LOCK;
					sunxi_individual_lock_enable(nor);
				}
			}
		}
	}

	/*
	 * If the Individual lock is not used and the Flash is locked,
	 * it will be unlocked globally
	 */
	if (!(nor->flags & SNOR_F_INDIVIDUAL_LOCK))
		if (sunxi_individual_lock_is_enable(nor))
			sunxi_individual_unlock_global(nor);

	return;
}

static int spi_nor_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	return nor->flash_lock(nor, ofs, len);
}

static int spi_nor_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	return nor->flash_unlock(nor, ofs, len);
}

static int spi_nor_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	return nor->flash_is_locked(nor, ofs, len);
}

#ifdef CONFIG_SPI_FLASH_SR
static int security_regiser_is_locked(struct mtd_info *mtd, u8 sr_num)
{
	u8 val, mask;
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	if (sr_num <= 0 || sr_num >= 4) {
		dev_dbg(nor->dev, "%s, error security register num %d\n", __func__, sr_num);
		return -1;
	}

	val = read_sr2(nor);
	mask = 1 << (LB_OFS + sr_num);

	return val & mask;
}

static int security_register_lock(struct mtd_info *mtd, u8 sr_num)
{
	u8 val, mask;
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	if (sr_num <= 0 || sr_num >= 4) {
		dev_dbg(nor->dev, "%s, error security register num %d\n", __func__, sr_num);
		return -1;
	}

	val = read_sr2(nor);
	mask = 1 << (LB_OFS + sr_num);
	val |= mask;
	write_sr2(nor, val);

	return 0;
}

static int security_regiser_read_data(struct mtd_info *mtd,
					loff_t addr, loff_t len, u_char *buf)
{
	u8 read_opcode;
	u8 read_dummy;
	u8 addr_width;
	int ret;
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	read_opcode = nor->read_opcode;
	read_dummy = nor->read_dummy;
	addr_width = nor->addr_width;

	nor->read_opcode = SR_OP_READ;
	nor->read_dummy = 8;
	nor->addr_width = 3;

	ret = nor->read(nor, addr, len, buf);

	nor->read_opcode = read_opcode;
	nor->read_dummy = read_dummy;
	nor->addr_width = addr_width;
	return ret;

}

static int security_regiser_write_data(struct mtd_info *mtd,
					loff_t addr, loff_t len, u_char *buf)
{
	u8 program_opcode;
	u8 addr_width;
	int ret;
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	program_opcode = nor->program_opcode;
	addr_width = nor->addr_width;

	nor->program_opcode = SR_OP_PROGRAM;
	nor->addr_width = 3;

	write_enable(nor);
	ret = nor->write(nor, addr, len, buf);

	nor->program_opcode = program_opcode;
	nor->addr_width = addr_width;
	return ret;
}

static int security_regiser_erase(struct mtd_info *mtd, loff_t addr)
{
	u8 program_opcode;
	int ret;
	struct spi_nor *nor = mtd_to_spi_nor(mtd);

	program_opcode = nor->program_opcode;
	nor->program_opcode = SR_OP_ERASE;

	write_enable(nor);
	ret = nor->write(nor, addr, 0, NULL);

	nor->program_opcode = program_opcode;
	return ret;
}
#endif

static const struct flash_info *spi_nor_read_id(struct spi_nor *nor)
{
	int			tmp;
	u8			id[SPI_NOR_MAX_ID_LEN];
	const struct flash_info	*info;

	tmp = nor->read_reg(nor, SPINOR_OP_RDID, id, SPI_NOR_MAX_ID_LEN);
	if (tmp < 0) {
		dev_dbg(nor->dev, "error %d reading JEDEC ID\n", tmp);
		return NULL;
	}

	info = spi_nor_ids;
	for (; info->name; info++) {
		if (info->id_len) {
			if (!memcmp(info->id, id, info->id_len))
				return info;
		}
	}

	dev_err(nor->dev, "unrecognized JEDEC id bytes: %02x, %02x, %02x\n",
		id[0], id[1], id[2]);
	return NULL;
}

static int spi_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret = 0;
	u32 single_die_size = 0;

	dev_dbg(nor->dev, "from 0x%08x, len %zd\n", (u32)from, len);

	if (nor->info->flags & SPI_NOR_STACK_DIE) {
		u8 total_stack_die = 2; //FIXME
		u8 select_die;
		single_die_size = nor->info->sector_size * nor->info->n_sectors / total_stack_die;
		select_die = (u32)from / single_die_size;

		ret = sunxi_select_die(nor, select_die);
		if (ret)
			goto read_err;

		from = (u32)from % single_die_size;
	}

	while (len) {
		loff_t addr = from;
		size_t read_len = len;

#ifdef CONFIG_SPI_FLASH_BAR
		u32 remain_len;

		ret = write_bar(nor, addr);
		if (ret < 0)
			return log_ret(ret);
		remain_len = (SZ_16M * (nor->bank_curr + 1)) - addr;

		if (len < remain_len)
			read_len = len;
		else
			read_len = remain_len;
#endif
		if (nor->info->flags & SPI_NOR_STACK_DIE)
			read_len = from + read_len > single_die_size ? single_die_size - from : read_len;
#ifdef CONFIG_SUNXI_SPIF
		if ((nor->info->flags & USE_IO_MODE) &&
				spi_nor_get_protocol_addr_nbits(nor->read_proto) != 1)
			ret = spi_nor_io_read_data(nor, addr, read_len, buf);
		else
#endif
			ret = nor->read(nor, addr, read_len, buf);
		if (ret == 0) {
			/* We shouldn't see 0-length reads */
			ret = -EIO;
			goto read_err;
		}
		if (ret < 0)
			goto read_err;

		*retlen += ret;
		buf += ret;
		from += ret;
		len -= ret;

		if ((nor->info->flags & SPI_NOR_STACK_DIE) && from >= single_die_size && len > 0) {
			u8 select_die = ++nor->active_stack_die;
			ret = sunxi_select_die(nor, select_die);
			if (ret)
				goto read_err;
			from = 0;
		}
	}
	ret = 0;

read_err:
#ifdef CONFIG_SPI_FLASH_BAR
	ret = clean_bar(nor);
#endif
	return ret;
}
#ifdef CONFIG_SPI_FLASH_SST
static int sst_write_byteprogram(struct spi_nor *nor, loff_t to, size_t len,
				 size_t *retlen, const u_char *buf)
{
	size_t actual;
	int ret = 0;

	for (actual = 0; actual < len; actual++) {
		nor->program_opcode = SPINOR_OP_BP;

		write_enable(nor);
		/* write one byte. */
		ret = nor->write(nor, to, 1, buf + actual);
		if (ret < 0)
			goto sst_write_err;
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
		to++;
	}

sst_write_err:
	write_disable(nor);
	return ret;
}

static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		     size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	struct spi_slave *spi = nor->spi;
	size_t actual;
	int ret;

	dev_dbg(nor->dev, "to 0x%08x, len %zd\n", (u32)to, len);
	if (spi->mode & SPI_TX_BYTE)
		return sst_write_byteprogram(nor, to, len, retlen, buf);

	write_enable(nor);

	nor->sst_write_second = false;

	actual = to % 2;
	/* Start write from odd address. */
	if (actual) {
		nor->program_opcode = SPINOR_OP_BP;

		/* write one byte. */
		ret = nor->write(nor, to, 1, buf);
		if (ret < 0)
			goto sst_write_err;
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
	}
	to += actual;

	/* Write out most of the data here. */
	for (; actual < len - 1; actual += 2) {
		nor->program_opcode = SPINOR_OP_AAI_WP;

		/* write two bytes. */
		ret = nor->write(nor, to, 2, buf + actual);
		if (ret < 0)
			goto sst_write_err;
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
		to += 2;
		nor->sst_write_second = true;
	}
	nor->sst_write_second = false;

	write_disable(nor);
	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto sst_write_err;

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(nor);

		nor->program_opcode = SPINOR_OP_BP;
		ret = nor->write(nor, to, 1, buf + actual);
		if (ret < 0)
			goto sst_write_err;
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
		write_disable(nor);
		actual += 1;
	}
sst_write_err:
	*retlen += actual;
	return ret;
}
#endif
/*
 * Write an address range to the nor chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int spi_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	size_t page_offset, page_remain, i;
	ssize_t ret = 0;
	size_t addr_offset = 0;
	u32 single_die_size = 0;

	if (nor->info->flags & SPI_NOR_STACK_DIE) {
		u8 total_stack_die = 2; //FIXME
		u8 select_die;
		single_die_size = nor->info->sector_size * nor->info->n_sectors / total_stack_die;
		select_die = (u32)to / single_die_size;

		ret = sunxi_select_die(nor, select_die);
		if (ret)
			goto write_err;

		to = (u32)to % single_die_size;
	}

	for (i = 0; i < len; ) {
		ssize_t written;
		loff_t addr = 0;
		if (nor->info->flags & SPI_NOR_STACK_DIE)
			addr = to + addr_offset;
		else
			addr = to + i;

		/*
		 * If page_size is a power of two, the offset can be quickly
		 * calculated with an AND operation. On the other cases we
		 * need to do a modulus operation (more expensive).
		 * Power of two numbers have only one bit set and we can use
		 * the instruction hweight32 to detect if we need to do a
		 * modulus (do_div()) or not.
		 */
		if (hweight32(nor->page_size) == 1) {
			page_offset = addr & (nor->page_size - 1);
		} else {
			u64 aux = addr;

			page_offset = do_div(aux, nor->page_size);
		}
		/* the size of data remaining on the first page */
		page_remain = min_t(size_t,
				    nor->page_size - page_offset, len - i);

#ifdef CONFIG_SPI_FLASH_BAR
		ret = write_bar(nor, addr);
		if (ret < 0)
			return ret;
#endif
		write_enable(nor);
		ret = nor->write(nor, addr, page_remain, buf + i);
		if (ret < 0)
			goto write_err;

		written = ret;

		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto write_err;
		*retlen += written;
		i += written;

		if (nor->info->flags & SPI_NOR_STACK_DIE) {
			addr_offset += written;
			if (to + addr_offset >= single_die_size && i < len) {
				u8 select_die = ++nor->active_stack_die;

				ret = sunxi_select_die(nor, select_die);
				if (ret)
					goto write_err;
				to = 0;
				addr_offset = 0;
			}
		}
	}

write_err:
#ifdef CONFIG_SPI_FLASH_BAR
	ret = clean_bar(nor);
#endif
	return ret;
}

static int spi_nor_has_lock_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	ssize_t ret;

	if (mtd->_unlock(mtd, to, len))
		dev_err(nor->dev, "write unlock err\n");

#ifdef CONFIG_SPI_FLASH_SST
	/* sst nor chips use AAI word program */
	if (nor->info->flags & SST_WRITE)
		ret = sst_write(mtd, to, len, retlen, buf);
	else
#endif
		ret = spi_nor_write(mtd, to, len, retlen, buf);

	mtd->_lock(mtd, 0, mtd->size);
	return ret;
}

#ifdef CONFIG_SPI_FLASH_PUYA1

static int puya_quad_enable(struct spi_nor *nor)
{
	u8 qeb_status;
	int ret;

	/* Check current Quad Enable bit value. */
	ret = read_cr(nor);
	if (ret < 0) {
		dev_dbg(dev, "error while reading configuration register\n");
		return -EINVAL;
	}

	if (ret & CR_QUAD_EN_PUYA)
		return 0;

	write_enable(nor);
	ret = write_sr(nor, ret | CR_QUAD_EN_PUYA);
	if (ret < 0)
		return ret;

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;
	/* read CR and check it */
	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_PUYA))) {
		dev_err(nor->dev, "Puya Quad bit not set\n");
		return -EINVAL;
	}

	return ret;
}
#endif

#ifdef CONFIG_SPI_FLASH_MACRONIX
/**
 * macronix_quad_enable() - set QE bit in Status Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Status Register.
 *
 * bit 6 of the Status Register is the QE bit for Macronix like QSPI memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int macronix_quad_enable(struct spi_nor *nor)
{
	int ret, val;

	val = read_sr(nor);
	if (val < 0)
		return val;
	if (val & SR_QUAD_EN_MX)
		return 0;

	write_enable(nor);

	write_sr(nor, val | SR_QUAD_EN_MX);

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;

	ret = read_sr(nor);
	if (!(ret > 0 && (ret & SR_QUAD_EN_MX))) {
		dev_err(nor->dev, "Macronix Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}
#endif

#if defined(CONFIG_SPI_FLASH_SPANSION) || defined(CONFIG_SPI_FLASH_WINBOND) || defined(CONFIG_SPI_FLASH_PUYA)
/**
 * spansion_read_cr_quad_enable() - set QE bit in Configuration Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Configuration Register.
 * This function should be used with QSPI memories supporting the Read
 * Configuration Register (35h) instruction.
 *
 * bit 1 of the Configuration Register is the QE bit for Spansion like QSPI
 * memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spansion_read_cr_quad_enable(struct spi_nor *nor)
{
	u8 sr_cr[2];
	int ret;

	/* Check current Quad Enable bit value. */
	ret = read_cr(nor);
	if (ret < 0) {
		dev_dbg(dev, "error while reading configuration register\n");
		return -EINVAL;
	}

	if (ret & CR_QUAD_EN_SPAN)
		return 0;

	sr_cr[1] = ret | CR_QUAD_EN_SPAN;

	/* Keep the current value of the Status Register. */
	ret = read_sr(nor);
	if (ret < 0) {
		dev_dbg(dev, "error while reading status register\n");
		return -EINVAL;
	}
	sr_cr[0] = ret;

	ret = write_sr_cr(nor, sr_cr);
	if (ret)
		return ret;

	/* Read back and check it. */
	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_SPAN))) {
		dev_dbg(nor->dev, "Spansion Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}


#if CONFIG_IS_ENABLED(SPI_FLASH_SFDP_SUPPORT)
/**
 * spansion_no_read_cr_quad_enable() - set QE bit in Configuration Register.
 * @nor:	pointer to a 'struct spi_nor'
 *
 * Set the Quad Enable (QE) bit in the Configuration Register.
 * This function should be used with QSPI memories not supporting the Read
 * Configuration Register (35h) instruction.
 *
 * bit 1 of the Configuration Register is the QE bit for Spansion like QSPI
 * memories.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spansion_no_read_cr_quad_enable(struct spi_nor *nor)
{
	u8 sr_cr[2];
	int ret;

	/* Keep the current value of the Status Register. */
	ret = read_sr(nor);
	if (ret < 0) {
		dev_dbg(nor->dev, "error while reading status register\n");
		return -EINVAL;
	}
	sr_cr[0] = ret;
	sr_cr[1] = CR_QUAD_EN_SPAN;

	return write_sr_cr(nor, sr_cr);
}

#endif /* CONFIG_SPI_FLASH_SFDP_SUPPORT */
#endif /* CONFIG_SPI_FLASH_SPANSION */

struct spi_nor_read_command {
	u8			num_mode_clocks;
	u8			num_wait_states;
	u8			opcode;
	enum spi_nor_protocol	proto;
};

struct spi_nor_pp_command {
	u8			opcode;
	enum spi_nor_protocol	proto;
};

enum spi_nor_read_command_index {
	SNOR_CMD_READ,
	SNOR_CMD_READ_FAST,
	SNOR_CMD_READ_1_1_1_DTR,

	/* Dual SPI */
	SNOR_CMD_READ_1_1_2,
	SNOR_CMD_READ_1_2_2,
	SNOR_CMD_READ_2_2_2,
	SNOR_CMD_READ_1_2_2_DTR,

	/* Quad SPI */
	SNOR_CMD_READ_1_1_4,
	SNOR_CMD_READ_1_4_4,
	SNOR_CMD_READ_4_4_4,
	SNOR_CMD_READ_1_4_4_DTR,

	/* Octo SPI */
	SNOR_CMD_READ_1_1_8,
	SNOR_CMD_READ_1_8_8,
	SNOR_CMD_READ_8_8_8,
	SNOR_CMD_READ_1_8_8_DTR,

	SNOR_CMD_READ_MAX
};

enum spi_nor_pp_command_index {
	SNOR_CMD_PP,

	/* Quad SPI */
	SNOR_CMD_PP_1_1_4,
	SNOR_CMD_PP_1_4_4,
	SNOR_CMD_PP_4_4_4,

	/* Octo SPI */
	SNOR_CMD_PP_1_1_8,
	SNOR_CMD_PP_1_8_8,
	SNOR_CMD_PP_8_8_8,

	SNOR_CMD_PP_MAX
};

struct spi_nor_flash_parameter {
	u64				size;
	u32				page_size;

	struct spi_nor_hwcaps		hwcaps;
	struct spi_nor_read_command	reads[SNOR_CMD_READ_MAX];
	struct spi_nor_pp_command	page_programs[SNOR_CMD_PP_MAX];

	int (*quad_enable)(struct spi_nor *nor);
};

static void
spi_nor_set_read_settings(struct spi_nor_read_command *read,
			  u8 num_mode_clocks,
			  u8 num_wait_states,
			  u8 opcode,
			  enum spi_nor_protocol proto)
{
	read->num_mode_clocks = num_mode_clocks;
	read->num_wait_states = num_wait_states;
	read->opcode = opcode;
	read->proto = proto;
}

static void
spi_nor_set_pp_settings(struct spi_nor_pp_command *pp,
			u8 opcode,
			enum spi_nor_protocol proto)
{
	pp->opcode = opcode;
	pp->proto = proto;
}

#if CONFIG_IS_ENABLED(SPI_FLASH_SFDP_SUPPORT)
/*
 * Serial Flash Discoverable Parameters (SFDP) parsing.
 */

/**
 * spi_nor_read_sfdp() - read Serial Flash Discoverable Parameters.
 * @nor:	pointer to a 'struct spi_nor'
 * @addr:	offset in the SFDP area to start reading data from
 * @len:	number of bytes to read
 * @buf:	buffer where the SFDP data are copied into (dma-safe memory)
 *
 * Whatever the actual numbers of bytes for address and dummy cycles are
 * for (Fast) Read commands, the Read SFDP (5Ah) instruction is always
 * followed by a 3-byte address and 8 dummy clock cycles.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_read_sfdp(struct spi_nor *nor, u32 addr,
			     size_t len, void *buf)
{
	u8 addr_width, read_opcode, read_dummy;
	int ret;

	read_opcode = nor->read_opcode;
	addr_width = nor->addr_width;
	read_dummy = nor->read_dummy;

	nor->read_opcode = SPINOR_OP_RDSFDP;
	nor->addr_width = 3;
	nor->read_dummy = 8;

	while (len) {
		ret = nor->read(nor, addr, len, (u8 *)buf);
		if (!ret || ret > len) {
			ret = -EIO;
			goto read_err;
		}
		if (ret < 0)
			goto read_err;

		buf += ret;
		addr += ret;
		len -= ret;
	}
	ret = 0;

read_err:
	nor->read_opcode = read_opcode;
	nor->addr_width = addr_width;
	nor->read_dummy = read_dummy;

	return ret;
}

struct sfdp_parameter_header {
	u8		id_lsb;
	u8		minor;
	u8		major;
	u8		length; /* in double words */
	u8		parameter_table_pointer[3]; /* byte address */
	u8		id_msb;
};

#define SFDP_PARAM_HEADER_ID(p)	(((p)->id_msb << 8) | (p)->id_lsb)
#define SFDP_PARAM_HEADER_PTP(p) \
	(((p)->parameter_table_pointer[2] << 16) | \
	 ((p)->parameter_table_pointer[1] <<  8) | \
	 ((p)->parameter_table_pointer[0] <<  0))

#define SFDP_BFPT_ID		0xff00	/* Basic Flash Parameter Table */
#define SFDP_SECTOR_MAP_ID	0xff81	/* Sector Map Table */

#define SFDP_SIGNATURE		0x50444653U
#define SFDP_JESD216_MAJOR	1
#define SFDP_JESD216_MINOR	0
#define SFDP_JESD216A_MINOR	5
#define SFDP_JESD216B_MINOR	6

struct sfdp_header {
	u32		signature; /* Ox50444653U <=> "SFDP" */
	u8		minor;
	u8		major;
	u8		nph; /* 0-base number of parameter headers */
	u8		unused;

	/* Basic Flash Parameter Table. */
	struct sfdp_parameter_header	bfpt_header;
};

/* Basic Flash Parameter Table */

/*
 * JESD216 rev B defines a Basic Flash Parameter Table of 16 DWORDs.
 * They are indexed from 1 but C arrays are indexed from 0.
 */
#define BFPT_DWORD(i)		((i) - 1)
#define BFPT_DWORD_MAX		16

/* The first version of JESB216 defined only 9 DWORDs. */
#define BFPT_DWORD_MAX_JESD216			9

/* 1st DWORD. */
#define BFPT_DWORD1_FAST_READ_1_1_2		BIT(16)
#define BFPT_DWORD1_ADDRESS_BYTES_MASK		GENMASK(18, 17)
#define BFPT_DWORD1_ADDRESS_BYTES_3_ONLY	(0x0UL << 17)
#define BFPT_DWORD1_ADDRESS_BYTES_3_OR_4	(0x1UL << 17)
#define BFPT_DWORD1_ADDRESS_BYTES_4_ONLY	(0x2UL << 17)
#define BFPT_DWORD1_DTR				BIT(19)
#define BFPT_DWORD1_FAST_READ_1_2_2		BIT(20)
#define BFPT_DWORD1_FAST_READ_1_4_4		BIT(21)
#define BFPT_DWORD1_FAST_READ_1_1_4		BIT(22)

/* 5th DWORD. */
#define BFPT_DWORD5_FAST_READ_2_2_2		BIT(0)
#define BFPT_DWORD5_FAST_READ_4_4_4		BIT(4)

/* 11th DWORD. */
#define BFPT_DWORD11_PAGE_SIZE_SHIFT		4
#define BFPT_DWORD11_PAGE_SIZE_MASK		GENMASK(7, 4)

/* 15th DWORD. */

/*
 * (from JESD216 rev B)
 * Quad Enable Requirements (QER):
 * - 000b: Device does not have a QE bit. Device detects 1-1-4 and 1-4-4
 *         reads based on instruction. DQ3/HOLD# functions are hold during
 *         instruction phase.
 * - 001b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         Writing only one byte to the status register has the side-effect of
 *         clearing status register 2, including the QE bit. The 100b code is
 *         used if writing one byte to the status register does not modify
 *         status register 2.
 * - 010b: QE is bit 6 of status register 1. It is set via Write Status with
 *         one data byte where bit 6 is one.
 *         [...]
 * - 011b: QE is bit 7 of status register 2. It is set via Write status
 *         register 2 instruction 3Eh with one data byte where bit 7 is one.
 *         [...]
 *         The status register 2 is read using instruction 3Fh.
 * - 100b: QE is bit 1 of status register 2. It is set via Write Status with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 *         In contrast to the 001b code, writing one byte to the status
 *         register does not modify status register 2.
 * - 101b: QE is bit 1 of status register 2. Status register 1 is read using
 *         Read Status instruction 05h. Status register2 is read using
 *         instruction 35h. QE is set via Writ Status instruction 01h with
 *         two data bytes where bit 1 of the second byte is one.
 *         [...]
 */
#define BFPT_DWORD15_QER_MASK			GENMASK(22, 20)
#define BFPT_DWORD15_QER_NONE			(0x0UL << 20) /* Micron */
#define BFPT_DWORD15_QER_SR2_BIT1_BUGGY		(0x1UL << 20)
#define BFPT_DWORD15_QER_SR1_BIT6		(0x2UL << 20) /* Macronix */
#define BFPT_DWORD15_QER_SR2_BIT7		(0x3UL << 20)
#define BFPT_DWORD15_QER_SR2_BIT1_NO_RD		(0x4UL << 20)
#define BFPT_DWORD15_QER_SR2_BIT1		(0x5UL << 20) /* Spansion */

struct sfdp_bfpt {
	u32	dwords[BFPT_DWORD_MAX];
};

/* Fast Read settings. */

static void
spi_nor_set_read_settings_from_bfpt(struct spi_nor_read_command *read,
				    u16 half,
				    enum spi_nor_protocol proto)
{
	read->num_mode_clocks = (half >> 5) & 0x07;
	read->num_wait_states = (half >> 0) & 0x1f;
	read->opcode = (half >> 8) & 0xff;
	read->proto = proto;
}

struct sfdp_bfpt_read {
	/* The Fast Read x-y-z hardware capability in params->hwcaps.mask. */
	u32			hwcaps;

	/*
	 * The <supported_bit> bit in <supported_dword> BFPT DWORD tells us
	 * whether the Fast Read x-y-z command is supported.
	 */
	u32			supported_dword;
	u32			supported_bit;

	/*
	 * The half-word at offset <setting_shift> in <setting_dword> BFPT DWORD
	 * encodes the op code, the number of mode clocks and the number of wait
	 * states to be used by Fast Read x-y-z command.
	 */
	u32			settings_dword;
	u32			settings_shift;

	/* The SPI protocol for this Fast Read x-y-z command. */
	enum spi_nor_protocol	proto;
};

static const struct sfdp_bfpt_read sfdp_bfpt_reads[] = {
	/* Fast Read 1-1-2 */
	{
		SNOR_HWCAPS_READ_1_1_2,
		BFPT_DWORD(1), BIT(16),	/* Supported bit */
		BFPT_DWORD(4), 0,	/* Settings */
		SNOR_PROTO_1_1_2,
	},

	/* Fast Read 1-2-2 */
	{
		SNOR_HWCAPS_READ_1_2_2,
		BFPT_DWORD(1), BIT(20),	/* Supported bit */
		BFPT_DWORD(4), 16,	/* Settings */
		SNOR_PROTO_1_2_2,
	},

	/* Fast Read 2-2-2 */
	{
		SNOR_HWCAPS_READ_2_2_2,
		BFPT_DWORD(5),  BIT(0),	/* Supported bit */
		BFPT_DWORD(6), 16,	/* Settings */
		SNOR_PROTO_2_2_2,
	},

	/* Fast Read 1-1-4 */
	{
		SNOR_HWCAPS_READ_1_1_4,
		BFPT_DWORD(1), BIT(22),	/* Supported bit */
		BFPT_DWORD(3), 16,	/* Settings */
		SNOR_PROTO_1_1_4,
	},

	/* Fast Read 1-4-4 */
	{
		SNOR_HWCAPS_READ_1_4_4,
		BFPT_DWORD(1), BIT(21),	/* Supported bit */
		BFPT_DWORD(3), 0,	/* Settings */
		SNOR_PROTO_1_4_4,
	},

	/* Fast Read 4-4-4 */
	{
		SNOR_HWCAPS_READ_4_4_4,
		BFPT_DWORD(5), BIT(4),	/* Supported bit */
		BFPT_DWORD(7), 16,	/* Settings */
		SNOR_PROTO_4_4_4,
	},
};

struct sfdp_bfpt_erase {
	/*
	 * The half-word at offset <shift> in DWORD <dwoard> encodes the
	 * op code and erase sector size to be used by Sector Erase commands.
	 */
	u32			dword;
	u32			shift;
};

static const struct sfdp_bfpt_erase sfdp_bfpt_erases[] = {
	/* Erase Type 1 in DWORD8 bits[15:0] */
	{BFPT_DWORD(8), 0},

	/* Erase Type 2 in DWORD8 bits[31:16] */
	{BFPT_DWORD(8), 16},

	/* Erase Type 3 in DWORD9 bits[15:0] */
	{BFPT_DWORD(9), 0},

	/* Erase Type 4 in DWORD9 bits[31:16] */
	{BFPT_DWORD(9), 16},
};

static int spi_nor_hwcaps_read2cmd(u32 hwcaps);

/**
 * spi_nor_parse_bfpt() - read and parse the Basic Flash Parameter Table.
 * @nor:		pointer to a 'struct spi_nor'
 * @bfpt_header:	pointer to the 'struct sfdp_parameter_header' describing
 *			the Basic Flash Parameter Table length and version
 * @params:		pointer to the 'struct spi_nor_flash_parameter' to be
 *			filled
 *
 * The Basic Flash Parameter Table is the main and only mandatory table as
 * defined by the SFDP (JESD216) specification.
 * It provides us with the total size (memory density) of the data array and
 * the number of address bytes for Fast Read, Page Program and Sector Erase
 * commands.
 * For Fast READ commands, it also gives the number of mode clock cycles and
 * wait states (regrouped in the number of dummy clock cycles) for each
 * supported instruction op code.
 * For Page Program, the page size is now available since JESD216 rev A, however
 * the supported instruction op codes are still not provided.
 * For Sector Erase commands, this table stores the supported instruction op
 * codes and the associated sector sizes.
 * Finally, the Quad Enable Requirements (QER) are also available since JESD216
 * rev A. The QER bits encode the manufacturer dependent procedure to be
 * executed to set the Quad Enable (QE) bit in some internal register of the
 * Quad SPI memory. Indeed the QE bit, when it exists, must be set before
 * sending any Quad SPI command to the memory. Actually, setting the QE bit
 * tells the memory to reassign its WP# and HOLD#/RESET# pins to functions IO2
 * and IO3 hence enabling 4 (Quad) I/O lines.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_parse_bfpt(struct spi_nor *nor,
			      const struct sfdp_parameter_header *bfpt_header,
			      struct spi_nor_flash_parameter *params)
{
	struct mtd_info *mtd = &nor->mtd;
	struct sfdp_bfpt bfpt;
	size_t len;
	int i, cmd, err;
	u32 addr;
	u16 half;

	/* JESD216 Basic Flash Parameter Table length is at least 9 DWORDs. */
	if (bfpt_header->length < BFPT_DWORD_MAX_JESD216)
		return -EINVAL;

	/* Read the Basic Flash Parameter Table. */
	len = min_t(size_t, sizeof(bfpt),
		    bfpt_header->length * sizeof(u32));
	addr = SFDP_PARAM_HEADER_PTP(bfpt_header);
	memset(&bfpt, 0, sizeof(bfpt));
	err = spi_nor_read_sfdp(nor,  addr, len, &bfpt);
	if (err < 0)
		return err;

	/* Fix endianness of the BFPT DWORDs. */
	for (i = 0; i < BFPT_DWORD_MAX; i++)
		bfpt.dwords[i] = le32_to_cpu(bfpt.dwords[i]);

	/* Number of address bytes. */
	switch (bfpt.dwords[BFPT_DWORD(1)] & BFPT_DWORD1_ADDRESS_BYTES_MASK) {
	case BFPT_DWORD1_ADDRESS_BYTES_3_ONLY:
		nor->addr_width = 3;
		break;

	case BFPT_DWORD1_ADDRESS_BYTES_4_ONLY:
		nor->addr_width = 4;
		break;

	default:
		break;
	}

	/* Flash Memory Density (in bits). */
	params->size = bfpt.dwords[BFPT_DWORD(2)];
	if (params->size & BIT(31)) {
		params->size &= ~BIT(31);

		/*
		 * Prevent overflows on params->size. Anyway, a NOR of 2^64
		 * bits is unlikely to exist so this error probably means
		 * the BFPT we are reading is corrupted/wrong.
		 */
		if (params->size > 63)
			return -EINVAL;

		params->size = 1ULL << params->size;
	} else {
		params->size++;
	}
	params->size >>= 3; /* Convert to bytes. */

	/* Fast Read settings. */
	for (i = 0; i < ARRAY_SIZE(sfdp_bfpt_reads); i++) {
		const struct sfdp_bfpt_read *rd = &sfdp_bfpt_reads[i];
		struct spi_nor_read_command *read;

		if (!(bfpt.dwords[rd->supported_dword] & rd->supported_bit)) {
			params->hwcaps.mask &= ~rd->hwcaps;
			continue;
		}

		params->hwcaps.mask |= rd->hwcaps;
		cmd = spi_nor_hwcaps_read2cmd(rd->hwcaps);
		read = &params->reads[cmd];
		half = bfpt.dwords[rd->settings_dword] >> rd->settings_shift;
		spi_nor_set_read_settings_from_bfpt(read, half, rd->proto);
	}

	/* Sector Erase settings. */
	for (i = 0; i < ARRAY_SIZE(sfdp_bfpt_erases); i++) {
		const struct sfdp_bfpt_erase *er = &sfdp_bfpt_erases[i];
		u32 erasesize;
		u8 opcode;

		half = bfpt.dwords[er->dword] >> er->shift;
		erasesize = half & 0xff;

		/* erasesize == 0 means this Erase Type is not supported. */
		if (!erasesize)
			continue;

		erasesize = 1U << erasesize;
		opcode = (half >> 8) & 0xff;
#ifdef CONFIG_MTD_SPI_NOR_USE_4K_SECTORS
		if (erasesize == SZ_4K) {
			nor->erase_opcode = opcode;
			mtd->erasesize = erasesize;
			break;
		}
#endif
		if (!mtd->erasesize || mtd->erasesize < erasesize) {
			nor->erase_opcode = opcode;
			mtd->erasesize = erasesize;
		}
	}

	/* Stop here if not JESD216 rev A or later. */
	if (bfpt_header->length < BFPT_DWORD_MAX)
		return 0;

	/* Page size: this field specifies 'N' so the page size = 2^N bytes. */
	params->page_size = bfpt.dwords[BFPT_DWORD(11)];
	params->page_size &= BFPT_DWORD11_PAGE_SIZE_MASK;
	params->page_size >>= BFPT_DWORD11_PAGE_SIZE_SHIFT;
	params->page_size = 1U << params->page_size;

	/* Quad Enable Requirements. */
	switch (bfpt.dwords[BFPT_DWORD(15)] & BFPT_DWORD15_QER_MASK) {
	case BFPT_DWORD15_QER_NONE:
		params->quad_enable = NULL;
		break;
#if defined(CONFIG_SPI_FLASH_SPANSION) || defined(CONFIG_SPI_FLASH_WINBOND)
	case BFPT_DWORD15_QER_SR2_BIT1_BUGGY:
	case BFPT_DWORD15_QER_SR2_BIT1_NO_RD:
		params->quad_enable = spansion_no_read_cr_quad_enable;
		break;
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX
	case BFPT_DWORD15_QER_SR1_BIT6:
		params->quad_enable = macronix_quad_enable;
		break;
#endif
#if defined(CONFIG_SPI_FLASH_SPANSION) || defined(CONFIG_SPI_FLASH_WINBOND) || defined(CONFIG_SPI_FLASH_PUYA)

	case BFPT_DWORD15_QER_SR2_BIT1:
		params->quad_enable = spansion_read_cr_quad_enable;
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * spi_nor_parse_sfdp() - parse the Serial Flash Discoverable Parameters.
 * @nor:		pointer to a 'struct spi_nor'
 * @params:		pointer to the 'struct spi_nor_flash_parameter' to be
 *			filled
 *
 * The Serial Flash Discoverable Parameters are described by the JEDEC JESD216
 * specification. This is a standard which tends to supported by almost all
 * (Q)SPI memory manufacturers. Those hard-coded tables allow us to learn at
 * runtime the main parameters needed to perform basic SPI flash operations such
 * as Fast Read, Page Program or Sector Erase commands.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_parse_sfdp(struct spi_nor *nor,
			      struct spi_nor_flash_parameter *params)
{
	const struct sfdp_parameter_header *param_header, *bfpt_header;
	struct sfdp_parameter_header *param_headers = NULL;
	struct sfdp_header header;
	size_t psize;
	int i, err;

	/* Get the SFDP header. */
	err = spi_nor_read_sfdp(nor, 0, sizeof(header), &header);
	if (err < 0)
		return err;

	/* Check the SFDP header version. */
	if (le32_to_cpu(header.signature) != SFDP_SIGNATURE ||
	    header.major != SFDP_JESD216_MAJOR)
		return -EINVAL;

	/*
	 * Verify that the first and only mandatory parameter header is a
	 * Basic Flash Parameter Table header as specified in JESD216.
	 */
	bfpt_header = &header.bfpt_header;
	if (SFDP_PARAM_HEADER_ID(bfpt_header) != SFDP_BFPT_ID ||
	    bfpt_header->major != SFDP_JESD216_MAJOR)
		return -EINVAL;

	/*
	 * Allocate memory then read all parameter headers with a single
	 * Read SFDP command. These parameter headers will actually be parsed
	 * twice: a first time to get the latest revision of the basic flash
	 * parameter table, then a second time to handle the supported optional
	 * tables.
	 * Hence we read the parameter headers once for all to reduce the
	 * processing time. Also we use kmalloc() instead of devm_kmalloc()
	 * because we don't need to keep these parameter headers: the allocated
	 * memory is always released with kfree() before exiting this function.
	 */
	if (header.nph) {
		psize = header.nph * sizeof(*param_headers);

		param_headers = kmalloc(psize, GFP_KERNEL);
		if (!param_headers)
			return -ENOMEM;

		err = spi_nor_read_sfdp(nor, sizeof(header),
					psize, param_headers);
		if (err < 0) {
			dev_err(dev, "failed to read SFDP parameter headers\n");
			goto exit;
		}
	}

	/*
	 * Check other parameter headers to get the latest revision of
	 * the basic flash parameter table.
	 */
	for (i = 0; i < header.nph; i++) {
		param_header = &param_headers[i];

		if (SFDP_PARAM_HEADER_ID(param_header) == SFDP_BFPT_ID &&
		    param_header->major == SFDP_JESD216_MAJOR &&
		    (param_header->minor > bfpt_header->minor ||
		     (param_header->minor == bfpt_header->minor &&
		      param_header->length > bfpt_header->length)))
			bfpt_header = param_header;
	}

	err = spi_nor_parse_bfpt(nor, bfpt_header, params);
	if (err)
		goto exit;

	/* Parse other parameter headers. */
	for (i = 0; i < header.nph; i++) {
		param_header = &param_headers[i];

		switch (SFDP_PARAM_HEADER_ID(param_header)) {
		case SFDP_SECTOR_MAP_ID:
			dev_info(dev, "non-uniform erase sector maps are not supported yet.\n");
			break;

		default:
			break;
		}

		if (err)
			goto exit;
	}

exit:
	kfree(param_headers);
	return err;
}
#else
static int spi_nor_parse_sfdp(struct spi_nor *nor,
			      struct spi_nor_flash_parameter *params)
{
	return -EINVAL;
}
#endif /* SPI_FLASH_SFDP_SUPPORT */

static int spi_nor_init_params(struct spi_nor *nor,
			       const struct flash_info *info,
			       struct spi_nor_flash_parameter *params)
{
	/* Set legacy flash parameters as default. */
	memset(params, 0, sizeof(*params));

	/* Set SPI NOR sizes. */
	params->size = info->sector_size * info->n_sectors;
	params->page_size = info->page_size;

	/* (Fast) Read settings. */
	params->hwcaps.mask |= SNOR_HWCAPS_READ;
	spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ],
				  0, 0, SPINOR_OP_READ,
				  SNOR_PROTO_1_1_1);

	if (!(info->flags & SPI_NOR_NO_FR)) {
		params->hwcaps.mask |= SNOR_HWCAPS_READ_FAST;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_FAST],
					  0, 8, SPINOR_OP_READ_FAST,
					  SNOR_PROTO_1_1_1);
	}

	if (info->flags & SPI_NOR_DUAL_READ) {
		params->hwcaps.mask |= SNOR_HWCAPS_READ_1_1_2;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_1_2],
					  0, 8, SPINOR_OP_READ_1_1_2,
					  SNOR_PROTO_1_1_2);
#ifdef CONFIG_SUNXI_SPIF
		if (info->flags & USE_IO_MODE) {
			if (info->flags & USE_RX_DTR) {
				params->hwcaps.mask |= SNOR_HWCAPS_READ_1_2_2_DTR;
				spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_2_2_DTR],
							  0, 0, SPINOR_OP_READ_1_2_2_DTR,
							  SNOR_PROTO_1_2_2_DTR);
			} else {
				params->hwcaps.mask |= SNOR_HWCAPS_READ_1_2_2;
				spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_2_2],
							  0, 0, SPINOR_OP_READ_1_2_2,
							  SNOR_PROTO_1_2_2);
			}
		}
#endif
	}

	if (info->flags & SPI_NOR_QUAD_READ) {
		params->hwcaps.mask |= SNOR_HWCAPS_READ_1_1_4;
		spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_1_4],
					  0, 8, SPINOR_OP_READ_1_1_4,
					  SNOR_PROTO_1_1_4);
#ifdef CONFIG_SUNXI_SPIF
		if (info->flags & USE_IO_MODE) {
			if (info->flags & USE_RX_DTR) {
				params->hwcaps.mask |= SNOR_HWCAPS_READ_1_4_4_DTR;
				spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_4_4_DTR],
							  0, 7, SPINOR_OP_READ_1_4_4_DTR,
							  SNOR_PROTO_1_4_4_DTR);
			} else {
				params->hwcaps.mask |= SNOR_HWCAPS_READ_1_4_4;
				spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_4_4],
							  0, 4, SPINOR_OP_READ_1_4_4,
							  SNOR_PROTO_1_4_4);
			}
		}
#endif
	}

	/* Page Program settings. */
	params->hwcaps.mask |= SNOR_HWCAPS_PP;
	spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP],
				SPINOR_OP_PP, SNOR_PROTO_1_1_1);

	if (info->flags & SPI_NOR_QUAD_READ) {
		params->hwcaps.mask |= SNOR_HWCAPS_PP_1_1_4;
		spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP_1_1_4],
					SPINOR_OP_PP_1_1_4, SNOR_PROTO_1_1_4);
	}

	/* Select the procedure to set the Quad Enable bit. */
	if (params->hwcaps.mask & (SNOR_HWCAPS_READ_QUAD |
				   SNOR_HWCAPS_PP_QUAD)) {
		switch (JEDEC_MFR(info)) {
#if defined(CONFIG_SPI_FLASH_MACRONIX) || defined(CONFIG_SPI_FLASH_XMC)
		case SNOR_MFR_MACRONIX:
		case SNOR_MFR_XMC:
			//SNOR_MFR_MICRON
			if (info->id[1] >> 4 == 'b' && JEDEC_MFR(info) == SNOR_MFR_XMC) //because XMC and MICRON id[0] equal
				return 0;
			params->quad_enable = macronix_quad_enable;
			break;
#endif
#if defined(CONFIG_SPI_FLASH_GIGADEVICE) || defined(CONFIG_SPI_FLASH_ADESTO) || defined(CONFIG_SPI_FLASH_PUYA)
		case SNOR_MFR_GIGADEVICE:
		case SNOR_MFR_ADESTO:
		case SNOR_MFR_PUYA:
		case SNOR_MFR_ZETTA:
		case SNOR_MFR_BOYA:
			params->quad_enable = gd_read_cr_quad_enable;
			break;
#endif
#ifdef CONFIG_SPI_FLASH_EON
		case SNOR_MFR_EON:
			switch (JEDEC_ID(info)) {
			case GM_64A_ID:
			case GM_128A_ID:
				params->quad_enable = gd_read_cr_quad_enable;
				break;
			default:
				params->quad_enable = Eon_quad_enable;
				break;
			}
#endif
//		case SNOR_MFR_ST:
		case SNOR_MFR_MICRON:
			break;

#if defined(CONFIG_SPI_FLASH_SPANSION) || defined(CONFIG_SPI_FLASH_WINBOND) || defined(CONFIG_SPI_FLASH_FM) \
			|| defined(CONFIG_SPI_FLASH_XT)
		case SNOR_MFR_SPANSION:
		case SNOR_MFR_WINBOND:
		case SNOR_MFR_XTX:
		case SNOR_MFR_FM:
			/* Kept only for backward compatibility purpose. */
			params->quad_enable = spansion_read_cr_quad_enable;
			break;
#endif
		default:
			dev_err(nor->dev, "SF: Need set QEB func for %02x flash\n",
				JEDEC_MFR(nor->info));
			break;
		}
	}

	/* Override the parameters with data read from SFDP tables. */
	nor->mtd.erasesize = 0;
	if ((info->flags & (SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)) &&
	    !(info->flags & SPI_NOR_SKIP_SFDP)) {
		struct spi_nor_flash_parameter sfdp_params;

		memcpy(&sfdp_params, params, sizeof(sfdp_params));
		if (spi_nor_parse_sfdp(nor, &sfdp_params)) {
			nor->mtd.erasesize = 0;
		} else {
			memcpy(params, &sfdp_params, sizeof(*params));
		}
	}

	return 0;
}

static int spi_nor_hwcaps2cmd(u32 hwcaps, const int table[][2], size_t size)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (table[i][0] == (int)hwcaps)
			return table[i][1];

	return -EINVAL;
}

static int spi_nor_hwcaps_read2cmd(u32 hwcaps)
{
	static const int hwcaps_read2cmd[][2] = {
		{ SNOR_HWCAPS_READ,		SNOR_CMD_READ },
		{ SNOR_HWCAPS_READ_FAST,	SNOR_CMD_READ_FAST },
		{ SNOR_HWCAPS_READ_1_1_1_DTR,	SNOR_CMD_READ_1_1_1_DTR },
		{ SNOR_HWCAPS_READ_1_1_2,	SNOR_CMD_READ_1_1_2 },
		{ SNOR_HWCAPS_READ_1_2_2,	SNOR_CMD_READ_1_2_2 },
		{ SNOR_HWCAPS_READ_2_2_2,	SNOR_CMD_READ_2_2_2 },
		{ SNOR_HWCAPS_READ_1_2_2_DTR,	SNOR_CMD_READ_1_2_2_DTR },
		{ SNOR_HWCAPS_READ_1_1_4,	SNOR_CMD_READ_1_1_4 },
		{ SNOR_HWCAPS_READ_1_4_4,	SNOR_CMD_READ_1_4_4 },
		{ SNOR_HWCAPS_READ_4_4_4,	SNOR_CMD_READ_4_4_4 },
		{ SNOR_HWCAPS_READ_1_4_4_DTR,	SNOR_CMD_READ_1_4_4_DTR },
		{ SNOR_HWCAPS_READ_1_1_8,	SNOR_CMD_READ_1_1_8 },
		{ SNOR_HWCAPS_READ_1_8_8,	SNOR_CMD_READ_1_8_8 },
		{ SNOR_HWCAPS_READ_8_8_8,	SNOR_CMD_READ_8_8_8 },
		{ SNOR_HWCAPS_READ_1_8_8_DTR,	SNOR_CMD_READ_1_8_8_DTR },
	};

	return spi_nor_hwcaps2cmd(hwcaps, hwcaps_read2cmd,
				  ARRAY_SIZE(hwcaps_read2cmd));
}

static int spi_nor_hwcaps_pp2cmd(u32 hwcaps)
{
	static const int hwcaps_pp2cmd[][2] = {
		{ SNOR_HWCAPS_PP,		SNOR_CMD_PP },
		{ SNOR_HWCAPS_PP_1_1_4,		SNOR_CMD_PP_1_1_4 },
		{ SNOR_HWCAPS_PP_1_4_4,		SNOR_CMD_PP_1_4_4 },
		{ SNOR_HWCAPS_PP_4_4_4,		SNOR_CMD_PP_4_4_4 },
		{ SNOR_HWCAPS_PP_1_1_8,		SNOR_CMD_PP_1_1_8 },
		{ SNOR_HWCAPS_PP_1_8_8,		SNOR_CMD_PP_1_8_8 },
		{ SNOR_HWCAPS_PP_8_8_8,		SNOR_CMD_PP_8_8_8 },
	};

	return spi_nor_hwcaps2cmd(hwcaps, hwcaps_pp2cmd,
				  ARRAY_SIZE(hwcaps_pp2cmd));
}

static int spi_nor_select_read(struct spi_nor *nor,
			       const struct spi_nor_flash_parameter *params,
			       u32 shared_hwcaps)
{
	int cmd, best_match = fls(shared_hwcaps & SNOR_HWCAPS_READ_MASK) - 1;
	const struct spi_nor_read_command *read;

	if (best_match < 0)
		return -EINVAL;

	cmd = spi_nor_hwcaps_read2cmd(BIT(best_match));
	if (cmd < 0)
		return -EINVAL;

	read = &params->reads[cmd];
	nor->read_opcode = read->opcode;
	nor->read_proto = read->proto;

	/*
	 * In the spi-nor framework, we don't need to make the difference
	 * between mode clock cycles and wait state clock cycles.
	 * Indeed, the value of the mode clock cycles is used by a QSPI
	 * flash memory to know whether it should enter or leave its 0-4-4
	 * (Continuous Read / XIP) mode.
	 * eXecution In Place is out of the scope of the mtd sub-system.
	 * Hence we choose to merge both mode and wait state clock cycles
	 * into the so called dummy clock cycles.
	 */
	nor->read_dummy = read->num_mode_clocks + read->num_wait_states;
	return 0;
}

static int spi_nor_select_pp(struct spi_nor *nor,
			     const struct spi_nor_flash_parameter *params,
			     u32 shared_hwcaps)
{
	int cmd, best_match = fls(shared_hwcaps & SNOR_HWCAPS_PP_MASK) - 1;
	const struct spi_nor_pp_command *pp;

	if (best_match < 0)
		return -EINVAL;

	cmd = spi_nor_hwcaps_pp2cmd(BIT(best_match));
	if (cmd < 0)
		return -EINVAL;

	pp = &params->page_programs[cmd];
	nor->program_opcode = pp->opcode;
	nor->write_proto = pp->proto;
	return 0;
}

static int spi_nor_select_erase(struct spi_nor *nor,
				const struct flash_info *info)
{
	struct mtd_info *mtd = &nor->mtd;

	/* Do nothing if already configured from SFDP. */
	if (mtd->erasesize)
		return 0;

#ifdef CONFIG_SPI_FLASH_USE_4K_SECTORS
	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		nor->erase_opcode = SPINOR_OP_BE_4K;
		mtd->erasesize = 4096;
	} else if (info->flags & SECT_4K_PMC) {
		nor->erase_opcode = SPINOR_OP_BE_4K_PMC;
		mtd->erasesize = 4096;
	} else
#endif
	{
		nor->erase_opcode = SPINOR_OP_SE;
		mtd->erasesize = info->sector_size;
	}
	return 0;
}

static int spi_nor_setup(struct spi_nor *nor, const struct flash_info *info,
			 const struct spi_nor_flash_parameter *params,
			 const struct spi_nor_hwcaps *hwcaps)
{
	u32 ignored_mask, shared_mask;
	bool enable_quad_io;
	int err;

	/*
	 * Keep only the hardware capabilities supported by both the SPI
	 * controller and the SPI flash memory.
	 */
	shared_mask = hwcaps->mask & params->hwcaps.mask;

	/* SPI n-n-n protocols are not supported yet. */
	ignored_mask = (SNOR_HWCAPS_READ_2_2_2 |
			SNOR_HWCAPS_READ_4_4_4 |
			SNOR_HWCAPS_PP_4_4_4);
	if (shared_mask & ignored_mask) {
		dev_dbg(nor->dev,
			"SPI n-n-n protocols are not supported yet.\n");
		shared_mask &= ~ignored_mask;
	}

	/* Select the (Fast) Read command. */
	err = spi_nor_select_read(nor, params, shared_mask);
	if (err) {
		dev_dbg(nor->dev,
			"can't select read settings supported by both the SPI controller and memory.\n");
		return err;
	}

	/* Select the Page Program command. */
	err = spi_nor_select_pp(nor, params, shared_mask);
	if (err) {
		dev_dbg(nor->dev,
			"can't select write settings supported by both the SPI controller and memory.\n");
		return err;
	}

	/* Select the Sector Erase command. */
	err = spi_nor_select_erase(nor, info);
	if (err) {
		dev_dbg(nor->dev,
			"can't select erase settings supported by both the SPI controller and memory.\n");
		return err;
	}

	/* Enable Quad I/O if needed. */
	enable_quad_io = (spi_nor_get_protocol_width(nor->read_proto) == 4 ||
			  spi_nor_get_protocol_width(nor->write_proto) == 4);
	if (enable_quad_io && params->quad_enable)
		nor->quad_enable = params->quad_enable;
	else
		nor->quad_enable = NULL;

	if (nor->addr_width == 4 && (JEDEC_MFR(info) == SNOR_MFR_SPANSION ||
			info->flags & SPI_NOR_4B_OPCODES))
		spi_nor_set_4byte_opcodes(nor, info);
	return 0;
}

static int sunxi_lock_init(struct spi_nor *nor)
{
	struct mtd_info *mtd = &nor->mtd;
	const struct flash_info *info = nor->info;

	/*
	 * If Burn mode Unlocks the Flash, Direct return
	 */
	if (get_boot_work_mode() == WORK_MODE_USB_PRODUCT ||
			get_boot_work_mode() == WORK_MODE_CARD_PRODUCT) {
		if (sunxi_individual_lock_is_enable(nor))
			sunxi_individual_unlock_global(nor);

		if (JEDEC_MFR(info) == SNOR_MFR_ST ||
			    JEDEC_MFR(info) == SNOR_MFR_MICRON ||
			    JEDEC_MFR(info) == SNOR_MFR_SST)
			stm_unlock(nor, 0, mtd->size);
		else
			sunxi_unlock(nor, 0, mtd->size);

		return 0;
	}

	/* NOR protection support for STmicro/Micron chips and similar */
	if (info->flags & SPI_NOR_HAS_LOCK) {
		sunxi_individual_lock_init(nor);

		if (info->flags & SPI_NOR_HAS_LOCK_HANDLE)
			nor->flags |= SNOR_F_HAS_LOCK_HANDLE;

		if (nor->flags & SNOR_F_INDIVIDUAL_LOCK) {
			nor->flash_lock = sunxi_individual_lock;
			nor->flash_unlock = sunxi_individual_unlock;
			nor->flash_is_locked = sunxi_individual_is_lock;
		} else {
			if (JEDEC_MFR(info) == SNOR_MFR_ST ||
			    JEDEC_MFR(info) == SNOR_MFR_MICRON ||
			    JEDEC_MFR(info) == SNOR_MFR_SST) {
				nor->flash_lock = stm_lock;
				nor->flash_unlock = stm_unlock;
				nor->flash_is_locked = stm_is_locked;
			} else {
				nor->flash_lock = sunxi_lock;
				nor->flash_unlock = sunxi_unlock;
				nor->flash_is_locked = sunxi_is_locked;
			}
		}
	} else {
		/*
		 * If the lock is not used and the Flash is locked,
		 * it will be unlocked globally
		 */
		if (sunxi_individual_lock_is_enable(nor))
			sunxi_individual_unlock_global(nor);

		if (JEDEC_MFR(info) == SNOR_MFR_ST ||
		    JEDEC_MFR(info) == SNOR_MFR_MICRON ||
		    JEDEC_MFR(info) == SNOR_MFR_SST)
			stm_unlock(nor, 0, mtd->size);
		else
			sunxi_unlock(nor, 0, mtd->size);
	}

	if (nor->flash_lock && nor->flash_unlock && nor->flash_is_locked) {
		mtd->_lock = spi_nor_lock;
		mtd->_unlock = spi_nor_unlock;
		mtd->_is_locked = spi_nor_is_locked;

		/* If the lock handle function is enabled, lock the flash */
		if (nor->flags & SNOR_F_HAS_LOCK_HANDLE) {
			pr_warn("enable lock handle function\n");
			mtd->_erase = spi_nor_has_lock_erase;
			mtd->_write = spi_nor_has_lock_write;
			mtd->_lock(mtd, 0, mtd->size);
		}
	}

	return 0;
}

static int spi_nor_init(struct spi_nor *nor)
{
	int err;

	if (nor->quad_enable) {
		err = nor->quad_enable(nor);
		if (err) {
			dev_err(nor->dev, "quad mode not supported\n");
			return err;
		}
	}

	if (nor->addr_width == 4 &&
	    (JEDEC_MFR(nor->info) != SNOR_MFR_SPANSION) &&
	    !(nor->info->flags & SPI_NOR_4B_OPCODES)) {
		/*
		 * If the RESET# pin isn't hooked up properly, or the system
		 * otherwise doesn't perform a reset command in the boot
		 * sequence, it's impossible to 100% protect against unexpected
		 * reboots (e.g., crashes). Warn the user (or hopefully, system
		 * designer) that this is bad.
		 */
		if (nor->flags & SNOR_F_BROKEN_RESET)
			printf("enabling reset hack; may not recover from unexpected reboots\n");
		set_4byte(nor, nor->info, 1);
	}

	return 0;
}

int spi_nor_scan(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter params;
	const struct flash_info *info = NULL;
	struct mtd_info *mtd = &nor->mtd;
	struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_PP,
	};
	struct spi_slave *spi = nor->spi;
	int ret;
	u8 total_stack_die = 2; //FIXME

	/* Reset SPI protocol for all commands. */
	nor->reg_proto = SNOR_PROTO_1_1_1;
	nor->read_proto = SNOR_PROTO_1_1_1;
	nor->write_proto = SNOR_PROTO_1_1_1;
	nor->read = spi_nor_read_data;
	nor->write = spi_nor_write_data;
	nor->read_reg = spi_nor_read_reg;
#ifdef CONFIG_SUNXI_SPIF
	nor->write_reg = spif_nor_write_reg;
#else
	nor->write_reg = spi_nor_write_reg;
#endif

	info = spi_nor_read_id(nor);
	if (IS_ERR_OR_NULL(info))
		return -ENOENT;

	if (spi->mode & SPI_RX_QUAD && info->flags & SPI_NOR_QUAD_READ) {
		hwcaps.mask |= (SNOR_HWCAPS_READ_1_1_4 |
				SNOR_HWCAPS_READ_1_4_4 |
				SNOR_HWCAPS_READ_1_4_4_DTR);

		if (spi->mode & SPI_TX_QUAD)
			hwcaps.mask |= (SNOR_HWCAPS_PP_1_1_4 |
					SNOR_HWCAPS_PP_1_4_4);
	} else if (spi->mode & SPI_RX_DUAL && info->flags & SPI_NOR_DUAL_READ) {
		hwcaps.mask |= SNOR_HWCAPS_READ_1_1_2;

		if (spi->mode & SPI_TX_DUAL)
			hwcaps.mask |= SNOR_HWCAPS_READ_1_2_2;
	}
	if (info->flags & SPI_NOR_OCTAL_READ) {
		hwcaps.mask |= (SNOR_HWCAPS_READ_1_1_8 |
				SNOR_HWCAPS_PP_1_1_8 |
				SNOR_HWCAPS_READ_8_8_8 |
				SNOR_HWCAPS_PP_8_8_8);
	}

	/* Parse the Serial Flash Discoverable Parameters table. */
	ret = spi_nor_init_params(nor, info, &params);
	if (ret)
		return ret;

	if (!mtd->name)
		mtd->name = info->name;
	mtd->priv = nor;
	mtd->type = MTD_NORFLASH;
	mtd->writesize = 1;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->size = params.size;
	mtd->_erase = spi_nor_erase;
	mtd->_force_erase = spi_nor_force_erase;
	mtd->_read = spi_nor_read;

#ifdef CONFIG_SPI_FLASH_SST
	/* sst nor chips use AAI word program */
	if (info->flags & SST_WRITE)
		mtd->_write = sst_write;
	else
#endif
		mtd->_write = spi_nor_write;

	if (info->flags & USE_FSR)
		nor->flags |= SNOR_F_USE_FSR;
	if (info->flags & SPI_NOR_HAS_TB)
		nor->flags |= SNOR_F_HAS_SR_TB;
	if (info->flags & NO_CHIP_ERASE)
		nor->flags |= SNOR_F_NO_OP_CHIP_ERASE;
	if (info->flags & USE_CLSR)
		nor->flags |= SNOR_F_USE_CLSR;

	if (info->flags & SPI_NOR_NO_ERASE)
		mtd->flags |= MTD_NO_ERASE;

	nor->page_size = params.page_size;
	mtd->writebufsize = nor->page_size;

	if (nor->addr_width) {
		/* already configured from SFDP */
	} else if (info->addr_width) {
		nor->addr_width = info->addr_width;
	} else if (mtd->size > SZ_16M) {
#ifndef CONFIG_SPI_FLASH_BAR
		/* enable 4-byte addressing if the device exceeds 16MiB */
		nor->addr_width = 4;
#else
	/* Configure the BAR - discover bank cmds and read current bank */
	nor->addr_width = 3;
	ret = read_bar(nor, info);
	if (ret < 0)
		return ret;
#endif
	} else {
		nor->addr_width = 3;
	}

	if (nor->addr_width > SPI_NOR_MAX_ADDR_WIDTH) {
		dev_dbg(dev, "address width is too large: %u\n",
			nor->addr_width);
		return -EINVAL;
	}

	/*
	 * Configure the SPI memory:
	 * - select op codes for (Fast) Read, Page Program and Sector Erase.
	 * - set the number of dummy cycles (mode cycles + wait states).
	 * - set the SPI protocols for register and memory accesses.
	 * - set the Quad Enable bit if needed (required by SPI x-y-4 protos).
	 */
	ret = spi_nor_setup(nor, info, &params, &hwcaps);
	if (ret)
		return ret;

	nor->name = mtd->name;
	nor->size = mtd->size;
	nor->erase_size = mtd->erasesize;
	nor->sector_size = mtd->erasesize;
	nor->info = info;

	if (info->flags & SPI_NOR_STACK_DIE) {
		ret = sunxi_select_die(nor, 0);
		if (ret)
			return ret;
	}

init_die:
	/* Send all the required SPI flash commands to initialize device */
	ret = spi_nor_init(nor);
	if (ret)
		return ret;

	sunxi_lock_init(nor);

	total_stack_die--;

	if ((info->flags & SPI_NOR_STACK_DIE) && total_stack_die) {
		s8 select_die = total_stack_die;
		ret = sunxi_select_die(nor, select_die);
		if (ret)
			return ret;
		else
			goto init_die;
	}

#if defined(CONFIG_SUNXI_SPIF) && defined(CONFIG_SUNXI_SPINOR)
	spif_init_dtr(nor->info->flags);

	if (get_boot_work_mode() == WORK_MODE_USB_PRODUCT ||
			get_boot_work_mode() == WORK_MODE_CARD_PRODUCT) {
		spif_update_right_delay_para(mtd);
	} else {
		spif_set_right_delay_para(mtd);
	}
#else
#if defined(CONFIG_SPI_SAMP_DL_EN) && defined(CONFIG_SUNXI_SPINOR)
	if (get_boot_work_mode() == WORK_MODE_USB_PRODUCT ||
			get_boot_work_mode() == WORK_MODE_CARD_PRODUCT)
		sunxi_update_right_delay_para(mtd);
	else {
		sunxi_set_right_delay_para(mtd);
	}
#else
	spi_init_clk(spi);
#endif
#endif /* CONFIG_SUNXI_SPIF */

#ifndef CONFIG_SPL_BUILD
	printf("SF: Detected %s(%s %s) with page size ",
			nor->name,
			info->flags & USE_IO_MODE ? "IO" : "",
			info->flags & USE_RX_DTR ? "DTR" : "");
	print_size(nor->page_size, ", erase size ");
	print_size(nor->erase_size, ", total ");
	print_size(nor->size, "");
	puts("\n");
#endif

	return 0;
}

/* U-Boot specific functions, need to extend MTD to support these */
int spi_flash_cmd_get_sw_write_prot(struct spi_nor *nor)
{
	int sr = read_sr(nor);

	if (sr < 0)
		return sr;

	return (sr >> 2) & 7;
}
