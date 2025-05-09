/*
 * (C) Copyright 2017-2018
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * wangwei <wangwei@allwinnertech.com>
 *
 * Configuration settings for the Allwinner sunxi series of boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <smc.h>
#include <sunxi_board.h>
#include "teesmc_v2.h"
#include <securestorage.h>
#include <efuse_map.h>
#include <memalign.h>
#include <sunxi_keybox.h>
#include <asm/arch/ce.h>

DECLARE_GLOBAL_DATA_PTR;

#define ARM_SVC_CALL_COUNT          0x8000ff00
#define ARM_SVC_UID                 0x8000ff01
/* 0x8000ff02 reserved */
#define ARM_SVC_VERSION             0x8000ff03
#define ARM_SVC_RUNNSOS             0x8000ff04

#ifdef CONFIG_SUNXI_ARM64
#define ARM_SVC_READ_SEC_REG        0x8000ff05
#define ARM_SVC_WRITE_SEC_REG       0x8000ff06
#else
#define ARM_SVC_READ_SEC_REG        OPTEE_SMC_READ_REG
#define ARM_SVC_WRITE_SEC_REG       OPTEE_SMC_WRITE_REG
#endif

#define PSCI_CPU_OFF                0x84000002
#define PSCI_CPU_ON_AARCH32         0x84000003
#define PSCI_SYSTEM_OFF             0x84000008

#define SUNXI_CPU_ON_AARCH32        0x84000010
#define SUNXI_CPU_OFF_AARCH32       0x84000011
#define SUNXI_CPU_WFI_AARCH32       0x84000012
/* arisc */
#define ARM_SVC_ARISC_STARTUP                   0x8000ff10
#define ARM_SVC_ARISC_WAIT_READY                0x8000ff11
#define ARM_SVC_ARISC_READ_PMU                  0x8000ff12
#define ARM_SVC_ARISC_WRITE_PMU                 0x8000ff13
#define ARM_SVC_ARISC_FAKE_POWER_OFF_REQ_ARCH32 0x83000019
#define ARM_SVC_FAKE_POWER_OFF       0x8000ff14
#define ARM_SVC_UBOOT_POWER_OFF       0x8000ff15

/* efuse */
#define ARM_SVC_EFUSE_READ         (0x8000fe00)
#define ARM_SVC_EFUSE_WRITE        (0x8000fe01)
#define ARM_SVC_EFUSE_PROBE_SECURE_ENABLE_AARCH32    (0x8000fe03)
#define ARM_SVC_EFUSE_CUSTOMER_RESERVED_HANDLE		(0x8000fe05)

/*
*note pResult is will
*/
u32 sunxi_smc_call_optee(ulong arg0, ulong arg1, ulong arg2,
		ulong arg3, ulong pResult)
{
	struct arm_smccc_res param = { 0 };
	arm_smccc_smc(arg0, arg1, arg2, arg3, 0, 0, 0, 0, &param);
	return param.a0;
}

/*
*note pResult is will
*/
u32 sunxi_smc_call_atf(ulong arg0, ulong arg1, ulong arg2, ulong arg3,
	ulong pResult)
{
	return __sunxi_smc_call(arg0, arg1, arg2, arg3);
}

u32 sunxi_smc_call(ulong arg0, ulong arg1, ulong arg2,
		ulong arg3, ulong pResult)
{
#ifdef CONFIG_SUNXI_ARM64
	return sunxi_smc_call_atf(arg0, arg1, arg2, arg3, pResult);
#else
	return sunxi_smc_call_optee(arg0, arg1, arg2, arg3, pResult);
#endif
}

int arm_svc_set_cpu_on(int cpu, uint entry)
{
	return sunxi_smc_call(PSCI_CPU_ON_AARCH32, cpu, entry, 0, 0);
}

int arm_svc_set_cpu_off(int cpu)
{
	return sunxi_smc_call(PSCI_CPU_OFF, cpu, 0, 0, 0);
}

/*for multi cluster*/
int sunxi_smc_set_cpu_entry(u32 entry, int cpu)
{
	struct arm_smccc_res res;
	arm_smccc_smc(OPTEE_SMC_PLATFORM_CMD,
		TEE_SET_CPU_ENTRY, entry, cpu, 0, 0, 0, 0, &res);
	return res.a0;
}

/*for multi cluster*/
int sunxi_smc_set_cpu_off(void)
{
	struct arm_smccc_res res;
	arm_smccc_smc(OPTEE_SMC_PLATFORM_CMD,
		TEE_SET_CPU_OFF, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}


int arm_svc_version(u32 *major, u32 *minor)
{
	u32 *pResult = NULL;
	int ret = 0;
	ret = sunxi_smc_call_atf(ARM_SVC_VERSION, 0, 0, 0, (ulong)&pResult);
	if (ret < 0)
		return ret;

	*major = pResult[0];
	*minor = pResult[1];
	return ret;
}

int arm_svc_call_count(void)
{
	return sunxi_smc_call_atf(ARM_SVC_CALL_COUNT, 0, 0, 0, 0);
}

int arm_svc_uuid(u32 *uuid)
{
	return sunxi_smc_call_atf(ARM_SVC_UID, 0, 0, 0, (ulong)uuid);
}

int arm_svc_run_os(ulong kernel, ulong fdt, ulong arg2)
{
	return sunxi_smc_call_atf(ARM_SVC_RUNNSOS, kernel, fdt, arg2, 0);
}

u32 arm_svc_read_sec_reg(ulong reg)
{
	return sunxi_smc_call(ARM_SVC_READ_SEC_REG, reg, 0, 0, 0);
}

int arm_svc_write_sec_reg(u32 val, ulong reg)
{
	sunxi_smc_call(ARM_SVC_WRITE_SEC_REG, reg, val, 0, 0);
	return 0;
}

int arm_svc_arisc_startup(ulong cfg_base)
{
	return sunxi_smc_call_atf(ARM_SVC_ARISC_STARTUP, cfg_base, 0, 0, 0);
}

int arm_svc_arisc_wait_ready(void)
{
	return sunxi_smc_call_atf(ARM_SVC_ARISC_WAIT_READY, 0, 0, 0, 0);
}

int arm_svc_arisc_fake_poweroff(void)
{
	return sunxi_smc_call(ARM_SVC_ARISC_FAKE_POWER_OFF_REQ_ARCH32, 0, 0, 0, 0);
}

int arm_svc_fake_poweroff(ulong dtb_base)
{
	return sunxi_smc_call(ARM_SVC_FAKE_POWER_OFF, dtb_base, 0, 0, 0);
}

int arm_svc_poweroff(void)
{
	return sunxi_smc_call(ARM_SVC_UBOOT_POWER_OFF, 0, 0, 0, 0);
}

int arm_svc_poweroff_charge(void)
{
	return sunxi_smc_call(PSCI_SYSTEM_OFF, 0, 0, 0, 0);
}

u32 arm_svc_arisc_read_pmu(ulong addr)
{
	return sunxi_smc_call_atf(ARM_SVC_ARISC_READ_PMU, addr, 0, 0, 0);
}

int arm_svc_arisc_write_pmu(ulong addr, u32 value)
{
	return sunxi_smc_call_atf(ARM_SVC_ARISC_WRITE_PMU, addr, value, 0, 0);
}

int arm_svc_efuse_read(void *key_buf, void *read_buf)
{
#if defined(CONFIG_SUNXI_ARM64)
	return sunxi_smc_call(ARM_SVC_EFUSE_READ,
		(ulong)key_buf, (ulong)read_buf, 0, 0);
#else
	struct arm_smccc_res res;
	efuse_key_info_t *efuse_info;
	int offset, len;
	memset(&res, 0, sizeof(res));
		/*probe share memory*/
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
	0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	efuse_info = (efuse_key_info_t *)res.a1;
	if (!efuse_info)
		return -1;

	memcpy(efuse_info, key_buf, sizeof(efuse_key_info_t));

	memset(&res, 0, sizeof(res));

	sunxi_smc_call(OPTEE_SMC_EFUSE_OP, EFUSE_OP_RD,
		(ulong)efuse_info, 0, 0);

	offset = ROUND(sizeof(efuse_key_info_t), 8);
	len = *(int *)((u32)efuse_info + SUNXI_KEY_NAME_LEN);
	memcpy(read_buf, (void *)((u32)efuse_info + offset), len);
	return len;
#endif
}

#ifdef CONFIG_SUNXI_SET_EFUSE_POWER
extern int set_efuse_voltage(int vol);
#endif
int arm_svc_efuse_write(void *key_buf)
{
	int ret = -1;

#ifdef CONFIG_SUNXI_SET_EFUSE_POWER
	/* set efuse voltage before burn efuse */
	set_efuse_voltage(1);
#endif

#if defined(CONFIG_SUNXI_ARM64)
	ret = sunxi_smc_call(ARM_SVC_EFUSE_WRITE,
				(ulong)key_buf, 0, 0, 0);
#else
	struct arm_smccc_res res;
	efuse_key_info_t *efuse_info;
	efuse_key_info_t *wd = (efuse_key_info_t *)key_buf;
	int offset;
	memset(&res, 0, sizeof(res));
	/*probe share memory*/
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
	0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		goto out;
	}

	efuse_info = (efuse_key_info_t *)res.a1;
	if (!efuse_info)
		goto out;

	memcpy(efuse_info, key_buf, sizeof(efuse_key_info_t));
	offset = ROUND(sizeof(efuse_key_info_t), 8);
	memcpy((void *)((u32)efuse_info + offset), wd->key_data,
			wd->len);
	efuse_info->key_data = (u8 *)((u32)efuse_info + offset);
	memset(&res, 0, sizeof(res));

	/*For efuse, we can makesure 0x200 is enough*/
	flush_cache((ulong)efuse_info, 0x200);
	ret = sunxi_smc_call(OPTEE_SMC_EFUSE_OP, EFUSE_OP_WR,
		(ulong)efuse_info, 0, 0);
 out:
#endif

#ifdef CONFIG_SUNXI_SET_EFUSE_POWER
	/* close efuse power after burning */
	set_efuse_voltage(0);
#endif

	return ret;
}

int arm_svc_probe_secure_mode(void)
{
	return sunxi_smc_call_atf(ARM_SVC_EFUSE_PROBE_SECURE_ENABLE_AARCH32,
		0, 0, 0, 0);
}

int arm_svc_customer_encrypt(u32 customer_reserved_id)
{
	return sunxi_smc_call(ARM_SVC_EFUSE_CUSTOMER_RESERVED_HANDLE,
		customer_reserved_id, 0, 0, 0);
}

static  u32 smc_readl_normal(ulong addr)
{
	return readl(addr);
}
static  int smc_writel_normal(u32 value, ulong addr)
{
	writel(value, addr);
	return 0;
}

u32 (*smc_readl_pt)(ulong addr) = smc_readl_normal;
int (*smc_writel_pt)(u32 value, ulong addr) = smc_writel_normal;
uint32_t sunxi_smc_call_offset = 0xFFFFFFFF;

u32 smc_readl(ulong addr)
{
	return smc_readl_pt(addr);
}

void smc_writel(u32 val, ulong addr)
{
	smc_writel_pt(val, addr);
}

int smc_efuse_readl(void *key_buf, void *read_buf)
{
	printf("smc_efuse_readl is just a interface,you must override it\n");
	return 0;
}

int smc_efuse_writel(void *key_buf)
{
	printf("smc_efuse_writel is just a interface,you must override it\n");
	return 0;
}

int smc_tee_get_os_revision(uint32_t *major, uint32_t *minor)
{
	struct arm_smccc_res param = { 0 };

	arm_smccc_smc(OPTEE_SMC_CALL_GET_OS_REVISION, 0, 0, 0, 0, 0, 0, 0,
		      &param);
	*major = param.a0;
	*minor = param.a1;
	return 0;
}

int smc_tee_get_sunxi_call_offset(void)
{
	uint32_t major, minor;

	smc_tee_get_os_revision(&major, &minor);
	pr_msg("optee version: major:%d minor:%d\n", major, minor);
	if ((major > 3) || ((major == 3) && (minor >= 5))) {
		return SUNXI_OPTEE_SMC_OFFSET;
	} else {
		return 0;
	}
}

int smc_init(void)
{
	if (sunxi_probe_secure_os()) {
		sunxi_smc_call_offset = smc_tee_get_sunxi_call_offset();
	}

	return 0;
}

typedef struct _aes_function_info {
	uint32_t  decrypt;
	uint32_t  key_addr;
	uint32_t  key_len;
	uint32_t  iv_map;
	uint32_t  key_mode;
} aes_function_info;

struct smc_tee_ssk_addr_group {
	uint32_t out_tee;
	uint32_t in_tee;
	uint32_t private;
};

int smc_tee_ssk_encrypt(char *out_buf, char *in_buf, int len, int *out_len)
{
	struct arm_smccc_res param = { 0 };
	struct smc_tee_ssk_addr_group *tee_addr;
	int align_len = 0;

	align_len = ALIGN(len, CACHE_LINE_SIZE);
	if (align_len > SUNXI_SECURE_STORTAGE_BLOCK_SIZE - SUNXI_SECURE_STORTAGE_INFO_HEAD_LEN) {
		return -1;
	}

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	memset((void *)tee_addr->in_tee, 0x0, align_len);
	memcpy((void *)tee_addr->in_tee, in_buf, len);
	flush_cache(tee_addr->in_tee, align_len);
	flush_cache(tee_addr->out_tee, align_len);
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));

	memset(&param, 0, sizeof(param));
#ifdef CONFIG_SECURE_STORAGE_KEY_HUK
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_HUK_ENCRYPT,
		(uint32_t)tee_addr, align_len, 0, 0, 0, 0, &param);
#else
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_SSK_ENCRYPT,
		(uint32_t)tee_addr, align_len, 0, 0, 0, 0, &param);
#endif
	if (param.a0 != 0) {
		printf("smc tee encrypt with ssk failed with: %ld", param.a0);

		return -1;
	}
	memcpy(out_buf, (void *)tee_addr->out_tee, align_len);
	*out_len = align_len;
	return 0;
}

int smc_tee_ssk_decrypt(char *out_buf, char *in_buf, int len)
{
	struct arm_smccc_res param = { 0 };
	struct smc_tee_ssk_addr_group *tee_addr;

	if (len > SUNXI_SECURE_STORTAGE_BLOCK_SIZE - SUNXI_SECURE_STORTAGE_INFO_HEAD_LEN) {
		return -1;
	}
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	memcpy((void *)tee_addr->in_tee, in_buf, len);
	memset(&param, 0, sizeof(param));
	flush_cache(tee_addr->in_tee, len);
	flush_cache(tee_addr->out_tee, len);
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));

#ifdef CONFIG_SECURE_STORAGE_KEY_HUK
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_HUK_DECRYPT,
		(uint32_t)tee_addr, len, 0, 0, 0, 0, &param);
#else
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_SSK_DECRYPT,
		(uint32_t)tee_addr, len, 0, 0, 0, 0, &param);
#endif
	if (param.a0 != 0) {
		printf("smc tee decrypt with ssk failed with: %ld", param.a0);

		return param.a0;
	}
	memcpy(out_buf, (void *)tee_addr->out_tee, len);

	return 0;
}

int smc_tee_rssk_encrypt(char *out_buf, char *in_buf, int len, int *out_len)
{
	struct smc_tee_ssk_addr_group *tee_addr;
	int align_len = 0;
	struct arm_smccc_res param = { 0 };

	align_len = ALIGN(len, CACHE_LINE_SIZE);

	if (align_len > SUNXI_SECURE_STORTAGE_BLOCK_SIZE - SUNXI_SECURE_STORTAGE_INFO_HEAD_LEN) {
		return -1;
	}
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	memset((void *)tee_addr->in_tee, 0x0, align_len);
	memcpy((void *)tee_addr->in_tee, in_buf, len);
	flush_cache(tee_addr->in_tee, align_len);
	flush_cache(tee_addr->out_tee, align_len);
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));

	memset(&param, 0, sizeof(param));
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_RSSK_ENCRYPT,
		(uint32_t)tee_addr, align_len, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("%s failed with: %ld", __func__, param.a0);

		return param.a0;
	}
	memcpy(out_buf, (void *)tee_addr->out_tee, align_len);
	*out_len = align_len;
	return 0;
}

int smc_aes_rssk_decrypt_to_keysram(void)
{
	struct smc_tee_ssk_addr_group *tee_addr;
	struct arm_smccc_res param = { 0 };

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	memset(&param, 0, sizeof(param));
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_RSSK_DECRYPT,
		0, SUNXI_HDCP_KEY_LEN, 0, 0, 0, 0, &param);

	if (param.a0 != 0) {
		printf("smc tee decrypt with ssk failed with: %ld", param.a0);
		return param.a0;
	}
	return 0;

}

int _smc_tee_hdcp_key_encrypt(char *out_buf, int dst_len, char *in_buf,
			     int data_len, enum AW_HDCP_KEY_TYPE_EN type)
{
	struct smc_tee_ssk_addr_group *tee_addr;
	aes_function_info *p_aes_info = NULL;
	struct arm_smccc_res param    = { 0 };

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		pr_err("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	tee_addr->private = tee_addr->out_tee + 4096;

	p_aes_info = (aes_function_info *)tee_addr->private;
	p_aes_info->decrypt = AES_ENCRYPT;
	p_aes_info->key_len = 128 / 8; /* key len is 128 bit */
	p_aes_info->key_mode = AES_MODE_CBC;

	if (data_len > SUNXI_SECURE_STORTAGE_BLOCK_SIZE) {
		pr_err("data len is max than buf size\n ");
		return -2;
	}
	memcpy((void *)tee_addr->in_tee, in_buf, data_len);
	memset(&param, 0, sizeof(param));

	flush_cache(tee_addr->in_tee, ALIGN(data_len, CACHE_LINE_SIZE));
	flush_cache(tee_addr->out_tee, ALIGN(dst_len, CACHE_LINE_SIZE));
	flush_cache(tee_addr->private,
		    ALIGN(sizeof(aes_function_info), CACHE_LINE_SIZE));
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_HDCP_KEY_ENCRYPT,
		(uint32_t)tee_addr, data_len, dst_len, type, 0, 0, &param);
	if (param.a0 != 0) {
		pr_err("smc tee decrypt with ssk failed with: %ld", param.a0);
		return param.a0;
	}

	memcpy(out_buf, (void *)tee_addr->out_tee, dst_len);

	return 0;
}

int smc_tee_hdcp_key_encrypt(char *out_buf, int dst_len, char *in_buf,
			     int data_len, int hdcpkey_type)
{
	return _smc_tee_hdcp_key_encrypt(out_buf, dst_len, in_buf, data_len,
				  hdcpkey_type);
}

int smc_tee_keybox_store(const char *name, char *in_buf, int len)
{
	sunxi_secure_storage_info_t *key_box;
	struct arm_smccc_res param = { 0 };

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	if (len > SUNXI_SECURE_STORTAGE_BLOCK_SIZE) {
		return -1;
	}
	key_box = (sunxi_secure_storage_info_t *)param.a1;
	memcpy(key_box, in_buf, len);
#if 0 /*debug info*/
	printf("len=%d\n", len);
	printf("name=%s\n", key_box->name);
	printf("len=%d\n",  key_box->len);
	printf("encrypt=%d\n", key_box->encrypted);
	printf("write_protect=%d\n", key_box->write_protect);
	printf("******************\n");
#endif
	if (strcmp(name, key_box->name)) {
		pr_err("name of key %s not match, key data corrupted\n", name);
		return -1;
	}

#ifdef CONFIG_SECURE_STORAGE_KEY_HUK
	key_box->encrypted = (key_box->encrypted & KEYBOX_ENC_MASK)
				| (KEYBOX_ENC_PARAM_CBC_192_HUK << KEYBOX_ENC_PARAM_OFFSET);
#endif

	flush_cache((ulong)key_box, sizeof(sunxi_secure_storage_info_t));

	memset(&param, 0, sizeof(param));
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_KEYBOX_STORE,
		(uint32_t)key_box, 0, 0, 0, 0, 0, &param);

	if (param.a0 != 0) {
		printf("smc tee keybox store failed with: %ld", param.a0);

		return param.a0;
	}

	return 0;
}

int smc_tee_probe_drm_configure(ulong *drm_base, ulong *drm_size)
{
	struct arm_smccc_res param = { 0 };
	arm_smccc_smc(OPTEE_SMC_GET_DRM_INFO, 0, 0, 0, 0, 0, 0, 0, &param);

	if (param.a0 != 0) {
		printf("drm config service not available: %X", (uint)param.a0);
		return param.a0;
	}

	*drm_base = param.a1;
	*drm_size = param.a2;

	return 0;
}

int smc_tee_check_hash(const char *name, u8 *hash)
{
#define NAME_MAX_SIZE 16
	struct arm_smccc_res res;
	char *smc_name, *smc_hash;

	memset(&res, 0, sizeof(res));
	/* probe share memory */
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE, 0, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}
	if (!res.a1)
		return -1;
	smc_name = (char *)res.a1;
	strncpy(smc_name, name, NAME_MAX_SIZE);
	smc_name[NAME_MAX_SIZE - 1] = 0;
	smc_hash		    = (char *)((u32)smc_name + NAME_MAX_SIZE);
	memcpy(smc_hash, hash, 32);

	flush_cache((ulong)smc_name,
		    ALIGN(NAME_MAX_SIZE + 32, CACHE_LINE_SIZE));
	return sunxi_smc_call(OPTEE_SMC_SUNXI_HASH_OP, (ulong)smc_name,
			      (ulong)smc_hash, 0, 0);
}

int smc_tee_setup_mips(uint32_t load_addr, uint32_t mips_size)
{
	struct arm_smccc_res param = { 0 };
	arm_smccc_smc(OPTEE_SMC_SUNXI_SETUP_MIPS, load_addr, mips_size, 0, 0, 0,
		      0, 0, &param);
	if (param.a0 != 0) {
		pr_err("%s failed with: %ld", __func__, param.a0);
		return param.a0;
	}
	return 0;
}

int smc_tee_inform_fdt(uint64_t base, uint32_t size)
{
	struct arm_smccc_res param = { 0 };

	flush_cache((ulong)base, ALIGN(size, CACHE_LINE_SIZE));

	arm_smccc_smc(OPTEE_SMC_SUNXI_INFORM_FDT, base, 0, size, 0, 0, 0, 0, &param);

	ulong fdt_blob = ((ulong)gd->fdt_blob) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
	invalidate_dcache_range((ulong)(fdt_blob),
							(ulong)(ALIGN(((ulong)gd->fdt_blob + gd->fdt_size),
							CONFIG_SYS_CACHELINE_SIZE)));
	if (param.a0 != 0) {
		pr_msg("%s failed with: %lx\n", __func__, param.a0);
		return param.a0;
	}

	return 0;
}

int smc_tee_rkp_op(int sub_cmd, void *input, size_t input_size, void *output)
{
	struct arm_smccc_res param = { 0 };
	uint8_t *buffer;

	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE, 0, 0, 0, 0, 0, 0,
		      &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	buffer = (uint8_t *)param.a1;
	memcpy(buffer, input, input_size);

	flush_cache((ulong)buffer, ALIGN(input_size, CACHE_LINE_SIZE));

	memset(&param, 0, sizeof(param));
	arm_smccc_smc(OPTEE_SMC_SUNXI_RKP, sub_cmd, (unsigned long)buffer, 0, 0,
		      0, 0, 0, &param);

	if (param.a0 > 0x80000000) {
		printf("%s failed with: %ld\n", __func__, param.a0);

		return param.a0;
	} else {
		if (param.a0 && output) {
			memcpy(output, buffer, param.a0);
		}
		return 0;
	}

	return 0;
}

int smc_tee_pkf_encrypt(char *out_buf, char *in_buf, int len, int *out_len)
{
	struct arm_smccc_res param = { 0 };
	struct smc_tee_ssk_addr_group *tee_addr;
	int align_len = 0;

	align_len = ALIGN(len, CACHE_LINE_SIZE);
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PROBE_SHM_BASE,
		0, 0, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("smc tee probe share memory base failed\n ");
		return -1;
	}

	tee_addr = (struct smc_tee_ssk_addr_group *)param.a1;
	tee_addr->in_tee = param.a1 + 0x100;
	tee_addr->out_tee = tee_addr->in_tee + 4096;
	memset((void *)tee_addr->in_tee, 0x0, align_len);
	memcpy((void *)tee_addr->in_tee, in_buf, len);
	flush_cache(tee_addr->in_tee, align_len);
	flush_cache(tee_addr->out_tee, align_len);
	flush_cache((uint32_t)tee_addr,
		    ALIGN(sizeof(struct smc_tee_ssk_addr_group),
			  CACHE_LINE_SIZE));

	memset(&param, 0, sizeof(param));
	arm_smccc_smc(OPTEE_SMC_CRYPT, TEESMC_PKF_ENCRYPT,
		(uint32_t)tee_addr, align_len, 0, 0, 0, 0, &param);
	if (param.a0 != 0) {
		printf("%s failed with: %ld", __func__, param.a0);

		return -1;
	}
	memcpy(out_buf, (void *)tee_addr->out_tee, align_len);
	*out_len = align_len;
	return 0;
}
