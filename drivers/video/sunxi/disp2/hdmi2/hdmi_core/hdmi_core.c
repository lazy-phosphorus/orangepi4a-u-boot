/*
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include "hdmi_core.h"
#include "api/api.h"
#if defined(__LINUX_PLAT__)
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <video/sunxi_metadata.h>
#endif

static u32 hdcp_irq;

static uintptr_t hdmi_reg_base;
static struct hdmi_tx_core *p_core;

struct disp_video_timings info;
static struct disp_hdmi_mode hdmi_mode_tbl[] = {
	{DISP_TV_MOD_480I,                HDMI_VIC_720x480I_16_9,     },
	{DISP_TV_MOD_576I,                HDMI_VIC_720x576I_16_9,     },
	{DISP_TV_MOD_480P,                HDMI_VIC_720x480P60_16_9,   },
	{DISP_TV_MOD_576P,                HDMI_VIC_720x576P_16_9,     },
	{DISP_TV_MOD_720P_50HZ,           HDMI_VIC_1280x720P50,       },
	{DISP_TV_MOD_720P_60HZ,           HDMI_VIC_1280x720P60,       },
	{DISP_TV_MOD_1080I_50HZ,          HDMI_VIC_1920x1080I50,      },
	{DISP_TV_MOD_1080I_60HZ,          HDMI_VIC_1920x1080I60,      },
	{DISP_TV_MOD_1080P_24HZ,          HDMI_VIC_1920x1080P24,      },
	{DISP_TV_MOD_1080P_50HZ,          HDMI_VIC_1920x1080P50,      },
	{DISP_TV_MOD_1080P_60HZ,          HDMI_VIC_1920x1080P60,      },
	{DISP_TV_MOD_1080P_25HZ,          HDMI_VIC_1920x1080P25,      },
	{DISP_TV_MOD_1080P_30HZ,          HDMI_VIC_1920x1080P30,      },
	{DISP_TV_MOD_1080P_24HZ_3D_FP,    HDMI1080P_24_3D_FP,},
	{DISP_TV_MOD_720P_50HZ_3D_FP,     HDMI720P_50_3D_FP, },
	{DISP_TV_MOD_720P_60HZ_3D_FP,     HDMI720P_60_3D_FP, },
	{DISP_TV_MOD_3840_2160P_30HZ,     HDMI_VIC_3840x2160P30, },
	{DISP_TV_MOD_3840_2160P_25HZ,     HDMI_VIC_3840x2160P25, },
	{DISP_TV_MOD_3840_2160P_24HZ,     HDMI_VIC_3840x2160P24, },
	{DISP_TV_MOD_4096_2160P_24HZ,     HDMI_VIC_4096x2160P24, },
	{DISP_TV_MOD_4096_2160P_25HZ,	  HDMI_VIC_4096x2160P25, },
	{DISP_TV_MOD_4096_2160P_30HZ,	  HDMI_VIC_4096x2160P30, },

	{DISP_TV_MOD_3840_2160P_50HZ,     HDMI_VIC_3840x2160P50,},
	{DISP_TV_MOD_4096_2160P_50HZ,     HDMI_VIC_4096x2160P50,},

	{DISP_TV_MOD_3840_2160P_60HZ,     HDMI_VIC_3840x2160P60, },
	{DISP_TV_MOD_4096_2160P_60HZ,     HDMI_VIC_4096x2160P60, },
	{DISP_TV_MOD_2560_1440P_60HZ,     HDMI_VIC_2560x1440P60, },
	{DISP_TV_MOD_1440_2560P_70HZ,     HDMI_VIC_1440x2560P70, },
	{DISP_TV_MOD_1080_1920P_60HZ,     HDMI_VIC_1080x1920P60, },

	/* VESA Timing Start */
	{DISP_VESA_MOD_640_480P_60,       HDMI_VIC_640x480P60,         },
	{DISP_VESA_MOD_800_600P_56,       HDMI_VIC_800x600P56,         },
	{DISP_VESA_MOD_800_600P_60,       HDMI_VIC_800x600P60,         },
	{DISP_VESA_MOD_1024_768P_60,      HDMI_VIC_1024x768P60,        },
	{DISP_VESA_MOD_1024_768P_70,      HDMI_VIC_1024x768P70,        },
	{DISP_VESA_MOD_1152_864P_75,      HDMI_VIC_1152x864P75_4x3,    },
	{DISP_VESA_MOD_1280_768P_60,      HDMI_VIC_1280x768P60_5x4,    },
	{DISP_VESA_MOD_1280_800P_60,      HDMI_VIC_1280x800P60_16x10,  },
	{DISP_VESA_MOD_1280_960P_60,      HDMI_VIC_1280x960P60_4x3,    },
	{DISP_VESA_MOD_1280_1024P_60,     HDMI_VIC_1280x1024P60_5x4,   },
	{DISP_VESA_MOD_1360_768P_60,      HDMI_VIC_1360x768P60_16x9,   },
	{DISP_VESA_MOD_1366_768P_60,      HDMI_VIC_1366x768P60_16x9,   },
	{DISP_VESA_MOD_1400_1050P_60,     HDMI_VIC_1400x1050P60_4x3,   },
	{DISP_VESA_MOD_1440_900P_60,      HDMI_VIC_1440x900P60_16x10,  },
	{DISP_VESA_MOD_1600_900P_60,      HDMI_VIC_1600x900P60_16x9,   },
	{DISP_VESA_MOD_1680_1050P_60,     HDMI_VIC_1680x1050P60_16x10, },
	{DISP_VESA_MOD_1920_1200P_60,     HDMI_VIC_1920x1200P60_16x10, },
	{DISP_VESA_MOD_1920_1440P_60,     HDMI_VIC_1920x1440P60_4x3,   },
};

static struct disp_hdmi_mode hdmi_mode_tbl_4_3[] = {
	{DISP_TV_MOD_480I,                HDMI_VIC_720x480I_4_3,     },
	{DISP_TV_MOD_576I,                HDMI_VIC_720x576I_4_3,     },
	{DISP_TV_MOD_480P,                HDMI_VIC_720x480P60_4_3,   },
	{DISP_TV_MOD_576P,                HDMI_VIC_720x576P_4_3,     },
};

static struct hdmi_sink_blacklist sink_blacklist[] = {
	{ { {0x36, 0x74}, {0x4d, 0x53, 0x74, 0x61, 0x72, 0x20, 0x44, 0x65, 0x6d, 0x6f, 0x0a, 0x20, 0x20}, 0x38},        /*sink*/
	{ {DISP_TV_MOD_3840_2160P_30HZ, 2}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} } },  /*issue*/

	{ { {0x2d, 0xee}, {0x4b, 0x4f, 0x4e, 0x41, 0x4b, 0x20, 0x54, 0x56, 0x0a, 0x20, 0x20, 0x20, 0x20}, 0xa5},
	{ {DISP_TV_MOD_3840_2160P_30HZ, 2}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} } },

	{ { {0x2d, 0xe1}, {0x54, 0x56, 0x5f, 0x4d, 0x4f, 0x4e, 0x49, 0x54, 0x4f, 0x52, 0x0a, 0x20, 0x20}, 0x28},
	{ {DISP_TV_MOD_1080P_24HZ_3D_FP, 0x00}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} } },

	{ { {0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0},
	{ {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} } },
};

static int hdmitx_set_phy(struct hdmi_tx_core *core, int phy);
static u32 svd_user_config(u32 code, u32 refresh_rate);
static void print_videoinfo(videoParams_t *pVideo);
static void print_audioinfo(audioParams_t *audio);

static void hdmitx_sleep(int us)
{
	udelay(us);
}

void hdmi_core_set_base_addr(uintptr_t reg_base)
{
	hdmi_reg_base = reg_base;
}

uintptr_t hdmi_core_get_base_addr(void)
{
	return hdmi_reg_base;
}

/*******************************
 * HDMI TX Support functions   *
 ******************************/
void set_platform(struct hdmi_tx_core *core)
{
	p_core = core;
}

struct hdmi_tx_core *get_platform(void)
{
	return p_core;
}

void hdmitx_write(uintptr_t addr, u32 data)
{
	if (hdmi_clk_enable_mask == 0)
		return;
#ifndef CONFIG_MACH_SUN20IW1
	asm volatile("dsb st");
#endif
	*((volatile u8 *)(hdmi_core_get_base_addr() + (addr >> 2))) = data;
}

u32 hdmitx_read(uintptr_t addr)
{
	if (hdmi_clk_enable_mask == 0)
		return 0;

	return *((volatile u8 *)(hdmi_core_get_base_addr() + (addr >> 2)));
}
#if defined(__LINUX_PLAT__)

/************************HDCP*******************************/
void set_hdcp_on_core(struct hdmi_tx_core *core, u8 on)
{
	core->dev_func.set_hdcp_on(on);
}

u8 get_hdcp_on_core(struct hdmi_tx_core *core)
{
	return core->dev_func.get_hdcp_on();
}

void set_hdcp_encrypt_on_core(struct hdmi_tx_core *core, u8 on)
{
	core->dev_func.set_hdcp_encrypt_on(on);
}

u8 get_hdcp_encrypt_on_core(struct hdmi_tx_core *core)
{
	return  core->dev_func.get_hdcp_encrypt_on();
}

u8 get_hdcp_type_core(struct hdmi_tx_core *core)
{
	return  core->dev_func.get_hdcp_type();
}


/* return 0: hdcp encrypting status is good
* return -1: hdcp encrypt failed
*/
int get_hdcp_status_core(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_hdcp_status();
}
#endif

/************************HDCP*******************************/

void resistor_calibration_core(struct hdmi_tx_core *core, u32 reg, u32 data)
{
	 hdmitx_sleep(1000);
	core->dev_func.resistor_calibration(reg, data);
}

u32 hdmi_core_get_hpd_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();
	return core->dev_func.dev_hpd_status();
}

void hdmi_core_set_phy_reg_base(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.set_phy_base_addr(hdmi_reg_base);
}
/**
 * @short Set PHY number
 * @param[in] core Main structure
 * @param[in] phy PHY number:301 or 309
 * @return 0 if successful or -1 if failure
 */
static  int hdmitx_set_phy(struct hdmi_tx_core *core, int phy)
{
	if ((phy > 500) || (phy < 100)) {
		HDMI_INFO_MSG("PHY value outside of range: %d\n", phy);
		return -1;
	}

	core->hdmi_tx_phy = phy;
	return 0;
}
#if defined(__LINUX_PLAT__)

void core_init_hdcp(struct hdmi_mode *cfg)
{
	hdcpParams_t *pHdcp = &(cfg->pHdcp);

	pHdcp->mEnable11Feature = -1;
	pHdcp->mRiCheck = -1;
	pHdcp->mI2cFastMode = -1;
	pHdcp->mEnhancedLinkVerification = -1;
	pHdcp->maxDevices = 0;
	pHdcp->mKsvListBuffer = NULL;
	pHdcp->mAksv = NULL;
	pHdcp->mKeys = NULL;
	pHdcp->mSwEncKey = NULL;
}

/*static void core_init_video(struct hdmi_mode *cfg)
{
	videoParams_t *video = &(cfg->pVideo);

	memset(video, 0, sizeof(videoParams_t));

	video->mHdmi = MODE_UNDEFINED;
	video->mEncodingOut = ENC_UNDEFINED;
	video->mEncodingIn = ENC_UNDEFINED;
	video->mColorResolution = COLOR_DEPTH_INVALID;
	video->mColorimetry = (u8)(~0);
	video->mEndTopBar = (u16)(~0);
	video->mStartBottomBar = (u16)(~0);
	video->mStartRightBar = (u16)(~0);

	video->mDtd.mLimitedToYcc420 = 0xFF;
	video->mDtd.mYcc420 = 0xFF;
	video->mDtd.mInterlaced = 0xFF;
	video->mDtd.mHBorder = 0xFFFF;
	video->mDtd.mHSyncPolarity = 0xFF;
	video->mDtd.mVBorder = 0xFFFF;
	video->mDtd.mVSyncPolarity = 0xFF;
}*/
#endif
void core_init_audio(struct hdmi_mode *cfg)
{
	audioParams_t *audio = &(cfg->pAudio);

	memset(audio, 0, sizeof(audioParams_t));

	audio->mInterfaceType = I2S;
	audio->mCodingType = PCM;
	audio->mSamplingFrequency = 44100;
	audio->mChannelAllocation = 0;
	audio->mChannelNum = 2;
	audio->mSampleSize = 16;
	audio->mClockFsFactor = 64;
	audio->mPacketType = PACKET_NOT_DEFINED;
	audio->mDmaBeatIncrement = DMA_NOT_DEFINED;
}

static int _api_init(struct hdmi_tx_core *core)
{
	hdmitx_api_init("HDMI Tx App");

	/* Register functions into api layer */
	register_system_functions(&(core->sys_functions));
	register_bsp_functions(&(core->dev_access));

	hdmi_core_set_phy_reg_base();
	return 0;
}

int hdmi_tx_core_init(struct hdmi_tx_core *core, int phy)
{

	core->hdmi_tx_phy = phy;

	/* Set global variable */
	set_platform(core);

	/* Data allocation */
	core->mode.edid = kzalloc(sizeof(struct edid), GFP_KERNEL);
	if (!core->mode.edid) {
		pr_info("Could not allocated hdmi_tx_core\n");
		return -1;
	}
	memset(core->mode.edid, 0, sizeof(struct edid));

	core->mode.edid_ext = kzalloc(128, GFP_KERNEL);
	if (!core->mode.edid_ext) {
		kfree(core->mode.edid);
		pr_info("Could not allocated edid_ext\n");
		return -1;
	}
	memset(core->mode.edid_ext, 0, 128);

	core->mode.sink_cap = kzalloc(sizeof(sink_edid_t), GFP_KERNEL);
	if (!core->mode.sink_cap) {
		kfree(core->mode.edid);
		kfree(core->mode.edid_ext);
		pr_info("Could not allocated sink\n");
		return -1;
	}
	memset(core->mode.sink_cap, 0, sizeof(sink_edid_t));

	/* set device access functions */
	strcpy(core->dev_access.name, "HDMI TX Device Access");
	core->dev_access.read = hdmitx_read;
	core->dev_access.write = hdmitx_write;

	/* set system abstraction functions */
	strcpy(core->sys_functions.name, "HDMI TX System Functions");
	core->sys_functions.sleep = hdmitx_sleep;

	if (hdmitx_set_phy(core, phy)) {
		HDMI_INFO_MSG("Invalid PHY model %d\n", phy);
		return -1;
	}

	_api_init(core);
#if defined(__LINUX_PLAT__)
	core->dev_func.set_hdcp_on(0);
#endif
	return 0;
}

void hdmi_core_exit(struct hdmi_tx_core *core)
{
	kfree(core->mode.edid);
	kfree(core->mode.edid_ext);
	kfree(core->mode.sink_cap);
	kfree(core);
	hdmitx_api_exit();
}

/********************************************************/
/********************HDCP HANDLE***************************/
/********************************************************/
void hdcp_handler_core(void)
{
	struct hdmi_tx_core *p_core = get_platform();
	int temp = 0;
	u8 hdcp_type = 0;

	LOG_TRACE();
	hdcp_type = get_hdcp_type_core(p_core);
	if (hdcp_type == 0)
		p_core->dev_func.hdcp_event_handler(&temp, hdcp_irq);
	else
		pr_info("Process hdcp2.2 event\n");
}

u8 get_hdcp22_status_core(void)
{
	return 0;
}

void hdmi_configure_core(struct hdmi_tx_core *core)
{
	print_videoinfo(&(core->mode.pVideo));
	print_audioinfo(&(core->mode.pAudio));
	core->dev_func.main_config(&core->mode.pVideo, &core->mode.pAudio,
			&core->mode.pProduct, &core->mode.pHdcp, core->hdmi_tx_phy);
}

/*
*@code: svd <1-107, 0x80+4, 0x80+19, 0x80+32 0x201>
*@refresh_rate: [optional] Hz*1000,which is 1000 times than 1 Hz
*/
static u32 svd_user_config(u32 code, u32 refresh_rate)
{
	struct hdmi_tx_core *core = get_platform();
	dtd_t dtd;

	if (core == NULL) {
		pr_info("ERROR:Empty hdmi core structure\n");
		return -1;
	}

	/*memset(&core->mode.pVideo, 0, sizeof(videoParams_t));*/

	if (code < 1 || code > 0x300) {
		pr_info("ERROR:VIC Code is out of range\n");
		return -1;
	}

	if (refresh_rate == 0) {
		dtd.mCode = code;
		/* Ensure Pixel clock is 0 to get the default refresh rate */
		dtd.mPixelClock = 0;
		HDMI_INFO_MSG("HDMI_WARN:Code %d with default refresh rate %d\n",
					code, dtd_get_refresh_rate(&dtd)/1000);
	} else {
		HDMI_INFO_MSG("HDMI_WARN:Code %d with refresh rate %d\n",
				code, dtd_get_refresh_rate(&dtd)/1000);
	}

	if (core->dev_func.dtd_fill(&dtd, code, refresh_rate) == false) {
		HDMI_INFO_MSG("Can not find detailed timing\n");
		return -1;
	}

	memcpy(&core->mode.pVideo.mDtd, &dtd, sizeof(dtd_t));
	edid_set_video_prefered_core();

	if ((code == 0x80 + 19) || (code == 0x80 + 4) || (code == 0x80 + 32)) {
		core->mode.pVideo.mHdmiVideoFormat = 0x02;
		core->mode.pVideo.m3dStructure = 0;
	} else if (core->mode.pVideo.mHdmi_code) {
		core->mode.pVideo.mHdmiVideoFormat = 0x01;
		core->mode.pVideo.m3dStructure = 0;
	} else {
		core->mode.pVideo.mHdmiVideoFormat = 0x0;
		core->mode.pVideo.m3dStructure = 0;
	}

	return 0;
}

static void print_videoinfo(videoParams_t *pVideo)
{
	u32 refresh_rate = dtd_get_refresh_rate(&pVideo->mDtd);

	if (pVideo->mCea_code)
		pr_info("[HDMI2.0]CEA VIC=%d: ", pVideo->mCea_code);
	else
		pr_info("[HDMI2.0]HDMI VIC=%d: ", pVideo->mHdmi_code);
	if (pVideo->mDtd.mInterlaced)
		pr_info("%dx%di", pVideo->mDtd.mHActive,
						pVideo->mDtd.mVActive*2);
	else
		pr_info("%dx%dp", pVideo->mDtd.mHActive,
						pVideo->mDtd.mVActive);
	pr_info("@%d Hz ", refresh_rate/1000);
	pr_info("%d:%d, ", pVideo->mDtd.mHImageSize,
					pVideo->mDtd.mVImageSize);
	pr_info("%d-bpp ", pVideo->mColorResolution);
	pr_info("%s\n", getEncodingString(pVideo->mEncodingIn));
	switch (pVideo->mColorimetry) {
	case 0:
		pr_info("WARN:you haven't set an colorimetry\n");
		break;
	case ITU601:
		pr_info("BT601\n");
		break;
	case ITU709:
		pr_info("BT709\n");
		break;
	case EXTENDED_COLORIMETRY:
		if (pVideo->mExtColorimetry == BT2020_Y_CB_CR)
			pr_info("BT2020_Y_CB_CR\n");
		else
			pr_info("WARN:extended color space standard %d undefined\n", pVideo->mExtColorimetry);
		break;
	default:
		pr_info("WARN:color space standard %d undefined\n", pVideo->mColorimetry);
	}
	if (pVideo->pb == NULL)
		return;

	switch (pVideo->pb->eotf) {
	case SDR_LUMINANCE_RANGE:
		pr_info("eotf:SDR_LUMINANCE_RANGE\n");
		break;
	case HDR_LUMINANCE_RANGE:
		pr_info("eotf:HDR_LUMINANCE_RANGE\n");
		break;
	case SMPTE_ST_2084:
		pr_info("eotf:SMPTE_ST_2084\n");
		break;
	case HLG:
		pr_info("eotf:HLG\n");
		break;
	default:
		pr_info("Unknow eotf\n");
		break;
	}
}


static void print_audioinfo(audioParams_t *audio)
{
	pr_info("Audio interface type = %s\n",
			audio->mInterfaceType == I2S ? "I2S" :
			audio->mInterfaceType == SPDIF ? "SPDIF" :
			audio->mInterfaceType == HBR ? "HBR" :
			audio->mInterfaceType == GPA ? "GPA" :
			audio->mInterfaceType == DMA ? "DMA" : "---");

	pr_info("Audio coding = %s\n", audio->mCodingType == PCM ? "PCM" :
		audio->mCodingType == AC3 ? "AC3" :
		audio->mCodingType == MPEG1 ? "MPEG1" :
		audio->mCodingType == MP3 ? "MP3" :
		audio->mCodingType == MPEG2 ? "MPEG2" :
		audio->mCodingType == AAC ? "AAC" :
		audio->mCodingType == DTS ? "DTS" :
		audio->mCodingType == ATRAC ? "ATRAC" :
		audio->mCodingType == ONE_BIT_AUDIO ? "ONE BIT AUDIO" :
		audio->mCodingType == DOLBY_DIGITAL_PLUS ? "DOLBY DIGITAL +" :
		audio->mCodingType == DTS_HD ? "DTS HD" :
		audio->mCodingType == MAT ? "MAT" : "---");
	pr_info("Audio frequency = %dHz\n", audio->mSamplingFrequency);
	pr_info("Audio sample size = %d\n", audio->mSampleSize);
	pr_info("Audio FS factor = %d\n", audio->mClockFsFactor);
	pr_info("Audio ChannelAllocationr = %d\n", audio->mChannelAllocation);
	pr_info("Audio mChannelNum = %d\n", audio->mChannelNum);
}

void video_apply(struct hdmi_tx_core *core)
{
	if (core == NULL) {
		HDMI_INFO_MSG("HDMI_ERROR:Improper arguments");
		return;
	}

	print_videoinfo(&(core->mode.pVideo));
	if (!core->dev_func.main_config(&core->mode.pVideo,
		&core->mode.pAudio, &core->mode.pProduct,
			&core->mode.pHdcp, core->hdmi_tx_phy)) {
		HDMI_ERROR_MSG("\n");
	}
}

static enum updated_type hdmi_check_updated(struct hdmi_tx_core *core, struct disp_device_config *config)
{
	enum updated_type ret = NOT_UPDATED;

	if (config->mode != core->config.mode) {
		if (((core->config.mode == DISP_TV_MOD_720P_60HZ_3D_FP) ||
			(core->config.mode == DISP_TV_MOD_720P_50HZ_3D_FP) ||
			(core->config.mode == DISP_TV_MOD_1080P_24HZ_3D_FP)) &&
			(config->mode != DISP_TV_MOD_720P_60HZ_3D_FP) &&
			(config->mode != DISP_TV_MOD_720P_60HZ_3D_FP) &&
			(config->mode != DISP_TV_MOD_720P_60HZ_3D_FP))
			ret = EXIT3D_UPDATED;
		else
			ret = NORMAL_UPDATED;
	} else if ((config->format != core->config.format) || (config->bits != core->config.bits)) {
		ret = NORMAL_UPDATED;
	} else if ((config->eotf != core->config.eotf) || (config->cs != core->config.cs)) {
		ret = SMOOTH_UPDATED;
	} else {
		ret = NOT_UPDATED;
	}

	return ret;
}

s32 set_static_config(struct disp_device_config *config)
{
	u32 data_bit = 0;
	struct hdmi_tx_core *core = get_platform();
	videoParams_t *pVideo = &core->mode.pVideo;

	LOG_TRACE();

	hdmi_reconfig_format_by_blacklist(config);
	pVideo->update = hdmi_check_updated(core, config);
#if defined(__LINUX_PLAT__)

	pr_info("[HDMI receive params]: tv mode: 0x%x format:0x%x data bits:0x%x eotf:0x%x cs:0x%x "
				"dvi_hdmi:%x range:%x scan:%x aspect_ratio:%x\n",
				config->mode, config->format, config->bits, config->eotf, config->cs,
				config->dvi_hdmi, config->range, config->scan, config->aspect_ratio);
#endif
	memset(pVideo, 0, sizeof(videoParams_t));
	/*set vic mode and dtd*/
	hdmi_set_display_mode(config->mode);

	/*set encoding mode*/
	pVideo->mEncodingIn = config->format;
	pVideo->mEncodingOut = config->format;

	/*set data bits*/
	if ((config->bits >= 0) && (config->bits < 3))
		data_bit = 8 + 2 * config->bits;
	if (config->bits == 3)
		data_bit = 16;
	pVideo->mColorResolution = (u8)data_bit;

	/*set eotf*/
	if (config->eotf) {
		if (core->mode.pVideo.pb == NULL) {
			pVideo->pb = kmalloc(sizeof(fc_drm_pb_t), GFP_KERNEL);
			if (pVideo->pb) {
				memset(pVideo->pb, 0, sizeof(fc_drm_pb_t));
			} else {
				pr_info("Can not alloc memory for dynamic range and mastering infoframe\n");
				return -1;
			}
		}
		if (pVideo->pb) {
			pVideo->pb->r_x = 0x33c2;
			pVideo->pb->r_y = 0x86c4;
			pVideo->pb->g_x = 0x1d4c;
			pVideo->pb->g_y = 0x0bb8;
			pVideo->pb->b_x = 0x84d0;
			pVideo->pb->b_y = 0x3e80;
			pVideo->pb->w_x = 0x3d13;
			pVideo->pb->w_y = 0x4042;
			pVideo->pb->luma_max = 0x03e8;
			pVideo->pb->luma_min = 0x1;
			pVideo->pb->mcll = 0x03e8;
			pVideo->pb->mfll = 0x0190;
		}


		switch (config->eotf) {
		case DISP_EOTF_GAMMA22:
			pVideo->mHdr = 0;
			pVideo->pb->eotf = SDR_LUMINANCE_RANGE;
			break;
		case DISP_EOTF_SMPTE2084:
			pVideo->mHdr = 1;
			pVideo->pb->eotf = SMPTE_ST_2084;
			break;
		case DISP_EOTF_ARIB_STD_B67:
			pVideo->mHdr = 1;
			pVideo->pb->eotf = HLG;
			break;
		default:
			break;
		}

	} else {
		if (config->mode < 4)
			pVideo->mColorimetry = ITU601;
		else
			pVideo->mColorimetry = ITU709;

	}

	/*set color space*/
	switch (config->cs) {
	case DISP_UNDEF:
		pVideo->mColorimetry = 0;
		break;
	case DISP_BT601:
		pVideo->mColorimetry = ITU601;
		break;
	case DISP_BT709:
		pVideo->mColorimetry = ITU709;
		break;
	case DISP_BT2020NC:
		pVideo->mColorimetry = EXTENDED_COLORIMETRY;
		pVideo->mExtColorimetry = BT2020_Y_CB_CR;
		break;
	default:
		pVideo->mColorimetry = 0;
		break;
	}

#if defined(__LINUX_PLAT__)
	/*set output mode: hdmi or avi*/
	switch (config->dvi_hdmi) {
	case DISP_DVI_HDMI_UNDEFINED:
		pVideo->mHdmi = HDMI;
		break;
	case DISP_DVI:
		pVideo->mHdmi = DVI;
		break;
	case DISP_HDMI:
		pVideo->mHdmi = HDMI;
		break;
	default:
		pVideo->mHdmi = HDMI;
		pr_info("Error: Unkonw ouput mode!\n");
		break;
	}

	/*set clor range: defult/limited/full*/
	switch (config->range) {
	case DISP_COLOR_RANGE_DEFAULT:
		pVideo->mRgbQuantizationRange = 0;
		break;
	case DISP_COLOR_RANGE_0_255:
		pVideo->mRgbQuantizationRange = 2;
		break;
	case DISP_COLOR_RANGE_16_235:
		pVideo->mRgbQuantizationRange = 1;
		break;
	default:
		pr_info("Error: Unkonw color range!\n");
		break;
	}

	/*set scan info*/
	pVideo->mScanInfo = config->scan;

	/*set aspect ratio*/
	if (config->aspect_ratio == 0)
		pVideo->mActiveFormatAspectRatio = 8;
	else
		pVideo->mActiveFormatAspectRatio = config->aspect_ratio;
#else
	pVideo->mHdmi = HDMI;
#endif
	memcpy(&core->config, config, sizeof(struct disp_device_config));
	return 0;
}

s32 get_static_config(struct disp_device_config *config)
{
	u32 i;
	enum disp_tv_mode tv_mode = 0;
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();
	for (i = 0; i < sizeof(hdmi_mode_tbl)/sizeof(struct disp_hdmi_mode); i++) {
			if (hdmi_mode_tbl[i].hdmi_mode == core->mode.pVideo.mDtd.mCode) {
					tv_mode = hdmi_mode_tbl[i].mode;
					break;
			}
	}
	config->mode = tv_mode;

	config->format = core->mode.pVideo.mEncodingIn;

	if ((core->mode.pVideo.mColorResolution >= 8) && (core->mode.pVideo.mColorResolution < 16))
		config->bits = (core->mode.pVideo.mColorResolution - 8)/2;
	if (core->mode.pVideo.mColorResolution == 16)
		config->bits  = 4;

	if (core->mode.pVideo.pb != NULL) {

		switch (core->mode.pVideo.pb->eotf) {
		case SDR_LUMINANCE_RANGE:
			config->eotf = DISP_EOTF_GAMMA22;
			break;
		case SMPTE_ST_2084:
			config->eotf = DISP_EOTF_SMPTE2084;
			break;
		case HLG:
			config->eotf = DISP_EOTF_ARIB_STD_B67;
			break;
		default:
			break;
		}

		config->cs = DISP_BT2020NC;
	} else {
		if (tv_mode < 4)
			config->cs = DISP_BT601;
		else
			config->cs = DISP_BT709;

	}

	return 0;
}


#if defined(__LINUX_PLAT__)
s32 set_dynamic_config(struct disp_device_dynamic_config *config)
{
	void *hdr_buff_addr;
	int ret = 0;
	struct dma_buf *dmabuf;
	struct hdmi_tx_core *core = get_platform();
	videoParams_t *pVideo = &core->mode.pVideo;
	struct hdr_static_metadata *hdr_smetadata;

	u8 *temp = kmalloc(config->metadata_size, GFP_KERNEL);
	fc_drm_pb_t *dynamic_pb = kmalloc(sizeof(fc_drm_pb_t), GFP_KERNEL);

	LOG_TRACE();

	if ((get_drv_hpd_state() == 0) || (hdmi_clk_enable_mask == 0)) {
		kfree(temp);
		kfree(dynamic_pb);
		return -1;
	}

	if ((temp == NULL) || (dynamic_pb == NULL))
		return -1;

	if (!((pVideo->pb->eotf == SMPTE_ST_2084)
	   || (pVideo->pb->eotf == HLG))) {
		kfree(temp);
		kfree(dynamic_pb);
		return -1;
	}

	/*get the virtual addr of metadata*/
	dmabuf = dma_buf_get(config->metadata_fd);
	if (IS_ERR(dmabuf)) {
		pr_info("dma_buf_get failed\n");
		kfree(temp);
		kfree(dynamic_pb);
		return -1;
	}
	ret = dma_buf_begin_cpu_access(dmabuf, 0, config->metadata_size,
						DMA_FROM_DEVICE);
	if (ret) {
		dma_buf_put(dmabuf);
		pr_info("dmabuf cpu aceess failed\n");
		kfree(temp);
		kfree(dynamic_pb);
		return ret;
	}
	hdr_buff_addr = dma_buf_kmap(dmabuf, 0);
	if (!hdr_buff_addr) {
		pr_info("dma_buf_kmap failed\n");
		dma_buf_end_cpu_access(dmabuf, 0, config->metadata_size,
							DMA_FROM_DEVICE);
		dma_buf_put(dmabuf);
		kfree(temp);
		kfree(dynamic_pb);
		return -1;
	}

	/*obtain metadata*/
	memcpy((void *)temp, hdr_buff_addr, config->metadata_size);
	dma_buf_kunmap(dmabuf, 0, dmabuf);
	dma_buf_end_cpu_access(dmabuf, 0, config->metadata_size,
						DMA_FROM_DEVICE);
	dma_buf_put(dmabuf);

	dynamic_pb->eotf = core->mode.pVideo.pb->eotf;
	dynamic_pb->metadata = core->mode.pVideo.pb->metadata;

	hdr_smetadata = (struct hdr_static_metadata *)temp;
	dynamic_pb->r_x =
		hdr_smetadata->disp_master.display_primaries_x[0];
	dynamic_pb->r_y =
		hdr_smetadata->disp_master.display_primaries_y[0];
	dynamic_pb->g_x =
		hdr_smetadata->disp_master.display_primaries_x[1];
	dynamic_pb->g_y =
		hdr_smetadata->disp_master.display_primaries_y[1];
	dynamic_pb->b_x =
		hdr_smetadata->disp_master.display_primaries_x[2];
	dynamic_pb->b_y =
		hdr_smetadata->disp_master.display_primaries_y[2];
	dynamic_pb->w_x =
		hdr_smetadata->disp_master.white_point_x;
	dynamic_pb->w_y =
		hdr_smetadata->disp_master.white_point_y;
	dynamic_pb->luma_max =
		hdr_smetadata->disp_master.max_display_mastering_luminance
		/ 10000;
	dynamic_pb->luma_min =
		hdr_smetadata->disp_master.min_display_mastering_luminance;
	dynamic_pb->mcll =
		hdr_smetadata->maximum_content_light_level;
	dynamic_pb->mfll =
		hdr_smetadata->maximum_frame_average_light_level;

	/*send metadata*/
	core->dev_func.fc_drm_up(dynamic_pb);

	kfree(temp);
	kfree(dynamic_pb);

	return 0;
}

s32 get_dynamic_config(struct disp_device_dynamic_config *config)
{
	LOG_TRACE();
	return 0;
}
#endif
s32 hdmi_set_display_mode(u32 mode)
{
	u32 hdmi_mode;
	u32 i;
	bool find = false;

	HDMI_INFO_MSG("[hdmi_set_display_mode],mode:%d\n", mode);

	for (i = 0; i < sizeof(hdmi_mode_tbl)/sizeof(struct disp_hdmi_mode); i++) {
		if (hdmi_mode_tbl[i].mode == (enum disp_tv_mode)mode) {
			hdmi_mode = hdmi_mode_tbl[i].hdmi_mode;
			find = true;
			break;
		}
	}

	if (find) {
		/*user configure detailed timing according to vic*/
		if (svd_user_config(hdmi_mode, 0) != 0) {
			HDMI_ERROR_MSG("svd user config failed!\n");
			return -1;
		} else {
			HDMI_INFO_MSG("Set hdmi mode: %d\n", hdmi_mode);
			return 0;
		}
	} else {
		HDMI_ERROR_MSG("unsupported video mode %d when set display mode\n", mode);
		return -1;
	}

}

/*Check if the sink pluged in is in sink blacklist
* @sink_edid: point to the sink's edid block 0
* return: true: indicate the sink is in blacklist
*         false: indicate the sink is NOT in blacklist
*/
int hdmi_mode_blacklist_check(u8 *sink_edid)
{
	u32 i, j;

	for (i = 0; sink_blacklist[i].sink.checksum != 0; i++) {
		for (j = 0; j < 2; j++) {
			if (sink_blacklist[i].sink.mft_id[j] != sink_edid[8 + j])
				break;
		}
		if (j < 2)
			continue;

		for (j = 0; j < 13; j++) {
			if (sink_blacklist[i].sink.stib[j] != sink_edid[95 + j])
				break;
		}
		if (j < 13)
			continue;

		if (sink_blacklist[i].sink.checksum != sink_edid[127])
			return -1;
		else
			return i;
	}

	return -1;
}

s32 hdmi_mode_support(u32 mode)
{
	u32 hdmi_mode;
	u32 i;
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();

	if (core->blacklist_sink >= 0) {
		for (i = 0; i < 10; i++) {
			if (sink_blacklist[core->blacklist_sink].issue[i].tv_mode == mode)
				return sink_blacklist[core->blacklist_sink].issue[i].issue_type << 1;
		}
	}

	for (i = 0; i < sizeof(hdmi_mode_tbl)/sizeof(struct disp_hdmi_mode); i++) {
		if (hdmi_mode_tbl[i].mode == (enum disp_tv_mode)mode) {
			hdmi_mode = hdmi_mode_tbl[i].hdmi_mode;
			if (edid_sink_supports_vic_code(hdmi_mode) == true)
				return 1;
		}
	}
	for (i = 0; i < sizeof(hdmi_mode_tbl_4_3)/
		sizeof(struct disp_hdmi_mode); i++) {
		if (hdmi_mode_tbl_4_3[i].mode == (enum disp_tv_mode)mode) {
			hdmi_mode = hdmi_mode_tbl_4_3[i].hdmi_mode;
			if (edid_sink_supports_vic_code(hdmi_mode) == true)
				return 1;
			}
	}
	return 0;
}

void hdmi_reconfig_format_by_blacklist(struct disp_device_config *config)
{
	u32 i;
	u8 issue = 0;
	struct hdmi_tx_core *core = get_platform();

	if (core->blacklist_sink < 0)
		return;

	for (i = 0; i < 10; i++) {
		if (sink_blacklist[core->blacklist_sink].issue[i].tv_mode == config->mode)
			issue = sink_blacklist[core->blacklist_sink].issue[i].issue_type;
	}

	if ((issue & 0x02) &&
			((config->format == (enum disp_csc_type)YCC444) ||
			 (config->format == (enum disp_csc_type)YCC422))) {
		/* Sink not support yuv on this mode */
		config->format = RGB;
		HDMI_INFO_MSG("Sink is on blacklist and not support YUV on mode %d\n", config->mode);
	}
}

s32 hdmi_get_hpd_status(void)
{
	u32 hpd = 0;

	hpd = hdmi_core_get_hpd_state();
	if (hpd == 0) {
		mdelay(20);
		hpd = hdmi_core_get_hpd_state();
		HDMI_INFO_MSG("delay 20ms and re-get hpd state: %d.\n", hpd);
	}
	return hpd;
}

s32 hdmi_core_get_csc_type(void)
{
	u32 set_encoding = 0;
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();

	set_encoding = core->mode.pVideo.mEncodingIn;
	HDMI_INFO_MSG("set encode=%d\n", set_encoding);

	return set_encoding;
}


s32 hdmi_get_video_timming_info(struct disp_video_timings **video_info)
{
	dtd_t *dtd = NULL;
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();

	dtd = &core->mode.pVideo.mDtd;
	info.vic = dtd->mCode;
	info.tv_mode = 0;

	info.pixel_clk = (dtd->mPixelClock) * 1000 / (dtd->mPixelRepetitionInput+1);
	if ((info.vic == 6) || (info.vic == 7) || (info.vic == 21) || (info.vic == 22))
		info.pixel_clk = (dtd->mPixelClock) * 1000 / (dtd->mPixelRepetitionInput + 1) / (dtd->mInterlaced + 1);
	info.pixel_repeat = dtd->mPixelRepetitionInput;
	info.x_res = (dtd->mHActive) / (dtd->mPixelRepetitionInput+1);
	if (dtd->mInterlaced == 1)
		info.y_res = (dtd->mVActive) * 2;
	else if (dtd->mInterlaced == 0)
		info.y_res = dtd->mVActive;

	info.hor_total_time = (dtd->mHActive + dtd->mHBlanking) / (dtd->mPixelRepetitionInput+1);
	info.hor_back_porch = (dtd->mHBlanking - dtd->mHSyncOffset - dtd->mHSyncPulseWidth) / (dtd->mPixelRepetitionInput+1);
	info.hor_front_porch = (dtd->mHSyncOffset) / (dtd->mPixelRepetitionInput+1);
	info.hor_sync_time = (dtd->mHSyncPulseWidth) / (dtd->mPixelRepetitionInput+1);

	if (dtd->mInterlaced == 1)
		info.ver_total_time = (dtd->mVActive + dtd->mVBlanking) * 2 + 1;
	else if (dtd->mInterlaced == 0)
		info.ver_total_time = dtd->mVActive + dtd->mVBlanking;
	info.ver_back_porch = dtd->mVBlanking - dtd->mVSyncOffset - dtd->mVSyncPulseWidth;
	info.ver_front_porch = dtd->mVSyncOffset;
	info.ver_sync_time = dtd->mVSyncPulseWidth;

	info.hor_sync_polarity = dtd->mHSyncPolarity;
	info.ver_sync_polarity = dtd->mVSyncPolarity;
	info.b_interlace = dtd->mInterlaced;

	if (dtd->mCode == HDMI1080P_24_3D_FP) {
		info.y_res = (dtd->mVActive) * 2;
		info.vactive_space = 45;
		info.trd_mode = 1;
	} else if (dtd->mCode == HDMI720P_50_3D_FP) {
		info.y_res = (dtd->mVActive) * 2;
		info.vactive_space = 30;
		info.trd_mode = 1;
	} else if (dtd->mCode == HDMI720P_60_3D_FP) {
		info.y_res = (dtd->mVActive) * 2;
		info.vactive_space = 30;
		info.trd_mode = 1;
	} else {
		info.vactive_space = 0;
		info.trd_mode = 0;
	}

	*video_info = &info;
	return 0;
}

void hpd_sense_enbale_core(struct hdmi_tx_core *core)
{
	core->dev_func.hpd_enable(1);
}

s32 hdmi_enable_core(void)
{
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();

	video_apply(core);

	return 0;
}

s32 hdmi_disable_core(void)
{
	struct hdmi_tx_core *core = get_platform();

	LOG_TRACE();

	/* Stand by */
	core->dev_func.avmute_enable(1);

	mdelay(100);

	core->dev_func.device_standby();
	/*dev_write(&p_core->hdmi_tx, FC_AVICONF0, 0x0);*/

	return 0;
}


/***********************************************************/
/*************************HDCP******************************/
/**********************************************************/
void hdcp_enable_core(struct hdmi_tx_core *core, u8 enable)
{
	if (enable)
		core->dev_func.hdcp_configure(&core->mode.pHdcp, &core->mode.pVideo);
	else
		core->dev_func.hdcp_disconfigure();
}

/***********************************************************/
/*************************Audio******************************/
/**********************************************************/
#if defined(CONFIG_SND_SUNXI_SOC_HDMIAUDIO)
s32 hdmi_core_audio_enable(u8 enable, u8 channel)
{
	struct hdmi_tx_core *p_core = get_platform();

	LOG_TRACE();
	HDMI_INFO_MSG("mode:%d   channel:%d\n", enable, channel);
	if (enable == 0)
		return 0;

	print_audioinfo(&(p_core->mode.pAudio));
	if (!p_core->dev_func.audio_config(&p_core->mode.pAudio,
						&p_core->mode.pVideo)) {
		HDMI_ERROR_MSG("API Configure ERROR\n");
		return -1;
	}

	return 0;
}
#endif
#if defined(CONFIG_SND_SUNXI_SOC_HDMIAUDIO)

s32 hdmi_set_audio_para(hdmi_audio_t *audio_para)
{
	struct hdmi_tx_core *p_core = get_platform();
	audioParams_t *pAudio = &p_core->mode.pAudio;

	LOG_TRACE();

	memset(pAudio, 0, sizeof(audioParams_t));
	if (audio_para->hw_intf < 2)
		pAudio->mInterfaceType = audio_para->hw_intf;
	else
		pr_info("Unknow hardware interface type\n");
	pAudio->mCodingType = audio_para->data_raw;
	if (pAudio->mCodingType < 1)
		pAudio->mCodingType = PCM;
	pAudio->mSamplingFrequency = audio_para->sample_rate;
	pAudio->mChannelAllocation = audio_para->ca;
	pAudio->mChannelNum = audio_para->channel_num;
	pAudio->mSampleSize = audio_para->sample_bit;
	pAudio->mClockFsFactor = audio_para->fs_between;

	return 0;
}
#endif
void register_func_to_hdmi_core(struct hdmi_dev_func func)
{
	struct hdmi_tx_core *core = get_platform();

	core->dev_func = func;
}


u32 hdmi_core_get_rxsense_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_phy_rxsense_state();
}

u32 hdmi_core_get_phy_pll_lock_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_phy_pll_lock_state();
}

u32 hdmi_core_get_phy_power_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_phy_power_state();
}

u32 hdmi_core_get_tmds_mode(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_tmds_mode();
}

u32 hdmi_core_get_scramble_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_scramble_state();
}

u32 hdmi_core_get_avmute_state(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_avmute_state();

}

u32 hdmi_core_get_pixelrepetion(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_pixelrepetion();

}

u32 hdmi_core_get_color_depth(void)
{
	struct hdmi_tx_core *core = get_platform();

	return  core->dev_func.get_color_depth();

}

u32 hdmi_core_get_colorimetry(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_colorimetry();

}

u32 hdmi_core_get_pixel_format(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_pixel_format();

}

u32 hdmi_core_get_video_code(void)
{
	struct hdmi_tx_core *core = get_platform();
	u32 code = 0;

	code = core->dev_func.get_video_code();
	if (code == 0) {
		code = core->mode.pVideo.mDtd.mCode;
	}

	return code;
}

u32 hdmi_core_get_audio_layout(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_audio_layout();

}


u32 hdmi_core_get_audio_channel_count(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_audio_channel_count();

}

u32 hdmi_core_get_audio_sample_freq(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_audio_sample_freq();

}

u32 hdmi_core_get_audio_sample_size(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_audio_sample_size();

}

u32 hdmi_core_get_audio_n(void)
{
	struct hdmi_tx_core *core = get_platform();

	return core->dev_func.get_audio_n();

}

void hdmi_core_avmute_enable(u8 enable)
{
	struct hdmi_tx_core *core = get_platform();

	core->dev_func.avmute_enable(enable);
}

void hdmi_core_phy_power_enable(u8 enable)
{
	struct hdmi_tx_core *core = get_platform();

	core->dev_func.phy_power_enable(enable);
}

void hdmi_core_dvimode_enable(u8 enable)
{
	struct hdmi_tx_core *core = get_platform();

	core->dev_func.dvimode_enable(enable);
}

