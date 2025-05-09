/*
 * drivers/video/sunxi/disp2/disp/de/include/include.h
 *
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _DISP_INCLUDE_H_
#define _DISP_INCLUDE_H_

//#define __LINUX_PLAT__
#define __UBOOT_PLAT__

#if defined(__LINUX_PLAT__)
#include <linux/module.h>
#include "linux/kernel.h"
#include "linux/mm.h"
#include <asm/uaccess.h>
#include <asm/memory.h>
#include <asm/unistd.h>
#include "linux/semaphore.h"
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create()??kthread_run()
#include <linux/err.h> //IS_ERR()??PTR_ERR()
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "asm-generic/int-ll64.h"
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm.h>
#include <asm/div64.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/compat.h>
#include <sunxi_disp_param.h>

#include <video/sunxi_display2.h>
#include <video/sunxi_metadata.h>
#include "../disp_sys_intf.h"
#include <video/sunxi_metadata.h>

s32 bsp_disp_get_print_level(void);

#if 1
#define __inf(msg...)       do {if (bsp_disp_get_print_level()) {printk(KERN_WARNING "[DISP] %s,line:%d:", __func__, __LINE__); printk(msg); } } while (0)
#define __msg(msg...)       do {if (bsp_disp_get_print_level()) {printk(KERN_WARNING "[DISP] %s,line:%d:", __func__, __LINE__); printk(msg); } } while (0)
#define __wrn(msg...)       do {{printk(KERN_WARNING "[DISP] %s,line:%d:", __func__, __LINE__); printk(msg); } } while (0)
#define __here__            do {if (bsp_disp_get_print_level() == 2) {printk(KERN_WARNING "[DISP] %s,line:%d\n", __func__, __LINE__); } } while (0)
#define __debug(msg...)     do {if (bsp_disp_get_print_level() == 2) {printk(KERN_WARNING "[DISP] %s,line:%d:", __func__, __LINE__); printk(msg); } } while (0)
#else
#define __inf(msg...)  printk(msg)
#define __msg(msg...)  printk(msg)
#define __wrn(msg...)  printk(msg)
#define __here__
#define __debug(msg...)
#endif

#endif//end of define __LINUX_PLAT__

#ifdef __UBOOT_PLAT__
#include <common.h>
#include <malloc.h>
#include <sunxi_display2.h>
#include <sunxi_metadata.h>
#include <sys_config.h>
/*#include <asm/arch/intc.h>*/
#include <pwm.h>
#include <asm/arch/timer.h>
/*#include <asm/arch/platform.h>*/
#include <linux/list.h>
/*#include <asm/memory.h>*/
#include <div64.h>
#include <fdt_support.h>
#include <sunxi_power/axp.h>
#include "asm/io.h"
#include <linux/compat.h>
#include "../disp_sys_intf.h"
#include "disp_features.h"
#include <asm/atomic.h>
#include <sunxi_disp_param.h>
#define OSAL_PRINTF

#define __inf(msg...)
#define __msg(msg...)
#define __wrn(fmt, msg...) pr_notice(fmt, ##msg)
#define __here__
#define __debug(msg...)

#define DP printk("[DEBUG] %s, %s, %d \n", __FILE__, __func__, __LINE__);

#define false 0
#define true 1
#endif /* __UBOOT_PLAT__ */

#if defined(__LINUX_PLAT__)
#define DE_INF __inf
#define DE_MSG __msg
#define DE_WRN __wrn
#define DE_DBG __debug
#define DISP_IRQ_RETURN IRQ_HANDLED
#else
#define DE_INF(msg...)
#define DE_MSG __msg
#define DE_WRN __wrn
#define DE_DBG __debug
#ifndef DISP_IRQ_RETURN
#define DISP_IRQ_RETURN DIS_SUCCESS
#endif
#endif

#define DEFAULT_PRINT_LEVLE 0
#if defined(CONFIG_FPGA_V4_PLATFORM) || defined(CONFIG_FPGA_V7_PLATFORM) || defined(CONFIG_A67_FPGA)
#define __FPGA_DEBUG__
#endif

#ifndef SETMASK
#define SETMASK(width, shift)   ((width?((-1U) >> (32-width)):0)  << (shift))
#endif

#ifndef CLRMASK
#define CLRMASK(width, shift)   (~(SETMASK(width, shift)))
#endif

#ifndef GET_BITS
#define GET_BITS(shift, width, reg) (((reg)&SETMASK(width, shift)) >> (shift))
#endif

#ifndef SET_BITS
#define SET_BITS(shift, width, reg, val)                                       \
	(((reg)&CLRMASK(width, shift)) | (val << (shift)))
#endif

#define DISPALIGN(value, align) ((align == 0) ? value : (((value) + ((align) - 1)) & ~((align) - 1)))

#ifndef abs
#define abs(x) (((x) & 0x80000000) ? (0 - (x)) : (x))
#endif

#define LCD_GAMMA_TABLE_SIZE (256 * sizeof(unsigned int))

#define ONE_SEC 1000000000ull

typedef struct {
	unsigned int   lcd_gamma_en;
	unsigned int   lcd_gamma_tbl[256];
	unsigned int   lcd_cmap_en;
	unsigned int   lcd_cmap_tbl[2][3][4];
	unsigned int   lcd_bright_curve_tbl[256];
} panel_extend_para;

typedef enum {
	DIS_SUCCESS = 0,
	DIS_FAIL =  -1,
	DIS_PARA_FAILED =  -2,
	DIS_PRIO_ERROR =  -3,
	DIS_OBJ_NOT_INITED =  -4,
	DIS_NOT_SUPPORT =  -5,
	DIS_NO_RES =  -6,
	DIS_OBJ_COLLISION =  -7,
	DIS_DEV_NOT_INITED =  -8,
	DIS_DEV_SRAM_COLLISION =  -9,
	DIS_TASK_ERROR = -10,
	DIS_PRIO_COLLSION = -11
} disp_return_value;

/*basic data information definition*/
enum disp_layer_feat {
	DISP_LAYER_FEAT_GLOBAL_ALPHA        = 1 << 0,
	DISP_LAYER_FEAT_PIXEL_ALPHA         = 1 << 1,
	DISP_LAYER_FEAT_GLOBAL_PIXEL_ALPHA  = 1 << 2,
	DISP_LAYER_FEAT_PRE_MULT_ALPHA      = 1 << 3,
	DISP_LAYER_FEAT_COLOR_KEY           = 1 << 4,
	DISP_LAYER_FEAT_ZORDER              = 1 << 5,
	DISP_LAYER_FEAT_POS                 = 1 << 6,
	DISP_LAYER_FEAT_3D                  = 1 << 7,
	DISP_LAYER_FEAT_SCALE               = 1 << 8,
	DISP_LAYER_FEAT_DE_INTERLACE        = 1 << 9,
	DISP_LAYER_FEAT_COLOR_ENHANCE       = 1 << 10,
	DISP_LAYER_FEAT_DETAIL_ENHANCE      = 1 << 11,
};

typedef enum {
	DISP_PIXEL_TYPE_RGB = 0x0,
	DISP_PIXEL_TYPE_YUV = 0x1,
} disp_pixel_type;

typedef enum {
	LAYER_ATTR_DIRTY       = 0x00000001,
	LAYER_VI_FC_DIRTY      = 0x00000002,
	LAYER_HADDR_DIRTY      = 0x00000004,
	LAYER_SIZE_DIRTY       = 0x00000008,
	BLEND_ENABLE_DIRTY     = 0x00000010,
	BLEND_ATTR_DIRTY       = 0x00000020,
	BLEND_CTL_DIRTY        = 0x00000040,
	BLEND_OUT_DIRTY        = 0x00000080,
	LAYER_ATW_DIRTY        = 0x00000100,
	LAYER_HDR_DIRTY        = 0x00000200,
	LAYER_ALL_DIRTY        = 0x000003ff,
} disp_layer_dirty_flags;

typedef enum {
	MANAGER_ENABLE_DIRTY     = 0x00000001,
	MANAGER_CK_DIRTY         = 0x00000002,
	MANAGER_BACK_COLOR_DIRTY = 0x00000004,
	MANAGER_SIZE_DIRTY       = 0x00000008,
	MANAGER_COLOR_RANGE_DIRTY = 0x00000010,
	MANAGER_COLOR_SPACE_DIRTY = 0x00000020,
	MANAGER_BLANK_DIRTY      = 0x00000040,
	MANAGER_KSC_DIRTY = 0x00000080,
	MANAGER_ALL_DIRTY = 0x000000ff,
} disp_manager_dirty_flags;

/* disp_fb_info_inner - image buffer info on the inside
 *
 * @addr: buffer address for each plane
 * @size: size<width,height> for each buffer, unit:pixels
 * @align: align for each buffer, unit:bytes
 * @format: pixel format
 * @color_space: color space
 * @trd_right_addr: the right-eye buffer address for each plane,
 *                  valid when frame-packing 3d buffer input
 * @pre_multiply: indicate the pixel use premultiplied alpha
 * @crop: crop rectangle for buffer to be display
 * @flag: indicate stereo/non-stereo buffer
 * @scan: indicate interleave/progressive scan type, and the scan order
 * @metadata_buf: the phy_address to the buffer contained metadata for fbc/hdr
 * @metadata_size: the size of metadata buffer, unit:bytes
 * @metadata_flag: the flag to indicate the type of metadata buffer
 *	0     : no metadata
 *	1 << 0: hdr static metadata
 *	1 << 1: hdr dynamic metadata
 *	1 << 4:	frame buffer compress(fbc) metadata
 *	x     : all type could be "or" together
 */
struct disp_fb_info_inner {
	unsigned long long       addr[3];
	struct disp_rectsz       size[3];
	unsigned int             align[3];
	enum disp_pixel_format   format;
	enum disp_color_space    color_space;
	unsigned int             trd_right_addr[3];
	bool                     pre_multiply;
	struct disp_rect64       crop;
	enum disp_buffer_flags   flags;
	enum disp_scan_flags     scan;
	enum disp_eotf           eotf;
	unsigned int             fbd_en;
	unsigned long long       metadata_buf;
	unsigned int             metadata_size;
	unsigned int             metadata_flag;
	struct dma_buf           *metadata_dmabuf;
	struct sunxi_metadata    *p_metadata;
	struct afbc_header       *p_afbc_header;
};

/**
 * disp_snr_info
 */
struct disp_snr_info_inner {
	unsigned char en;
	unsigned char demo_en;
	struct disp_rect demo_win;
	unsigned char y_strength;
	unsigned char u_strength;
	unsigned char v_strength;
	unsigned char th_ver_line;
	unsigned char th_hor_line;
};

/* disp_layer_info_inner - layer info on the inside
 *
 * @mode: buffer/clolor mode, when in color mode, the layer is widthout buffer
 * @zorder: the zorder of layer, 0~max-layer-number
 * @alpha_mode:
 *	0: pixel alpha;
 *	1: global alpha
 *	2: mixed alpha, compositing width pixel alpha before global alpha
 * @alpha_value: global alpha value, valid when alpha_mode is not pixel alpha
 * @screen_win: the rectangle on the screen for fb to be display
 * @b_trd_out: indicate if 3d display output
 * @out_trd_mode: 3d output mode, valid when b_trd_out is true
 * @color: the color value to be display, valid when layer is in color mode
 * @fb: the framebuffer info related width the layer, valid when in buffer mode
 * @id: frame id, the user could get the frame-id display currently by
 *	DISP_LAYER_GET_FRAME_ID ioctl
 * @atw: asynchronous time wrap information
 */
struct disp_layer_info_inner {
	enum disp_layer_mode      mode;
	unsigned char             zorder;
	unsigned char             alpha_mode;
	unsigned char             alpha_value;
	struct disp_rect          screen_win;
	bool                      b_trd_out;
	enum disp_3d_out_mode     out_trd_mode;
	union {
		unsigned int               color;
		struct disp_fb_info_inner  fb;
	};

	unsigned int              id;
	struct disp_atw_info      atw;
#if defined(DE_VERSION_V33X) || defined(DE_VERSION_V35X)
	int transform;
	struct disp_snr_info_inner snr;
#endif
};

/* disp_layer_config_inner - layer config on the inside
 *
 * @info: layer info
 * @enable: indicate to enable/disable the layer
 * @channel: the channel index of the layer, 0~max-channel-number
 * @layer_id: the layer index of the layer widthin it's channel
 */
struct disp_layer_config_inner {
	struct disp_layer_info_inner info;
	bool enable;
	unsigned int channel;
	unsigned int layer_id;
};

struct disp_device_info {
	u32 line_reg;
	u32 line_shift;
	u32 line_mask;
	u32 safe_line;
	u32 sleep_us;
};

struct disp_amp_preview_channel_info {
	unsigned long alpha_reg;
	unsigned long alpha_shift;
	unsigned long addr_reg;
	unsigned int layer_reg_size;
	unsigned long fill_color_en_reg;
	unsigned int fill_color_en_bit;
	unsigned long fill_color_reg;
};

/* disp_layer_config_ops - operations for layer config
 *
 * @vmap:vmap a block contigous phys memory into virtual space
 * @vunmap: release virtual mapping obtained by vmap()
 */
struct disp_layer_config_ops {
	void *(*vmap)(unsigned long phys_addr, unsigned long size);
	void (*vunmap)(const void *vaddr);
};

struct disp_layer_config_data {
	struct disp_layer_config_inner config;
	disp_layer_dirty_flags flag;
	struct disp_layer_config_ops ops;
};

struct disp_manager_info {
	struct disp_color back_color;
	struct disp_colorkey ck;
	struct disp_rect size;
	enum disp_csc_type cs;
	enum disp_color_space color_space;
	u32 color_range;
	u32 interlace;
	bool enable;
	u32 disp_device;//disp of device
	u32 hwdev_index;//indicate the index of timing controller
	bool blank;//true: disable all layer; false: enable layer according to layer_config.enable
	u32 de_freq;
	enum disp_eotf eotf; /* sdr/hdr10/hlg */
	enum disp_data_bits data_bits;
	u32 device_fps;
	struct disp_ksc_info ksc;
};

struct disp_manager_data {
	struct disp_manager_info config;
	disp_manager_dirty_flags flag;
};

struct disp_clk_info {
		u32                     clk;
		u32                     clk_div;
		u32                     h_clk;
		u32                     clk_src;
		u32                     clk_div2;

		u32                     clk_p;
		u32                     clk_div_p;
		u32                     h_clk_p;
		u32                     clk_src_p;

		u32                     ahb_clk;
		u32                     h_ahb_clk;
		u32                     dram_clk;
		u32                     h_dram_clk;

		bool                    enabled;
};

struct disp_enhance_info {
	/*
	 * enhance parameters : 0~10, bigger value, stronger enhance level
	 * mode : combination of enhance_mode and dev_type
	 * enhance_mode : bit31~bit16 of mode
	 *              : 0-disable; 1-enable; 2-demo(enable half window)
	 * dev_type : bit15~bit0 of mode
	 *          : 0-lcd; 1-tv(hdmi, cvbs, vga, ypbpr)
	 */

	u32         bright;
	u32         contrast;
	u32         saturation;
	u32         hue;
	u32         edge;
	u32         detail;
	u32         denoise;
	u32         mode;
	u32         sharp;	//0-off; 1~3-on.
	u32         auto_contrast;	//0-off; 1~3-on.
	u32					auto_color;	//0-off; 1-on.
	u32         fancycolor_red; //0-Off; 1-2-on.
	u32         fancycolor_green;//0-Off; 1-2-on.
	u32         fancycolor_blue;//0-Off; 1-2-on.
	struct disp_rect   window;
	u32         enable;
	struct disp_rect size;
	u32         demo_enable;//1: enable demo mode
};

typedef enum {
	ENH_NONE_DIRTY       = 0x0,
	ENH_ENABLE_DIRTY     = 0x1 << 0,  /* enable dirty */
	ENH_SIZE_DIRTY       = 0x1 << 1,  /* size dirty */
	ENH_FORMAT_DIRTY     = 0x1 << 2,  /* overlay format dirty */
	ENH_BYPASS_DIRTY     = 0x1 << 3, /* bypass dirty */
	ENH_INIT_DIRTY       = 0x1 << 8,  /* initial parameters dirty */
	ENH_MODE_DIRTY       = 0X1 << 9,  /* enhance mode dirty */
	ENH_BRIGHT_DIRTY     = 0x1 << 10,  /* brightness level dirty */
	ENH_CONTRAST_DIRTY   = 0x1 << 11,  /* contrast level dirty */
	ENH_EDGE_DIRTY       = 0x1 << 12,  /* edge level dirty */
	ENH_DETAIL_DIRTY     = 0x1 << 13,  /* detail level dirty */
	ENH_SAT_DIRTY        = 0x1 << 14,  /* saturation level dirty */
	ENH_DNS_DIRTY        = 0x1 << 15, /* de-noise level dirty */
	ENH_USER_DIRTY       = 0xf00,     /* dirty by user */
	ENH_ALL_DIRTY        = 0xffff      /* all dirty */
} disp_enhance_dirty_flags;

struct disp_enhance_config {
	struct disp_enhance_info info;
	disp_enhance_dirty_flags flags;
};

typedef enum {
	SMBL_DIRTY_NONE      = 0x00000000,
	SMBL_DIRTY_ENABLE    = 0x00000001,
	SMBL_DIRTY_WINDOW    = 0x00000002,
	SMBL_DIRTY_SIZE      = 0x00000004,
	SMBL_DIRTY_BL        = 0x00000008,
	SMBL_DIRTY_ALL       = 0x0000000F,
} disp_smbl_dirty_flags;

struct disp_smbl_info {
	struct disp_rect                window;
	u32                      enable;
	struct disp_rect              size;
	u32                      backlight;
	u32                      backlight_dimming;
	disp_smbl_dirty_flags    flags;
};

struct disp_csc_config {
	u32 in_fmt;
	u32 in_mode;
	u32 in_color_range;
	u32 out_fmt;
	u32 out_mode;
	u32 out_color_range;
	u32 brightness;
	u32 contrast;
	u32 saturation;
	u32 hue;
	u32 enhance_mode;
	u32 color;
	u32 in_eotf;
	u32 out_eotf;
};

enum {
	DE_RGB = 0,
	DE_YUV = 1,
};

typedef enum {
  CAPTURE_DIRTY_ADDRESS   = 0x00000001,
  CAPTURE_DIRTY_WINDOW	  = 0x00000002,
  CAPTURE_DIRTY_SIZE	  = 0x00000004,
  CAPTURE_DIRTY_ALL	  = 0x00000007,
} disp_capture_dirty_flags;

/* disp_s_frame_inner - display simple frame buffer
 *
 * @format: pixel format of fb
 * @size: size for each plane
 * @crop: crop zone to be fill image data
 * @fd: dma_buf fd
 * @addr: buffer addr for each plane
 */
struct disp_s_frame_inner {
	enum disp_pixel_format format;
	struct disp_rectsz size[3];
	struct disp_rect crop;
	unsigned long long addr[3];
	int fd;
};

/* disp_capture_config - configuration for capture function
 *
 * @in_frame: input frame information
 * @out_frame: output framebuffer infomation
 * @disp: indicate which disp channel to be capture
 * @flags: caputre flags
 */
struct disp_capture_config {
	struct disp_s_frame_inner in_frame;     /* only format/size/crop valid */
	struct disp_s_frame_inner out_frame;
	u32 disp;               /* which disp channel to be capture */
	disp_capture_dirty_flags flags;
};

typedef enum {
	LCD_IF_HV			  = 0,
	LCD_IF_CPU			= 1,
	LCD_IF_LVDS			= 3,
	LCD_IF_DSI			= 4,
	LCD_IF_EDP      = 5,
	LCD_IF_EXT_DSI  = 6,
} disp_lcd_if;

typedef enum {
	LCD_HV_IF_PRGB_1CYC		  = 0,  //parallel hv
	LCD_HV_IF_SRGB_3CYC		  = 8,  //serial hv
	LCD_HV_IF_DRGB_4CYC		  = 10, //Dummy RGB
	LCD_HV_IF_RGBD_4CYC		  = 11, //RGB Dummy
	LCD_HV_IF_CCIR656_2CYC	= 12,
} disp_lcd_hv_if;

typedef enum {
	LCD_HV_SRGB_SEQ_RGB_RGB	= 0,
	LCD_HV_SRGB_SEQ_RGB_BRG	= 1,
	LCD_HV_SRGB_SEQ_RGB_GBR	= 2,
	LCD_HV_SRGB_SEQ_BRG_RGB	= 4,
	LCD_HV_SRGB_SEQ_BRG_BRG	= 5,
	LCD_HV_SRGB_SEQ_BRG_GBR	= 6,
	LCD_HV_SRGB_SEQ_GRB_RGB	= 8,
	LCD_HV_SRGB_SEQ_GRB_BRG	= 9,
	LCD_HV_SRGB_SEQ_GRB_GBR	= 10,
} disp_lcd_hv_srgb_seq;

typedef enum {
	LCD_HV_SYUV_SEQ_YUYV	= 0,
	LCD_HV_SYUV_SEQ_YVYU	= 1,
	LCD_HV_SYUV_SEQ_UYUV	= 2,
	LCD_HV_SYUV_SEQ_VYUY	= 3,
} disp_lcd_hv_syuv_seq;

typedef enum {
	LCD_HV_SYUV_FDLY_0LINE	= 0,
	LCD_HV_SRGB_FDLY_2LINE	= 1, /*ccir pal*/
	LCD_HV_SRGB_FDLY_3LINE	= 2, /*ccir ntsc*/
} disp_lcd_hv_syuv_fdly;

typedef enum {
	LCD_CPU_IF_RGB666_18PIN = 0,
	LCD_CPU_IF_RGB666_9PIN  = 10,
	LCD_CPU_IF_RGB666_6PIN  = 12,
	LCD_CPU_IF_RGB565_16PIN = 8,
	LCD_CPU_IF_RGB565_8PIN  = 14,
} disp_lcd_cpu_if;

typedef enum {
	LCD_CPU_AUTO_MODE    = 0,
	LCD_CPU_TRIGGER_MODE = 1,
} disp_lcd_cpu_mode;

typedef enum {
	LCD_TE_DISABLE	= 0,
	LCD_TE_RISING		= 1,
	LCD_TE_FALLING  = 2,
} disp_lcd_te;

typedef enum {
	LCD_LVDS_IF_SINGLE_LINK		= 0,
	LCD_LVDS_IF_DUAL_LINK		  = 1,
	LCD_LVDS_IF_DUAL_LINK_SAME_SRC = 2,
} disp_lcd_lvds_if;

typedef enum {
	LCD_LVDS_8bit		= 0,
	LCD_LVDS_6bit		= 1,
} disp_lcd_lvds_colordepth;

typedef enum {
	LCD_LVDS_MODE_NS		  = 0,
	LCD_LVDS_MODE_JEIDA		= 1,
} disp_lcd_lvds_mode;

typedef enum {
	LCD_DSI_IF_VIDEO_MODE	  = 0,
	LCD_DSI_IF_COMMAND_MODE	= 1,
	LCD_DSI_IF_BURST_MODE   = 2,
} disp_lcd_dsi_if;

typedef enum {
	LCD_DSI_1LANE			= 1,
	LCD_DSI_2LANE			= 2,
	LCD_DSI_3LANE			= 3,
	LCD_DSI_4LANE			= 4,
} disp_lcd_dsi_lane;

typedef enum {
	LCD_DSI_FORMAT_RGB888	  = 0,
	LCD_DSI_FORMAT_RGB666	  = 1,
	LCD_DSI_FORMAT_RGB666P	= 2,
	LCD_DSI_FORMAT_RGB565	  = 3,
} disp_lcd_dsi_format;

typedef enum {
	LCD_FRM_BYPASS	= 0,
	LCD_FRM_RGB666	= 1,
	LCD_FRM_RGB565	= 2,
} disp_lcd_frm;

typedef enum {
	LCD_CMAP_B0	= 0x0,
	LCD_CMAP_G0	= 0x1,
	LCD_CMAP_R0	= 0x2,
	LCD_CMAP_B1	= 0x4,
	LCD_CMAP_G1	= 0x5,
	LCD_CMAP_R1	= 0x6,
	LCD_CMAP_B2	= 0x8,
	LCD_CMAP_G2	= 0x9,
	LCD_CMAP_R2	= 0xa,
	LCD_CMAP_B3	= 0xc,
	LCD_CMAP_G3	= 0xd,
	LCD_CMAP_R3	= 0xe,
} disp_lcd_cmap_color;

typedef struct {
	unsigned int lp_clk_div;
	unsigned int hs_prepare;
	unsigned int hs_trail;
	unsigned int clk_prepare;
	unsigned int clk_zero;
	unsigned int clk_pre;
	unsigned int clk_post;
	unsigned int clk_trail;
	unsigned int hs_dly_mode;
	unsigned int hs_dly;
	unsigned int lptx_ulps_exit;
	unsigned int hstx_ana0;
	unsigned int hstx_ana1;
} __disp_dsi_dphy_timing_t;

/**
 * lcd tcon mode(dual tcon drive dual dsi)
 */
enum disp_lcd_tcon_mode {
	DISP_TCON_NORMAL_MODE = 0,
	DISP_TCON_MASTER_SYNC_AT_FIRST_TIME,
	DISP_TCON_MASTER_SYNC_EVERY_FRAME,
	DISP_TCON_SLAVE_MODE,
	DISP_TCON_DUAL_DSI,
};

enum disp_lcd_dsi_port {
	DISP_LCD_DSI_SINGLE_PORT = 0,
	DISP_LCD_DSI_DUAL_PORT,
};

typedef enum {
	DISP_LCD_DSI_CLK_CONTINUOUS = 0,
	DISP_LCD_DSI_CLK_NON_CONTINUOUS = 1,
} disp_lcd_dsi_clk_mode;

enum div_flag {
	 INCREASE        = 1,
	 DECREASE        = -1,
};

struct clk_div_ajust {
	enum div_flag clk_div_increase_or_decrease;
	int div_multiple;
};

typedef struct {
	bool is_enabled;
	enum disp_output_type convert_type;
	enum disp_tv_mode cur_mode;
} disp_panel_convert_info;

typedef struct {
	disp_lcd_if              lcd_if;

	disp_lcd_hv_if           lcd_hv_if;
	disp_lcd_hv_srgb_seq     lcd_hv_srgb_seq;
	disp_lcd_hv_syuv_seq     lcd_hv_syuv_seq;
	disp_lcd_hv_syuv_fdly    lcd_hv_syuv_fdly;

	disp_lcd_lvds_if         lcd_lvds_if;
	disp_lcd_lvds_colordepth lcd_lvds_colordepth; //color depth, 0:8bit; 1:6bit
	disp_lcd_lvds_mode       lcd_lvds_mode;
	unsigned int             lcd_lvds_io_polarity;

	disp_lcd_cpu_if          lcd_cpu_if;
	disp_lcd_te              lcd_cpu_te;
	disp_lcd_cpu_mode        lcd_cpu_mode;

	disp_lcd_dsi_if          lcd_dsi_if;
	disp_lcd_dsi_lane        lcd_dsi_lane;
	disp_lcd_dsi_format      lcd_dsi_format;
	unsigned int             lcd_dsi_eotp;
	unsigned int             lcd_dsi_vc;
	disp_lcd_te              lcd_dsi_te;
	enum disp_lcd_dsi_port   lcd_dsi_port_num;
	disp_lcd_dsi_clk_mode    lcd_dsi_clk_mode;
	unsigned int lcd_dsi_clk_if;
	enum disp_lcd_tcon_mode  lcd_tcon_mode;
	unsigned int             lcd_slave_stop_pos;
	unsigned int             lcd_sync_pixel_num;
	unsigned int             lcd_sync_line_num;
	unsigned int             lcd_slave_tcon_num;
	unsigned int             lcd_tcon_en_odd_even;
	unsigned int             hs_trail_set;

	unsigned int             lcd_dsi_dphy_timing_en;
	__disp_dsi_dphy_timing_t *lcd_dsi_dphy_timing_p;

	unsigned int lcd_fsync_en;
	unsigned int lcd_fsync_act_time;
	unsigned int lcd_fsync_dis_time;
	unsigned int lcd_fsync_pol;

	unsigned int            lcd_dclk_freq;
	unsigned int            lcd_x; //horizontal resolution
	unsigned int            lcd_y; //vertical resolution
	unsigned int            lcd_width; //width of lcd in mm
	unsigned int            lcd_height;//height of lcd in mm
	unsigned int            lcd_xtal_freq;

	unsigned int            lcd_pwm_used;
	unsigned int            lcd_pwm_ch;
	unsigned int            lcd_pwm_freq;
	unsigned int            lcd_pwm_pol;
	unsigned int            lcd_backlight_delay;

	unsigned int            lcd_rb_swap;
	unsigned int            lcd_rgb_endian;

	unsigned int            lcd_vt;
	unsigned int            lcd_ht;
	unsigned int            lcd_vbp;
	unsigned int            lcd_hbp;
	unsigned int            lcd_vspw;
	unsigned int            lcd_hspw;

	unsigned int            lcd_interlace;
	unsigned int            lcd_hv_clk_phase;
	unsigned int            lcd_hv_sync_polarity;

	unsigned int            lcd_frm;
	unsigned int            lcd_gamma_en;
	unsigned int            lcd_cmap_en;
	unsigned int            lcd_bright_curve_en;
	unsigned int            lcd_start_delay;

	char                    lcd_size[8]; //e.g. 7.9, 9.7
	char                    lcd_model_name[32];

	unsigned int            tcon_index; //not need to config for user
	unsigned int            lcd_fresh_mode;//not need to config for user
	unsigned int            lcd_dclk_freq_original; //not need to config for user
	unsigned int            ccir_clk_div; /*not need to config for user*/
	unsigned int            input_csc;
	unsigned int lcd_hv_data_polarity;
	struct clk_div_ajust tcon_clk_div_ajust;
	disp_panel_convert_info convert_info;
} disp_panel_para;

typedef struct {
	enum disp_tv_mode tv_mode;
	disp_panel_para panel_info;
} disp_panel_para_mapping;

typedef enum {
	DISP_MOD_DE = 0,
#if defined(CONFIG_INDEPENDENT_DE)
	DISP_MOD_DE1,
#endif
	DISP_MOD_DEVICE, //for timing controller common module
#if defined(CONFIG_INDEPENDENT_DE)
	DISP_MOD_DEVICE1,
#endif
	DISP_MOD_LCD0,
	DISP_MOD_LCD1,
	DISP_MOD_LCD2,
	DISP_MOD_LCD3,
	DISP_MOD_LCD4,
	DISP_MOD_DSI0,
	DISP_MOD_DSI1,
	DISP_MOD_DSI2,
	DISP_MOD_DSI3,
#if defined(DE_VERSION_V35X)
	DSIP_MOD_COMBPHY0,
	DSIP_MOD_COMBPHY1,
#endif
	DISP_MOD_HDMI,
	DISP_MOD_LVDS,
	DISP_MOD_LVDS1,
	DISP_MOD_EINK,
	DISP_MOD_EDMA,
#if defined(CONFIG_INDEPENDENT_DE)
	DISP_MOD_DPSS0,
	DISP_MOD_DPSS1,
#endif
	DISP_MOD_NUM,
} disp_mod_id;

typedef struct {
	int sync; //1: sync width bootloader
	int disp; //output disp at bootloader period
	int type; //output type at bootloader period
	int mode; //output mode at bootloader period
} disp_bootloader_info;

#define DEBUG_TIME_SIZE 100
typedef struct {
	unsigned long         sync_time[DEBUG_TIME_SIZE];//for_debug
	unsigned int          sync_time_index;//for_debug
	unsigned int          skip_cnt;
	unsigned int          error_cnt;//under flow .ect
	unsigned int          irq_cnt;
	unsigned int          vsync_cnt;
	unsigned int          vsync_skip_cnt;
} disp_health_info;

typedef struct {
	uintptr_t reg_base[DISP_MOD_NUM];
	u32 irq_no[DISP_MOD_NUM];
	struct clk *mclk[DISP_MOD_NUM];

	s32 (*disp_int_process)(u32 sel);
	s32 (*vsync_event)(u32 sel);
	s32 (*start_process)(void);
	s32 (*capture_event)(u32 sel);
	s32 (*shadow_protect)(u32 sel, bool protect);
	disp_bootloader_info boot_info;
	struct disp_feat_init feat_init;
} disp_bsp_init_para;

typedef void (*LCD_FUNC) (unsigned int sel);
typedef void (*EDP_FUNC) (unsigned int sel);

typedef struct lcd_function {
	LCD_FUNC func;
	unsigned int delay;//ms
} disp_lcd_function;

#define LCD_MAX_SEQUENCES 7
typedef struct lcd_flow {
    disp_lcd_function func[LCD_MAX_SEQUENCES];
    unsigned int func_num;
    unsigned int cur_step;
} disp_lcd_flow;

typedef struct {
	void (*cfg_panel_info)(panel_extend_para *info);
	int (*cfg_open_flow)(unsigned int sel);
	int (*cfg_close_flow)(unsigned int sel);
	int (*lcd_user_defined_func)(unsigned int sel, unsigned int para1, unsigned int para2, unsigned int para3);
	int (*set_bright)(unsigned int sel, unsigned int bright);
	int (*get_panel_para_mapping)(const disp_panel_para_mapping **);
} disp_lcd_panel_fun;

typedef struct {
	//basic adjust
	u32         bright;
	u32         contrast;
	u32         saturation;
	u32         hue;
	u32         mode;
	//ehnance
	u32         sharp;	//0-off; 1~3-on.
	u32         auto_contrast;	//0-off; 1~3-on.
	u32					auto_color;	//0-off; 1-on.
	u32         fancycolor_red; //0-Off; 1-2-on.
	u32         fancycolor_green;//0-Off; 1-2-on.
	u32         fancycolor_blue;//0-Off; 1-2-on.
	struct disp_rect   window;
	u32         enable;
} disp_enhance_para;

struct disp_device {
	struct list_head list;
	/* data fields */
	char name[32];
	u32 disp;
	u32 hwdev_index;//indicate the index of hw device(timing controller)
	u32 fix_timing;
	enum disp_output_type type;
	struct disp_manager *manager;
	struct disp_video_timings timings;
	struct work_struct close_eink_panel_work;
	void *priv_data;

	/* function fileds  */
	/* init: script init && clock init && pwm init && register irq
	 * exit: clock exit && unregister irq
	 */
	s32 (*init)(struct disp_device *dispdev);
	s32 (*exit)(struct disp_device *dispdev);

	s32 (*set_manager)(struct disp_device *dispdev, struct disp_manager *mgr);
	s32 (*unset_manager)(struct disp_device *dispdev);

	s32 (*enable)(struct disp_device *dispdev);
	s32 (*fake_enable)(struct disp_device *dispdev);
	s32 (*sw_enable)(struct disp_device *dispdev);
	s32 (*disable)(struct disp_device *dispdev);
	s32 (*is_enabled)(struct disp_device *dispdev);
	s32 (*is_used)(struct disp_device *dispdev);
	s32 (*get_resolution)(struct disp_device *dispdev, u32 *xres, u32 *yres);
	s32 (*get_dimensions)(struct disp_device *dispdev, u32 *width, u32 *height);
	s32 (*set_timings)(struct disp_device *dispdev, struct disp_video_timings *timings);
	s32 (*get_timings)(struct disp_device *dispdev, struct disp_video_timings *timings);
	s32 (*check_timings)(struct disp_device *dispdev, struct disp_video_timings *timings);
	s32 (*detect)(struct disp_device *dispdev);
	s32 (*set_detect)(struct disp_device *dispdev, bool hpd);
	s32 (*get_status)(struct disp_device *dispdev);
	s32 (*get_fps)(struct disp_device *dispdev);
	bool (*is_in_safe_period)(struct disp_device *dispdev);
	s32 (*get_input_csc)(struct disp_device *dispdev);
	s32 (*get_input_color_range)(struct disp_device *dispdev);
	s32 (*is_interlace)(struct disp_device *dispdev);
	int (*get_preview_device_info)(struct disp_device *dispdev, struct disp_device_info *info);

	/* power manager */
	s32 (*early_suspend)(struct disp_device *dispdev);
	s32 (*late_resume)(struct disp_device *dispdev);
	s32 (*suspend)(struct disp_device *dispdev);
	s32 (*resume)(struct disp_device *dispdev);

	s32 (*dump)(struct disp_device *dispdev, char *buf);

	/* HDMI /TV */
	s32 (*get_work_mode)(struct disp_device *dispdev);
	s32 (*set_mode)(struct disp_device *dispdev, u32 mode);
	s32 (*get_mode)(struct disp_device *dispdev);
	s32 (*set_static_config)(struct disp_device *dispdev,
				 struct disp_device_config *config);
	s32 (*get_static_config)(struct disp_device *dispdev,
				 struct disp_device_config *config);
	s32 (*set_dynamic_config)(struct disp_device *dispdev,
				  struct disp_device_dynamic_config *config);
	s32 (*get_dynamic_config)(struct disp_device *dispdev,
				  struct disp_device_dynamic_config *config);
	/*
	 * check_config_dirty
	 * check if the config is not the same with current one
	 */
	bool (*check_config_dirty)(struct disp_device *dispdev,
				   struct disp_device_config *config);
	s32 (*get_support_mode)(struct disp_device *dispdev, u32 init_mode);
	s32 (*check_support_mode)(struct disp_device *dispdev, u32 mode);
	s32 (*set_func)(struct disp_device *dispdev, struct disp_device_func *func);
	s32 (*set_tv_func)(struct disp_device *dispdev, struct disp_tv_func *func);
	s32 (*set_enhance_mode)(struct disp_device *dispdev, u32 mode);

	/* LCD */
	s32 (*set_bright)(struct disp_device *dispdev, u32 bright);
	s32 (*get_bright)(struct disp_device *dispdev);
	s32 (*backlight_enable)(struct disp_device *dispdev);
	s32 (*backlight_disable)(struct disp_device *dispdev);
	s32 (*pwm_enable)(struct disp_device *dispdev);
	s32 (*pwm_disable)(struct disp_device *dispdev);
	s32 (*power_enable)(struct disp_device *dispdev, u32 power_id);
	s32 (*power_disable)(struct disp_device *dispdev, u32 power_id);
	s32 (*tcon_enable)(struct disp_device *dispdev);
	s32 (*tcon_disable)(struct disp_device *dispdev);
	s32 (*set_bright_dimming)(struct disp_device *dispdev, u32 dimming);

	disp_lcd_flow *(*get_open_flow)(struct disp_device *dispdev);
	disp_lcd_flow *(*get_close_flow)(struct disp_device *dispdev);
	int (*switch_compat_panel)(struct disp_device *dispdev, unsigned int index);
	s32 (*pin_cfg)(struct disp_device *dispdev, u32 bon);
	s32 (*set_gamma_tbl)(struct disp_device *dispdev, u32 *tbl, u32 size);
	s32 (*enable_gamma)(struct disp_device *dispdev);
	s32 (*disable_gamma)(struct disp_device *dispdev);
	s32 (*set_panel_func)(struct disp_device *lcd, char *name, disp_lcd_panel_fun *lcd_cfg);
	s32 (*set_open_func)(struct disp_device *lcd, LCD_FUNC func, u32 delay);
	s32 (*set_close_func)(struct disp_device *lcd, LCD_FUNC func, u32 delay);
	int (*gpio_set_value)(struct disp_device *dispdev, unsigned int io_index, u32 value);
	/**** add for lcd read gpio level id by lxm 20220310 ***/
	s32 (*gpio_get_value)(struct disp_device *dispdev, unsigned int io_index);
	int (*gpio_set_direction)(struct disp_device *dispdev, unsigned int io_index, u32 direction);
	int (*get_panel_info)(struct disp_device *dispdev, disp_panel_para *info);
	void (*show_builtin_patten)(struct disp_device *dispdev, u32 patten);
};

/* manager */
struct disp_manager {
	/* data fields */
	char name[32];
	u32 disp;
	u32 num_chns;
	u32 num_layers;
	struct disp_device *device;
	struct disp_smbl *smbl;
	struct disp_enhance *enhance;
	struct disp_capture *cptr;

	struct list_head lyr_list;

	/* function fields */
	s32 (*enable)(struct disp_manager *mgr);
	s32 (*sw_enable)(struct disp_manager *mgr);
	s32 (*disable)(struct disp_manager *mgr);
	s32 (*is_enabled)(struct disp_manager *mgr);
	s32 (*blank)(struct disp_manager *mgr, bool blank);

	/* init: clock init && reg init && register irq
	 * exit: clock exit && unregister irq
	 */
	s32 (*init)(struct disp_manager *mgr);
	s32 (*exit)(struct disp_manager *mgr);

	s32 (*set_back_color)(struct disp_manager *mgr,	struct disp_color *bk_color);
	s32 (*get_back_color)(struct disp_manager *mgr,	struct disp_color *bk_color);
	s32 (*set_color_key)(struct disp_manager *mgr, struct disp_colorkey *ck);
	s32 (*get_color_key)(struct disp_manager *mgr, struct disp_colorkey *ck);

	s32 (*get_screen_size)(struct disp_manager *mgr, u32 *width, u32 *height);
	s32 (*set_screen_size)(struct disp_manager *mgr, u32 width, u32 height);
	s32 (*get_clk_rate)(struct disp_manager *mgr);

	/* layer mamage */
	s32 (*set_layer_config)(struct disp_manager *mgr, struct disp_layer_config *config, unsigned int layer_num);
	s32 (*get_layer_config)(struct disp_manager *mgr, struct disp_layer_config *config, unsigned int layer_num);
	s32 (*set_layer_config2)(struct disp_manager *mgr,
				 struct disp_layer_config2 *config,
				 unsigned int layer_num);
	s32 (*get_layer_config2)(struct disp_manager *mgr,
				 struct disp_layer_config2 *config,
				 unsigned int layer_num);
	s32 (*set_output_color_range)(struct disp_manager *mgr, u32 color_range);
	s32 (*get_output_color_range)(struct disp_manager *mgr);
	s32 (*update_color_space)(struct disp_manager *mgr);

	s32 (*apply)(struct disp_manager *mgr);
	s32 (*force_apply)(struct disp_manager *mgr);
	s32 (*update_regs)(struct disp_manager *mgr);
	s32 (*sync)(struct disp_manager *mgr);
	s32 (*tasklet)(struct disp_manager *mgr);

	/* debug interface, dump manager info */
	s32 (*dump)(struct disp_manager *mgr, char *buf);
	s32 (*reg_protect)(struct disp_manager *mgr, bool protect);
	s32 (*set_ksc_para)(struct disp_manager *mgr,
				      struct disp_ksc_info *pinfo);
};

struct disp_layer {
	/* data fields */
	char name[32];
	u32 disp;
	u32 chn;
	u32 id;

	//enum disp_layer_feat caps;
	struct disp_manager *manager;
	struct list_head list;
	void *data;

	/* function fileds */

//	s32 (*is_support_caps)(struct disp_layer* layer, enum disp_layer_feat caps);
	s32 (*is_support_format)(struct disp_layer *layer, enum disp_pixel_format fmt);
	s32 (*set_manager)(struct disp_layer *layer, struct disp_manager *mgr);
	s32 (*unset_manager)(struct disp_layer *layer);

	s32 (*check)(struct disp_layer *layer, struct disp_layer_config *config);
	s32 (*check2)(struct disp_layer *layer,
		      struct disp_layer_config2 *config);
	s32 (*save_and_dirty_check)(struct disp_layer *layer, struct disp_layer_config *config);
	s32 (*save_and_dirty_check2)(struct disp_layer *layer,
				     struct disp_layer_config2 *config);
	s32 (*get_config)(struct disp_layer *layer, struct disp_layer_config *config);
	s32 (*get_config2)(struct disp_layer *layer,
			   struct disp_layer_config2 *config);
	s32 (*apply)(struct disp_layer *layer);
	s32 (*force_apply)(struct disp_layer *layer);
	s32 (*is_dirty)(struct disp_layer *layer);
	s32 (*dirty_clear)(struct disp_layer *layer);

	/* init: NULL
	 * exit: NULL
	 */
	s32 (*init)(struct disp_layer *layer);
	s32 (*exit)(struct disp_layer *layer);

	s32 (*get_frame_id)(struct disp_layer *layer);

	s32 (*dump)(struct disp_layer *layer, char *buf);
};

struct disp_smbl {
	/* static fields */
	char *name;
	u32 disp;
	u32 backlight;
	struct disp_manager *manager;

	/*
	 * The following functions do not block:
	 *
	 * is_enabled
	 * set_layer_info
	 * get_layer_info
	 *
	 * The rest of the functions may block and cannot be called from
	 * interrupt context
	 */

	s32 (*enable)(struct disp_smbl *smbl);
	s32 (*disable)(struct disp_smbl *smbl);
	bool (*is_enabled)(struct disp_smbl *smbl);
	s32 (*set_manager)(struct disp_smbl *smbl, struct disp_manager *mgr);
	s32 (*unset_manager)(struct disp_smbl *smbl);
	s32 (*update_backlight)(struct disp_smbl *smbl, unsigned int bl);

	/* init: NULL
	 * exit: NULL
	 */
	s32 (*init)(struct disp_smbl *smbl);
	s32 (*exit)(struct disp_smbl *smbl);

	s32 (*apply)(struct disp_smbl *smbl);
	s32 (*update_regs)(struct disp_smbl *smbl);
	s32 (*force_apply)(struct disp_smbl *smbl);
	s32 (*sync)(struct disp_smbl *smbl);
	s32 (*tasklet)(struct disp_smbl *smbl);

	s32 (*set_window)(struct disp_smbl *smbl, struct disp_rect *window);
	s32 (*get_window)(struct disp_smbl *smbl, struct disp_rect *window);
	s32 (*dump)(struct disp_smbl *smbl, char *buf);
};

struct disp_enhance {
	/* static fields */
	char *name;
	u32 disp;
	struct disp_manager *manager;

	/*
	 * The following functions do not block:
	 *
	 * is_enabled
	 * set_layer_info
	 * get_layer_info
	 *
	 * The rest of the functions may block and cannot be called from
	 * interrupt context
	 */

	s32 (*enable)(struct disp_enhance *enhance);
	s32 (*disable)(struct disp_enhance *enhance);
	bool (*is_enabled)(struct disp_enhance *enhance);
	s32 (*set_manager)(struct disp_enhance *enhance, struct disp_manager *mgr);
	s32 (*unset_manager)(struct disp_enhance *enhance);

	/* init: NULL
	 * exit: NULL
	 */
	s32 (*init)(struct disp_enhance *enhance);
	s32 (*exit)(struct disp_enhance *enhance);

	s32 (*apply)(struct disp_enhance *enhance);
	s32 (*update_regs)(struct disp_enhance *enhance);
	s32 (*force_apply)(struct disp_enhance *enhance);
	s32 (*sync)(struct disp_enhance *enhance);
	s32 (*tasklet)(struct disp_enhance *enhance);

	/* power manager */
	s32 (*early_suspend)(struct disp_enhance *enhance);
	s32 (*late_resume)(struct disp_enhance *enhance);
	s32 (*suspend)(struct disp_enhance *enhance);
	s32 (*resume)(struct disp_enhance *enhance);

	s32 (*set_bright)(struct disp_enhance *enhance, u32 val);
	s32 (*set_saturation)(struct disp_enhance *enhance, u32 val);
	s32 (*set_contrast)(struct disp_enhance *enhance, u32 val);
	s32 (*set_hue)(struct disp_enhance *enhance, u32 val);
	s32 (*set_edge)(struct disp_enhance *enhance, u32 val);
	s32 (*set_detail)(struct disp_enhance *enhance, u32 val);
	s32 (*set_denoise)(struct disp_enhance *enhance, u32 val);
	s32 (*set_mode)(struct disp_enhance *enhance, u32 val);
	s32 (*set_window)(struct disp_enhance *enhance, struct disp_rect *window);
	s32 (*get_bright)(struct disp_enhance *enhance);
	s32 (*get_saturation)(struct disp_enhance *enhance);
	s32 (*get_contrast)(struct disp_enhance *enhance);
	s32 (*get_hue)(struct disp_enhance *enhance);
	s32 (*get_edge)(struct disp_enhance *enhance);
	s32 (*get_detail)(struct disp_enhance *enhance);
	s32 (*get_denoise)(struct disp_enhance *enhance);
	s32 (*get_mode)(struct disp_enhance *enhance);
	s32 (*get_window)(struct disp_enhance *enhance, struct disp_rect *window);
	s32 (*set_para)(struct disp_enhance *enhance, disp_enhance_para *para);
	s32 (*demo_enable)(struct disp_enhance *enhance);
	s32 (*demo_disable)(struct disp_enhance *enhance);
	s32 (*dump)(struct disp_enhance *enhance, char *buf);
};

struct disp_capture {
	char *name;
	u32 disp;
	struct disp_manager *manager;

	s32 (*set_manager)(struct disp_capture *cptr, struct disp_manager *mgr);
	s32 (*unset_manager)(struct disp_capture *cptr);
	s32 (*start)(struct disp_capture *cptr);
	s32 (*commmit)(struct disp_capture *cptr, struct disp_capture_info *info);
	s32 (*stop)(struct disp_capture *cptr);
	s32 (*sync)(struct disp_capture *cptr);
	s32 (*tasklet)(struct disp_capture *cptr);
	s32 (*init)(struct disp_capture *cptr);
	s32 (*exit)(struct disp_capture *cptr);
	s32 (*query)(struct disp_capture *cptr);//0: finish, other: fail/err

	/* inner interface */
	s32 (*apply)(struct disp_capture *cptr);
};

//extern s32 disp_delay_ms(u32 ms);
//extern s32 disp_delay_us(u32 us);
struct ee_img {
	unsigned long addr;
	unsigned int pitch;
	unsigned int w;     /*image width*/
	unsigned int h;     /*image height*/
};

struct rect_size {
	u32 width;
	u32 height;
	u32 align;
};

#if defined(CONFIG_EINK_PANEL_USED)
struct area_info {
	unsigned int x_top;
	unsigned int y_top;
	unsigned int x_bottom;
	unsigned int y_bottom;
};

struct eink_timing_param {
	unsigned int lbl;
	unsigned int lel;
	unsigned int lsl;
	unsigned int fbl;
	unsigned int fel;
	unsigned int fsl;
	unsigned int width;  /*image width*/
	unsigned int height;  /*image height*/
};

enum eink_flash_mode {
	LOCAL,
	GLOBAL,
	INIT
};

enum buf_use_state {
	FREE,
	USED
};

enum eink_update_mode {
	/*GLOBAL*/
	EINK_INIT_MODE = 0x01,
	EINK_DU_MODE = 0x02,
	EINK_GC16_MODE = 0x04,
	EINK_A2_MODE = 0x10,
	EINK_GC16_LOCAL_MODE = 0x84,

	/*LOCAL*/
	EINK_DU_RECT_MODE = 0x402,
	EINK_GC16_RECT_MODE = 0x404,
	EINK_A2_RECT_MODE = 0x410,
	EINK_GC16_LOCAL_RECT_MODE = 0x484,
};

struct eink_8bpp_image {
	enum eink_update_mode	update_mode;
	enum eink_flash_mode	flash_mode;
	enum buf_use_state		state;
	void					*vaddr;
	void					*paddr;
	bool					window_calc_enable;
	struct rect_size			size;
	struct area_info			update_area;
};

struct eink_init_param {
	bool					used;
	u8					eink_moudule_type;
	u8					eink_version_type;
	u8					eink_ctrl_data_type;
	u8					eink_bits;   /*0->3bits,1->4bits,2->5bits*/
	u8					eink_mode;   /*0->8data,1->16data*/
	struct eink_timing_param		timing;
	char				wavefile_path[32];
};

enum  eink_bit_num {
	EINK_BIT_1 = 0x01,
	EINK_BIT_2 = 0x02,
	EINK_BIT_3 = 0x03,
	EINK_BIT_4 = 0x04,
	EINK_BIT_5 = 0x05
};

/*#define EINK_FLUSH_TIME_TEST*/

struct disp_eink_manager {
	unsigned int disp;
	unsigned int test;
	int	tcon_flag;
	int	eink_panel_temperature;

	volatile unsigned int flush_continue_flag;
	struct tasklet_struct sync_tasklet;
	struct tasklet_struct decode_tasklet;
	/*wait_queue_head_t decode_taske_queue;*/
	struct work_struct decode_work;
	struct eink_private *private_data;
	struct disp_manager *mgr;
	/*struct eink_buffer_manager* buffer_mgr;*/
	struct pipeline_manager *pipeline_mgr;
	struct format_manager *convert_mgr;
	struct task_struct *detect_fresh_task;
	struct task_struct *debug_task;
	struct mutex standby_lock;

	int (*eink_update)(struct disp_eink_manager *manager,
			   struct eink_8bpp_image *cimage);
	int (*enable)(struct disp_eink_manager *);
	int (*disable)(struct disp_eink_manager *);
	int (*op_skip)(struct disp_eink_manager *manager, u32 skip);
	int (*suspend)(struct disp_eink_manager *);
	int (*resume)(struct disp_eink_manager *);
	void (*clearwd)(struct disp_eink_manager *, int);
	/*for debug*/
	int (*decode)(struct disp_eink_manager *, int);
	int (*set_temperature)(struct disp_eink_manager *manager, unsigned int temp);
	unsigned int (*get_temperature)(struct disp_eink_manager *manager);
};

struct image_format {
	enum disp_pixel_format format;
	unsigned int width;
	unsigned int height;
	unsigned long addr1;
	unsigned long addr2;
	unsigned long addr3;
};

struct format_manager {
	unsigned int disp;
	unsigned int irq_num;
	volatile unsigned int write_back_finish;
	/*	wait_queue_head_t write_back_queue;*/
	struct clk *clk;
	int (*enable)(unsigned int id);
	int (*disable)(unsigned int id);
	int (*start_convert)(unsigned int id, struct disp_layer_config *config,
			     unsigned int layer_num, struct image_format *dest);
};
#endif
#endif
