/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2019 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 *	File name   :       eink_reg.h
 *
 *	Description :       eink engine 2.0 struct declaration
 *
 *	History     :       2019/03/20 liuli   initial version
 *
 */
#ifndef __EINK_REG_H__
#define __EINK_REG_H__

#include "../../../sunxi_eink/sunxi_eink.h"

union __eink_top_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int panel_bit_mode	:2;
		unsigned int r0			:2;
		unsigned int out_mode		:1;
		unsigned int r1			:3;
		unsigned int panel_scan_mode	:2;
		unsigned int panel_split_dir	:1;
		unsigned int r2			:1;
		unsigned int panel_scan_dir	:2;
		unsigned int r3			:2;
		unsigned int burst_len		:2;
		unsigned int r4			:2;
		unsigned int timing_gen_en	:1;
		unsigned int r5			:3;
		unsigned int offline_en		:1;
		unsigned int panel_out_clk_flip	:1;
		unsigned int data_reverse	:1;
		unsigned int work_mode		:1;
		unsigned int follow_en		:1;
		unsigned int sram_gating_mode	:1;
		unsigned int bist_en		:1;
		unsigned int reset		:1;
	} bits;
};

union __eink_start_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int ee_start		:1;
		unsigned int r0			:31;
	} bits;
};

union __eink_upd_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_pic_en		:1;
		unsigned int r0			:3;
		unsigned int upd_all_en		:1;
		unsigned int r1			:3;
		unsigned int upd_bit_fmt	:2;
		unsigned int r2			:22;
	} bits;
};

union __eink_bw_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int bwc_en		:1;
		unsigned int r0			:3;
		unsigned int bwc_pos_num	:7;
		unsigned int r1			:21;
	} bits;
};

union __eink_irq_en_reg_t {
	unsigned int dwval;
	struct {
		unsigned int eink_frame_fin_en		:1;
		unsigned int r0				:3;
		unsigned int upd_pic_accept_irq_en	:1;
		unsigned int r1				:3;
		unsigned int upd_pic_fin_irq_en		:1;
		unsigned int r2				:3;
		unsigned int pipe_fin_irq_en		:1;
		unsigned int r3				:3;
		unsigned int edma_frame_fin_en		:1;
		unsigned int r4				:7;
		unsigned int ee_fin_irq_en		:1;
		unsigned int r5				:3;
		unsigned int edma_fin_irq_en		:1;
		unsigned int r6				:3;
	} bits;
};

union __eink_irq_status_reg_t {
	unsigned int dwval;
	struct {
		unsigned int eink_frame_fin	:1;
		unsigned int r0			:3;
		unsigned int upd_pic_accept	:1;
		unsigned int r1			:3;
		unsigned int upd_pic_fin	:1;
		unsigned int r2			:3;
		unsigned int pipe_fin		:1;
		unsigned int r3			:3;
		unsigned int edma_frame_fin	:1;
		unsigned int r4			:3;
		unsigned int upd_coll_occur	:1;
		unsigned int r5			:3;
		unsigned int ee_fin		:1;
		unsigned int r6			:3;
		unsigned int edma_fin		:1;
		unsigned int r7			:3;
	} bits;
};

union __eink_error_status_reg_t {
	unsigned int dwval;
	struct {
		unsigned int eink_wd_underflow	:1;
		unsigned int r0			:3;
		unsigned int edma_wb_overflow	:1;
		unsigned int r1			:27;
	} bits;
};

union __eink_version_reg_t {
	unsigned int dwval;
	struct {
		unsigned int r1p0	:16;/* default 0x10 */
		unsigned int p2		:16;/* default 0x200 */
	} bits;
};

union __eink_upd_img_size_reg_t {
	unsigned int dwval;
	struct {
		unsigned int width		:12;
		unsigned int r0			:4;
		unsigned int height		:12;
		unsigned int r1			:4;
	} bits;
};

union __eink_upd_img_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int addr		:32;
	} bits;
};

union __eink_upd_img_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_img_haddr	:8;
		unsigned int r0			:24;
	} bits;
};

union __eink_upd_img_pitch_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_img_pitch	:16;
		unsigned int r0			:16;
	} bits;
};

union __eink_rmi_in_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int addr		:32;
	} bits;
};

union __eink_rmi_in_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int addr		:8;
		unsigned int r0			:24;
	} bits;
};

union __eink_upd_win0_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_win_left	:12;
		unsigned int r0			:4;
		unsigned int upd_win_top	:12;
		unsigned int r1			:4;
	} bits;
};

union __eink_upd_win1_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_win_right	:12;
		unsigned int r0			:4;
		unsigned int upd_win_bottom	:12;
		unsigned int r1			:4;
	} bits;
};

union __eink_upd_pipe_id_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_pipe_id	:6;
		unsigned int r0			:26;
	} bits;
};

union __eink_upd_coll_win0_reg_t {
	unsigned int dwval;
	struct {
		unsigned int coll_win_left	:12;
		unsigned int r0			:4;
		unsigned int coll_win_top	:12;
		unsigned int r1			:4;
	} bits;
};

union __eink_upd_coll_win1_reg_t {
	unsigned int dwval;
	struct {
		unsigned int coll_win_right	:12;
		unsigned int r0			:4;
		unsigned int coll_win_bottom	:12;
		unsigned int r1			:4;
	} bits;
};

union __eink_upd_coll_status0_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_coll_status0	:32;
	} bits;
};

union __eink_upd_coll_status1_reg_t {
	unsigned int dwval;
	struct {
		unsigned int upd_coll_status1	:32;
	} bits;
};

union __eink_fin_pipe_id0_reg_t {
	unsigned int dwval;
	struct {
		unsigned int	fin_pipe_id0	:32;
	} bits;
};

union __eink_fin_pipe_id1_reg_t {
	unsigned int dwval;
	struct {
		unsigned int	fin_pipe_id1	:32;
	} bits;
};

union __eink_rmi_out_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int rmi_out_laddr	:32;/* Default: 0x0; */
	} bits;
};

union __eink_rmi_out_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int rmi_out_haddr	:8;/* Default: 0x0; */
		unsigned int res0		:24;/* Default: ; */
	} bits;
};

union __eink_timing_param0_reg_t {
	unsigned int dwval;
	struct {
		unsigned int line_lsl		:  8;	   /* Default: 0x0; */
		unsigned int line_lbl		:  8;	   /* Default: 0x0; */
		unsigned int line_lel		:  9;	   /* Default: 0x0; */
		unsigned int res0		:  7;     /* Default: ; */
	} bits;
};

union __eink_timing_param1_reg_t {
	unsigned int dwval;
	struct {
		unsigned int frm_fsl		:  8;	   /* Default: 0x0; */
		unsigned int frm_fbl		:  8;	   /* Default: 0x0; */
		unsigned int frm_fel		:  9;	   /* Default: 0x0; */
		unsigned int res0		:  7;     /* Default: ; */
	} bits;
};

union __eink_timing_param2_reg_t {
	unsigned int dwval;
	struct {
		unsigned int line_ldl		:  12;	   /* Default: 0x0; */
		unsigned int r0			:  4;
		unsigned int frm_fdl		:  12;	   /* Default: 0x0; */
		unsigned int r1			:  4;
	} bits;
};

union __eink_timing_param3_reg_t {
	unsigned int dwval;
	struct {
		unsigned int gdck_sta		:  12;	   /* Default: 0x0; */
		unsigned int r0			:  4;
		unsigned int lgonl		:  12;	   /* Default: 0x0; */
		unsigned int r1			:  4;
	} bits;
};

union __eink_timing_param4_reg_t {
	unsigned int dwval;
	struct {
		unsigned int gdoe_start_line	:  8;	   /* Default: 0x0; */
		unsigned int r0			:  24;
	} bits;
};

union __eink_pipe_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int pipe_en		:1;/* default: 0x0; */
		unsigned int r0			:7;
		unsigned int pipe_frm_num	:8;/* default: 0x0; */
		unsigned int pipe_frm_cnt	:8;/* default: 0x0; */
		unsigned int pipe_wf_haddr	:8;/* default: 0x0; */
	} bits;
};

union __eink_pipe_wf_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int wf_laddr		:32;/* default: 0x0; */
	} bits;
};

union __eink_timing_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int timing_laddr		:32;/* default: 0x0; */
	} bits;
};

union __eink_timing_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int timing_haddr		:8;/* default: 0x0; */
		unsigned int r0				:24;/* default: 0x0; */
	} bits;
};

union __eink_timing_pitch_reg_t {
	unsigned int dwval;
	struct {
		unsigned int timing_pitch	:16;/* default: 0x0; */
		unsigned int res0		:16;
	} bits;
};

union __eink_timing_size_reg_t {
	unsigned int dwval;
	struct {
		unsigned int width		:12;/* default: 0x0; */
		unsigned int res0		:4;
		unsigned int height		:12;
		unsigned int res1		:4;
	} bits;
};

union __eink_wd_start_coor_reg_t {
	unsigned int dwval;
	struct {
		unsigned int start_coor_x	:12;/* default: 0x0; */
		unsigned int r0			:4;
		unsigned int start_coor_y	:12;/* default: 0x0; */
		unsigned int r1			:4;
	} bits;
};

union __eink_dec_wd_pitch_reg_t {
	unsigned int dwval;
	struct {
		unsigned int pitch		:  16;	   /* Default: 0x0; */
		unsigned int r0			:  16;
	} bits;
};

union __eink_dec_wd_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int dec_wd_laddr	:  32;	   /* Default: 0x0; */
	} bits;
};

union __eink_dec_wd_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int dec_wd_haddr	:  8;	   /* Default: 0x0; */
		unsigned int r0			:  24;	   /* Default: 0x0; */
	} bits;
};

union __eink_edma_wb_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int wb_en		:1;	   /* Default: 0x0; */
		unsigned int r0			:3;	   /* Default: 0x0; */
		unsigned int self_wb_mode	:1;	   /* Default: 0x0; */
		unsigned int r1			:3;	   /* Default: 0x0; */
		unsigned int wb_frm_cnt		:8;
		unsigned int r2			:16;
	} bits;
};

union __eink_edma_start_ctrl_reg_t {
	unsigned int dwval;
	struct {
		unsigned int en			:1;	   /* Default: 0x0; */
		unsigned int r0			:31;	   /* Default: 0x0; */
	} bits;
};

union __eink_edma_wd_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int edma_wd_laddr	:  32;	   /* Default: 0x0; */
	} bits;
};

union __eink_edma_wd_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int edma_wd_haddr	:8;	   /* Default: 0x0; */
		unsigned int r0			:24;
	} bits;
};

union __eink_edma_wd_pitch_reg_t {
	unsigned int dwval;
	struct {
		unsigned int pitch		:16;	   /* Default: 0x0; */
		unsigned int r0			:16;
	} bits;
};

union __eink_edma_wd_size_reg_t {
	unsigned int dwval;
	struct {
		unsigned int edma_wd_width	:12;	   /* Default: 0x0; */
		unsigned int r0			:4;
		unsigned int edma_wd_height	:12;	   /* Default: 0x0; */
		unsigned int r1			:4;
	} bits;
};

union __eink_edma_wb_laddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int edma_wb_laddr	:32;	   /* Default: 0x0; */
	} bits;
};

union __eink_edma_wb_haddr_reg_t {
	unsigned int dwval;
	struct {
		unsigned int edma_wd_haddr	:8;	   /* Default: 0x0; */
		unsigned int r0			:24;
	} bits;
};

struct __eink_reg_t {
	union __eink_top_ctrl_reg_t		top_ctrl;	/* 0x0000 */
	union __eink_start_ctrl_reg_t		start_ctrl;	/* 0x0004 */
	union __eink_upd_ctrl_reg_t		upd_ctrl;	/* 0x0008 */
	union __eink_irq_en_reg_t		irq_en;		/* 0x000c */
	union __eink_irq_status_reg_t		irq_status;	/* 0x0010 */
	union __eink_error_status_reg_t		error_status;	/* 0x0014 */
	union __eink_bw_ctrl_reg_t		bw_ctrl;	/* 0x0018 */
	union __eink_version_reg_t		version;	/* 0x001c */
	union __eink_upd_img_size_reg_t		upd_img_size;	/* 0x0020 */
	union __eink_upd_img_laddr_reg_t	upd_img_laddr;	/* 0x0024 */
	union __eink_upd_img_haddr_reg_t	upd_img_haddr;	/* 0x0028 */
	union __eink_upd_img_pitch_reg_t	upd_img_pitch;	/* 0x002c */
	union __eink_rmi_in_laddr_reg_t		rmi_in_laddr;	/* 0x0030 */
	union __eink_rmi_in_haddr_reg_t		rmi_in_haddr;	/* 0x0034 */
	union __eink_upd_win0_reg_t		upd_win0;	/* 0x0038 */
	union __eink_upd_win1_reg_t		upd_win1;	/* 0x003c */
	union __eink_upd_pipe_id_reg_t		upd_pipe_id;	/* 0x0040 */
	unsigned int res1;					/* 0x0044 */
	union __eink_upd_coll_win0_reg_t	coll_win0;	/* 0x0048 */
	union __eink_upd_coll_win1_reg_t	coll_win1;	/* 0x004c */
	union __eink_upd_coll_status0_reg_t	coll_status0;	/* 0x0050 */
	union __eink_upd_coll_status1_reg_t	coll_status1;	/* 0x0054 */
	union __eink_fin_pipe_id0_reg_t		fin_pipe_id0;	/* 0x0058 */
	union __eink_fin_pipe_id1_reg_t		fin_pipe_id1;	/* 0x005c */
	union __eink_rmi_out_laddr_reg_t	rmi_out_laddr;	/* 0x0060 */
	union __eink_rmi_out_haddr_reg_t	rmi_out_haddr;	/* 0x0064 */
	union __eink_timing_param0_reg_t	timing_param0;	/* 0x0068 */
	union __eink_timing_param1_reg_t	timing_param1;	/* 0x006c */
	union __eink_timing_param2_reg_t	timing_param2;	/* 0x0070 */
	union __eink_timing_param3_reg_t	timing_param3;	/* 0x0074 */
	union __eink_timing_param4_reg_t	timing_param4;	/* 0x0078 */
	unsigned int res2;					/* 0x007c */
	union __eink_pipe_ctrl_reg_t		pipe_ctrl[64];	/* 0x0080~0x017c */
	union __eink_pipe_wf_laddr_reg_t	pipe_wf_laddr[64];/*0x0180~0x027c*/
	union __eink_timing_laddr_reg_t		timing_laddr;	/* 0x0280 */
	union __eink_timing_haddr_reg_t		timing_haddr;	/* 0x0284 */
	union __eink_timing_pitch_reg_t		timing_pitch;	/* 0x0288 */
	union __eink_timing_size_reg_t		timing_size;	/* 0x028c */
	union __eink_wd_start_coor_reg_t	wd_start_coor;	/* 0x0290 */
	union __eink_dec_wd_pitch_reg_t		dec_wd_pitch;	/* 0x0294 */
	union __eink_dec_wd_laddr_reg_t		dec_wd_laddr;	/* 0x0298 */
	union __eink_dec_wd_haddr_reg_t		dec_wd_haddr;	/* 0x029c */
	union __eink_edma_wb_ctrl_reg_t		edma_wb_ctrl;	/* 0x02a0 */
	union __eink_edma_start_ctrl_reg_t	edma_start;	/* 0x02a4 */
	union __eink_edma_wd_laddr_reg_t	edma_wd_laddr;	/* 0x02a8 */
	union __eink_edma_wd_haddr_reg_t	edma_wd_haddr;	/* 0x02ac */
	union __eink_edma_wd_pitch_reg_t	edma_wd_pitch;	/* 0x02b0 */
	union __eink_edma_wb_laddr_reg_t	edma_wb_laddr;	/* 0x02b4 */
	union __eink_edma_wb_haddr_reg_t	edma_wb_haddr;	/* 0x02b8 */
	unsigned int res4;					/* 0x02bc */
};

struct timing_buf_cfg {
	unsigned long	    addr;
	u32		    pitch;
	u32		    width;
	u32		    height;
};

struct upd_pic_cfg {
	unsigned long	    addr;
	u32		    upd_all_en;
	u32		    pitch;
	struct upd_pic_size size;
	struct upd_win	    upd_win;
	enum upd_pixel_fmt  out_fmt;
};


extern int eink_set_reg_base(void __iomem *reg_base);
extern unsigned long eink_get_reg_base(void);
extern int eink_start(void);
extern int eink_set_rmi_inaddr(unsigned long inaddr);
extern int eink_set_rmi_outaddr(unsigned long outaddr);
extern int eink_set_out_panel_mode(struct eink_panel_info  *panel_info);
extern int eink_set_data_reverse(void);
extern int eink_set_panel_scan_dir(unsigned int scan_dir);
extern unsigned int eink_get_dec_cnt(u32 pipe_no);
extern int eink_prepare_decode(unsigned long wav_paddr, struct eink_panel_info *info);
extern int eink_irq_enable(void);
extern int eink_irq_query(void);
extern int eink_pipe_enable(unsigned int pipe_no);
extern int eink_pipe_disable(unsigned int pipe_no);

extern int eink_edma_cfg(struct eink_panel_info *info);
extern int eink_edma_set_wavaddr(unsigned long wav_addr);
extern int eink_edma_start(void);

extern int eink_config_timing_buf(struct timing_buf_cfg *cfg);

extern int eink_set_rmi_inaddr(unsigned long inaddr);
extern int eink_set_rmi_outaddr(unsigned long outaddr);

extern int eink_pipe_config(struct pipe_info_node *info);
extern int eink_pipe_config_wavefile(unsigned long wav_file_addr, unsigned int pipe_no);
extern u64 eink_get_fin_pipe_id(void);
extern void eink_reset_fin_pipe_id(u64 reg_val);
unsigned int eink_get_error_status(void);

extern int eink_set_upd_all_en(unsigned int en);

extern int eink_get_coll_status0(void);
extern int eink_get_coll_status1(void);
extern int eink_get_coll_win(struct upd_win *win);

extern int eink_config_timing_param(struct timing_info *info);
extern int eink_timing_gen_enable(void);
extern int eink_timing_config_coor(u32 x, u32 y);
#endif
