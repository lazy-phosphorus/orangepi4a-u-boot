/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "eink_reg.h"

static volatile struct __eink_reg_t *ee_base;

u8 __bits_num_config_to_reg(u32 bit_num)
{
	u8 eink_bits = 0;

	switch (bit_num) {
	case EINK_BIT_5:
		eink_bits = 2;
		break;
	case EINK_BIT_4:
		eink_bits = 1;
		break;
	default:
		eink_bits = 0;
		break;
	}
	return eink_bits;
}

u8 __data_len_config_to_reg(u32 data_len)
{
	u8 eink_mode = 0;

	switch (data_len) {
	case 8:
		eink_mode = 0;
		break;
	case 16:
		eink_mode = 1;
		break;
	default:
		eink_mode = 0;
		break;
	}
	return eink_mode;
}

int eink_set_reg_base(void __iomem *reg_base)
{
	ee_base = (struct __eink_reg_t *)reg_base;

	return 0;
}

unsigned long eink_get_reg_base(void)
{

	DRM_INFO("ee base=0x%p\n", ee_base);

	return (unsigned long)ee_base;
}

/* reset and stop reset */
int eink_dev_reset(void)
{
	const unsigned int time = 10;/* wait 10 us*/

	ee_base->top_ctrl.bits.reset = 1;
	udelay(time);
	ee_base->top_ctrl.bits.reset = 0;
	return 0;
}

int eink_set_out_panel_mode(struct eink_panel_info  *panel_info)
{
	unsigned int tmp = 0;
	u8 bit_num = 0, data_len = 0;
	u8 panel_bit = 0, out_mode = 0;

	data_len = panel_info->data_len;
	bit_num = panel_info->bit_num;

	panel_bit = __bits_num_config_to_reg(bit_num);
	out_mode = __data_len_config_to_reg(data_len);

	DRM_INFO("in data len=%d, bit_num=%d, out_mode=%d, panel_bit=%d\n",
			data_len, bit_num, out_mode, panel_bit);

	tmp = ((out_mode & 0x1) << 4) | (panel_bit & 0x3);
	ee_base->top_ctrl.dwval |= tmp;

	return 0;
}

int eink_set_data_reverse(void)
{
	DRM_INFO("DATA reverse!\n");
	ee_base->top_ctrl.bits.data_reverse = 1;
	return 0;
}

/* 0 single 1 dual 2 quad */
int eink_set_panel_scan_mode(unsigned int scan_mode)
{

	ee_base->top_ctrl.bits.panel_scan_mode = scan_mode;

	return 0;
}

/* 0 left-top, 1 left-bottom, 2 right-top, 3 right-bottom
 * when scan_mode is 0 this can be only set to 0
 */
int eink_set_panel_scan_dir(unsigned int scan_dir)
{

	ee_base->top_ctrl.bits.panel_scan_dir = scan_dir;

	return 0;
}

int eink_set_panel_split_dir(unsigned int split_dir)
{

	ee_base->top_ctrl.bits.panel_split_dir = split_dir;

	return 0;
}

/* Y3/4/5/8 */
int eink_set_upd_mode(unsigned char upd_pic_mode)
{
	unsigned int tmp;

	tmp = (upd_pic_mode & 0x3) << 2;
	ee_base->top_ctrl.dwval |= tmp;

	return 0;
}

int eink_timing_gen_enable(void)
{

	ee_base->top_ctrl.bits.timing_gen_en = 1;

	return 0;
}

int eink_offline_enable(unsigned int en)
{
#ifdef OFFLINE_SINGLE_MODE
	ee_base->top_ctrl.bits.work_mode = 1;
#endif
	ee_base->top_ctrl.bits.offline_en = en;

	return 0;
}

/*div of eink clk max 32(5bit)*/
int eink_set_bwc_ctrl(unsigned int en, unsigned int pos_num)
{

	ee_base->bw_ctrl.bits.bwc_en = en;
	ee_base->bw_ctrl.bits.bwc_pos_num = pos_num;

	return 0;
}

int eink_prepare_decode(unsigned long wav_paddr, struct eink_panel_info *info)
{
	unsigned int tmp = 0;

	tmp = (info->data_len == 16) ? ((info->width >> 3)) : ((info->width >> 2));

	ee_base->dec_wd_pitch.bits.pitch = tmp;

	ee_base->dec_wd_laddr.dwval = wav_paddr;

	return 0;
}

int eink_start(void)
{
	ee_base->start_ctrl.bits.ee_start = 1;
	return 0;
}

int eink_edma_start(void)
{
	ee_base->edma_start.bits.en = 1;
	return 0;
}

int eink_set_upd_bit_fmt(u32 fmt)
{
	u32 upd_fmt = 0;

	switch (fmt) {
	case EINK_Y8:
		upd_fmt = 0x3;
		break;
	case EINK_Y5:
		upd_fmt = 0x2;
		break;
	case EINK_Y4:
		upd_fmt = 0x1;
		break;
	case EINK_Y3:
		upd_fmt = 0x0;
		break;
	default:
		pr_err("format not support use Y8");
		upd_fmt = 0x3;
	};
	ee_base->upd_ctrl.bits.upd_bit_fmt = upd_fmt;
	return 0;
}

int eink_upd_pic_enable(void)
{
	ee_base->upd_ctrl.bits.upd_pic_en = 1;
	return 0;
}

int eink_set_upd_all_en(unsigned int en)
{
	ee_base->upd_ctrl.bits.upd_all_en = en;
	return 0;
}

int eink_edma_fin_irq_enable(unsigned int en)
{
	ee_base->irq_en.bits.edma_fin_irq_en = en;
	return 0;
}

int eink_pipe_fin_irq_enable(unsigned int en)
{
	ee_base->irq_en.bits.pipe_fin_irq_en = en;
	return 0;
}

int eink_upd_pic_fin_irq_enable(unsigned int en)
{
	ee_base->irq_en.bits.upd_pic_fin_irq_en = en;
	return 0;
}

int eink_upd_pic_accept_irq_enable(unsigned int en)
{
	ee_base->irq_en.bits.upd_pic_accept_irq_en = en;
	return 0;
}

int eink_fin_irq_enable(unsigned int en)
{
	ee_base->irq_en.bits.ee_fin_irq_en = en;
	return 0;
}

int eink_irq_enable(void)
{
	DRM_INFO("pipe all irq enable.\n");
	eink_upd_pic_accept_irq_enable(1);
	eink_upd_pic_fin_irq_enable(1);
	eink_pipe_fin_irq_enable(1);
#ifdef OFFLINE_SINGLE_MODE
	eink_edma_fin_irq_enable(1);
#endif
	eink_fin_irq_enable(1);
	return 0;
}

#if 0
int eink_irq_query(void)
{
	unsigned int ee_fin, upd_pic_accept, upd_pic_fin, pipe_fin, edma_fin,
		     upd_coll_occur, edma_wb_overflow, eink_wd_underflow;
	unsigned int reg_val = 0, reg_val1 = 0;

	reg_val = ee_base->irq_status.dwval;
	DRM_INFO("reg_val = 0x%x\n", reg_val);

	upd_pic_accept = reg_val & 0x10;
	upd_pic_fin = reg_val & 0x100;
	pipe_fin = reg_val & 0x1000;
	upd_coll_occur = reg_val & 0x100000;
	ee_fin = reg_val & 0x1000000;
	edma_fin = reg_val & 0x10000000;

	reg_val1 = ee_base->error_status.dwval;
	eink_wd_underflow = reg_val1 & 0x1;
	edma_wb_overflow = reg_val1 & 0x10;

	if (upd_pic_accept == 0x10) {
		ee_base->irq_status.bits.upd_pic_accept = 1;
		return 1;
	}

	if (upd_pic_fin == 0x100) {
		ee_base->irq_status.bits.upd_pic_fin = 1;
		if (upd_coll_occur == 0x100000) {
			ee_base->irq_status.bits.upd_coll_occur = 1;
			return 5;
		}
		return 2;
	}

	if (pipe_fin == 0x1000) {
		ee_base->irq_status.bits.pipe_fin = 1;
		return 3;
	}
/* */
	if (ee_fin == 0x1000000) {
		ee_base->irq_status.bits.ee_fin = 1;
		return 0;
	}

	if (edma_fin == 0x10000000) {
		ee_base->irq_status.bits.edma_fin = 1;
		return 4;
	}

	if (eink_wd_underflow == 0x1) {
		ee_base->error_status.bits.eink_wd_underflow = 1;
		return 6;
	}

	if (edma_wb_overflow == 0x10) {
		ee_base->error_status.bits.edma_wb_overflow = 1;
		return 7;
	}

	return -1;
}
#endif
unsigned int eink_get_error_status(void)
{
	unsigned int reg_val = 0, edma_wb_overflow, eink_wd_underflow;
	reg_val = ee_base->error_status.dwval;
	eink_wd_underflow = reg_val & 0x1;
	edma_wb_overflow = reg_val & 0x10;

	if (eink_wd_underflow == 0x1) {
		ee_base->error_status.bits.eink_wd_underflow = 1;
	}

	if (edma_wb_overflow == 0x10) {
		ee_base->error_status.bits.edma_wb_overflow = 1;
	}

	return reg_val;
}

int eink_irq_query(void)
{
	unsigned int ee_fin, upd_pic_accept, upd_pic_fin, pipe_fin, edma_fin,
		     upd_coll_occur;
	unsigned int reg_val = 0;

	reg_val = ee_base->irq_status.dwval;

	upd_pic_accept = reg_val & 0x10;
	upd_pic_fin = reg_val & 0x100;
	pipe_fin = reg_val & 0x1000;
	upd_coll_occur = reg_val & 0x100000;
	ee_fin = reg_val & 0x1000000;
	edma_fin = reg_val & 0x10000000;


	if (upd_pic_accept == 0x10) {
		ee_base->irq_status.bits.upd_pic_accept = 1;
	}

	if (upd_pic_fin == 0x100) {
		ee_base->irq_status.bits.upd_pic_fin = 1;
		if (upd_coll_occur == 0x100000) {
			ee_base->irq_status.bits.upd_coll_occur = 1;
		}
	}

	if (pipe_fin == 0x1000) {
		ee_base->irq_status.bits.pipe_fin = 1;
	}
/* */
	if (ee_fin == 0x1000000) {
		ee_base->irq_status.bits.ee_fin = 1;
	}

	if (edma_fin == 0x10000000) {
		ee_base->irq_status.bits.edma_fin = 1;
	}

	return reg_val;
}

u64 eink_get_fin_pipe_id(void)
{
	u64 val = 0, val1 = 0;

	val = ee_base->fin_pipe_id0.dwval;

	DRM_INFO("Finish pipe val = 0x%llx\n", val);
	val1 = ee_base->fin_pipe_id1.dwval;
	DRM_INFO("Finish ex-pipe val = 0x%llx\n", val1);
	val |= val1 << 32;
	return val;
}

void eink_reset_fin_pipe_id(u64 reg_val)
{

	ee_base->fin_pipe_id0.dwval = reg_val & 0xffffffff;
	ee_base->fin_pipe_id1.dwval = (reg_val & 0xffffffff00000000) >> 32;
	DRM_INFO("reg_val = 0x%llx, id0 = 0x%x, id1 = 0x%x\n",
			reg_val, ee_base->fin_pipe_id0.dwval,
			ee_base->fin_pipe_id1.dwval);
}

int eink_irq_query_index(void)
{
	/*
	unsigned int idx_irq;
	unsigned reg_val = 0;

	reg_val = EINK_RUINT32(ee_base + EE_IRQ);
	idx_irq = reg_val&0x2;

	if (idx_irq == 0x2) {
		EINK_WUINT32(reg_val&0x1e, ee_base + EE_IRQ);
		return 1;
	}
	*/
	return -1;

}

int eink_pipe_enable(unsigned int pipe_no)
{
	ee_base->pipe_ctrl[pipe_no].dwval |= 0x1;

	return 0;
}

int eink_pipe_disable(unsigned int pipe_no)
{
	DRM_INFO("pipe %d disable\n", pipe_no);
	ee_base->pipe_ctrl[pipe_no].bits.pipe_en = 0;

	return 0;
}

unsigned int eink_get_dec_cnt(u32 pipe_no)
{
	u32 dec_cnt = 0;

	dec_cnt = ee_base->pipe_ctrl[pipe_no].bits.pipe_frm_cnt;
	return dec_cnt;
}

int eink_upd_pic_config(struct upd_pic_cfg *cfg)
{
	DRM_INFO("img addr=0x%x, size=(%d x %d), pitch=%d, (%d, %d)~(%d, %d), format=0x%x\n",
			(unsigned int)cfg->addr, cfg->size.width, cfg->size.height,
			cfg->pitch, cfg->upd_win.left, cfg->upd_win.top,
			cfg->upd_win.right, cfg->upd_win.bottom, cfg->out_fmt);

	ee_base->upd_img_laddr.dwval = cfg->addr;
	ee_base->upd_img_size.bits.width = ((cfg->size.width - 1) & 0xfff);
	ee_base->upd_img_size.bits.height = ((cfg->size.height - 1) & 0xfff);
	ee_base->upd_img_pitch.dwval = (cfg->pitch & 0xffff);/* byte align */

	ee_base->upd_win0.bits.upd_win_left = (cfg->upd_win.left & 0xfff);
	ee_base->upd_win0.bits.upd_win_top = (cfg->upd_win.top & 0xfff);
	ee_base->upd_win1.bits.upd_win_right = (cfg->upd_win.right & 0xfff);
	ee_base->upd_win1.bits.upd_win_bottom = (cfg->upd_win.bottom & 0xfff);
	eink_set_upd_bit_fmt(cfg->out_fmt);
	return 0;
}

int eink_pipe_config(struct pipe_info_node *info)
{
	u32 pipe_no = 0;
	struct upd_pic_cfg cfg;

	/* set pipe num */
	pipe_no = info->pipe_id;

	ee_base->upd_pipe_id.dwval = (pipe_no & 0x3f);
	/* ee_base->pipe_ctrl[pipe_no].bits.pipe_frm_num = info->total_frames; */
	/* ee_base->pipe_ctrl[pipe_no].bits.pipe_frm_cnt = info->dec_frame_cnt; */

#ifdef WAVEDATA_DEBUG
	/* a defualt wav file gc16 mode frames = 51 */
	if (info->upd_mode == EINK_GC16_MODE)
		info->total_frames = 51;
#endif
	ee_base->pipe_ctrl[pipe_no].dwval = ((info->total_frames - 1) << 8) | (info->dec_frame_cnt << 16);

	/* set upd pic*/
	memcpy(&cfg.upd_win, &info->upd_win, sizeof(struct upd_win));
	memcpy(&cfg.size, &info->img->size, sizeof(struct upd_pic_size));
	cfg.addr = (unsigned long)info->img->paddr;
	cfg.pitch = info->img->pitch;
	cfg.out_fmt = info->img->out_fmt;

	eink_upd_pic_config(&cfg);
	eink_upd_pic_enable();

	return 0;
}

int eink_pipe_config_wavefile(unsigned long wav_file_addr, unsigned int pipe_no)
{
	ee_base->pipe_wf_laddr[pipe_no].dwval = wav_file_addr;
	return 0;
}

int eink_set_rmi_inaddr(unsigned long inaddr)
{
	ee_base->rmi_in_laddr.dwval = inaddr;
	ee_base->rmi_in_haddr.dwval = 0;
	return 0;
}

int eink_set_rmi_outaddr(unsigned long outaddr)
{
	ee_base->rmi_out_laddr.dwval = outaddr;
	ee_base->rmi_out_haddr.dwval = 0;
	return 0;
}

int eink_get_coll_win(struct upd_win *win)
{
	win->left = ee_base->coll_win0.bits.coll_win_left;
	win->top = ee_base->coll_win0.bits.coll_win_top;
	win->right = ee_base->coll_win1.bits.coll_win_right;
	win->bottom = ee_base->coll_win1.bits.coll_win_bottom;

	return 0;
}

int eink_get_coll_status0(void)
{
	unsigned int status0;

	status0 = ee_base->coll_status0.dwval;

	return status0;
}

int eink_get_coll_status1(void)
{
	unsigned int status1;

	status1 = ee_base->coll_status1.dwval;

	return status1;
}

int eink_set_dec_wd_addr(unsigned long addr)
{
	ee_base->dec_wd_laddr.dwval = addr;

	return 0;
}

int eink_config_timing_param(struct timing_info *info)
{
	u32 hsync = 0, vsync = 0, coor_x = 0, coor_y = 0;

	DRM_INFO("Timing config !\n");

	DRM_INFO("lsl=%d, lbl=%d, lel=%d, ldl=%d\n", info->lsl,
			info->lbl, info->lel, info->ldl);

	ee_base->timing_param0.bits.line_lsl = info->lsl;
	ee_base->timing_param0.bits.line_lbl = info->lbl;
	ee_base->timing_param0.bits.line_lel = info->lel;
	ee_base->timing_param2.bits.line_ldl = info->ldl;

	ee_base->timing_param1.bits.frm_fsl = info->fsl;
	ee_base->timing_param1.bits.frm_fbl = info->fbl;
	ee_base->timing_param1.bits.frm_fel = info->fel;
	ee_base->timing_param2.bits.frm_fdl = info->fdl;

	ee_base->timing_param3.bits.lgonl = info->lgonl;
	ee_base->timing_param3.bits.gdck_sta = info->gdck_sta;

	ee_base->timing_param4.bits.gdoe_start_line = info->gdoe_start_line;

	hsync = info->lsl + info->lbl + info->ldl + info->lel;
	vsync = info->fsl + info->fbl + info->fdl + info->fel;
	coor_x = info->lsl + info->lbl;
	coor_y = info->fsl + info->fbl;

	ee_base->wd_start_coor.bits.start_coor_x = coor_x;
	ee_base->wd_start_coor.bits.start_coor_y = coor_y;
	ee_base->timing_size.bits.width = hsync - 1;
	ee_base->timing_pitch.dwval = hsync;
	ee_base->timing_size.bits.height = vsync - 1;

	DRM_INFO("lsl=%d, lbl=%d, lel=%d, ldl=%d, x=%d, y=%d, w=%d, h=%d\n", ee_base->timing_param0.bits.line_lsl,
			ee_base->timing_param0.bits.line_lbl, ee_base->timing_param0.bits.line_lel,
			ee_base->timing_param2.bits.line_ldl, ee_base->wd_start_coor.bits.start_coor_x,
			ee_base->wd_start_coor.bits.start_coor_y, ee_base->timing_size.bits.width,
			ee_base->timing_size.bits.height);
	return 0;
}

int eink_config_timing_buf(struct timing_buf_cfg *cfg)
{
	DRM_INFO("pitch = 0x%x, w=0x%x, h=0x%x\n",
			cfg->pitch, cfg->width, cfg->height);
	ee_base->timing_laddr.dwval = cfg->addr;
	ee_base->timing_pitch.dwval = cfg->pitch;
	ee_base->timing_size.bits.width = cfg->width;
	ee_base->timing_size.bits.height = cfg->height;
	return 0;
}

int eink_timing_config_coor(u32 x, u32 y)
{
	ee_base->wd_start_coor.bits.start_coor_x = x;
	ee_base->wd_start_coor.bits.start_coor_y = y;
	return 0;
}

int eink_edma_cfg(struct eink_panel_info *info)
{
	unsigned int tmp = 0;

	tmp = (info->data_len == 16) ? ((info->width >> 3)) : ((info->width >> 2));

	ee_base->edma_wd_pitch.bits.pitch = tmp;

	return 0;
}

int eink_edma_set_wavaddr(unsigned long wav_addr)
{
	ee_base->edma_wd_laddr.dwval = wav_addr;
	return 0;
}

/* wb_en: default :0, enablen write back for debug */
int eink_set_wb(unsigned char wb_en, unsigned long wb_addr)
{
	/*
	unsigned int tmp;

	tmp = 0x1 << 31;
	tmp |= (0x1 << 16);
	tmp |= wb_en ? (0x3 << 8) : 0x0;
	tmp |= 0x1;
	EINK_WUINT32(tmp, ee_base + EDMA_GLB_CTL);
	EINK_WUINT32(wb_addr, ee_base + EDMA_WB_ADDR);
	*/

	return 0;
}

int eink_edma_wb_en(int en)
{
	ee_base->edma_wb_ctrl.bits.wb_en = en;
	return 0;
}

int eink_edma_wb_frm_cnt(unsigned int frm_cnt)
{
	ee_base->edma_wb_ctrl.bits.wb_frm_cnt = frm_cnt;
	return 0;
}

int eink_edma_self_wb(void)
{
	ee_base->edma_wb_ctrl.bits.self_wb_mode = 1;
	return 0;
}

int eink_edma_wd_addr(unsigned long paddr, u32 pitch)
{
	ee_base->edma_wd_laddr.dwval = paddr;
	ee_base->edma_wd_pitch.dwval = pitch;
	return 0;
}

int eink_edma_wb_addr(unsigned long addr)
{
	ee_base->edma_wb_laddr.dwval = addr;
	return 0;
}

