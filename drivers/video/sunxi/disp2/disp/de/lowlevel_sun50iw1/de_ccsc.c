/*
 * drivers/video/sunxi/disp2/disp/de/lowlevel_sun50iw1/de_ccsc.c
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
#include "de_rtmx.h"
#include "de_csc_type.h"
#include "de_vep_table.h"
#include "de_csc.h"
#include "de_enhance.h"

#define CCSC00_OFST	0xAA050 	//device0 channel0 (or device1 channel0 support vep)
#define CCSC01_OFST 0xFA050		//device0 channel1 (or device1 channel1 support vep)
#define CCSC10_OFST 0xA0000		//device1 channel0 not support vep
#define CCSC11_OFST 0xF0000		//device1 channel1 not support vep
#define ICSC0_OFST  0xA0040
#define ICSC1_OFST  0xF0040

static volatile __csc_reg_t *ccsc_dev[DEVICE_NUM][CHN_NUM];
static volatile __icsc_reg_t *icsc_dev[DEVICE_NUM][CHN_NUM];
static de_reg_blocks csc_block[DEVICE_NUM][CHN_NUM];
static de_reg_blocks icsc_block[DEVICE_NUM][CHN_NUM];

static unsigned int vi_num[DEVICE_NUM];
static unsigned int vep_support[DEVICE_NUM][CHN_NUM];

static int de_ccsc_set_reg_base(unsigned int sel, unsigned int chno, void *base)
{
	__inf("sel=%d, chno=%d, base=0x%p\n", sel, chno, base);
	ccsc_dev[sel][chno] = (__csc_reg_t *)base;

	return 0;
}

static int de_icsc_set_reg_base(unsigned int sel, unsigned int chno, void *base)
{
	__inf("sel=%d, chno=%d, base=0x%p\n", sel, chno, base);
	icsc_dev[sel][chno] = (__icsc_reg_t *)base;

	return 0;
}

int de_ccsc_apply(unsigned int sel, unsigned int ch_id, struct disp_csc_config *config)
{
	int csc_coeff[12];
	unsigned int in_fmt, in_mode, out_fmt, out_mode;

	//enable FCE CSC when UI(RGB format) input
	if (vep_support[sel][ch_id])
	{
		if (config->in_fmt == DE_RGB && config->out_fmt == DE_RGB)
		{
			icsc_dev[sel][ch_id]->bypass.bits.enable = 1;
			icsc_block[sel][ch_id].dirty = 1;

			in_fmt = DE_YUV;
			in_mode = DE_ENHANCE;
			out_fmt = DE_RGB;
			out_mode = DE_BT601;
		}
		else
		{
			icsc_dev[sel][ch_id]->bypass.bits.enable = 0;
			icsc_block[sel][ch_id].dirty = 1;

			in_fmt = config->in_fmt;
			in_mode = config->in_mode;
			out_fmt = config->out_fmt;
			out_mode = config->out_mode;
		}

	}
	else
	{
		in_fmt = config->in_fmt;
		in_mode = config->in_mode;
		out_fmt = config->out_fmt;
		out_mode = config->out_mode;
	}

	de_csc_coeff_calc(in_fmt, in_mode, out_fmt, out_mode,
						config->brightness, config->contrast, config->saturation, config->hue,
						config->out_color_range, csc_coeff);

	ccsc_dev[sel][ch_id]->c00.dwval = *(csc_coeff);
	ccsc_dev[sel][ch_id]->c01.dwval = *(csc_coeff + 1);
	ccsc_dev[sel][ch_id]->c02.dwval = *(csc_coeff + 2);
	ccsc_dev[sel][ch_id]->c03.dwval = *(csc_coeff + 3) + 0x200;
	ccsc_dev[sel][ch_id]->c10.dwval = *(csc_coeff + 4);
	ccsc_dev[sel][ch_id]->c11.dwval = *(csc_coeff + 5);
	ccsc_dev[sel][ch_id]->c12.dwval = *(csc_coeff + 6);
	ccsc_dev[sel][ch_id]->c13.dwval = *(csc_coeff + 7) + 0x200;
	ccsc_dev[sel][ch_id]->c20.dwval = *(csc_coeff + 8);
	ccsc_dev[sel][ch_id]->c21.dwval = *(csc_coeff + 9);
	ccsc_dev[sel][ch_id]->c22.dwval = *(csc_coeff + 10);
	ccsc_dev[sel][ch_id]->c23.dwval = *(csc_coeff + 11) + 0x200;

	ccsc_dev[sel][ch_id]->bypass.bits.enable = 1; //always enable csc
	csc_block[sel][ch_id].dirty = 1;

	return 0;
}

int de_ccsc_update_regs(unsigned int sel)
{
	int ch_id;
	for (ch_id=0; ch_id<vi_num[sel]; ch_id++)
	{
		if (csc_block[sel][ch_id].dirty == 0x1){
			memcpy((void *)csc_block[sel][ch_id].off,csc_block[sel][ch_id].val,csc_block[sel][ch_id].size);
			csc_block[sel][ch_id].dirty = 0x0;}
		if (vep_support[sel][ch_id])
		{
			if (icsc_block[sel][ch_id].dirty == 0x1){
			memcpy((void *)icsc_block[sel][ch_id].off,icsc_block[sel][ch_id].val,icsc_block[sel][ch_id].size);
			icsc_block[sel][ch_id].dirty = 0x0;}
		}
	}
	return 0;
}

int de_ccsc_init(disp_bsp_init_para *para)
{
	uintptr_t base, base_ofst;
	void *memory;
	int screen_id, ch_id, device_num;

	device_num = de_feat_get_num_devices();

	for (screen_id=0; screen_id<device_num; screen_id++)
		vi_num[screen_id] = de_feat_get_num_vi_chns(screen_id);

	for (screen_id=0; screen_id<device_num; screen_id++)
		for (ch_id=0; ch_id<vi_num[screen_id]; ch_id++)
		{
			vep_support[screen_id][ch_id] = de_feat_is_support_vep_by_chn(screen_id, ch_id);

			if (screen_id==0)
			{
				base_ofst = (ch_id==0)?CCSC00_OFST:CCSC01_OFST;
			}
			else
			{
				if (vep_support[screen_id][ch_id])
				{
					base_ofst = (ch_id==0)?CCSC00_OFST:CCSC01_OFST;
				}
				else
				{
					base_ofst = (ch_id==0)?CCSC10_OFST:CCSC11_OFST;
				}
			}

			base = para->reg_base[DISP_MOD_DE] + (screen_id+1)*0x00100000 + base_ofst;

			memory = (void *)kmalloc(sizeof(__csc_reg_t), GFP_KERNEL | __GFP_ZERO);
			if (NULL == memory) {
				__wrn("malloc Ccsc[%d][%d] memory fail! size=0x%x\n", screen_id, ch_id, (unsigned int)sizeof(__csc_reg_t));
				return -1;
			}

			csc_block[screen_id][ch_id].off			= base;
			csc_block[screen_id][ch_id].val			= memory;
			csc_block[screen_id][ch_id].size		= 0x40;
			csc_block[screen_id][ch_id].dirty 		= 1;

			de_ccsc_set_reg_base(screen_id, ch_id, memory);

			//input csc
			if (vep_support[screen_id][ch_id])
			{
				base_ofst = (ch_id==0)?ICSC0_OFST:ICSC1_OFST;

				base = para->reg_base[DISP_MOD_DE] + (screen_id+1)*0x00100000 + FCE_OFST + 0x40;

				__inf("sel %d, Icsc_base[%d]=0x%p\n", screen_id, ch_id, (void*)base);

				memory = (void *)kmalloc(sizeof(__icsc_reg_t), GFP_KERNEL | __GFP_ZERO);
				if (NULL == memory) {
					__wrn("malloc Icsc[%d][%d] memory fail! size=0x%x\n", screen_id, ch_id, (unsigned int)sizeof(__icsc_reg_t));
					return -1;
				}

				icsc_block[screen_id][ch_id].off		= base;
				icsc_block[screen_id][ch_id].val		= memory;
				icsc_block[screen_id][ch_id].size		= 0x04;
				icsc_block[screen_id][ch_id].dirty 		= 0;

				de_icsc_set_reg_base(screen_id, ch_id, memory);
			}

		}

	return 0;
}

typedef struct __SCAL_MATRIX4X4
{
	__s64 x00;
	__s64 x01;
	__s64 x02;
	__s64 x03;
	__s64 x10;
	__s64 x11;
	__s64 x12;
	__s64 x13;
	__s64 x20;
	__s64 x21;
	__s64 x22;
	__s64 x23;
	__s64 x30;
	__s64 x31;
	__s64 x32;
	__s64 x33;
}__scal_matrix4x4;

inline int IntRightShift(int datain, unsigned int shiftbit)
{
	int dataout;
	int tmp;
	tmp = (shiftbit>=1)?(1<<(shiftbit-1)):0;
	if (datain >= 0)
	{
		dataout = (datain + tmp)>> shiftbit;
	}
	else
	{
		dataout = - ((-datain + tmp) >> shiftbit);
	}

	return dataout;
}


inline __s64 IntRightShift64(__s64 datain, unsigned int shiftbit)
{
	__s64 dataout;
	__s64 tmp;
	tmp = (shiftbit>=1)?(1<<(shiftbit-1)):0;
	if (datain >= 0)
	{
		dataout = (datain + tmp)>> shiftbit;
	}
	else
	{
		dataout = - ((-datain + tmp) >> shiftbit);
	}

	return dataout;
}

static s32 iDE_SCAL_Matrix_Mul(__scal_matrix4x4 *in1, __scal_matrix4x4 in2, __scal_matrix4x4 *result)
{

	result->x00 = IntRightShift64(in1->x00 * in2.x00 + in1->x01 * in2.x10 + in1->x02 * in2.x20 + in1->x03 * in2.x30 ,10);
	result->x01 = IntRightShift64(in1->x00 * in2.x01 + in1->x01 * in2.x11 + in1->x02 * in2.x21 + in1->x03 * in2.x31 ,10);
	result->x02 = IntRightShift64(in1->x00 * in2.x02 + in1->x01 * in2.x12 + in1->x02 * in2.x22 + in1->x03 * in2.x32 ,10);
	result->x03 = IntRightShift64(in1->x00 * in2.x03 + in1->x01 * in2.x13 + in1->x02 * in2.x23 + in1->x03 * in2.x33 ,10);
	result->x10 = IntRightShift64(in1->x10 * in2.x00 + in1->x11 * in2.x10 + in1->x12 * in2.x20 + in1->x13 * in2.x30 ,10);
	result->x11 = IntRightShift64(in1->x10 * in2.x01 + in1->x11 * in2.x11 + in1->x12 * in2.x21 + in1->x13 * in2.x31 ,10);
	result->x12 = IntRightShift64(in1->x10 * in2.x02 + in1->x11 * in2.x12 + in1->x12 * in2.x22 + in1->x13 * in2.x32 ,10);
	result->x13 = IntRightShift64(in1->x10 * in2.x03 + in1->x11 * in2.x13 + in1->x12 * in2.x23 + in1->x13 * in2.x33 ,10);
	result->x20 = IntRightShift64(in1->x20 * in2.x00 + in1->x21 * in2.x10 + in1->x22 * in2.x20 + in1->x23 * in2.x30 ,10);
	result->x21 = IntRightShift64(in1->x20 * in2.x01 + in1->x21 * in2.x11 + in1->x22 * in2.x21 + in1->x23 * in2.x31 ,10);
	result->x22 = IntRightShift64(in1->x20 * in2.x02 + in1->x21 * in2.x12 + in1->x22 * in2.x22 + in1->x23 * in2.x32 ,10);
	result->x23 = IntRightShift64(in1->x20 * in2.x03 + in1->x21 * in2.x13 + in1->x22 * in2.x23 + in1->x23 * in2.x33 ,10);
	result->x30 = IntRightShift64(in1->x30 * in2.x00 + in1->x31 * in2.x10 + in1->x32 * in2.x20 + in1->x33 * in2.x30 ,10);
	result->x31 = IntRightShift64(in1->x30 * in2.x01 + in1->x31 * in2.x11 + in1->x32 * in2.x21 + in1->x33 * in2.x31 ,10);
	result->x32 = IntRightShift64(in1->x30 * in2.x02 + in1->x31 * in2.x12 + in1->x32 * in2.x22 + in1->x33 * in2.x32 ,10);
	result->x33 = IntRightShift64(in1->x30 * in2.x03 + in1->x31 * in2.x13 + in1->x32 * in2.x23 + in1->x33 * in2.x33 ,10);

	return 0;
}

//normal case:
//display a SD video: 											infmt = DE_YUV, incscmod = BT_601, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//display a HD video: 											infmt = DE_YUV, incscmod = BT_709, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//display a JPEG picture:										infmt = DE_YUV, incscmod = BT_YCC, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//display a UI (RGB format)	with ENHANCE enable					infmt = DE_YUV, incscmod = BT_ENHANCE, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//output to TV with HDMI in RGB mode: 							infmt = DE_RGB, incscmod = BT_601, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_16_235
//output to PC with HDMI in RGB mode:							infmt = DE_RGB, incscmod = BT_601, outfmt = DE_RGB, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//output to TV with HDMI in YCbCr mode, 480i/576i/480p/576p: 	infmt = DE_RGB, incscmod = BT_601, outfmt = DE_YUV, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//output to TV with HDMI in YCbCr mode, 720p/1080p/2160p: 		infmt = DE_RGB, incscmod = BT_601, outfmt = DE_YUV, outcscmod = BT_709, out_color_range = DISP_COLOR_RANGE_0_255
//output to TV with CVBS:										infmt = DE_RGB, incscmod = BT_601, outfmt = DE_YUV, outcscmod = BT_601, out_color_range = DISP_COLOR_RANGE_0_255
//bypass:														outfmt = infmt, outcscmod = incscmod, out_color_range = DISP_COLOR_RANGE_0_255, brightness=contrast=saturation=hue=50
int de_csc_coeff_calc(unsigned int infmt, unsigned int incscmod, unsigned int outfmt, unsigned int outcscmod,
								unsigned int brightness, unsigned int contrast, unsigned int saturation, unsigned int hue,
								unsigned int out_color_range, int *csc_coeff)
{
	__scal_matrix4x4 enhancecoeff, tmpcoeff;
	__scal_matrix4x4 *coeff[5], in0coeff, *in1coeff;
	int oper, i;
	int i_bright, i_contrast, i_saturation, i_hue, sinv, cosv;

	oper = 0;

	//BYPASS
	if (infmt == outfmt && incscmod == outcscmod && out_color_range == DISP_COLOR_RANGE_0_255 && brightness == 50 && contrast == 50 && saturation == 50 && hue == 50)
	{
		memcpy(csc_coeff, bypass_csc, 48);
		return 0;
	}

	//NON-BYPASS
	if (infmt == DE_RGB)
	{
		//convert to YCbCr
		if (outfmt == DE_RGB)
		{
			coeff[oper] = (__scal_matrix4x4 *)(r2y + 0x20);
			oper++;
		}
		else
		{
			if (outcscmod == DE_BT601){
				coeff[oper] = (__scal_matrix4x4 *)(r2y);
				oper++;
			}
			else if (outcscmod == DE_BT709){
				coeff[oper] = (__scal_matrix4x4 *)(r2y + 0x20);
				oper++;
			}
			//else //not support DE_YCC output
		}
	}
	else
	{
		if (incscmod != outcscmod && outfmt == DE_YUV)
		{
			if (incscmod == DE_BT601 && outcscmod == DE_BT709){
				coeff[oper] = (__scal_matrix4x4 *)(y2y);
				oper++;
			}
			else if (incscmod == DE_BT709 && outcscmod == DE_BT601){
				coeff[oper] = (__scal_matrix4x4 *)(y2y + 0x20);
				oper++;
			}
		}
	}

	if (brightness != 50 || contrast != 50 || saturation != 50 || hue != 50)
	{
		brightness = brightness>100?100:brightness;
		contrast = contrast>100?100:contrast;
		saturation = saturation>100?100:saturation;
		hue = hue>100?100:hue;

		i_bright = (int)(brightness*64/100);
		i_saturation = (int)(saturation*64/100);
		i_contrast = (int)(contrast*64/100);
		i_hue = (int)(hue*64/100);

		sinv = sin_cos[i_hue&0x3f];
		cosv = sin_cos[64 + (i_hue&0x3f)];

		//calculate enhance matrix
		enhancecoeff.x00 = i_contrast << 7;
		enhancecoeff.x01 = 0;
		enhancecoeff.x02 = 0;
		enhancecoeff.x03 = (((i_bright - 32) + 16) <<12) - ( i_contrast << 11);
		enhancecoeff.x10 = 0;
		enhancecoeff.x11 = (i_contrast * i_saturation * cosv) >> 5;
		enhancecoeff.x12 = (i_contrast * i_saturation * sinv) >> 5;
		enhancecoeff.x13 = (1<<19) - ((enhancecoeff.x11 + enhancecoeff.x12)<<7);
		enhancecoeff.x20 = 0;
		enhancecoeff.x21 = (-i_contrast * i_saturation * sinv)>> 5;
		enhancecoeff.x22 = (i_contrast * i_saturation * cosv) >> 5;
		enhancecoeff.x23 = (1<<19) - ((enhancecoeff.x22 + enhancecoeff.x21)<<7);
		enhancecoeff.x30 = 0;
		enhancecoeff.x31 = 0;
		enhancecoeff.x32 = 0;
		enhancecoeff.x33 = 4096;

		coeff[oper] = (__scal_matrix4x4 *)(&enhancecoeff);
		oper++;

	}

	if (outfmt == DE_RGB)
	{
		if (infmt == DE_RGB)
		{
			coeff[oper] = (__scal_matrix4x4 *)(y2r + 0x20);
			oper++;

			if (out_color_range == DISP_COLOR_RANGE_16_235)
			{
				coeff[oper] = (__scal_matrix4x4 *)(r2r);
				oper++;
			}
		}
		else
		{
			if (out_color_range == DISP_COLOR_RANGE_16_235)
			{
				if (incscmod == DE_BT601){
					coeff[oper] = (__scal_matrix4x4 *)(y2r + 0x80);
					oper++;
				}
				else if (incscmod == DE_BT709){
					coeff[oper] = (__scal_matrix4x4 *)(y2r + 0xa0);
					oper++;
				}
			}
			else
			{
				if (incscmod == DE_BT601){
					coeff[oper] = (__scal_matrix4x4 *)(y2r);
					oper++;
				}
				else if (incscmod == DE_BT709){
					coeff[oper] = (__scal_matrix4x4 *)(y2r + 0x20);
					oper++;
				}
				else if (incscmod == DE_YCC){
					coeff[oper] = (__scal_matrix4x4 *)(y2r + 0x40);
					oper++;
				}
				else if (incscmod == DE_ENHANCE){
					coeff[oper] = (__scal_matrix4x4 *)(y2r + 0x60);
					oper++;
				}
			}
		}
	}

	//matrix multiply
	if (oper==0)
	{
		csc_coeff = bypass_csc;
	}
	else if (oper==1)
	{
		for (i=0;i<12;i++)
			*(csc_coeff + i) = IntRightShift64((int)(*((__s64 *)coeff[0] + i)), oper<<1);
	}
	else
	{
		in0coeff = *(coeff[0]);
		for (i=1;i<oper;i++)
		{
			in1coeff = coeff[i];
			iDE_SCAL_Matrix_Mul(in1coeff, in0coeff, &tmpcoeff);
			in0coeff = tmpcoeff;
		}

		for (i=0;i<12;i++)
			*(csc_coeff + i) = IntRightShift64((int)(*((__s64 *)(&tmpcoeff) + i)), oper<<1);
	}

	return 0;

}


