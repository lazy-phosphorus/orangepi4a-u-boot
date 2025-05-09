/*
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "packets.h"
#include "audio.h"

#define ACP_PACKET_SIZE	16
#define ISRC_PACKET_SIZE	16

#define FC_GMD_PB_SIZE			28

void fc_drm_disable(hdmi_tx_dev_t *dev);

void fc_acp_type(hdmi_tx_dev_t *dev, u8 type)
{
	LOG_TRACE1(type);
	dev_write(dev, FC_ACP0, type);
}

void fc_acp_type_dependent_fields(hdmi_tx_dev_t *dev, u8 *fields,
							u8 fieldsLength)
{
	u8 c = 0;

	LOG_TRACE1(fields[0]);
	if (fieldsLength > (FC_ACP1 - FC_ACP16 + 1)) {
		fieldsLength = (FC_ACP1 - FC_ACP16 + 1);
		HDMI_INFO_MSG("WARN:ACP Fields Truncated\n");
	}

	for (c = 0; c < fieldsLength; c++)
		dev_write(dev, FC_ACP1 - c, fields[c]);
}

void fc_RgbYcc(hdmi_tx_dev_t *dev, u8 type)
{
	LOG_TRACE1(type);
	dev_write_mask(dev, FC_AVICONF0,
			FC_AVICONF0_RGC_YCC_INDICATION_MASK, type);
}

void fc_ScanInfo(hdmi_tx_dev_t *dev, u8 left)
{
	LOG_TRACE1(left);
	dev_write_mask(dev, FC_AVICONF0,
			FC_AVICONF0_SCAN_INFORMATION_MASK, left);
}

void fc_Colorimetry(hdmi_tx_dev_t *dev, unsigned cscITU)
{
	LOG_TRACE1(cscITU);
	dev_write_mask(dev, FC_AVICONF1, FC_AVICONF1_COLORIMETRY_MASK, cscITU);
}

void fc_PicAspectRatio(hdmi_tx_dev_t *dev, u8 ar)
{
	LOG_TRACE1(ar);
	dev_write_mask(dev, FC_AVICONF1,
			FC_AVICONF1_PICTURE_ASPECT_RATIO_MASK, ar);
}

void fc_ActiveAspectRatioValid(hdmi_tx_dev_t *dev, u8 valid)
{
	LOG_TRACE1(valid);
	dev_write_mask(dev, FC_AVICONF0,
			FC_AVICONF0_ACTIVE_FORMAT_PRESENT_MASK, valid);
}

void fc_ActiveFormatAspectRatio(hdmi_tx_dev_t *dev, u8 left)
{
	LOG_TRACE1(left);
	dev_write_mask(dev, FC_AVICONF1,
			FC_AVICONF1_ACTIVE_ASPECT_RATIO_MASK, left);
}

void fc_IsItContent(hdmi_tx_dev_t *dev, u8 it)
{
	LOG_TRACE1(it);
	dev_write_mask(dev, FC_AVICONF2,
			FC_AVICONF2_IT_CONTENT_MASK, (it ? 1 : 0));
}

void fc_ExtendedColorimetry(hdmi_tx_dev_t *dev, u8 extColor)
{
	LOG_TRACE1(extColor);
	dev_write_mask(dev, FC_AVICONF2,
			FC_AVICONF2_EXTENDED_COLORIMETRY_MASK, extColor);
	dev_write_mask(dev, FC_AVICONF1, FC_AVICONF1_COLORIMETRY_MASK, 0x3);
}

void fc_QuantizationRange(hdmi_tx_dev_t *dev, u8 range)
{
	LOG_TRACE1(range);
	dev_write_mask(dev, FC_AVICONF2,
			FC_AVICONF2_QUANTIZATION_RANGE_MASK, range);
}

void fc_NonUniformPicScaling(hdmi_tx_dev_t *dev, u8 scale)
{
	LOG_TRACE1(scale);
	dev_write_mask(dev, FC_AVICONF2,
			FC_AVICONF2_NON_UNIFORM_PICTURE_SCALING_MASK, scale);
}

void fc_VideoCode(hdmi_tx_dev_t *dev, u8 code)
{
	LOG_TRACE1(code);
	dev_write(dev, FC_AVIVID, code);
}

void fc_HorizontalBarsValid(hdmi_tx_dev_t *dev, u8 validity)
{
	dev_write_mask(dev, FC_AVICONF0,
		FC_AVICONF0_BAR_INFORMATION_MASK & 0x8, (validity ? 1 : 0));
}

void fc_HorizontalBars(hdmi_tx_dev_t *dev, u16 endTop, u16 startBottom)
{
	LOG_TRACE2(endTop, startBottom);
	dev_write(dev, FC_AVIETB0, (u8) (endTop));
	dev_write(dev, FC_AVIETB1, (u8) (endTop >> 8));
	dev_write(dev, FC_AVISBB0, (u8) (startBottom));
	dev_write(dev, FC_AVISBB1, (u8) (startBottom >> 8));
}

void fc_VerticalBarsValid(hdmi_tx_dev_t *dev, u8 validity)
{
	dev_write_mask(dev, FC_AVICONF0,
		FC_AVICONF0_BAR_INFORMATION_MASK & 0x4, (validity ? 1 : 0));
}

void fc_VerticalBars(hdmi_tx_dev_t *dev, u16 endLeft, u16 startRight)
{
	LOG_TRACE2(endLeft, startRight);
	dev_write(dev, FC_AVIELB0, (u8) (endLeft));
	dev_write(dev, FC_AVIELB1, (u8) (endLeft >> 8));
	dev_write(dev, FC_AVISRB0, (u8) (startRight));
	dev_write(dev, FC_AVISRB1, (u8) (startRight >> 8));
}

void fc_OutPixelRepetition(hdmi_tx_dev_t *dev, u8 pr)
{
	LOG_TRACE1(pr);
	dev_write_mask(dev, FC_PRCONF, FC_PRCONF_OUTPUT_PR_FACTOR_MASK, pr);
}

u32 fc_GetInfoFrameSatus(hdmi_tx_dev_t *dev)
{
	return dev_read(dev, FC_AVICONF0);
}

void fc_avi_config(hdmi_tx_dev_t *dev, videoParams_t *videoParams)
{
	u16 endTop = 0;
	u16 startBottom = 0;
	u16 endLeft = 0;
	u16 startRight = 0;
	dtd_t *dtd = &videoParams->mDtd;

	LOG_TRACE();

	if (videoParams->mEncodingOut == RGB) {
		HDMI_INFO_MSG("rgb\n");
		fc_RgbYcc(dev, 0);
	} else if (videoParams->mEncodingOut == YCC422) {
		HDMI_INFO_MSG("ycc422\n");
		fc_RgbYcc(dev, 1);
	} else if (videoParams->mEncodingOut == YCC444) {
		HDMI_INFO_MSG("ycc444\n");
		fc_RgbYcc(dev, 2);
	} else if (videoParams->mEncodingOut == YCC420) {
		HDMI_INFO_MSG("ycc420\n");
		fc_RgbYcc(dev, 3);
	}

	fc_ActiveFormatAspectRatio(dev, 0x8);

	HDMI_INFO_MSG("infoframe status before %x\n",
			fc_GetInfoFrameSatus(dev));

	fc_ScanInfo(dev, videoParams->mScanInfo);

	if ((dtd->mHImageSize != 0 || dtd->mVImageSize != 0)
		&& (dtd->mHImageSize > dtd->mVImageSize)) {
		u8 pic = (dtd->mHImageSize * 10) % dtd->mVImageSize;
		/* 16:9 or 4:3 */
		fc_PicAspectRatio(dev, (pic > 5) ? 2 : 1);
	} else {
		/* No Data */
		fc_PicAspectRatio(dev, 0);
	}

	fc_IsItContent(dev, videoParams->mItContent);

	fc_QuantizationRange(dev, videoParams->mRgbQuantizationRange);
	fc_NonUniformPicScaling(dev, videoParams->mNonUniformScaling);

#if defined(__LINUX_PLAT__)
	fc_VideoCode(dev, videoParams->mCea_code);
#else
	if (dtd->mCode != (u8) (-1) && dtd->mCode < 0x200) {
		fc_VideoCode(dev, dtd->mCode);
	} else
		fc_VideoCode(dev, 0);
#endif
	if (videoParams->mColorimetry == EXTENDED_COLORIMETRY) {
		/* ext colorimetry valid */
		if (videoParams->mExtColorimetry != (u8) (-1)) {
			fc_ExtendedColorimetry(dev, videoParams->mExtColorimetry);
			fc_Colorimetry(dev, videoParams->mColorimetry);/*EXT-3*/
		} else {
			fc_Colorimetry(dev, 0);	/* No Data */
		}
	} else {
		/* NODATA-0/ 601-1/ 709-2/ EXT-3 */
		fc_Colorimetry(dev, videoParams->mColorimetry);
	}
	if (videoParams->mActiveFormatAspectRatio != 0) {
		fc_ActiveFormatAspectRatio(dev, videoParams->mActiveFormatAspectRatio);
		fc_ActiveAspectRatioValid(dev, 1);
	} else {
		fc_ActiveFormatAspectRatio(dev, 0);
		fc_ActiveAspectRatioValid(dev, 0);
	}
	if (videoParams->mEndTopBar != (u16) (-1) ||
			videoParams->mStartBottomBar != (u16) (-1)) {

		if (videoParams->mEndTopBar != (u16) (-1))
			endTop = videoParams->mEndTopBar;
		if (videoParams->mStartBottomBar != (u16) (-1))
			startBottom = videoParams->mStartBottomBar;

		fc_HorizontalBars(dev, endTop, startBottom);
		fc_HorizontalBarsValid(dev, 1);
	} else {
		fc_HorizontalBarsValid(dev, 0);
	}
	if (videoParams->mEndLeftBar != (u16) (-1) ||
		videoParams->mStartRightBar != (u16) (-1)) {
		if (videoParams->mEndLeftBar != (u16) (-1))
			endLeft = videoParams->mEndLeftBar;

		if (videoParams->mStartRightBar != (u16) (-1))
			startRight = videoParams->mStartRightBar;

		fc_VerticalBars(dev, endLeft, startRight);
		fc_VerticalBarsValid(dev, 1);
	} else {
		fc_VerticalBarsValid(dev, 0);
	}
	fc_OutPixelRepetition(dev, (dtd->mPixelRepetitionInput + 1) *
				(videoParams->mPixelRepetitionFactor + 1) - 1);
	HDMI_INFO_MSG("infoframe status before after:%x\n",
			fc_GetInfoFrameSatus(dev));
}



void fc_gamut_Profile(hdmi_tx_dev_t *dev, u8 profile)
{
	LOG_TRACE1(profile);
	dev_write_mask(dev, FC_GMD_HB, FC_GMD_HB_GMDGBD_PROFILE_MASK, profile);
}

void fc_gamut_AffectedSeqNo(hdmi_tx_dev_t *dev, u8 no)
{
	LOG_TRACE1(no);
	dev_write_mask(dev, FC_GMD_HB,
			FC_GMD_HB_GMDAFFECTED_GAMUT_SEQ_NUM_MASK, no);
}

void fc_gamut_PacketsPerFrame(hdmi_tx_dev_t *dev, u8 packets)
{
	LOG_TRACE1(packets);
	dev_write_mask(dev, FC_GMD_CONF,
			FC_GMD_CONF_GMDPACKETSINFRAME_MASK, packets);
}

void fc_gamut_PacketLineSpacing(hdmi_tx_dev_t *dev, u8 lineSpacing)
{
	LOG_TRACE1(lineSpacing);
	dev_write_mask(dev, FC_GMD_CONF,
			FC_GMD_CONF_GMDPACKETLINESPACING_MASK, lineSpacing);
}

void fc_gamut_Content(hdmi_tx_dev_t *dev, const u8 *content, u8 length)
{
	u8 i = 0;

	LOG_TRACE1(content[0]);
	if (length > (FC_GMD_PB_SIZE)) {
		length = (FC_GMD_PB_SIZE);
		HDMI_INFO_MSG("WARN:Gamut Content Truncated");
	}

	for (i = 0; i < length; i++)
		dev_write(dev, FC_GMD_PB0 + (i*4), content[i]);
}

void fc_gamut_enable_tx(hdmi_tx_dev_t *dev, u8 enable)
{
	LOG_TRACE1(enable);
	if (enable)
		enable = 1; /* ensure value is 1 */
	dev_write_mask(dev, FC_GMD_EN, FC_GMD_EN_GMDENABLETX_MASK, enable);
}

void fc_gamut_UpdatePacket(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	dev_write_mask(dev, FC_GMD_UP, FC_GMD_UP_GMDUPDATEPACKET_MASK, 1);
}

u8 fc_gamut_CurrentSeqNo(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	return (u8)(dev_read(dev, FC_GMD_STAT) & 0xF);
}


void fc_gamut_config(hdmi_tx_dev_t *dev)
{
	/* P0 */
	fc_gamut_Profile(dev, 0x0);

	/* P0 */
	fc_gamut_PacketsPerFrame(dev, 0x1);
	fc_gamut_PacketLineSpacing(dev, 0x1);
}

void fc_gamut_packet_config(hdmi_tx_dev_t *dev, const u8 *gbdContent, u8 length)
{
	fc_gamut_enable_tx(dev, 1);
	/*sequential*/
	fc_gamut_AffectedSeqNo(dev, (fc_gamut_CurrentSeqNo(dev) + 1) % 16);
	fc_gamut_Content(dev, gbdContent, length);
	fc_gamut_UpdatePacket(dev); /* set next_field to 1 */
}

void fc_isrc_status(hdmi_tx_dev_t *dev, u8 code)
{
	LOG_TRACE1(code);
	dev_write_mask(dev, FC_ISCR1_0, FC_ISCR1_0_ISRC_STATUS_MASK, code);
}

void fc_isrc_valid(hdmi_tx_dev_t *dev, u8 validity)
{
	LOG_TRACE1(validity);
	dev_write_mask(dev, FC_ISCR1_0, FC_ISCR1_0_ISRC_VALID_MASK,
							(validity ? 1 : 0));
}

void fc_isrc_cont(hdmi_tx_dev_t *dev, u8 isContinued)
{
	LOG_TRACE1(isContinued);
	dev_write_mask(dev, FC_ISCR1_0, FC_ISCR1_0_ISRC_CONT_MASK,
						(isContinued ? 1 : 0));
}

void fc_isrc_isrc1_codes(hdmi_tx_dev_t *dev, u8 *codes, u8 length)
{
	u8 c = 0;

	LOG_TRACE1(codes[0]);
	if (length > (FC_ISCR1_1 - FC_ISCR1_16 + 1)) {
		length = (FC_ISCR1_1 - FC_ISCR1_16 + 1);
		HDMI_INFO_MSG("WARN:ISCR1 Codes Truncated\n");
	}

	for (c = 0; c < length; c++)
		dev_write(dev, FC_ISCR1_1 - c, codes[c]);
}

void fc_isrc_isrc2_codes(hdmi_tx_dev_t *dev, u8 *codes, u8 length)
{
	u8 c = 0;

	LOG_TRACE1(codes[0]);
	if (length > (FC_ISCR2_0 - FC_ISCR2_15 + 1)) {
		length = (FC_ISCR2_0 - FC_ISCR2_15 + 1);
		HDMI_INFO_MSG("WARN:ISCR2 Codes Truncated\n");
	}

	for (c = 0; c < length; c++)
		dev_write(dev, FC_ISCR2_0 - c, codes[c]);
}

void fc_packets_QueuePriorityHigh(hdmi_tx_dev_t *dev, u8 value)
{
	LOG_TRACE1(value);
	dev_write_mask(dev, FC_CTRLQHIGH,
				FC_CTRLQHIGH_ONHIGHATTENDED_MASK, value);
}

void fc_packets_QueuePriorityLow(hdmi_tx_dev_t *dev, u8 value)
{
	LOG_TRACE1(value);
	dev_write_mask(dev, FC_CTRLQLOW, FC_CTRLQLOW_ONLOWATTENDED_MASK, value);
}

void fc_packets_MetadataFrameInterpolation(hdmi_tx_dev_t *dev, u8 value)
{
	LOG_TRACE1(value);
	dev_write_mask(dev, FC_DATAUTO1,
			FC_DATAUTO1_AUTO_FRAME_INTERPOLATION_MASK, value);
}

void fc_packets_MetadataFramesPerPacket(hdmi_tx_dev_t *dev, u8 value)
{
	LOG_TRACE1(value);
	dev_write_mask(dev, FC_DATAUTO2,
			FC_DATAUTO2_AUTO_FRAME_PACKETS_MASK, value);
}

void fc_packets_MetadataLineSpacing(hdmi_tx_dev_t *dev, u8 value)
{
	LOG_TRACE1(value);
	dev_write_mask(dev, FC_DATAUTO2,
			FC_DATAUTO2_AUTO_LINE_SPACING_MASK, value);
}

void fc_packets_AutoSend(hdmi_tx_dev_t *dev, u8 enable, u8 mask)
{
	LOG_TRACE2(enable, mask);
	dev_write_mask(dev, FC_DATAUTO0, (1 << mask), (enable ? 1 : 0));
}

void fc_packets_ManualSend(hdmi_tx_dev_t *dev, u8 mask)
{
	LOG_TRACE1(mask);
	dev_write_mask(dev, FC_DATMAN, (1 << mask), 1);
}

void fc_packets_disable_all(hdmi_tx_dev_t *dev)
{
	uint32_t value = (uint32_t)(~(BIT(ACP_TX) | BIT(ISRC1_TX) |
			BIT(ISRC2_TX) | BIT(SPD_TX) | BIT(VSD_TX)));

	LOG_TRACE();
	dev_write(dev, FC_DATAUTO0, value & dev_read(dev, FC_DATAUTO0));
}

void fc_packets_metadata_config(hdmi_tx_dev_t *dev)
{
	fc_packets_MetadataFrameInterpolation(dev, 1);
	fc_packets_MetadataFramesPerPacket(dev, 1);
	fc_packets_MetadataLineSpacing(dev, 1);
}

void fc_spd_VendorName(hdmi_tx_dev_t *dev, const u8 *data,
					unsigned short length)
{
	unsigned short i = 0;

	LOG_TRACE();
	for (i = 0; i < length; i++)
		dev_write(dev, FC_SPDVENDORNAME0 + (i*4), data[i]);
}

void fc_spd_ProductName(hdmi_tx_dev_t *dev, const u8 *data,
						unsigned short length)
{
	unsigned short i = 0;

	LOG_TRACE();
	for (i = 0; i < length; i++)
		dev_write(dev, FC_SPDPRODUCTNAME0 + (i*4), data[i]);
}

void fc_spd_SourceDeviceInfo(hdmi_tx_dev_t *dev, u8 code)
{
	LOG_TRACE1(code);
	dev_write(dev, FC_SPDDEVICEINF, code);
}

int fc_spd_config(hdmi_tx_dev_t *dev, fc_spd_info_t *spd_data)
{
	const unsigned short pSize = 8;
	const unsigned short vSize = 16;

	LOG_TRACE();

	if (spd_data == NULL) {
		HDMI_ERROR_MSG("Improper argument: spd_data\n");
		return FALSE;
	}

	fc_packets_AutoSend(dev, 0, SPD_TX);/*prevent sending half the info.*/

	if (spd_data->vName == 0) {
		error_set(ERR_INVALID_PARAM_VENDOR_NAME);
		HDMI_ERROR_MSG("invalid parameter\n");
		return FALSE;
	}
	if (spd_data->vLength > vSize) {
		spd_data->vLength = vSize;
		HDMI_INFO_MSG("vendor name truncated\n");
	}
	if (spd_data->pName == 0) {
		error_set(ERR_INVALID_PARAM_PRODUCT_NAME);
		HDMI_ERROR_MSG("invalid parameter\n");
		return FALSE;
	}
	if (spd_data->pLength > pSize) {
		spd_data->pLength = pSize;
		HDMI_INFO_MSG("product name truncated\n");
	}

	fc_spd_VendorName(dev, spd_data->vName, spd_data->vLength);
	fc_spd_ProductName(dev, spd_data->pName, spd_data->pLength);

	fc_spd_SourceDeviceInfo(dev, spd_data->code);

	if (spd_data->autoSend)
		fc_packets_AutoSend(dev, spd_data->autoSend, SPD_TX);
	else
		fc_packets_ManualSend(dev, SPD_TX);

	return TRUE;
}

void fc_vsd_vendor_OUI(hdmi_tx_dev_t *dev, u32 id)
{
	LOG_TRACE1(id);
	dev_write(dev, (FC_VSDIEEEID0), id);
	dev_write(dev, (FC_VSDIEEEID1), id >> 8);
	dev_write(dev, (FC_VSDIEEEID2), id >> 16);
}

u8 fc_vsd_vendor_payload(hdmi_tx_dev_t *dev, const u8 *data,
						unsigned short length)
{
	const unsigned short size = 24;
	unsigned i = 0;

	LOG_TRACE();
	if (data == 0) {
		HDMI_INFO_MSG("invalid parameter\n");
		return -1;
	}
	if (length > size) {
		length = size;
		HDMI_INFO_MSG("vendor payload truncated\n");
	}
	for (i = 0; i < length; i++)
		dev_write(dev, (FC_VSDPAYLOAD0 + (i*4)), data[i]);

	return 0;
}

void fc_vsif_enable(hdmi_tx_dev_t *dev, u8 enable)
{
	dev_write_mask(dev, FC_PACKET_TX_EN, FC_PACKET_TX_EN_AUT_TX_EN_MASK, enable);
}

int packets_Initialize(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	packets_DisableAllPackets(dev);
	return TRUE;
}

/*packets configure is the same as infoframe configure*/
int packets_Configure(hdmi_tx_dev_t *dev, videoParams_t *video,
						productParams_t *prod)
{
	u32 oui = 0;
	u8 struct_3d = 0;
	u8 data[3];
	u8 *vendor_payload = prod->mVendorPayload;
	u8 payload_length = prod->mVendorPayloadLength;

	LOG_TRACE();

	if (dev->snps_hdmi_ctrl.hdmi_on == 0) {
		pr_info("DVI mode selected: packets not configured\n");
		return TRUE;
	}

	if (video->mHdmiVideoFormat == 2) {
		struct_3d = video->m3dStructure;
		HDMI_INFO_MSG("3D packets configure\n");

		/* frame packing || tab || sbs */
		if ((struct_3d == 0) || (struct_3d == 6) || (struct_3d == 8)) {
			data[0] = video->mHdmiVideoFormat << 5; /* PB4 */
			data[1] = struct_3d << 4; /* PB5 */
			data[2] = video->m3dExtData << 4;
			/* HDMI Licensing, LLC */
			packets_VendorSpecificInfoFrame(dev, 0x000C03, data,
							sizeof(data), 1);
			fc_vsif_enable(dev, 1);
		} else {
			error_set(ERR_3D_STRUCT_NOT_SUPPORTED);
			HDMI_ERROR_MSG("3D structure not supported %d\n",
							 struct_3d);
			return FALSE;
		}
	} else if ((video->mHdmiVideoFormat == 0x1) || (video->mHdmiVideoFormat == 0x0)) {
		if (prod != 0) {
			fc_spd_info_t spd_data;

			spd_data.vName = prod->mVendorName;
			spd_data.vLength = prod->mVendorNameLength;
			spd_data.pName = prod->mProductName;
			spd_data.pLength = prod->mProductNameLength;
			spd_data.code = prod->mSourceType;
			spd_data.autoSend = 1;

			oui = prod->mOUI;
			fc_spd_config(dev, &spd_data);
			packets_VendorSpecificInfoFrame(dev, oui, vendor_payload,
					payload_length, 1);
			fc_vsif_enable(dev, 1);
		} else {
				HDMI_INFO_MSG("No product info provided: not configured\n");
		}
	} else {
		pr_info("error: unknow video format\n");
		fc_vsif_enable(dev, 0);
	}

	fc_packets_metadata_config(dev);

	/* default phase 1 = true */
	dev_write_mask(dev, FC_GCP, FC_GCP_DEFAULT_PHASE_MASK,
			((video->mPixelPackingDefaultPhase == 1) ? 1 : 0));

	fc_gamut_config(dev);

	fc_avi_config(dev, video);

	/** Colorimetry */
	packets_colorimetry_config(dev, video);

	if (video->mHdr) {
		HDMI_INFO_MSG("Is HDR video format\n");
		fc_drm_up(dev, video->pb);
	} else {
		fc_drm_disable(dev);
	}

	return TRUE;
}

void packets_AudioContentProtection(hdmi_tx_dev_t *dev, u8 type,
				const u8 *fields, u8 length, u8 autoSend)
{
	u8 newFields[ACP_PACKET_SIZE];
	u16 i = 0;
	/* LOG_TRACE(); */
	fc_packets_AutoSend(dev, 0, ACP_TX);

	fc_acp_type(dev, type);

	for (i = 0; i < length; i++)
		newFields[i] = fields[i];

	if (length < ACP_PACKET_SIZE) {
		for (i = length; i < ACP_PACKET_SIZE; i++)
			newFields[i] = 0;	/* Padding */

		length = ACP_PACKET_SIZE;
	}
	fc_acp_type_dependent_fields(dev, newFields, length);
	if (!autoSend)
		fc_packets_ManualSend(dev, ACP_TX);
	else
		fc_packets_AutoSend(dev, autoSend, ACP_TX);

}

void packets_IsrcPackets(hdmi_tx_dev_t *dev, u8 initStatus,
				const u8 *codes, u8 length, u8 autoSend)
{
	u16 i = 0;
	u8 newCodes[ISRC_PACKET_SIZE * 2];

	LOG_TRACE();

	fc_packets_AutoSend(dev, 0, ISRC1_TX);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);

	fc_isrc_status(dev, initStatus);

	for (i = 0; i < length; i++)
		newCodes[i] = codes[i];

	if (length > ISRC_PACKET_SIZE) {
		for (i = length; i < (ISRC_PACKET_SIZE * 2); i++)
			newCodes[i] = 0;	/* Padding */

		length = (ISRC_PACKET_SIZE * 2);

		fc_isrc_isrc2_codes(dev, newCodes +
				(ISRC_PACKET_SIZE * sizeof(u8)),
				length - ISRC_PACKET_SIZE);
		fc_isrc_cont(dev, 1);

		fc_packets_AutoSend(dev, autoSend, ISRC2_TX);

		if (!autoSend)
			fc_packets_ManualSend(dev, ISRC2_TX);

	}
	if (length < ISRC_PACKET_SIZE) {
		for (i = length; i < ISRC_PACKET_SIZE; i++)
			newCodes[i] = 0;	/* Padding */


		length = ISRC_PACKET_SIZE;

		fc_isrc_cont(dev, 0);
	}

	fc_isrc_isrc1_codes(dev, newCodes, length);	/* first part only */
	fc_isrc_valid(dev, 1);

	fc_packets_AutoSend(dev, autoSend, ISRC1_TX);

	if (!autoSend)
		fc_packets_ManualSend(dev, ISRC1_TX);
}

void packets_AvMute(hdmi_tx_dev_t *dev, u8 enable)
{
	LOG_TRACE1(enable);
	dev_write_mask(dev, FC_GCP, FC_GCP_SET_AVMUTE_MASK, (enable ? 1 : 0));
	dev_write_mask(dev, FC_GCP, FC_GCP_CLEAR_AVMUTE_MASK, (enable ? 0 : 1));
}

void packets_IsrcStatus(hdmi_tx_dev_t *dev, u8 status)
{
	LOG_TRACE();
	fc_isrc_status(dev, status);
}

void packets_StopSendAcp(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, ACP_TX);
}

void packets_StopSendIsrc1(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, ISRC1_TX);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);
}

void packets_StopSendIsrc2(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_isrc_cont(dev, 0);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);
}

void packets_StopSendSpd(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, SPD_TX);
}

void packets_StopSendVsd(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, VSD_TX);
}

void packets_DisableAllPackets(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	fc_packets_disable_all(dev);
}

int packets_VendorSpecificInfoFrame(hdmi_tx_dev_t *dev, u32 oui,
				const u8 *payload, u8 length, u8 autoSend)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev,  0, VSD_TX);/*prevent sending half the info.*/
	fc_vsd_vendor_OUI(dev, oui);
	if (fc_vsd_vendor_payload(dev, payload, length))
		return FALSE;	/* DEFINE ERROR */

	if (autoSend)
		fc_packets_AutoSend(dev, autoSend, VSD_TX);
	else
		fc_packets_ManualSend(dev, VSD_TX);

	return TRUE;
}

u8 packets_AudioMetaDataPacket(hdmi_tx_dev_t *dev,
				audioMetaDataPacket_t *audioMetaDataPckt)
{
	halAudioMultistream_MetaDataPacket_Header(dev, audioMetaDataPckt);
	halAudioMultistream_MetaDataPacketBody(dev, audioMetaDataPckt);
	return TRUE;
}

void packets_colorimetry_config(hdmi_tx_dev_t *dev, videoParams_t *video)
{
	u8 gamut_metadata[28] = {0};
	int gdb_color_space = 0;

	fc_gamut_enable_tx(dev, 0);

	if (video->mColorimetry == EXTENDED_COLORIMETRY) {
		if (video->mExtColorimetry == XV_YCC601) {
			gdb_color_space = 1;
		} else if (video->mExtColorimetry == XV_YCC709) {
			gdb_color_space = 2;
			HDMI_INFO_MSG("xv ycc709\n");
		} else if (video->mExtColorimetry == S_YCC601) {
			gdb_color_space = 3;
		} else if (video->mExtColorimetry == ADOBE_YCC601) {
			gdb_color_space = 3;
		} else if (video->mExtColorimetry == ADOBE_RGB) {
			gdb_color_space = 3;
		}

		if (video->mColorimetryDataBlock == TRUE) {
			gamut_metadata[0] = (1 << 7) | gdb_color_space;
			fc_gamut_packet_config(dev, gamut_metadata,
					(sizeof(gamut_metadata) / sizeof(u8)));
		}
	}
}

void fc_drm_up(hdmi_tx_dev_t *dev, fc_drm_pb_t *pb)
{
	int timeout = 10;
	u32 status = 0;

	/*Configure Dynamic Range and Mastering infoFrame*/
	if (pb != 0) {
		dev_write(dev, FC_DRM_PB0, pb->eotf & 0x07);
		dev_write(dev, FC_DRM_PB1, pb->metadata & 0x07);
		dev_write(dev, FC_DRM_PB2, (pb->r_x >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB3, (pb->r_x >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB4, (pb->r_y >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB5, (pb->r_y >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB6, (pb->g_x >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB7, (pb->g_x >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB8, (pb->g_y >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB9, (pb->g_y >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB10, (pb->b_x >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB11, (pb->b_x >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB12, (pb->b_y >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB13, (pb->b_y >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB14, (pb->w_x >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB15, (pb->w_x >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB16, (pb->w_y >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB17, (pb->w_y >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB18, (pb->luma_max >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB19, (pb->luma_max >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB20, (pb->luma_min >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB21, (pb->luma_min >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB22, (pb->mcll >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB23, (pb->mcll >> 8) & 0xff);
		dev_write(dev, FC_DRM_PB24, (pb->mfll >> 0) & 0xff);
		dev_write(dev, FC_DRM_PB25, (pb->mfll >> 8) & 0xff);
	 }
	dev_write_mask(dev, FC_DRM_HB0, FC_DRM_UP_FC_DRM_HB_MASK, 0x01);
	dev_write_mask(dev, FC_DRM_HB1, FC_DRM_UP_FC_DRM_HB_MASK, 26);
	dev_write_mask(dev, FC_PACKET_TX_EN, FC_PACKET_TX_EN_DRM_TX_EN_MASK, 0x1);
	do {
		snps_sleep(10);
		status = dev_read_mask(dev, FC_DRM_UP, FC_DRM_UP_DRMPACKETUPDATE_MASK);
	} while (status && (timeout--));
	dev_write_mask(dev, FC_DRM_UP,  FC_DRM_UP_DRMPACKETUPDATE_MASK, 0x1);
}

void fc_drm_disable(hdmi_tx_dev_t *dev)
{
	dev_write_mask(dev, FC_PACKET_TX_EN, FC_PACKET_TX_EN_DRM_TX_EN_MASK, 0x0);
}

u8 fc_Colorimetry_get(hdmi_tx_dev_t *dev)
{
	u8 colorimetry = 0;
	LOG_TRACE();
	colorimetry = dev_read_mask(dev,
			FC_AVICONF1,
			FC_AVICONF1_COLORIMETRY_MASK);
	if (colorimetry == 3)
		return (colorimetry + dev_read_mask(dev, FC_AVICONF2,
					FC_AVICONF2_EXTENDED_COLORIMETRY_MASK));


	return colorimetry;
}
u8 fc_RgbYcc_get(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	return dev_read_mask(dev, FC_AVICONF0,
			FC_AVICONF0_RGC_YCC_INDICATION_MASK);
}

u8 fc_VideoCode_get(hdmi_tx_dev_t *dev)
{
	LOG_TRACE();
	return dev_read_mask(dev, FC_AVIVID, FC_AVIVID_FC_AVIVID_MASK);
}

void fc_VideoCode_set(hdmi_tx_dev_t *dev, u8 data)
{
	LOG_TRACE();
	return dev_write_mask(dev, FC_AVIVID, FC_AVIVID_FC_AVIVID_MASK, data);
}

/*
* get vsif data
* data[0]: hdmi_format filed in vsif
* data[1]: hdmi_vic or 3d strcture filed in vsif
*/
void fc_vsif_get(hdmi_tx_dev_t *dev, u8 *data)
{
	data[0] = dev_read(dev, FC_VSDPAYLOAD0);
	data[1] = dev_read(dev, FC_VSDPAYLOAD0 + 0x4);
}

/*
* set vsif data
* data[0]: hdmi_format filed in vsif
* data[1]: hdmi_vic or 3d strcture filed in vsif
*/
void fc_vsif_set(hdmi_tx_dev_t *dev, u8 *data)
{
	 dev_write(dev, FC_VSDPAYLOAD0, data[0]);
	 dev_write(dev, FC_VSDPAYLOAD0 + 0x4, data[1]);
}

