/*
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "edid.h"
#include "access.h"

#define EDID_I2C_ADDR		0x50
#define EDID_I2C_SEGMENT_ADDR	0x30

const unsigned DTD_SIZE = 0x12;

static supported_dtd_t _dtd[] = {
/* pclk  I/P Ha   Hb                          Va   Vb */
	{60000, {  1, 0, 0, 0, 25200, 0,  640,  160, 0,  4,   16,  96, 0,  480, 45, 0, 3, 10,  2, 0} },
	{59940, {  1, 0, 0, 0, 25175, 0,  640,  160, 0,  4,   16,  96, 0,  480, 45, 0, 3, 10,  2, 0} },
	{60000, {  2, 0, 0, 0, 27000, 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0} },
	{59940, {  2, 0, 0, 0, 27000, 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0} },
	{60000, {  3, 0, 0, 0, 27000, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{59940, {  3, 0, 0, 0, 27000, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{60000, {  4, 0, 0, 0, 74250, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{59940, {  4, 0, 0, 0, 74176, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{60000, {  5, 0, 0, 0, 74250, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{59940, {  5, 0, 0, 0, 74176, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{60000, {  6, 0, 0, 1, 27000, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{59940, {  6, 0, 0, 1, 27000, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{60000, {  7, 0, 0, 1, 27000, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{59940, {  7, 0, 0, 1, 27000, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{60000, {  8, 0, 0, 1, 27000, 0, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{60054, {  8, 0, 0, 1, 27000, 0, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{59826, {  8, 0, 0, 1, 27000, 0, 1440,  276, 0,  4,   38, 124, 0,  240, 23, 0, 3,  5,  3, 0} },
	{60000, {  9, 0, 0, 1, 27000, 0, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{60054, {  9, 0, 0, 1, 27000, 0, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{59826, {  9, 0, 0, 1, 27000, 0, 1440,  276, 0, 16,   38, 124, 0,  240, 23, 0, 9,  5,  3, 0} },
	{60000, { 10, 0, 0, 0, 54054, 1, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0} },
	{59940, { 11, 0, 0, 0, 54000, 1, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0} },
	{60000, { 12, 0, 0, 0, 54054, 0, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0} },
	{60054, { 12, 0, 0, 0, 54000, 0, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0} },
	{59826, { 12, 0, 0, 0, 54000, 0, 2880,  552, 0,  4,   76, 248, 0,  240, 23, 0, 3,  5,  3, 0} },
	{60000, { 13, 0, 0, 0, 54054, 0, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0} },
	{60054, { 13, 0, 0, 0, 54000, 0, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0} },
	{59826, { 13, 0, 0, 0, 54000, 0, 2880,  552, 0, 16,   76, 248, 0,  240, 23, 0, 9,  5,  3, 0} },
	{60000, { 14, 0, 0, 0, 54054, 0, 1440,  276, 0,  4,   32, 124, 0,  480, 45, 0, 3,  9,  6, 0} },
	{59940, { 14, 0, 0, 0, 54000, 0, 1440,  276, 0,  4,   32, 124, 0,  480, 45, 0, 3,  9,  6, 0} },
	{60000, { 15, 0, 0, 0, 54054, 0, 1440,  276, 0, 16,   32, 124, 0,  480, 45, 0, 9,  9,  6, 0} },
	{59940, { 15, 0, 0, 0, 54000, 0, 1440,  276, 0, 16,   32, 124, 0,  480, 45, 0, 9,  9,  6, 0} },
	{60000, { 16, 0, 0, 0, 148500, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{59940, { 16, 0, 0, 0, 148352, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{50000, { 17, 0, 0, 0, 27000, 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0} },
	{50000, { 18, 0, 0, 0, 27000, 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0} },
	{50000, { 19, 0, 0, 0, 74250, 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{50000, { 20, 0, 0, 0, 74250, 1, 1920,  720, 0, 16,  528,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{50000, { 21, 0, 0, 1, 27000, 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0} },
	{50000, { 22, 0, 0, 1, 27000, 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0} },
	{50000, { 23, 0, 0, 1, 27000, 0, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0} },
	{50000, { 23, 0, 0, 1, 27000, 0, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0} },
	{49920, { 23, 0, 0, 1, 27000, 0, 1440,  288, 0,  4,   24, 126, 0,  288, 25, 0, 3,  3,  3, 0} },
	{50000, { 24, 0, 0, 1, 27000, 0, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0} },
	{50000, { 24, 0, 0, 1, 27000, 0, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0} },
	{49920, { 24, 0, 0, 1, 27000, 0, 1440,  288, 0, 16,   24, 126, 0,  288, 25, 0, 9,  3,  3, 0} },
	{50000, { 25, 0, 0, 0, 54000, 1, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0} },
	{50000, { 26, 0, 0, 0, 54000, 1, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0} },
	{50000, { 27, 0, 0, 0, 54000, 0, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0} },
	{49920, { 27, 0, 0, 0, 54000, 0, 2880,  576, 0,  4,   48, 252, 0,  288, 25, 0, 3,  3,  3, 0} },
	{50000, { 27, 0, 0, 0, 54000, 0, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0} },
	{50000, { 28, 0, 0, 0, 54000, 0, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0} },
	{49920, { 28, 0, 0, 0, 54000, 0, 2880,  576, 0, 16,   48, 252, 0,  288, 25, 0, 9,  3,  3, 0} },
	{50000, { 28, 0, 0, 0, 54000, 0, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0} },
	{50000, { 29, 0, 0, 0, 54000, 0, 1440,  288, 0,  4,   24, 128, 0,  576, 49, 0, 3,  5,  5, 0} },
	{50000, { 30, 0, 0, 0, 54000, 0, 1440,  288, 0, 16,   24, 128, 0,  576, 49, 0, 9,  5,  5, 0} },
	{50000, { 31, 0, 0, 0, 148500, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{24000, { 32, 0, 0, 0, 74250, 0, 1920,  830, 0, 16,  638,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{23976, { 32, 0, 0, 0, 74176, 0, 1920,  830, 0, 16,  638,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{25000, { 33, 0, 0, 0, 74250, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{30000, { 34, 0, 0, 0, 74250, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{29970, { 34, 0, 0, 0, 74176, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{60000, { 35, 0, 0, 0, 108108, 0, 2880,  552, 0,  4,   64, 248, 0,  480, 45, 0, 3,  9,  6, 0} },
	{59940, { 35, 0, 0, 0, 108000, 0, 2880,  552, 0,  4,   64, 248, 0,  480, 45, 0, 3,  9,  6, 0} },
	{60000, { 36, 0, 0, 0, 108108, 0, 2880,  552, 0, 16,   64, 248, 0,  480, 45, 0, 9,  9,  6, 0} },
	{59940, { 36, 0, 0, 0, 108100, 0, 2880,  552, 0, 16,   64, 248, 0,  480, 45, 0, 9,  9,  6, 0} },
	{50000, { 37, 0, 0, 0, 108000, 0, 2880,  576, 0,  4,   48, 256, 0,  576, 49, 0, 3,  5,  5, 0} },
	{50000, { 38, 0, 0, 0, 108000, 0, 2880,  576, 0, 16,   48, 256, 0,  576, 49, 0, 9,  5,  5, 0} },
	{50000, { 39, 0, 0, 0, 72000, 1, 1920,  384, 0, 16,   32, 168, 1,  540, 85, 0, 9, 23,  5, 0} },
	{100000, { 40, 0, 0, 0, 148500, 1, 1920,  720, 0, 16,  528,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{100000, { 41, 0, 0, 0, 148500, 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{100000, { 42, 0, 0, 0, 54000, 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0} },
	{100000, { 43, 0, 0, 0, 54000, 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0} },
	{100000, { 44, 0, 0, 1, 54000, 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0} },
	{100000, { 45, 0, 0, 1, 54000, 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0} },
	{120000, { 46, 0, 0, 0, 148500, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{119880, { 46, 0, 0, 0, 148352, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1} },
	{120000, { 47, 0, 0, 0, 148500, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{120000, { 48, 0, 0, 0, 54054, 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0} },
	{120000, { 49, 0, 0, 0, 54054, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{119880, { 49, 0, 0, 0, 54000, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{120000, { 50, 0, 0, 1, 54054, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{119880, { 50, 0, 0, 1, 54000, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{120000, { 51, 0, 0, 1, 54054, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{119880, { 51, 0, 0, 1, 54000, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{200000, { 52, 0, 0, 0, 108000, 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0} },
	{200000, { 53, 0, 0, 0, 108000, 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0} },
	{200000, { 54, 0, 0, 1, 108000, 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0} },
	{200000, { 55, 0, 0, 1, 108000, 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0} },
	{240000, { 56, 0, 0, 0, 108100, 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0} },
	{240000, { 57, 0, 0, 0, 108100, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{239760, { 57, 0, 0, 0, 108000, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0} },
	{240000, { 58, 0, 0, 1, 108100, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{239760, { 58, 0, 0, 1, 108000, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0} },
	{240000, { 59, 0, 0, 1, 108100, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{239760, { 59, 0, 0, 1, 108000, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0} },
	{24000, { 60, 0, 0, 0, 59400, 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{23970, { 60, 0, 0, 0, 59341, 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{25000, { 61, 0, 0, 0, 74250, 0, 1280, 2680, 0, 16, 2420,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{30000, { 62, 0, 0, 0, 74250, 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{29970, { 62, 0, 0, 0, 74176, 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{120000, { 63, 0, 0, 0, 297000, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{119880, { 63, 0, 0, 0, 296703, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{100000, { 64, 0, 0, 0, 297000, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{50000, { 68, 0, 0, 0, 74250, 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{60000, { 69, 0, 0, 0, 74250, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{50000, { 75, 0, 0, 0, 148500, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{60000, { 76, 0, 0, 0, 148500, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{24000, { 93, 0, 0, 0, 297000, 0, 3840, 1660, 0, 16, 1276,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{25000, { 94, 0, 0, 0, 297000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{30000, { 95, 0, 0, 0, 297000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{50000, { 96, 0, 0, 0, 594000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{60000, { 97, 0, 0, 0, 594000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{24000, { 98, 0, 0, 0, 297000, 0, 4096, 1404, 0, 16, 1020,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{25000, { 99, 0, 0, 0, 297000, 0, 4096, 1184, 0, 16, 968,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{30000, { 100, 0, 0, 0, 297000, 0, 4096, 304, 0, 16, 88,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{50000, { 101, 0, 0, 0, 594000, 0, 4096, 1184, 0, 16, 968,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{60000, { 102, 0, 0, 0, 594000, 0, 4096, 304, 0, 16, 88,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{30000, { 103, 0, 0, 0, 297000, 0, 3840, 1660, 0, 16, 1276,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{30000, { 104, 0, 0, 0, 297000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{30000, { 105, 0, 0, 0, 297000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{50000, { 106, 0, 0, 0, 594000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },
	{60000, { 107, 0, 0, 0, 594000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1} },

	/*For 3D*/
	{23976, {(32 + 0x80), 0, 0, 0, 148500, 0, 1920,  830, 0, 16,  638,  44, 1, 1080, 45, 0, 9,  4,  5, 1} },
	{50000, {(19 + 0x80), 0, 0, 0, 148500, 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1} },
	{60000, {(4 + 0x80), 0, 0, 0, 148500, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1} },

	/*for vesa format*/
	{60000, {0x201, 0, 0, 0, 233500, 0, 2560, 928, 0, 16, 192, 272, 0, 1440, 53, 0, 9, 3, 5, 1} },
	{70000, {0x202, 0, 0, 0, 280000, 0, 1440, 110, 0, 9, 68, 16, 1, 2560, 16, 0, 16, 6, 6, 1} },
	{60000, {0x203, 0, 0, 0, 142250, 0, 1080, 132, 0, 9, 64, 4, 1, 1920, 36, 0, 16, 16, 4, 0} },

	/************************* for vesa format ********************************/
	/* 640x480@60Hz */
	{60000, {0x209, 0, 0, 0, 25175,  0,  640, 144, 0,  4,   8,  96, 0,  480,  29, 0,  3, 2, 2, 0} },
	/* 800x600@60Hz */
	{60000, {0x204, 0, 0, 0, 40000,  0,  800, 256, 0,  4,  40, 128, 1,  600,  28, 0,  3, 1, 4, 1} },
	/* 800x600@56Hz */
	{56000, {0x205, 0, 0, 0, 36000,  0,  800, 224, 0,  4,  24,  72, 1,  600,  25, 0,  3, 1, 2, 1} },
	/* 1024x768@60Hz */
	{60000, {0x20f, 0, 0, 0, 65000,  0, 1024, 320, 0,  4,  24, 136, 0,  768,  38, 0,  3, 3, 6, 0} },
	/* 1024x768@70Hz */
	{70000, {0x20e, 0, 0, 0, 75000,  0, 1024, 304, 0,  4,  24, 136, 0,  768,  38, 0,  3, 3, 6, 0} },
	/* 1152x864@75Hz */
	{75000, {0x215, 0, 0, 0, 108000, 0, 1152, 448, 0,  4,  64, 128, 1,  864,  36, 0,  3, 1, 3, 1} },
	/* 1280x768@60Hz */
	{60000, {0x216, 0, 0, 0, 68250,  0, 1280, 160, 0,  5,  48,  32, 1,  768,  22, 0,  4, 3, 7, 0} },
	/* 1280x800@60Hz */
	{60000, {0x217, 0, 0, 0, 71000,  0, 1280, 160, 0, 16,  48,  32, 1,  800,  23, 0, 10, 3, 6, 0} },
	/* 1280x960@60Hz */
	{60000, {0x218, 0, 0, 0, 108000, 0, 1280, 520, 0,  4,  96, 112, 1,  960,  40, 0,  3, 1, 3, 1} },
	/* 1280x1024@60Hz */
	{60000, {0x219, 0, 0, 0, 108000, 0, 1280, 408, 0,  5,  48, 112, 1, 1024,  42, 0,  4, 1, 3, 1} },
	/* 1360x768@60Hz */
	{60000, {0x21a, 0, 0, 0, 85500,  0, 1360, 432, 0, 16,  64, 112, 1, 768,   27, 0,  9, 3, 6, 1} },
	/* 1366x768@60Hz */
	{60000, {0x21b, 0, 0, 0, 72000,  0, 1366, 134, 0, 16,  14, 56,  1, 768,   32, 0,  9, 1, 3, 1} },
	/* 1400x1050@60Hz */
	{60000, {0x21c, 0, 0, 0, 101000, 0, 1400, 160, 0,  4,  48, 32,  1, 1050,  30, 0,  3, 3, 4, 0} },
	/* 1440x900@60Hz */
	{60000, {0x21d, 0, 0, 0, 88750,  0, 1440, 160, 0, 16,  48, 32,  1, 900, 26, 0,  10, 3, 6, 0} },
	/* 1600x900@60Hz */
	{60000, {0x21e, 0, 0, 0, 108000, 0, 1600, 200, 0, 16,  24, 80,  1,  900, 100, 0,  9, 1, 3, 1} },
	/* 1680x1050@60Hz */
	{60000, {0x21f, 0, 0, 0, 119000, 0, 1680, 160, 0, 16,  48, 32,  1, 1050,  30, 0, 10, 3, 6, 0} },
	/* 1920x1200@60Hz */
	{60000, {0x220, 0, 0, 0, 154000, 0, 1920, 160, 0, 16,  48, 32,  1, 1200,  35, 0, 10, 3, 6, 0} },
	/* 1920x1440@60Hz */
	{60000, {0x221, 0, 0, 0, 234000, 0, 1920, 680, 0, 4,  128, 208, 0, 1440, 60, 0, 3, 1, 3, 1} },

	{    0, {    0, 0, 0, 0,      0, 0,    0,   0, 0,  0,   0,   0, 0,    0,   0, 0,  0, 0, 0, 0} },
};

#if defined(__LINUX_PLAT__)

static speaker_alloc_code_t alloc_codes[] = {
		{1,  0},
		{3,  1},
		{5,  2},
		{7,  3},
		{17, 4},
		{19, 5},
		{21, 6},
		{23, 7},
		{9,  8},
		{11, 9},
		{13, 10},
		{15, 11},
		{25, 12},
		{27, 13},
		{29, 14},
		{31, 15},
		{73, 16},
		{75, 17},
		{77, 18},
		{79, 19},
		{33, 20},
		{35, 21},
		{37, 22},
		{39, 23},
		{49, 24},
		{51, 25},
		{53, 26},
		{55, 27},
		{41, 28},
		{43, 29},
		{45, 30},
		{47, 31},
		{0, 0}
};


int dtd_parse(hdmi_tx_dev_t *dev, dtd_t *dtd, u8 data[18])
{
	/* LOG_TRACE(); */

	dtd->mCode = -1;
	dtd->mPixelRepetitionInput = 0;
	dtd->mLimitedToYcc420 = 0;
	dtd->mYcc420 = 0;

	dtd->mPixelClock = 1000 * byte_to_word(data[1], data[0]);/*[10000Hz]*/
	if (dtd->mPixelClock < 0x01) {	/* 0x0000 is defined as reserved */
		return false;
	}

	dtd->mHActive = concat_bits(data[4], 4, 4, data[2], 0, 8);
	dtd->mHBlanking = concat_bits(data[4], 0, 4, data[3], 0, 8);
	dtd->mHSyncOffset = concat_bits(data[11], 6, 2, data[8], 0, 8);
	dtd->mHSyncPulseWidth = concat_bits(data[11], 4, 2, data[9], 0, 8);
	dtd->mHImageSize = concat_bits(data[14], 4, 4, data[12], 0, 8);
	dtd->mHBorder = data[15];

	dtd->mVActive = concat_bits(data[7], 4, 4, data[5], 0, 8);
	dtd->mVBlanking = concat_bits(data[7], 0, 4, data[6], 0, 8);
	dtd->mVSyncOffset = concat_bits(data[11], 2, 2, data[10], 4, 4);
	dtd->mVSyncPulseWidth = concat_bits(data[11], 0, 2, data[10], 0, 4);
	dtd->mVImageSize = concat_bits(data[14], 0, 4, data[13], 0, 8);
	dtd->mVBorder = data[16];

	if (bit_field(data[17], 4, 1) != 1) {/*if not DIGITAL SYNC SIGNAL DEF*/
		HDMI_ERROR_MSG("Invalid DTD Parameters\n");
		return false;
	}
	if (bit_field(data[17], 3, 1) != 1) {/*if not DIGITAL SEPATATE SYNC*/
		HDMI_ERROR_MSG("Invalid DTD Parameters\n");
		return false;
	}
	/* no stereo viewing support in HDMI */
	dtd->mInterlaced = bit_field(data[17], 7, 1) == 1;
	dtd->mVSyncPolarity = bit_field(data[17], 2, 1) == 1;
	dtd->mHSyncPolarity = bit_field(data[17], 1, 1) == 1;
	return true;
}
#endif
/**
 * @short Get the DTD structure that contains the video parameters
 * @param[in] code VIC code to search for
 * @param[in] refreshRate  1HZ * 1000
 * @return returns a pointer to the DTD structure or NULL if not supported.
 * If refreshRate=0 then the first (default)
 *parameters are returned for the VIC code.
 */
dtd_t *get_dtd(u32 code, u32 refreshRate)
{
	int i = 0;

	for (i = 0; _dtd[i].dtd.mCode != 0; i++) {
		if (_dtd[i].dtd.mCode == code) {
			if (refreshRate == 0)
				return &_dtd[i].dtd;

			if (refreshRate == _dtd[i].refresh_rate)
				return &_dtd[i].dtd;
		}
	}
	return NULL;
}

int dtd_fill(hdmi_tx_dev_t *dev, dtd_t *dtd, u32 code, u32 refreshRate)
{
	dtd_t *p_dtd = NULL;

	HDMI_INFO_MSG("vic mode=%d\n", code);
	p_dtd = get_dtd(code, refreshRate);
	if (p_dtd == NULL) {
		HDMI_INFO_MSG("VIC code [%d] with refresh rate [%dHz] is not supported\n", code, refreshRate);
		return false;
	}
	p_dtd->mLimitedToYcc420 = false;
	p_dtd->mYcc420 = false;

	memcpy(dtd, p_dtd, sizeof(dtd_t));

	return true;
}

/*refresh rate unit: 1HZ * 1000  */
u32 dtd_get_refresh_rate(dtd_t *dtd)
{
	int i = 0;

	for (i = 0; _dtd[i].dtd.mCode != 0; i++) {
		if (_dtd[i].dtd.mCode == dtd->mCode) {
			if ((dtd->mPixelClock == 0) ||
				(_dtd[i].dtd.mPixelClock == dtd->mPixelClock))
				return _dtd[i].refresh_rate;
		}
	}
	return 0;
}
#if defined(__LINUX_PLAT__)

void monitor_range_limits_reset(hdmi_tx_dev_t *dev, monitorRangeLimits_t *mrl)
{
	mrl->mMinVerticalRate = 0;
	mrl->mMaxVerticalRate = 0;
	mrl->mMinHorizontalRate = 0;
	mrl->mMaxHorizontalRate = 0;
	mrl->mMaxPixelClock = 0;
	mrl->mValid = FALSE;
}

void colorimetry_data_block_reset(hdmi_tx_dev_t *dev,
					colorimetryDataBlock_t *cdb)
{
	cdb->mByte3 = 0;
	cdb->mByte4 = 0;
	cdb->mValid = false;
}


int colorimetry_data_block_parse(hdmi_tx_dev_t *dev,
				colorimetryDataBlock_t *cdb, u8 *data)
{
	LOG_TRACE();
	colorimetry_data_block_reset(dev, cdb);
	if ((data != 0) && (bit_field(data[0], 0, 5) == 0x03) &&
		(bit_field(data[0], 5, 3) == 0x07)
			&& (bit_field(data[1], 0, 7) == 0x05)) {
		cdb->mByte3 = data[2];
		cdb->mByte4 = data[3];
		cdb->mValid = true;
		return true;
	}
	return false;
}

void hdr_metadata_data_block_reset(hdmi_tx_dev_t *dev,
		struct hdr_static_metadata_data_block *hdr_metadata)
{
	memset(hdr_metadata, 0, sizeof(struct hdr_static_metadata_data_block));
}

int hdr_static_metadata_block_parse(hdmi_tx_dev_t *dev,
		struct hdr_static_metadata_data_block *hdr_metadata, u8 *data)
{
	LOG_TRACE();
	hdr_metadata_data_block_reset(dev, hdr_metadata);
	if ((data != 0) && (bit_field(data[0], 0, 5) > 1)
		&& (bit_field(data[0], 5, 3) == 0x07)
		&& (data[1] == 0x06)) {
		hdr_metadata->et_n = bit_field(data[2], 0, 5);
		hdr_metadata->sm_n = data[3];

		if (bit_field(data[0], 0, 5) > 3)
			hdr_metadata->dc_max_lum_data = data[4];
		if (bit_field(data[0], 0, 5) > 4)
			hdr_metadata->dc_max_fa_lum_data = data[5];
		if (bit_field(data[0], 0, 5) > 5)
			hdr_metadata->dc_min_lum_data = data[6];

		return true;
	}
	return false;
}

void hdmiforumvsdb_reset(hdmi_tx_dev_t *dev, hdmiforumvsdb_t *forumvsdb)
{
	forumvsdb->mValid = FALSE;
	forumvsdb->mIeee_Oui = 0;
	forumvsdb->mVersion = 0;
	forumvsdb->mMaxTmdsCharRate = 0;
	forumvsdb->mSCDC_Present = FALSE;
	forumvsdb->mRR_Capable = FALSE;
	forumvsdb->mLTS_340Mcs_scramble = FALSE;
	forumvsdb->mIndependentView = FALSE;
	forumvsdb->mDualView = FALSE;
	forumvsdb->m3D_OSD_Disparity = FALSE;
	forumvsdb->mDC_30bit_420 = FALSE;
	forumvsdb->mDC_36bit_420 = FALSE;
	forumvsdb->mDC_48bit_420 = FALSE;
}

int hdmiforumvsdb_parse(hdmi_tx_dev_t *dev, hdmiforumvsdb_t *forumvsdb,
								u8 *data)
{
	u16 blockLength;

	LOG_TRACE();
	hdmiforumvsdb_reset(dev, forumvsdb);
	if (data == 0)
		return FALSE;

	if (bit_field(data[0], 5, 3) != 0x3) {
		HDMI_ERROR_MSG("Invalid datablock tag\n");
		return FALSE;
	}
	blockLength = bit_field(data[0], 0, 5);
	if (blockLength < 7) {
		HDMI_ERROR_MSG("Invalid minimum length\n");
		return FALSE;
	}
	if (byte_to_dword(0x00, data[3], data[2], data[1]) !=
	    0xC45DD8) {
		HDMI_ERROR_MSG("HDMI IEEE registration identifier not valid\n");
		return FALSE;
	}
	forumvsdb->mVersion = bit_field(data[4], 0, 7);
	forumvsdb->mMaxTmdsCharRate = bit_field(data[5], 0, 7);
	forumvsdb->mSCDC_Present = bit_field(data[6], 7, 1);
	forumvsdb->mRR_Capable = bit_field(data[6], 6, 1);
	forumvsdb->mLTS_340Mcs_scramble = bit_field(data[6], 3, 1);
	forumvsdb->mIndependentView = bit_field(data[6], 2, 1);
	forumvsdb->mDualView = bit_field(data[6], 1, 1);
	forumvsdb->m3D_OSD_Disparity = bit_field(data[6], 0, 1);
	forumvsdb->mDC_30bit_420 = bit_field(data[7], 2, 1);
	forumvsdb->mDC_36bit_420 = bit_field(data[7], 1, 1);
	forumvsdb->mDC_48bit_420 = bit_field(data[7], 0, 1);
	forumvsdb->mValid = TRUE;

#if 1
	HDMI_INFO_MSG("version %d\n", bit_field(data[4], 0, 7));
	HDMI_INFO_MSG("Max_TMDS_Charater_rate %d\n",
		    bit_field(data[5], 0, 7));
	HDMI_INFO_MSG("SCDC_Present %d\n", bit_field(data[6], 7, 1));
	HDMI_INFO_MSG("RR_Capable %d\n", bit_field(data[6], 6, 1));
	HDMI_INFO_MSG("LTE_340Mcsc_scramble %d\n",
		    bit_field(data[6], 3, 1));
	HDMI_INFO_MSG("Independent_View %d\n", bit_field(data[6], 2, 1));
	HDMI_INFO_MSG("Dual_View %d\n", bit_field(data[6], 1, 1));
	HDMI_INFO_MSG("3D_OSD_Disparity %d\n", bit_field(data[6], 0, 1));
	HDMI_INFO_MSG("DC_48bit_420 %d\n", bit_field(data[7], 2, 1));
	HDMI_INFO_MSG("DC_36bit_420 %d\n", bit_field(data[7], 1, 1));
	HDMI_INFO_MSG("DC_30bit_420 %d\n", bit_field(data[7], 0, 1));
#endif
	return TRUE;
}

void hdmivsdb_reset(hdmi_tx_dev_t *dev, hdmivsdb_t *vsdb)
{
	int i, j = 0;

	vsdb->mPhysicalAddress = 0;
	vsdb->mSupportsAi = FALSE;
	vsdb->mDeepColor30 = FALSE;
	vsdb->mDeepColor36 = FALSE;
	vsdb->mDeepColor48 = FALSE;
	vsdb->mDeepColorY444 = FALSE;
	vsdb->mDviDual = FALSE;
	vsdb->mMaxTmdsClk = 0;
	vsdb->mVideoLatency = 0;
	vsdb->mAudioLatency = 0;
	vsdb->mInterlacedVideoLatency = 0;
	vsdb->mInterlacedAudioLatency = 0;
	vsdb->mId = 0;
	vsdb->mContentTypeSupport = 0;
	vsdb->mHdmiVicCount = 0;
	for (i = 0; i < MAX_HDMI_VIC; i++)
		vsdb->mHdmiVic[i] = 0;

	vsdb->m3dPresent = FALSE;
	for (i = 0; i < MAX_VIC_WITH_3D; i++) {
		for (j = 0; j < MAX_HDMI_3DSTRUCT; j++)
			vsdb->mVideo3dStruct[i][j] = 0;
	}
	for (i = 0; i < MAX_VIC_WITH_3D; i++) {
		for (j = 0; j < MAX_HDMI_3DSTRUCT; j++)
			vsdb->mDetail3d[i][j] = ~0;
	}
	vsdb->mValid = FALSE;
}

int edid_parser_CeaExtReset(hdmi_tx_dev_t *dev, sink_edid_t *edidExt)
{
	unsigned i = 0;

	edidExt->edid_m20Sink = false;
#if 1
	for (i = 0; i < sizeof(edidExt->edid_mMonitorName); i++)
		edidExt->edid_mMonitorName[i] = 0;

	edidExt->edid_mBasicAudioSupport = false;
	edidExt->edid_mUnderscanSupport = false;
	edidExt->edid_mYcc422Support = false;
	edidExt->edid_mYcc444Support = false;
	edidExt->edid_mYcc420Support = false;
	edidExt->edid_mDtdIndex = 0;
	edidExt->edid_mSadIndex = 0;
	edidExt->edid_mSvdIndex = 0;
#endif
	hdmivsdb_reset(dev, &edidExt->edid_mHdmivsdb);
	hdmiforumvsdb_reset(dev, &edidExt->edid_mHdmiForumvsdb);
	monitor_range_limits_reset(dev, &edidExt->edid_mMonitorRangeLimits);
	video_cap_data_block_reset(dev,
				&edidExt->edid_mVideoCapabilityDataBlock);
	colorimetry_data_block_reset(dev, &edidExt->edid_mColorimetryDataBlock);
	hdr_metadata_data_block_reset(dev, &edidExt->edid_hdr_static_metadata_data_block);
	speaker_alloc_data_block_reset(dev,
				&edidExt->edid_mSpeakerAllocationDataBlock);
	return true;
}


int hdmivsdb_parse(hdmi_tx_dev_t *dev, hdmivsdb_t *vsdb, u8 *data)
{
	u8 blockLength = 0;
	unsigned videoInfoStart = 0;
	unsigned hdmi3dStart = 0;
	unsigned hdmiVicLen = 0;
	unsigned hdmi3dLen = 0;
	unsigned spanned3d = 0;
	unsigned i = 0;
	unsigned j = 0;

	LOG_TRACE();
	hdmivsdb_reset(dev, vsdb);
	if (data == 0)
		return FALSE;

	if (bit_field(data[0], 5, 3) != 0x3) {
		HDMI_ERROR_MSG("Invalid datablock tag\n");
		return FALSE;
	}
	blockLength = bit_field(data[0], 0, 5);
	if (blockLength < 5) {
		HDMI_ERROR_MSG("Invalid minimum length\n");
		return FALSE;
	}
	if (byte_to_dword(0x00, data[3], data[2], data[1]) != 0x000C03) {
		HDMI_ERROR_MSG("HDMI IEEE registration identifier not valid\n");
		return FALSE;
	}
	hdmivsdb_reset(dev, vsdb);
	vsdb->mId = 0x000C03;
	vsdb->mPhysicalAddress = byte_to_word(data[4], data[5]);
	/* parse extension fields if they exist */
	if (blockLength > 5) {
		vsdb->mSupportsAi = bit_field(data[6], 7, 1) == 1;
		vsdb->mDeepColor48 = bit_field(data[6], 6, 1) == 1;
		vsdb->mDeepColor36 = bit_field(data[6], 5, 1) == 1;
		vsdb->mDeepColor30 = bit_field(data[6], 4, 1) == 1;
		vsdb->mDeepColorY444 = bit_field(data[6], 3, 1) == 1;
		vsdb->mDviDual = bit_field(data[6], 0, 1) == 1;
	} else {
		vsdb->mSupportsAi = FALSE;
		vsdb->mDeepColor48 = FALSE;
		vsdb->mDeepColor36 = FALSE;
		vsdb->mDeepColor30 = FALSE;
		vsdb->mDeepColorY444 = FALSE;
		vsdb->mDviDual = FALSE;
	}
	vsdb->mMaxTmdsClk = (blockLength > 6) ? data[7] : 0;
	vsdb->mVideoLatency = 0;
	vsdb->mAudioLatency = 0;
	vsdb->mInterlacedVideoLatency = 0;
	vsdb->mInterlacedAudioLatency = 0;
	if (blockLength > 7) {
		if (bit_field(data[8], 7, 1) == 1) {
			if (blockLength < 10) {
				HDMI_ERROR_MSG("Invalid length - latencies are not valid\n");
				return FALSE;
			}
			if (bit_field(data[8], 6, 1) == 1) {
				if (blockLength < 12) {
					HDMI_ERROR_MSG("Invalid length - Interlaced latencies are not valid\n");
					return FALSE;
				} else {
					vsdb->mVideoLatency = data[9];
					vsdb->mAudioLatency = data[10];
					vsdb->mInterlacedVideoLatency
								= data[11];
					vsdb->mInterlacedAudioLatency
								= data[12];
					videoInfoStart = 13;
				}
			} else {
				vsdb->mVideoLatency = data[9];
				vsdb->mAudioLatency = data[10];
				vsdb->mInterlacedVideoLatency = 0;
				vsdb->mInterlacedAudioLatency = 0;
				videoInfoStart = 11;
			}
		} else {	/* no latency data */
			vsdb->mVideoLatency = 0;
			vsdb->mAudioLatency = 0;
			vsdb->mInterlacedVideoLatency = 0;
			vsdb->mInterlacedAudioLatency = 0;
			videoInfoStart = 9;
		}
		vsdb->mContentTypeSupport = bit_field(data[8], 0, 4);
	}
	/* additional video format capabilities are described */
	if (bit_field(data[8], 5, 1) == 1) {
		vsdb->mImageSize = bit_field(data[videoInfoStart], 3, 2);
		hdmiVicLen = bit_field(data[videoInfoStart + 1], 5, 3);
		hdmi3dLen = bit_field(data[videoInfoStart + 1], 0, 5);
		for (i = 0; i < hdmiVicLen; i++)
			vsdb->mHdmiVic[i] = data[videoInfoStart + 2 + i];

		vsdb->mHdmiVicCount = hdmiVicLen;
		if (bit_field(data[videoInfoStart], 7, 1) == 1) {/*3d present*/
			vsdb->m3dPresent = TRUE;
			hdmi3dStart = videoInfoStart + hdmiVicLen + 2;
			/* 3d multi 00 -> both 3d_structure_all
			and 3d_mask_15 are NOT present */
			/* 3d mutli 11 -> reserved */
			if (bit_field(data[videoInfoStart], 5, 2) == 1) {
				/* 3d multi 01 */
				/* 3d_structure_all is present but 3d_mask_15 not present */
				for (j = 0; j < 16; j++) {
					/* j spans 3d structures */
					if (bit_field(data[hdmi3dStart
						+ (j / 8)], (j % 8), 1) == 1) {
						for (i = 0; i < 16; i++)
							vsdb->mVideo3dStruct[i][(j < 8)	? j+8 : j - 8] = 1;
					}
				}
				spanned3d = 2;
				/*hdmi3dStart += 2;
				   hdmi3dLen -= 2; */
			} else if (bit_field(data[videoInfoStart], 5, 2) == 2) {
				/* 3d multi 10 */
				/* 3d_structure_all and 3d_mask_15 are present */
				for (j = 0; j < 16; j++) {
					for (i = 0; i < 16; i++) {
						if (bit_field(data[hdmi3dStart + 2 + (i / 8)], (i % 8), 1) == 1)
							vsdb->mVideo3dStruct[(i < 8) ? i + 8 : i - 8][(j < 8) ? j + 8 : j - 8] = bit_field(data[hdmi3dStart + (j / 8)], (j % 8), 1);
					}
				}
				spanned3d = 4;
			}
			if (hdmi3dLen > spanned3d) {
				hdmi3dStart += spanned3d;
				for (i = 0, j = 0; i < (hdmi3dLen - spanned3d); i++) {
					vsdb->mVideo3dStruct[bit_field(data[hdmi3dStart + i + j], 4, 4)][bit_field(data[hdmi3dStart + i + j], 0, 4)] = 1;
					if (bit_field(data[hdmi3dStart + i + j], 4, 4) > 7) {
						j++;
						vsdb->mDetail3d[bit_field(data[hdmi3dStart + i + j], 4, 4)][bit_field(data[hdmi3dStart + i + j], 4, 4)] = bit_field(data[hdmi3dStart + i + j], 4, 4);
					}
				}
			}
		} else {	/* 3d NOT present */
			vsdb->m3dPresent = FALSE;
		}
	}
	vsdb->mValid = TRUE;
	return TRUE;
}

void sad_reset(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	sad->mFormat = 0;
	sad->mMaxChannels = 0;
	sad->mSampleRates = 0;
	sad->mByte3 = 0;
}

int sad_parse(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad, u8 *data)
{
	LOG_TRACE();
	sad_reset(dev, sad);
	if (data != 0) {
		sad->mFormat = bit_field(data[0], 3, 4);
		sad->mMaxChannels = bit_field(data[0], 0, 3) + 1;
		sad->mSampleRates = bit_field(data[1], 0, 7);
		sad->mByte3 = data[2];
		return TRUE;
	}
	return FALSE;
}

int sad_support32k(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 0, 1) == 1) ? TRUE : FALSE;
}

int sad_support44k1(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 1, 1) == 1) ? TRUE : FALSE;
}

int sad_support48k(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 2, 1) == 1) ? TRUE : FALSE;
}

int sad_support88k2(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 3, 1) ==
		1) ? TRUE : FALSE;
}

int sad_support96k(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 4, 1) == 1) ? TRUE : FALSE;
}

int sad_support176k4(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 5, 1) == 1) ? TRUE : FALSE;
}

int sad_support192k(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	return (bit_field(sad->mSampleRates, 6, 1) == 1) ? TRUE : FALSE;
}

int sad_support16bit(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	if (sad->mFormat == 1)
		return (bit_field(sad->mByte3, 0, 1) == 1) ? TRUE : FALSE;

	HDMI_INFO_MSG("Information is not valid for this format\n");
	return FALSE;
}

int sad_support20bit(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	if (sad->mFormat == 1)
		return (bit_field(sad->mByte3, 1, 1) == 1) ? TRUE : FALSE;
	HDMI_INFO_MSG("Information is not valid for this format\n");
	return FALSE;
}

int sad_support24bit(hdmi_tx_dev_t *dev, shortAudioDesc_t *sad)
{
	if (sad->mFormat == 1)
		return (bit_field(sad->mByte3, 2, 1) == 1) ? TRUE : FALSE;
	HDMI_INFO_MSG("Information is not valid for this format\n");
	return FALSE;
}

void svd_reset(hdmi_tx_dev_t *dev, shortVideoDesc_t *svd)
{
	svd->mNative = FALSE;
	svd->mCode = 0;
}

int svd_parse(hdmi_tx_dev_t *dev, shortVideoDesc_t *svd, u8 data)
{
	svd_reset(dev, svd);
	svd->mNative = (bit_field(data, 7, 1) == 1) ? TRUE : FALSE;
	svd->mCode = bit_field(data, 0, 7);
	svd->mLimitedToYcc420 = 0;
	svd->mYcc420 = 0;
	return TRUE;
}


void speaker_alloc_data_block_reset(hdmi_tx_dev_t *dev,
				speakerAllocationDataBlock_t *sadb)
{
	sadb->mByte1 = 0;
	sadb->mValid = FALSE;
}

int speaker_alloc_data_block_parse(hdmi_tx_dev_t *dev,
				speakerAllocationDataBlock_t *sadb, u8 *data)
{
	LOG_TRACE();
	speaker_alloc_data_block_reset(dev, sadb);
	if ((data != 0) && (bit_field(data[0], 0, 5) == 0x03) &&
				(bit_field(data[0], 5, 3) == 0x04)) {
		sadb->mByte1 = data[1];
		sadb->mValid = TRUE;
		return TRUE;
	}
	return FALSE;
}

u8 get_channell_alloc_code(hdmi_tx_dev_t *dev,
			speakerAllocationDataBlock_t *sadb)
{
	int i = 0;

	for (i = 0; alloc_codes[i].byte != 0; i++) {
		if (sadb->mByte1 == alloc_codes[i].byte)
			return alloc_codes[i].code;
	}
	return (u8)-1;
}

void video_cap_data_block_reset(hdmi_tx_dev_t *dev,
					videoCapabilityDataBlock_t *vcdb)
{
	vcdb->mQuantizationRangeSelectable = FALSE;
	vcdb->mPreferredTimingScanInfo = 0;
	vcdb->mItScanInfo = 0;
	vcdb->mCeScanInfo = 0;
	vcdb->mValid = FALSE;
}

int video_cap_data_block_parse(hdmi_tx_dev_t *dev,
			videoCapabilityDataBlock_t *vcdb, u8 *data)
{
	LOG_TRACE();
	video_cap_data_block_reset(dev, vcdb);
	/* check tag code and extended tag */
	if ((data != 0) && (bit_field(data[0], 5, 3) == 0x7) &&
		(bit_field(data[1], 0, 8) == 0x0) &&
			(bit_field(data[0], 0, 5) == 0x2)) {
		/* so far VCDB is 2 bytes long */
		vcdb->mCeScanInfo = bit_field(data[2], 0, 2);
		vcdb->mItScanInfo = bit_field(data[2], 2, 2);
		vcdb->mPreferredTimingScanInfo = bit_field(data[2], 4, 2);
		vcdb->mQuantizationRangeSelectable =
				(bit_field(data[2], 6, 1) == 1) ? TRUE : FALSE;
		vcdb->mValid = TRUE;
		return TRUE;
	}
	return FALSE;
}


int _edid_checksum(u8 *edid)
{
	int i, checksum = 0;

	for (i = 0; i < EDID_LENGTH; i++)
		checksum += edid[i];

	return checksum % 256; /* CEA-861 Spec */
}

int edid_read(hdmi_tx_dev_t *dev, struct edid *edid)
{
	int error = 0;
	const u8 header[] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};

	i2cddc_fast_mode(dev, 0);
	i2cddc_clk_config(dev, I2C_SFR_CLK, I2C_MIN_SS_SCL_LOW_TIME,
			I2C_MIN_SS_SCL_HIGH_TIME, I2C_MIN_FS_SCL_LOW_TIME,
			I2C_MIN_FS_SCL_HIGH_TIME);
	/* i2cddc_clk_config(dev, 5000, I2C_MIN_SS_SCL_LOW_TIME,
		I2C_MIN_SS_SCL_HIGH_TIME, I2C_MIN_FS_SCL_LOW_TIME,
		I2C_MIN_FS_SCL_HIGH_TIME); */
	/* i2cddc_clk_config(dev, 4700, I2C_MIN_SS_SCL_LOW_TIME,
		I2C_MIN_SS_SCL_HIGH_TIME, I2C_MIN_FS_SCL_LOW_TIME,
		I2C_MIN_FS_SCL_HIGH_TIME); */

	error = ddc_read(dev, EDID_I2C_ADDR, EDID_I2C_SEGMENT_ADDR,
						0, 0, 128, (u8 *)edid);
	if (error) {
		HDMI_ERROR_MSG("EDID read failed\n");
		return error;
	}
	error = memcmp((u8 *) edid, (u8 *) header, sizeof(header));
	if (error) {
		HDMI_ERROR_MSG("EDID header check failed\n");
		return error;
	}

	error = _edid_checksum((u8 *) edid);
	if (error) {
		HDMI_ERROR_MSG("EDID checksum failed\n");
		return error;
	}
	return 0;
}

int edid_extension_read(hdmi_tx_dev_t *dev, int block, u8 *edid_ext)
{
	int error = 0;
	/*to incorporate extensions we have to include the following
	- see VESA E-DDC spec. P 11 */
	u8 start_pointer = block / 2;
	/* pointer to segments of 256 bytes */
	u8 start_address = ((block % 2) * 0x80);
	/* offset in segment; first block 0-127; second 128-255 */

	error = ddc_read(dev, EDID_I2C_ADDR, EDID_I2C_SEGMENT_ADDR,
			start_pointer, start_address, 128, edid_ext);
	if (error) {
		HDMI_ERROR_MSG("EDID extension read failed");
		return error;
	}

	error = _edid_checksum(edid_ext);
	if (error) {
		HDMI_ERROR_MSG("EDID extension checksum failed");
		return error;
	}
	return 0;
}

static void edid_parser_updateYcc420(sink_edid_t *edidExt,
					u8 Ycc420All, u8 LimitedToYcc420All)
{
	u16 edid_cnt = 0;

	for (edid_cnt = 0; edid_cnt < edidExt->edid_mSvdIndex; edid_cnt++) {
		switch (edidExt->edid_mSvd[edid_cnt].mCode) {
		case 96:
		case 97:
		case 101:
		case 102:
		case 106:
		case 107:
			Ycc420All == 1 ?
				edidExt->edid_mSvd[edid_cnt].mYcc420 = Ycc420All : 0;
			LimitedToYcc420All == 1 ?
				edidExt->edid_mSvd[edid_cnt].mLimitedToYcc420 =
							LimitedToYcc420All : 0;
			break;
		}
	}
}


static int _edid_struture_parser(hdmi_tx_dev_t *dev, struct edid *edid,
						sink_edid_t *sink)
{
	int i;
	char *monitorName;

	if (edid->header[0] != 0) {
		HDMI_INFO_MSG("Invalid Header\n");
		return -1;
	}

	for (i = 0; i < 4; i++) {
		struct detailed_timing *detailed_timing = &(edid->detailed_timings[i]);

		if (detailed_timing->pixel_clock == 0) {
			struct detailed_non_pixel *npixel = &(detailed_timing->data.other_data);

			switch (npixel->type) {
			case EDID_DETAIL_MONITOR_NAME:
				monitorName = (char *) &(npixel->data.str.str);
				HDMI_INFO_MSG("Monitor name: %s\n", monitorName);
				break;
			case EDID_DETAIL_MONITOR_RANGE:
				break;

			}
		} else { /* Detailed Timing Definition */
			struct detailed_pixel_timing *ptiming = &(detailed_timing->data.pixel_data);

			HDMI_INFO_MSG(" time: %d\n", detailed_timing->pixel_clock * 10000);
			HDMI_INFO_MSG("hactive_hblank_hi: %d\n", ptiming->hactive_hblank_hi);
		}
	}
	return true;
}

static int edid_parser_ParseDataBlock(hdmi_tx_dev_t *dev, u8 *data,
						sink_edid_t *edidExt)
{
	u8 c = 0;
	shortAudioDesc_t tmpSad;
	shortVideoDesc_t tmpSvd;
	u8 tmpYcc420All = 0;
	u8 tmpLimitedYcc420All = 0;
	u32 ieeeId = 0;
	u8 extendedTag = 0;
	int i = 0;
	int edid_cnt = 0;
	int svdNr = 0;
	int icnt = 0;
	u8 tag = bit_field(data[0], 5, 3);
	u8 length = bit_field(data[0], 0, 5);

	tmpSvd.mLimitedToYcc420 = 0;
	tmpSvd.mYcc420 = 0;


	switch (tag) {
	case 0x1:		/* Audio Data Block */
		HDMI_INFO_MSG("EDID: Audio datablock parsing\n");
		for (c = 1; c < (length + 1); c += 3) {
			sad_parse(dev, &tmpSad, data + c);
			if (edidExt->edid_mSadIndex < (sizeof(edidExt->edid_mSad) / sizeof(shortAudioDesc_t)))
				edidExt->edid_mSad[edidExt->edid_mSadIndex++] = tmpSad;
			else
				HDMI_INFO_MSG("buffer full - SAD ignored\n");
		}
		break;
	case 0x2:		/* Video Data Block */
		HDMI_INFO_MSG("EDID: Video datablock parsing\n");
		for (c = 1; c < (length + 1); c++) {
			svd_parse(dev, &tmpSvd, data[c]);
			if (edidExt->edid_mSvdIndex < (sizeof(edidExt->edid_mSvd) / sizeof(shortVideoDesc_t)))
				edidExt->edid_mSvd[edidExt->edid_mSvdIndex++] = tmpSvd;
			else
				HDMI_INFO_MSG("buffer full - SVD ignored\n");
		}
		break;
	case 0x3:		/* Vendor Specific Data Block HDMI or HF */
		HDMI_INFO_MSG("EDID: VSDB HDMI and HDMI-F\n ");
		ieeeId = byte_to_dword(0x00, data[3], data[2], data[1]);
		if (ieeeId == 0x000C03) {	/* HDMI */
			if (hdmivsdb_parse(dev, &edidExt->edid_mHdmivsdb, data) != true) {
				HDMI_INFO_MSG("HDMI Vendor Specific Data Block corrupt\n");
				break;
			}
			HDMI_INFO_MSG("EDID HDMI VSDB parsed\n");
		} else {
			if (ieeeId == 0xC45DD8) {	/* HDMI-F */
				HDMI_INFO_MSG("Sink is HDMI 2.0 because haves HF-VSDB\n");
				edidExt->edid_m20Sink = true;
				if (hdmiforumvsdb_parse(dev, &edidExt->edid_mHdmiForumvsdb, data) != true) {
					HDMI_INFO_MSG("HDMI Vendor Specific Data Block corrupt\n");
					break;
				} else {
					HDMI_INFO_MSG("support hf-vsdb block!\n");
				}
			} else {
				HDMI_INFO_MSG("Vendor Specific Data Block not parsed ieeeId: 0x%x\n",
						ieeeId);
			}
		}
		break;
	case 0x4:		/* Speaker Allocation Data Block */
		HDMI_INFO_MSG("SAD block parsing");
		if (speaker_alloc_data_block_parse(dev, &edidExt->edid_mSpeakerAllocationDataBlock, data) != true)
			HDMI_INFO_MSG("Speaker Allocation Data Block corrupt\n");
		break;
	case 0x7:{
		HDMI_INFO_MSG("EDID CEA Extended field 0x07\n");
		extendedTag = data[1];
		switch (extendedTag) {
		case 0x00:	/* Video Capability Data Block */
			HDMI_INFO_MSG("Video Capability Data Block\n");
			if (video_cap_data_block_parse(dev, &edidExt->edid_mVideoCapabilityDataBlock, data) != true)
				HDMI_INFO_MSG("Video Capability Data Block corrupt\n");
			break;
		case 0x04:	/* HDMI Video Data Block */
			HDMI_INFO_MSG("HDMI Video Data Block\n");
			break;
		case 0x05:	/* Colorimetry Data Block */
			HDMI_INFO_MSG("Colorimetry Data Block");
			if (colorimetry_data_block_parse(dev, &edidExt->edid_mColorimetryDataBlock, data) != true)
				HDMI_INFO_MSG("Colorimetry Data Block corrupt\n");
			break;
		case 0x06:	/* HDR Static Metadata Data Block */
			HDMI_INFO_MSG("HDR Static Metadata Data Block");
			if (hdr_static_metadata_block_parse(dev, &edidExt->edid_hdr_static_metadata_data_block, data) != true)
				HDMI_INFO_MSG("HDR Static Metadata Data Block\n");
			break;
		case 0x12:	/* HDMI Audio Data Block */
			HDMI_INFO_MSG("HDMI Audio Data Block\n");
			break;
		case 0xe:
			/** If it is a YCC420 VDB then VICs can ONLY be displayed in YCC 4:2:0 */
			HDMI_INFO_MSG("YCBCR 4:2:0 Video Data Block\n");
			/** If Sink has YCC Datablocks it is HDMI 2.0 */
			edidExt->edid_m20Sink = true;
			tmpLimitedYcc420All = (bit_field(data[0], 0, 5) == 1 ? 1 : 0);
			edid_parser_updateYcc420(edidExt, tmpYcc420All, tmpLimitedYcc420All);
			for (i = 0; i < (bit_field(data[0], 0, 5) - 1); i++) {
				/** Length includes the tag byte*/
				tmpSvd.mCode = data[2 + i];
				tmpSvd.mNative = 0;
				tmpSvd.mLimitedToYcc420 = 1;
				for (edid_cnt = 0; edid_cnt < edidExt->edid_mSvdIndex; edid_cnt++) {
					if (edidExt->edid_mSvd[edid_cnt].mCode == tmpSvd.mCode) {
						edidExt->edid_mSvd[edid_cnt] =	tmpSvd;
						goto concluded;
					}
				}
				if (edidExt->edid_mSvdIndex <
					(sizeof(edidExt->edid_mSvd) /  sizeof(shortVideoDesc_t))) {
					edidExt->edid_mSvd[edidExt->edid_mSvdIndex] = tmpSvd;
					edidExt->edid_mSvdIndex++;
				} else {
					HDMI_INFO_MSG("buffer full - YCC 420 DTD ignored");
				}
concluded:;
			}
			break;
		case 0x0f:
			/** If it is a YCC420 CDB then VIC can ALSO be displayed in YCC 4:2:0 */
			edidExt->edid_m20Sink = true;
			HDMI_INFO_MSG("YCBCR 4:2:0 Capability Map Data Block");
			/* If YCC420 CMDB is bigger than 1, then there is SVD info to parse */
			if (bit_field(data[0], 0, 5) > 1) {
				for (icnt = 0; icnt < 8; icnt++) {
					/** Lenght includes the tag byte*/
					if ((bit_field(data[2], 1 >> icnt, 1) & 0x01)) {
						svdNr = icnt;
						tmpSvd.mCode = edidExt->edid_mSvd[svdNr - 1].mCode;
						tmpSvd.mYcc420 = 1;
						edidExt->edid_mSvd[svdNr - 1] = tmpSvd;
					}
				}
				/* Otherwise, all SVDs present at the Video Data Block support YCC420*/
			} else {
				tmpYcc420All = (bit_field(data[0], 0, 5) == 1 ? 1 : 0);
				edid_parser_updateYcc420(edidExt, tmpYcc420All, tmpLimitedYcc420All);
			}
			break;
		default:
			HDMI_INFO_MSG("Extended Data Block not parsed %d\n",
					extendedTag);
			break;
		}
		break;
	}
	default:
		HDMI_INFO_MSG("Data Block not parsed %d\n", tag);
		break;
	}
	return length + 1;
}


static int _edid_cea_extension_parser(hdmi_tx_dev_t *dev, u8 *buffer,
						sink_edid_t *edidExt)
{
	int i = 0;
	int c = 0;
	dtd_t tmpDtd;
	u8 offset = buffer[2];

	if (buffer[1] < 0x03) {
		HDMI_INFO_MSG("Invalid version for CEA Extension block,only rev 3 or higher is supported");
		return -1;
	}

	edidExt->edid_mYcc422Support = bit_field(buffer[3],	4, 1) == 1;
	edidExt->edid_mYcc444Support = bit_field(buffer[3],	5, 1) == 1;
	edidExt->edid_mBasicAudioSupport = bit_field(buffer[3], 6, 1) == 1;
	edidExt->edid_mUnderscanSupport = bit_field(buffer[3], 7, 1) == 1;
	if (offset != 4) {
		for (i = 4; i < offset; i += edid_parser_ParseDataBlock(dev, buffer + i, edidExt))
			;
	}
	/* last is checksum */
	for (i = offset, c = 0; i < (sizeof(buffer) - 1) && c < 6; i += DTD_SIZE, c++) {
		if (dtd_parse(dev, &tmpDtd, buffer + i) == true) {
			if (edidExt->edid_mDtdIndex < (sizeof(edidExt->edid_mDtd) / sizeof(dtd_t))) {
				edidExt->edid_mDtd[edidExt->edid_mDtdIndex++] = tmpDtd;
				HDMI_INFO_MSG("edid_mDtd code %d\n", edidExt->edid_mDtd[edidExt->edid_mDtdIndex].mCode);
				HDMI_INFO_MSG("edid_mDtd limited to Ycc420? %d\n", edidExt->edid_mDtd[edidExt->edid_mDtdIndex].mLimitedToYcc420);
				HDMI_INFO_MSG("edid_mDtd supports Ycc420? %d\n", edidExt->edid_mDtd[edidExt->edid_mDtdIndex].mYcc420);
			} else {
				HDMI_INFO_MSG("buffer full - DTD ignored\n");
			}
		}
	}
	return true;
}

int edid_parser(hdmi_tx_dev_t *dev, u8 *buffer, sink_edid_t *edidExt,
							u16 edid_size)
{
	int ret = 0;

	switch (buffer[0]) {
	case 0x00:
		ret = _edid_struture_parser(dev, (struct edid *) buffer,
								edidExt);
		break;
	case CEA_EXT:
		ret = _edid_cea_extension_parser(dev, buffer, edidExt);
		break;
	case VTB_EXT:
	case DI_EXT:
	case LS_EXT:
	case MI_EXT:
	default:
		HDMI_INFO_MSG("Block 0x%02x not supported\n", buffer[0]);
	}
	if (ret == true)
		return true;

	return false;
}
#endif
