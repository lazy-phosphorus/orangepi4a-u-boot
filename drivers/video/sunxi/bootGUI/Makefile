# SPDX-License-Identifier: GPL-2.0
obj-y	+= boot_gui.o
obj-y	+= fb_con.o
obj-y	+= canvas_utils.o
obj-y	+= dev_manage.o
obj-y	+= hdmi_manage.o
obj-y	+= video_hal.o
obj-y	+= video_misc_hal.o
obj-y	+= grallocator.o
obj-$(CONFIG_BOOT_GUI_TEST)   += boot_gui_test.o

ccflags-y += -I$(srctree)/$(obj)/../common/
