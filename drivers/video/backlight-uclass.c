// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <backlight.h>

int backlight_enable(struct udevice *dev)
{
	const struct backlight_ops *ops = backlight_get_ops(dev);

	if (!ops->enable)
		return -ENOSYS;

	return ops->enable(dev);
}
int backlight_disable(struct udevice *dev)
{
	const struct backlight_ops *ops = backlight_get_ops(dev);

	if (!ops->disable)
		return -ENOSYS;

	return ops->disable(dev);
}

UCLASS_DRIVER(backlight) = {
	.id		= UCLASS_PANEL_BACKLIGHT,
	.name		= "backlight",
};
