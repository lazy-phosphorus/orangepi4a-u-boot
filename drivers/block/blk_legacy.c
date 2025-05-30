// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <linux/err.h>

struct blk_driver *blk_driver_lookup_type(int if_type)
{
	struct blk_driver *drv = ll_entry_start(struct blk_driver, blk_driver);
	const int n_ents = ll_entry_count(struct blk_driver, blk_driver);
	struct blk_driver *entry;

	for (entry = drv; entry != drv + n_ents; entry++) {
		if (if_type == entry->if_type)
			return entry;
	}

	/* Not found */
	return NULL;
}

static struct blk_driver *blk_driver_lookup_typename(const char *if_typename)
{
	struct blk_driver *drv = ll_entry_start(struct blk_driver, blk_driver);
	const int n_ents = ll_entry_count(struct blk_driver, blk_driver);
	struct blk_driver *entry;

	for (entry = drv; entry != drv + n_ents; entry++) {
		if (!strcmp(if_typename, entry->if_typename))
			return entry;
	}

	/* Not found */
	return NULL;
}

const char *blk_get_if_type_name(enum if_type if_type)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_get_if_type_name(if_type);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);

	return drv ? drv->if_typename : NULL;
}

/**
 * get_desc() - Get the block device descriptor for the given device number
 *
 * @drv:	Legacy block driver
 * @devnum:	Device number (0 = first)
 * @descp:	Returns block device descriptor on success
 * @return 0 on success, -ENODEV if there is no such device, -ENOSYS if the
 * driver does not provide a way to find a device, or other -ve on other
 * error.
 */
static int get_desc(struct blk_driver *drv, int devnum, struct blk_desc **descp)
{
	if (drv->desc) {
		if (devnum < 0 || devnum >= drv->max_devs)
			return -ENODEV;
		*descp = &drv->desc[devnum];
		return 0;
	}
	if (!drv->get_dev)
		return -ENOSYS;

	return drv->get_dev(devnum, descp);
}

#ifdef CONFIG_HAVE_BLOCK_DEVICE
int blk_list_part(enum if_type if_type)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_list_part(if_type);
#endif

	struct blk_driver *drv;
	struct blk_desc *desc;
	int devnum, ok;
	bool first = true;

	drv = blk_driver_lookup_type(if_type);
	if (!drv)
		return -ENOSYS;
	for (ok = 0, devnum = 0; devnum < drv->max_devs; ++devnum) {
		if (get_desc(drv, devnum, &desc))
			continue;
		if (desc->part_type != PART_TYPE_UNKNOWN) {
			++ok;
			if (!first)
				putc('\n');
			part_print(desc);
			first = false;
		}
	}
	if (!ok)
		return -ENODEV;

	return 0;
}

int blk_print_part_devnum(enum if_type if_type, int devnum)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_print_part_devnum(if_type, devnum);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int ret;

	if (!drv)
		return -ENOSYS;
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	if (desc->type == DEV_TYPE_UNKNOWN)
		return -ENOENT;
	part_print(desc);

	return 0;
}

void blk_list_devices(enum if_type if_type)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_list_devices(if_type);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int i;

	if (!drv)
		return;
	for (i = 0; i < drv->max_devs; ++i) {
		if (get_desc(drv, i, &desc))
			continue;
		if (desc->type == DEV_TYPE_UNKNOWN)
			continue;  /* list only known devices */
		printf("Device %d: ", i);
		dev_print(desc);
	}
}

int blk_print_device_num(enum if_type if_type, int devnum)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_print_device_num(if_type, devnum);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int ret;

	if (!drv)
		return -ENOSYS;
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	printf("\n%s device %d: ", drv->if_typename, devnum);
	dev_print(desc);

	return 0;
}

int blk_show_device(enum if_type if_type, int devnum)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_show_device(if_type, devnum);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int ret;

	if (!drv)
		return -ENOSYS;
	printf("\nDevice %d: ", devnum);
	if (devnum >= drv->max_devs) {
		puts("unknown device\n");
		return -ENODEV;
	}
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	dev_print(desc);

	if (desc->type == DEV_TYPE_UNKNOWN)
		return -ENOENT;

	return 0;
}
#endif /* CONFIG_HAVE_BLOCK_DEVICE */

struct blk_desc *blk_get_devnum_by_type(enum if_type if_type, int devnum)
{
	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;

	if (!drv)
		return NULL;

	if (get_desc(drv, devnum, &desc))
		return NULL;

	return desc;
}

int blk_dselect_hwpart(struct blk_desc *desc, int hwpart)
{
#ifdef CONFIG_AW_BLK
	if (desc->if_type == IF_TYPE_NVME)
		return aw_blk_dselect_hwpart(desc, hwpart);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(desc->if_type);

	if (!drv)
		return -ENOSYS;
	if (drv->select_hwpart)
		return drv->select_hwpart(desc, hwpart);

	return 0;
}

struct blk_desc *blk_get_devnum_by_typename(const char *if_typename, int devnum)
{
#ifdef CONFIG_AW_BLK
	if (0 == strcmp(if_typename, "nvme"))
		return aw_blk_get_devnum_by_typename(if_typename, devnum);
#endif

	static struct blk_desc *desc;
	static char cache_if_typename[16] = {0};
	if ((desc == NULL) || (cache_if_typename[0] == 0) ||
		(strncmp(if_typename, cache_if_typename, max(strlen(if_typename), strlen(cache_if_typename))))) {
		strncpy(cache_if_typename, if_typename, strlen(if_typename));
		struct blk_driver *drv = blk_driver_lookup_typename(if_typename);

		if (!drv)
			return NULL;

		if (get_desc(drv, devnum, &desc))
			return NULL;
	}
	return desc;
}

ulong blk_read_devnum(enum if_type if_type, int devnum, lbaint_t start,
		      lbaint_t blkcnt, void *buffer)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_read_devnum(if_type, devnum, start, blkcnt, buffer);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	ulong n;
	int ret;

	if (!drv)
		return -ENOSYS;
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	n = desc->block_read(desc, start, blkcnt, buffer);
	if (IS_ERR_VALUE(n))
		return n;

	return n;
}

ulong blk_write_devnum(enum if_type if_type, int devnum, lbaint_t start,
		       lbaint_t blkcnt, const void *buffer)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_write_devnum(if_type, devnum, start, blkcnt, buffer);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int ret;

	if (!drv)
		return -ENOSYS;
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	return desc->block_write(desc, start, blkcnt, buffer);
}

int blk_select_hwpart_devnum(enum if_type if_type, int devnum, int hwpart)
{
#ifdef CONFIG_AW_BLK
	if (if_type == IF_TYPE_NVME)
		return aw_blk_select_hwpart_devnum(if_type, devnum, hwpart);
#endif

	struct blk_driver *drv = blk_driver_lookup_type(if_type);
	struct blk_desc *desc;
	int ret;

	if (!drv)
		return -ENOSYS;
	ret = get_desc(drv, devnum, &desc);
	if (ret)
		return ret;
	return drv->select_hwpart(desc, hwpart);
}
