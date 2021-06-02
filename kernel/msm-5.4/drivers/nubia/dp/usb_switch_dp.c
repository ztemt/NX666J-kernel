/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/soc/zte/usb_switch_dp.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


struct dp_switch_priv {
	struct platform_device *pdev;
	struct device *dev;
	unsigned int usb_mode;
	bool is_enabled;
} *pswitcher = NULL;


/*
* switcher_function_switch_event - configure FSA switch position based on event
*
* @node - phandle node to fsa4480 device
* @event - fsa_function enum
*
* Returns int on whether the switch happened or not
*/
int switcher_switch_event(struct device_node *node,
		   enum switcher_function event)
{
	if (!pswitcher)
	  return -EINVAL;
	pswitcher->usb_mode = event;
	switch (event) {
	case SWITCHER_MIC_GND_SWAP:
		break;
	case SWITCHER_USBC_ORIENTATION_CC1:
	case SWITCHER_USBC_ORIENTATION_CC2:
		pswitcher->is_enabled = 1;
		break;
	case SWITCHER_USBC_DISPLAYPORT_DISCONNECTED:
		pswitcher->is_enabled = 0;
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(switcher_switch_event);

static ssize_t dp_cc_statue_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	switch (pswitcher->usb_mode) {
	case SWITCHER_USBC_ORIENTATION_CC1:
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	case SWITCHER_USBC_ORIENTATION_CC2:
		return snprintf(buf, PAGE_SIZE, "%d\n", 2);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t dp_cc_statue_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	return count;
}

static ssize_t dp_enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pswitcher->is_enabled);
}

static ssize_t dp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	switch (val) {
	case 0:
	case 1:
		break;
	default:
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, dp_enable_show, dp_enable_store);
static DEVICE_ATTR(cc_statue, S_IWUSR | S_IRUGO, dp_cc_statue_show, dp_cc_statue_store);

static struct attribute *usb_switch_dp_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_cc_statue.attr,
	NULL
};

static struct attribute_group usb_switch_dp_attribute_group = {
	.attrs = usb_switch_dp_attributes
};

static int  nubia_usb_switch_dp_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node;

	if (!pdev) {
		return -EPROBE_DEFER;
	}

	node = pdev->dev.of_node;
	pswitcher = kzalloc(sizeof(struct dp_switch_priv), GFP_KERNEL);
	if (!pswitcher) {
		dev_err(&pdev->dev, "cannot allocate device memory.\n");
		return -ENOMEM;
	}
	pswitcher->dev = &pdev->dev;
	pswitcher->pdev = pdev;
	pswitcher->is_enabled = 0;

	ret = sysfs_create_group(&pswitcher->dev->kobj, &usb_switch_dp_attribute_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s error creating sysfs attr files\n", __func__);
	}
	return 0;

}

static int nubia_usb_switch_dp_remove(struct platform_device *pdev)
{
	if (!pswitcher)
		return -EINVAL;
	sysfs_remove_group(&pswitcher->dev->kobj, &usb_switch_dp_attribute_group);
	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "nubia,usb_switch_dp" },
	{ }
};

static struct platform_driver usb_switch_dp_driver = {
	.probe = nubia_usb_switch_dp_probe,
	.remove = nubia_usb_switch_dp_remove,
	.driver = {
		.name = "nubia,usb_switch_dp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_match),
	},
};

int __init nubia_usb_switch_dp_init(void)
{
	platform_driver_register(&usb_switch_dp_driver);
	return 0;
}

static void __exit nubia_usb_switch_dp_exit(void)
{
	platform_driver_unregister(&usb_switch_dp_driver);
}

module_init(nubia_usb_switch_dp_init);
module_exit(nubia_usb_switch_dp_exit);

MODULE_DESCRIPTION("nubia_dp driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nubia_dp");

