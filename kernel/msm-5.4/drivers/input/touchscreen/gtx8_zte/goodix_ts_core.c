 /*
  * Goodix Touchscreen Driver
  * Core layer of touchdriver architecture.
  *
  * Copyright (C) 2019 - 2020 Goodix, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be a reference
  * to you, when you are integrating the GOODiX's CTP IC into your system,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * General Public License for more details.
  *
  */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/fs.h>	/* XLK add here */
#include <linux/uaccess.h>/* XLK add here */
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFIER
#include <drm/drm_panel.h>
#endif

#include "goodix_ts_core.h"
#include "../tpd_ufp_mac.h"
#include "../zte_edge_suppression.h"

#if KERNEL_VERSION(2, 6, 38) < LINUX_VERSION_CODE
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif

#define GOOIDX_INPUT_PHYS	"goodix_ts/input0"
#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"

#define ZTE_SUPPRESS_LENGTH 900
#define DEFAULT_SUPPRESS_LEVEL 2
#define GOODIX_MAX_EDGE_LEVEL	4
#define GOODIX_HORIZONTAL_LEFT	5
#define GOODIX_HORIZONTAL_RIGHT	6

static int goodix_ts_remove(struct platform_device *pdev);
int goodix_start_later_init(struct goodix_ts_core *ts_core);
void goodix_ts_dev_release(void);
int goodix_ts_core_release(void);
static int goodix_set_edge_suppress(int level, int is_hor, void *priv_data);

extern void goodix_tpd_register_fw_class(struct goodix_ts_core *core_data);
/*extern void goodix_gsx_gesture_exit_gt9886(void);
extern void goodix_fwu_module_exit_gt9886(void);
extern void goodix_tools_exit_gt9886(void);
extern int goodix_tools_init_gt9886(void);
extern int goodix_gsx_gesture_init_gt9886(void);
extern int goodix_fwu_module_init(void);*/

struct goodix_module goodix_modules_gt9886;

static struct tp_point_log point_log[GOODIX_MAX_TOUCH];
static char finger_down[GOODIX_MAX_TOUCH] = {0};

bool rawdiff_mode = false; /* XLK add here */

#ifdef CONFIG_DRM_PANEL_NOTIFIER
static struct drm_panel *active_panel = NULL;
#endif

#ifdef CONFIG_DRM_PANEL_NOTIFIER
static int goodix_drm_register(struct device *dev)
{
	int retval = -1;
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(dev->of_node, "panel", NULL);
	if (count <= 0)
		return -ENODEV;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(dev->of_node, "panel", i);
		if (node != NULL)
			ts_err("node = %s\n", node->name);

		panel = of_drm_find_panel(node);
		if (!IS_ERR(panel)) {
			of_node_put(node);
			active_panel = panel;
			return 0;
		}

		if (PTR_ERR(panel) == -ENODEV) {
			ts_err("no device!\n");
			retval = -ENODEV;
		} else if (PTR_ERR(panel) == -EPROBE_DEFER) {
			ts_err("device has not been probed yet\n");
			retval = -EPROBE_DEFER;
		}

		of_node_put(node);
	}

	return retval;
}
#endif

/**
 * __do_register_ext_module - register external module
 * to register into touch core modules structure
 */
static void  __do_register_ext_module(struct work_struct *work)
{
	struct goodix_ext_module *module =
			container_of(work, struct goodix_ext_module, work);
	struct goodix_ext_module *ext_module, *next;
	struct list_head *insert_point = &goodix_modules_gt9886.head;

	ts_info("__do_register_ext_module IN");

	if (goodix_modules_gt9886.core_data &&
	    !goodix_modules_gt9886.core_data->initialized) {
		ts_err("core layer has exit");
		return;
	}

	if (!goodix_modules_gt9886.core_data) {
		/* waitting for core layer */
		if (!wait_for_completion_timeout(&goodix_modules_gt9886.core_comp,
						 25 * HZ)) {
			ts_err("Module [%s] timeout", module->name);
			return;
		}
	}

	/* driver probe failed */
	if (!goodix_modules_gt9886.core_data ||
	    !goodix_modules_gt9886.core_data->initialized) {
		ts_err("Can't register ext_module core error");
		return;
	}

	ts_info("start register ext_module");

	/* prority level *must* be set */
	if (module->priority == EXTMOD_PRIO_RESERVED) {
		ts_err("Priority of module [%s] needs to be set",
		       module->name);
		return;
	}

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (ext_module == module) {
				ts_info("Module [%s] already exists",
					module->name);
				mutex_unlock(&goodix_modules_gt9886.mutex);
				return;
			}
		}

		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			/* small value of priority have
			 * higher priority level
			 */
			if (ext_module->priority >= module->priority) {
				insert_point = &ext_module->list;
				break;
			}
		}
	}

	if (module->funcs && module->funcs->init) {
		if (module->funcs->init(goodix_modules_gt9886.core_data,
					module) < 0) {
			ts_err("Module [%s] init error",
			       module->name ? module->name : " ");
			mutex_unlock(&goodix_modules_gt9886.mutex);
			return;
		}
	}

	list_add(&module->list, insert_point->prev);
	goodix_modules_gt9886.count++;
	mutex_unlock(&goodix_modules_gt9886.mutex);

	ts_info("Module [%s] registered,priority:%u",
		module->name,
		module->priority);
}

/**
 * goodix_register_ext_module_gt9886 - interface for external module
 * to register into touch core modules structure
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int goodix_register_ext_module_gt9886(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;

	if (!goodix_modules_gt9886.initilized) {
		goodix_modules_gt9886.initilized = true;
		INIT_LIST_HEAD(&goodix_modules_gt9886.head);
		mutex_init(&goodix_modules_gt9886.mutex);
		init_completion(&goodix_modules_gt9886.core_comp);
	}

	ts_info("goodix_register_ext_module_gt9886 IN");

	INIT_WORK(&module->work, __do_register_ext_module);
	schedule_work(&module->work);

	ts_info("goodix_register_ext_module_gt9886 OUT");

	return 0;
}
EXPORT_SYMBOL_GPL(goodix_register_ext_module_gt9886);

/**
 * goodix_unregister_ext_module_gt9886 - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int goodix_unregister_ext_module_gt9886(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module, *next;
	bool found = false;

	if (!module)
		return -EINVAL;

	if (!goodix_modules_gt9886.initilized)
		return -EINVAL;

	if (!goodix_modules_gt9886.core_data)
		return -ENODEV;

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (ext_module == module) {
				found = true;
				break;
			}
		}
	} else {
		mutex_unlock(&goodix_modules_gt9886.mutex);
		return -EFAULT;
	}

	if (!found) {
		ts_err("Module [%s] never registed",
				module->name);
		mutex_unlock(&goodix_modules_gt9886.mutex);
		return -EFAULT;
	}

	list_del(&module->list);
	goodix_modules_gt9886.count--;
	mutex_unlock(&goodix_modules_gt9886.mutex);

	if (module->funcs && module->funcs->exit)
		module->funcs->exit(goodix_modules_gt9886.core_data, module);

	ts_info("Moudle [%s] unregistered",
		module->name ? module->name : " ");
	return 0;
}
EXPORT_SYMBOL_GPL(goodix_unregister_ext_module_gt9886);

static void goodix_remove_all_ext_modules(void)
{
	struct goodix_ext_module *ext_module, *next;

	if (!goodix_modules_gt9886.initilized || !goodix_modules_gt9886.core_data)
		return;

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			list_del(&ext_module->list);
			goodix_modules_gt9886.count--;
			if (ext_module->funcs && ext_module->funcs->exit)
				ext_module->funcs->exit(goodix_modules_gt9886.core_data,
							ext_module);
		}
	}

	mutex_unlock(&goodix_modules_gt9886.mutex);
}

static void goodix_ext_sysfs_release(struct kobject *kobj)
{
	ts_info("Kobject released!");
}

#define to_ext_module(kobj)	container_of(kobj,\
				struct goodix_ext_module, kobj)
#define to_ext_attr(attr)	container_of(attr,\
				struct goodix_ext_attribute, attr)

static ssize_t goodix_ext_sysfs_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->show)
		return ext_attr->show(module, buf);

	return -EIO;
}

static ssize_t goodix_ext_sysfs_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->store)
		return ext_attr->store(module, buf, count);

	return -EIO;
}

static const struct sysfs_ops goodix_ext_ops = {
	.show = goodix_ext_sysfs_show,
	.store = goodix_ext_sysfs_store
};

static struct kobj_type goodix_ext_ktype = {
	.release = goodix_ext_sysfs_release,
	.sysfs_ops = &goodix_ext_ops,
};

struct kobj_type *goodix_get_default_ktype_gt9886(void)
{
	return &goodix_ext_ktype;
}
EXPORT_SYMBOL_GPL(goodix_get_default_ktype_gt9886);

struct kobject *goodix_get_default_kobj_gt9886(void)
{
	struct kobject *kobj = NULL;

	if (goodix_modules_gt9886.core_data &&
			goodix_modules_gt9886.core_data->pdev)
		kobj = &goodix_modules_gt9886.core_data->pdev->dev.kobj;
	return kobj;
}
EXPORT_SYMBOL_GPL(goodix_get_default_kobj_gt9886);

/* debug fs */
struct debugfs_buf {
	struct debugfs_blob_wrapper buf;
	int pos;
	struct dentry *dentry;
} goodix_dbg;

void goodix_msg_printf(const char *fmt, ...)
{
	va_list args;
	int r;

	if (!goodix_dbg.dentry)
		return;
	if (goodix_dbg.pos < goodix_dbg.buf.size) {
		va_start(args, fmt);
		r = vscnprintf(goodix_dbg.buf.data + goodix_dbg.pos,
			       goodix_dbg.buf.size - 1, fmt, args);
		goodix_dbg.pos += r;
		va_end(args);
	}
}
EXPORT_SYMBOL_GPL(goodix_msg_printf);

static int goodix_debugfs_init(void)
{
	struct dentry *r_b;

	goodix_dbg.buf.size = PAGE_SIZE;
	goodix_dbg.pos = 0;
	goodix_dbg.buf.data = kzalloc(goodix_dbg.buf.size, GFP_KERNEL);
	if (goodix_dbg.buf.data == NULL) {
		pr_err("Debugfs init failed\n");
		goto exit;
	}
	r_b = debugfs_create_blob("goodix_ts", 0644, NULL, &goodix_dbg.buf);
	if (!r_b) {
		pr_err("Debugfs create failed\n");
		return -ENOENT;
	}
	goodix_dbg.dentry = r_b;

exit:
	return 0;
}

static void goodix_debugfs_exit(void)
{
	debugfs_remove(goodix_dbg.dentry);
	goodix_dbg.dentry = NULL;
	pr_info("Debugfs module exit\n");
}

/* XLK add here.begin */
#define RAWDATA_REG 0x7EFC
#define RAWDATA_LEN 5500
static int goodix_cache_rawdata(struct goodix_ts_device *ts_dev,
		u8 *data)
{
	return ts_dev->hw_ops->read_trans(ts_dev, RAWDATA_REG, data, RAWDATA_LEN);
}

#define FW_STATEV02_REG 0x4154
#define FW_STATEV02_LEN 12
static int goodix_cache_fwstate_v02(struct goodix_ts_device *ts_dev,
		u8 *data)
{
	return ts_dev->hw_ops->read_trans(ts_dev, FW_STATEV02_REG, data, FW_STATEV02_LEN);
}

#define FW_STATE2_REG 0x428E
#define FW_STATE2_LEN 29
static int goodix_cache_fwstate2(struct goodix_ts_device *ts_dev,
		u8 *data)
{
	return ts_dev->hw_ops->read_trans(ts_dev, FW_STATE2_REG, data, FW_STATE2_LEN);
}
/* XLK add here.end */

/* XLK add here.end add by zte */
#define TS_RAWDATA_CMD 0x4160
#define CACHE_RAWDATA_FRAME	5
#define BUF_ARRY_LEN 1000
int get_goodix_ts_rawdata(struct goodix_ts_core *core_data,
	char *buf_arry[], int list_len, int unit_len)
{
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd cmd;
	int cnt = 0;
	int ret;
	int i;
	int j;
	int k = 0;
	int m = 0;
	int rpt;
	u8 irq_flag;
	u8 *data = NULL;

	data = kzalloc(RAWDATA_LEN, GFP_KERNEL);
	if (!data) {
		ts_err("Failed to alloc data");
		return 0;
	}

	rawdiff_mode = true;

	memset(&cmd, 0, sizeof(cmd));
	cmd.initialized = true;
	cmd.cmd_reg = TS_RAWDATA_CMD;
	cmd.cmds[0] = 1;
	cmd.cmds[4] = 1;
	cmd.length = 5;
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		ts_err("Failed to switch RW mode");
		goto err_free;
	}

	msleep(50);
	/* clear irq_flag */
	irq_flag = 0;
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);

	for (j = 0; j < CACHE_RAWDATA_FRAME; j++) {
		rpt = 15;
		while (rpt--) {
			ts_dev->hw_ops->read_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);
			if (irq_flag & 0x80)
				break;
			usleep_range(5000, 5100);
		}
		if (rpt <= 0) {
			ts_err("rawdata is not ready");
			cnt = 0;
			goto err;
		}

		/* cache rawdata */
		ret = goodix_cache_rawdata(ts_dev, data);
		if (ret < 0) {
			ts_err("Faile to read rawdata");
			goto err;
		}

		/* one circle */
		cnt = snprintf(&buf_arry[m][0], unit_len, "rawdata_frame_%d:\n", j + 1);

		for (i = 0; i < RAWDATA_LEN; i++) {
			if (k > BUF_ARRY_LEN) {
				m++;
				k = 0;
				cnt = 0;
			}
			cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt,  "%02x,", data[i]);

			if ((i + 1) % 55 == 0)
				cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt, "\n");
			k++;
		}
		/* next */
		m++;
		k = 0;
		cnt = 0;

		/* cache fwstate_v02 */
		ret = goodix_cache_fwstate_v02(ts_dev, data);
		if (ret) {
			ts_err("Failed to read fwstate_v02");
			goto err;
		}

		cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt,  "fwstate_v02:\n");
		for (i = 0; i < FW_STATEV02_LEN; i++)
			cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt,  "%02x,", data[i]);
		cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt, "\n");

		ret = goodix_cache_fwstate2(ts_dev, data);
		if (ret) {
			ts_err("Failed to read fwstate2");
			cnt = 0;
			goto err;
		}

		cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt,  "fwstate2:\n");
		for (i = 0; i < FW_STATE2_LEN; i++)
			cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt,  "%02x,", data[i]);
		cnt += snprintf(&buf_arry[m][cnt], unit_len - cnt, "\n");

		/* clear irq_flag */
		irq_flag = 0;
		ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);

		/* next */
		m++;
		k = 0;
		cnt = 0;
	}

err:
	memset(&cmd, 0, sizeof(cmd));
	cmd.initialized = true;
	cmd.cmd_reg = TS_RAWDATA_CMD;
	cmd.length = 5;
	ts_dev->hw_ops->send_cmd(ts_dev, &cmd);

err_free:
	rawdiff_mode = false;
	kfree(data);

	return 0;
}
/* XLK add here.end */

/* show external module information */
static ssize_t goodix_ts_extmod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ext_module *module, *next;
	size_t offset = 0;
	int r;

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(module, next,
					 &goodix_modules_gt9886.head, list) {
			r = snprintf(&buf[offset], PAGE_SIZE,
				     "priority:%u module:%s\n",
				     module->priority, module->name);
			if (r < 0) {
				mutex_unlock(&goodix_modules_gt9886.mutex);
				return -EINVAL;
			}
			offset += r;
		}
	}

	mutex_unlock(&goodix_modules_gt9886.mutex);
	return offset;
}

/* show driver information */
static ssize_t goodix_ts_driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t goodix_ts_chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_version chip_ver;
	int r, cnt = 0;

	cnt += snprintf(buf, PAGE_SIZE, "TouchDeviceName:%s\n", ts_dev->name);
	if (ts_dev->hw_ops->read_version) {
		r = ts_dev->hw_ops->read_version(ts_dev, &chip_ver);
		if (!r && chip_ver.valid) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"PID:%s\nVID:%02x%02x%02x%02x\nSensID:%02x\n",
				chip_ver.pid, chip_ver.vid[0],
				chip_ver.vid[1], chip_ver.vid[2],
				chip_ver.vid[3], chip_ver.sensor_id);
		}
	}

	return cnt;
}

/* XLK add here.begin */
#define TS_RAWDATA_CMD 0x4160
#define CACHE_RAWDATA_FRAME	5
#define DEBUG_INFO_PATH	"/sdcard/goodix_debug.log"
static ssize_t goodix_ts_rawdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd cmd;
	struct file *fp;
	mm_segment_t fs = get_fs();
	loff_t pos = 0;
	ktime_t start, end;
	int cnt;
	int ret;
	int i;
	int j;
	int take_time;
	int rpt;
	u8 irq_flag;
	u8 *data = NULL;
	u8 *temp_buf = NULL;

	fp = filp_open(DEBUG_INFO_PATH, O_RDWR | O_CREAT, 0644);
	if (!fp) {
		ts_err("Failed to creat file");
		return 0;
	}

	data = kzalloc(RAWDATA_LEN, GFP_KERNEL);
	temp_buf = kzalloc(RAWDATA_LEN * 2, GFP_KERNEL);
	if (!data || !temp_buf) {
		ts_err("Failed to alloc data");
		kfree(data);
		kfree(temp_buf);
		filp_close(fp, NULL);
		return 0;
	}
	rawdiff_mode = true;

	start = ktime_get();
	memset(&cmd, 0, sizeof(cmd));
	cmd.initialized = true;
	cmd.cmd_reg = TS_RAWDATA_CMD;
	cmd.cmds[0] = 1;
	cmd.cmds[4] = 1;
	cmd.length = 5;
	ts_dev->hw_ops->send_cmd(ts_dev, &cmd);

	msleep(50);
	set_fs(KERNEL_DS);
	/* clear irq_flag */
	irq_flag = 0;
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);

	for (j = 0; j < CACHE_RAWDATA_FRAME; j++) {
		rpt = 15;
		while (rpt--) {
			ts_dev->hw_ops->read_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);
			if (irq_flag & 0x80)
				break;
			usleep_range(5000, 5100);
		}
		if (rpt <= 0) {
			ts_err("rawdata is not ready");
			cnt = 0;
			goto err;
		}

		/* cache rawdata */
		ret = goodix_cache_rawdata(ts_dev, data);
		if (ret) {
			ts_err("Faile to read rawdata");
			cnt = 0;
			goto err;
		}
		cnt = snprintf(temp_buf, PAGE_SIZE, "rawdata_frame_%d:\n", j + 1);
		for (i = 0; i < RAWDATA_LEN; i++) {
			cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "%02x,", data[i]);
			if ((i + 1) % 55 == 0)
				cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "\n");
		}

		/* cache fwstate_v02 */
		ret = goodix_cache_fwstate_v02(ts_dev, data);
		if (ret) {
			ts_err("Failed to read fwstate_v02");
			cnt = 0;
			goto err;
		}
		cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "fwstate_v02:\n");
		for (i = 0; i < FW_STATEV02_LEN; i++)
			cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "%02x,", data[i]);
		cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "\n");

		/* cache fwstate2 */
		ret = goodix_cache_fwstate2(ts_dev, data);
		if (ret) {
			ts_err("Failed to read fwstate2");
			cnt = 0;
			goto err;
		}
		cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "fwstate2:\n");
		for (i = 0; i < FW_STATE2_LEN; i++)
			cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "%02x,", data[i]);
		cnt += snprintf(&temp_buf[cnt], PAGE_SIZE, "\n");

		/* clear irq_flag */
		irq_flag = 0;
		ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);

		vfs_write(fp, temp_buf, cnt, &pos);
	}

	end = ktime_get();
	take_time = ktime_to_ms(ktime_sub(end, start));
	cnt = snprintf(buf, PAGE_SIZE, "log_path:%s\ntake_time:%dms\n",
		DEBUG_INFO_PATH, take_time);

err:
	kfree(data);
	kfree(temp_buf);
	filp_close(fp, NULL);
	set_fs(fs);
	memset(&cmd, 0, sizeof(cmd));
	cmd.initialized = true;
	cmd.cmd_reg = TS_RAWDATA_CMD;
	cmd.length = 5;
	ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	rawdiff_mode = false;

	return cnt;
}
/* XLK add here.end */

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	if (ts_dev->hw_ops->reset)
		ts_dev->hw_ops->reset(ts_dev);
	return count;

}

static ssize_t goodix_ts_read_cfg_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int ret, i, offset;
	char *cfg_buf;

	cfg_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!cfg_buf)
		return -ENOMEM;

	if (ts_dev->hw_ops->read_config)
		ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf);
	else
		ret = -EINVAL;

	if (ret > 0) {
		offset = 0;
		for (i = 0; i < ret; i++) {
			if (i != 0 && i % 20 == 0)
				buf[offset++] = '\n';
			offset += snprintf(&buf[offset], PAGE_SIZE - offset,
					   "%02x,", cfg_buf[i]);
		}
	}
	kfree(cfg_buf);
	if (ret <= 0)
		return ret;

	return offset;
}

static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if (a >= '0' && a <= '9')
		value = a - '0';
	else if (a >= 'A' && a <= 'F')
		value = a - 'A' + 0x0A;
	else if (a >= 'a' && a <= 'f')
		value = a - 'a' + 0x0A;
	else
		value = 0xff;

	return value;
}

static int goodix_ts_convert_0x_data(const u8 *buf, int buf_size,
				     unsigned char *out_buf, int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;
	u8 high, low;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}

	if (m_size <= 1) {
		ts_err("cfg file ERROR, valid data count:%d\n", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] != 'x' && buf[i] != 'X')
			continue;

		if (temp_index >= m_size) {
			ts_err("exchange cfg data error, overflow,"
			       "temp_index:%d,m_size:%d\n",
			       temp_index, m_size);
			return -EINVAL;
		}
		high = ascii2hex(buf[i + 1]);
		low = ascii2hex(buf[i + 2]);
		if (high == 0xff || low == 0xff) {
			ts_err("failed convert: 0x%x, 0x%x",
				buf[i + 1], buf[i + 2]);
			return -EINVAL;
		}
		out_buf[temp_index++] = (high << 4) + low;
	}
	return 0;
}

static ssize_t goodix_ts_send_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en, r;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	disable_irq(core_data->irq);

	/*request configuration*/
	r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d",
		       GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	} else
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (config == NULL)
		goto exit;

	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				      config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}

	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = TS_CFG_STABLE;

	if (ts_dev->hw_ops->send_config)
		ts_dev->hw_ops->send_config(ts_dev, config);

exit:
	enable_irq(core_data->irq);
	kfree(config);
	config = NULL;
	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}

	return count;
}

/* show irq information */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		     atomic_read(&core_data->irq_enabled) ?
		     "enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		     desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
		     "echo 0/1 > irq_info to disable/enable irq");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable(core_data, en);
	return count;
}

/*reg read/write */
static u16 rw_addr;
static u32 rw_len;
static u8 rw_flag;
static u8 store_buf[32];
static u8 show_buf[PAGE_SIZE];
static ssize_t goodix_ts_reg_rw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	if (!rw_addr || !rw_len) {
		ts_err("address(0x%x) and length(%d) cann't be null\n",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = ts_dev->hw_ops->read(ts_dev, rw_addr, show_buf, rw_len);
	if (ret) {
		ts_err("failed read addr(%x) length(%d)\n", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
				"failed read addr(%x), len(%d)\n",
				rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n",
			rw_addr, rw_len, rw_len, show_buf);
}

static ssize_t goodix_ts_reg_rw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	char *pos = NULL, *token = NULL;
	long result = 0;
	int ret, i;

	if (!buf || !count) {
		ts_err("invalid params\n");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err("string must start with 'r/w'\n");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid address info\n");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err("failed get addr info\n");
			goto err_out;
		}
		rw_addr = (u16)result;
		ts_info("rw addr is 0x%x\n", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid length info\n");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err("failed get length info\n");
			goto err_out;
		}
		rw_len = (u32)result;
		ts_info("rw length info is %d\n", rw_len);
		if (rw_len > sizeof(store_buf)) {
			ts_err("data len > %lu\n", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err("invalid data info\n");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err("failed get data[%d] info\n", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
			ts_info("get data[%d]=0x%x\n", i, store_buf[i]);
		}
	}
	ret = ts_dev->hw_ops->write(ts_dev, rw_addr, store_buf, rw_len);
	if (ret) {
		ts_err("failed write addr(%x) data %*ph\n", rw_addr,
			rw_len, store_buf);
		goto err_out;
	}

	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", rw_addr, rw_len, store_buf);

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;
}

static DEVICE_ATTR(extmod_info, S_IRUGO, goodix_ts_extmod_show, NULL);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_ts_driver_info_show, NULL);
static DEVICE_ATTR(chip_info, S_IRUGO, goodix_ts_chip_info_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, goodix_ts_reset_store);
static DEVICE_ATTR(send_cfg, S_IWUSR | S_IWGRP, NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, S_IRUGO, goodix_ts_read_cfg_show, NULL);
static DEVICE_ATTR(irq_info, S_IRUGO | S_IWUSR | S_IWGRP,
		   goodix_ts_irq_info_show, goodix_ts_irq_info_store);
static DEVICE_ATTR(reg_rw, S_IRUGO | S_IWUSR | S_IWGRP,
		   goodix_ts_reg_rw_show, goodix_ts_reg_rw_store);
static DEVICE_ATTR(get_rawdata, S_IRUGO, goodix_ts_rawdata_show, NULL);/* XLK add here */

static struct attribute *sysfs_attrs[] = {
	&dev_attr_extmod_info.attr,
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_reset.attr,
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_reg_rw.attr,
	&dev_attr_get_rawdata.attr,/* xlk add here */
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static ssize_t goodix_sysfs_config_write(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t pos, size_t count)
{
	struct platform_device *pdev = container_of(kobj_to_dev(kobj),
				struct platform_device, dev);
	struct goodix_ts_core *ts_core = platform_get_drvdata(pdev);
	struct goodix_ts_device *ts_dev = ts_core->ts_dev;
	struct goodix_ts_config *config = NULL;
	int ret;

	if (pos != 0 || count > GOODIX_CFG_MAX_SIZE) {
		ts_info("pos(%d) != 0, cfg size %zu", (int)pos, count);
		return -EINVAL;
	}

	config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (config == NULL)
		return -ENOMEM;

	memcpy(config->data, buf, count);
	config->length = count;
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	ret = ts_dev->hw_ops->send_config(ts_dev, config);
	if (ret) {
		count = -EINVAL;
		ts_err("send config failed %d", ret);
	} else {
		ts_info("send config success");
	}

	kfree(config);
	return count;
}

static ssize_t goodix_sysfs_config_read(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t pos, size_t size)
{
	struct platform_device *pdev = container_of(kobj_to_dev(kobj),
				struct platform_device, dev);
	struct goodix_ts_core *ts_core = platform_get_drvdata(pdev);
	struct goodix_ts_device *ts_dev = ts_core->ts_dev;
	int ret;

	ts_debug("pos = %d, size = %zu", (int)pos, size);

	if (pos != 0)
		return 0;

	if (ts_dev->hw_ops->read_config)
		ret = ts_dev->hw_ops->read_config(ts_dev, buf);
	else
		ret = -EINVAL;

	ts_debug("read config ret %d", ret);
	return ret;
}

static struct bin_attribute goodix_config_bin_attr = {
	.attr = {
		.name = "config_bin",
		.mode = S_IRUGO | S_IWUSR | S_IWGRP,
	},
	.size = GOODIX_CFG_MAX_SIZE,
	.read = goodix_sysfs_config_read,
	.write = goodix_sysfs_config_write,
};

static int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	int ret;

	ret = sysfs_create_bin_file(&core_data->pdev->dev.kobj,
				    &goodix_config_bin_attr);
	if (ret) {
		ts_err("failed create config bin attr");
		return ret;
	}

	ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (ret) {
		ts_err("failed create core sysfs group");
		sysfs_remove_bin_file(&core_data->pdev->dev.kobj,
				      &goodix_config_bin_attr);
		return ret;
	}

	return ret;
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_bin_file(&core_data->pdev->dev.kobj,
			      &goodix_config_bin_attr);
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* event notifier */
static BLOCKING_NOTIFIER_HEAD(ts_notifier_list);
/**
 * goodix_ts_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *  see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_register_notifier_gt9886(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_register_notifier_gt9886);

/**
 * goodix_ts_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_unregister_notifier_gt9886(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_unregister_notifier_gt9886);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_blocking_notify_gt9886(enum ts_notify_event evt, void *v)
{
	int ret;

	ret = blocking_notifier_call_chain(&ts_notifier_list,
			(unsigned long)evt, v);
	return ret;
}
EXPORT_SYMBOL_GPL(goodix_ts_blocking_notify_gt9886);

static void goodix_ts_report_pen(struct input_dev *dev,
		struct goodix_pen_data *pen_data)
{
	int i;

	if (pen_data->coords.status == TS_TOUCH) {
		input_report_key(dev, BTN_TOUCH, 1);
		input_report_key(dev, pen_data->coords.tool_type, 1);
	} else if (pen_data->coords.status == TS_RELEASE) {
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, pen_data->coords.tool_type, 0);
	}
	if (pen_data->coords.status) {
		input_report_abs(dev, ABS_X, pen_data->coords.x);
		input_report_abs(dev, ABS_Y, pen_data->coords.y);
		input_report_abs(dev, ABS_PRESSURE, pen_data->coords.p);
	}
	/* report pen button */
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		if (!pen_data->keys[i].status)
			continue;
		if (pen_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, pen_data->keys[i].code, 1);
		else if (pen_data->keys[i].status == TS_RELEASE)
			input_report_key(dev, pen_data->keys[i].code, 0);
	}
	input_sync(dev);
}

static inline void point_up_log(int index)
{
	if (finger_down[index]) {
		finger_down[index] = 0;
		pr_info("ufp_touch_point: touch_up id: %d, coord[%d, %d]\n",
				index, point_log[index].x, point_log[index].y);
	}
}

static inline void point_down_log(int x, int y, int index)
{
	struct goodix_ts_device *ts_device = goodix_modules_gt9886.core_data->ts_dev;

	if (!finger_down[index]) {
		finger_down[index] = 1;
		pr_info("ufp_touch_point: touch_down id: %d, coord[%d, %d]\n", index, x, y);
	}

	point_log[index].x = x;
	point_log[index].y = y;
	dev_dbg(ts_device->dev,
					"Finger %d: x = %d, y = %d\n",
					index, x, y);
}

static void goodix_ts_report_finger(struct input_dev *dev,
		struct goodix_touch_data *touch_data)
{
	unsigned int touch_num = touch_data->touch_num;
	static u32 pre_fin;
	int i;

	/*first touch down and last touch up condition*/
	if (touch_num && !pre_fin)
		input_report_key(dev, BTN_TOUCH, 1);
	else if (!touch_num && pre_fin)
		input_report_key(dev, BTN_TOUCH, 0);

	pre_fin = touch_num;

	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (!touch_data->coords[i].status)
			continue;
		if (touch_data->coords[i].status == TS_RELEASE) {
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
#if defined(CONFIG_TPD_UFP_MAC) && defined(POINT_SIMULAT_UF)
			uf_touch_report(-1, -1, i);
#endif
#if defined(CONFIG_TPD_UFP_MAC) && defined(ZTE_ONE_KEY)
			one_key_report(false, -1, -1, i);
#endif
			point_up_log(i);
			zte_touch_up(i);
			continue;
		}

/*
		input_mt_slot(dev, i);
		input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
		input_report_abs(dev, ABS_MT_POSITION_X,
				 touch_data->coords[i].x);
		input_report_abs(dev, ABS_MT_POSITION_Y,
				 touch_data->coords[i].y);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
				 touch_data->coords[i].w);
*/
		zte_touch_report(touch_data->coords[i].x, touch_data->coords[i].y, i,
						touch_data->coords[i].w, -1, dev);

#if defined(CONFIG_TPD_UFP_MAC) && defined(POINT_SIMULAT_UF)
		uf_touch_report(touch_data->coords[i].x,
					touch_data->coords[i].y,
					i);
#endif
#if defined(CONFIG_TPD_UFP_MAC) && defined(ZTE_ONE_KEY)
		one_key_report(true,
					touch_data->coords[i].x,
					touch_data->coords[i].y,
					i);
#endif

		point_down_log(touch_data->coords[i].x,
					touch_data->coords[i].y,
					i);
	}

	/* report panel key */
	for (i = 0; i < GOODIX_MAX_TP_KEY; i++) {
		if (!touch_data->keys[i].status)
			continue;
		if (touch_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, touch_data->keys[i].code, 1);
		else if (touch_data->keys[i].status == TS_RELEASE)
			input_report_key(dev, touch_data->keys[i].code, 0);
	}
	input_sync(dev);
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_device *ts_dev =  core_data->ts_dev;
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	u8 irq_flag = 0;
	int r;

	core_data->irq_trig_cnt++;
	/* inform external module */
	mutex_lock(&goodix_modules_gt9886.mutex);
	list_for_each_entry_safe(ext_module, next,
				 &goodix_modules_gt9886.head, list) {
		if (!ext_module->funcs->irq_event)
			continue;
		r = ext_module->funcs->irq_event(core_data, ext_module);
		if (r == EVT_CANCEL_IRQEVT) {
			mutex_unlock(&goodix_modules_gt9886.mutex);
			return IRQ_HANDLED;
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

	/* read touch data from touch device */
	r = ts_dev->hw_ops->event_handler(ts_dev, ts_event);
	if (likely(r >= 0)) {
		if (ts_event->event_type == EVENT_TOUCH) {
			/* report touch */
			goodix_ts_report_finger(core_data->input_dev,
					&ts_event->touch_data);
		}
		if (ts_dev->board_data.pen_enable &&
			ts_event->event_type == EVENT_PEN) {
			goodix_ts_report_pen(core_data->pen_dev,
					&ts_event->pen_data);
		}
	}

	/* clean irq flag */
	irq_flag = 0;
	/* XLK modify here.begin */
	if (!rawdiff_mode)
		ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);
	/* XLK modify here.end */

	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	/* if ts_bdata-> irq is invalid */
	if (ts_bdata->irq <= 0)
		core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	else
		core_data->irq = ts_bdata->irq;

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	r = devm_request_threaded_irq(&core_data->pdev->dev,
				      core_data->irq, NULL,
				      goodix_ts_threadirq_func,
				      ts_bdata->irq_flags | IRQF_ONESHOT,
				      GOODIX_CORE_DRIVER_NAME,
				      core_data);
	if (r < 0)
		ts_err("Failed to requeset threaded irq:%d", r);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return r;
}

/**
 * goodix_ts_irq_enable - Enable/Disable a irq
 * @core_data: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_enable(struct goodix_ts_core *core_data,
			bool enable)
{
	if (enable) {
		if (!atomic_cmpxchg(&core_data->irq_enabled, 0, 1)) {
			enable_irq(core_data->irq);
			ts_debug("Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&core_data->irq_enabled, 1, 0)) {
			disable_irq(core_data->irq);
			ts_debug("Irq disabled");
		}
	}

	return 0;
}
EXPORT_SYMBOL(goodix_ts_irq_enable);

/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata;
	struct device *dev = NULL;
	int r = 0;

	ts_info("Power init");
	/* dev:i2c client device or spi slave device*/
	dev =  core_data->ts_dev->dev;
	ts_bdata = board_data(core_data);

	core_data->avdd_enable_gpio = ts_bdata->avdd_enable_gpio;
	r = devm_gpio_request(dev, core_data->avdd_enable_gpio, "TP_AVDD_GPIO");
	if (r) {
		ts_err("failed to request avdd_enable_gpio");
		return r;
	}

	core_data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR_OR_NULL(core_data->vdd)) {
		r = PTR_ERR(core_data->vdd);
		ts_err("Failed to get regulator vdd:%d", r);
		core_data->vdd = NULL;
		return r;
	}

	return r;
}

/**
 * goodix_ts_power_on_gt9886 - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_on_gt9886(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	ts_info("Device power on");
	if (core_data->power_on)
		return 0;

	r = gpio_direction_output(core_data->avdd_enable_gpio, 1);
	if (r) {
		ts_err("Failed to enable avdd power:%d", r);
		return r;
	}

	r = regulator_enable(core_data->vdd);
	if (!r) {
		ts_info("regulator vdd enable SUCCESS");
		if (ts_bdata->power_on_delay_us)
			usleep_range(ts_bdata->power_on_delay_us,
				     ts_bdata->power_on_delay_us);
	} else {
		ts_err("Failed to enable vdd power:%d", r);
		return r;
	}

	core_data->power_on = 1;
	return 0;
}

/**
 * goodix_ts_power_off_gt9886 - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_off_gt9886(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	ts_info("Device power off");
	if (!core_data->power_on)
		return 0;

	r = gpio_direction_output(core_data->avdd_enable_gpio, 0);
	if (r) {
		ts_err("Failed to disable avdd power:%d", r);
	}

	if (core_data->vdd) {
		r = regulator_disable(core_data->vdd);
		if (!r) {
			ts_info("regulator vdd disable SUCCESS");
			if (ts_bdata->power_off_delay_us)
				usleep_range(ts_bdata->power_off_delay_us,
					     ts_bdata->power_off_delay_us);
		} else {
			ts_err("Failed to disable vdd power:%d", r);
			return r;
		}
	}

	core_data->power_on = 0;
	return 0;
}

#ifdef CONFIG_PINCTRL
/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_ts_core *core_data)
{
	int r = 0;

	/* get pinctrl handler from of node */
	core_data->pinctrl = devm_pinctrl_get(core_data->ts_dev->dev);
	if (IS_ERR_OR_NULL(core_data->pinctrl)) {
		ts_info("Failed to get pinctrl handler[need confirm]");
		core_data->pinctrl = NULL;
		return -EINVAL;
	}
	ts_debug("success get pinctrl");
	/* active state */
	core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		r = PTR_ERR(core_data->pin_sta_active);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_ACTIVE, r);
		core_data->pin_sta_active = NULL;
		goto exit_pinctrl_put;
	}
	ts_debug("success get avtive pinctrl state");

	/* suspend state */
	core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		r = PTR_ERR(core_data->pin_sta_suspend);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_SUSPEND, r);
		core_data->pin_sta_suspend = NULL;
		goto exit_pinctrl_put;
	}
	ts_debug("success get suspend pinctrl state");

	return 0;
exit_pinctrl_put:
	devm_pinctrl_put(core_data->pinctrl);
	core_data->pinctrl = NULL;
	return r;
}
#endif

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 *	reset_gpio and irq_gpio number are obtained from goodix_ts_device
 *  which created in hardware layer driver. e.g.goodix_xx_i2c.c
 *	A goodix_ts_device should set those two fileds to right value
 *	before registed to touch core driver.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
				  GPIOF_OUT_INIT_HIGH, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
				  GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}

	return 0;
}

/**
 * goodix_input_set_params - set input parameters
 */
static void goodix_ts_set_input_params(struct input_dev *input_dev,
		struct goodix_ts_board_data *ts_bdata)
{
	int i;

	if (ts_bdata->swap_axis)
		swap(ts_bdata->panel_max_x, ts_bdata->panel_max_y);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, ts_bdata->panel_max_w, 0, 0);

	if (ts_bdata->panel_max_key) {
		for (i = 0; i < ts_bdata->panel_max_key; i++)
			input_set_capability(input_dev, EV_KEY,
					     ts_bdata->panel_key_map[i]);
	}
}

/**
 * goodix_ts_input_dev_config - Requset and config a input device
 *  then register it to input sybsystem.
 *  NOTE that some hardware layer may provide a input device
 *  (ts_dev->input_dev not NULL).
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int r;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	input_dev->name = GOODIX_CORE_DRIVER_NAME;
	input_dev->phys = GOOIDX_INPUT_PHYS;
	input_dev->id.product = 0xDEAD;
	input_dev->id.vendor = 0xBEEF;
	input_dev->id.version = 10427;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* set input parameters */
	goodix_ts_set_input_params(input_dev, ts_bdata);

#ifdef INPUT_TYPE_B_PROTOCOL
#if KERNEL_VERSION(3, 7, 0) < LINUX_VERSION_CODE
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH,
				INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		input_free_device(input_dev);
		return r;
	}

	return 0;
}

static int goodix_ts_pen_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *pen_dev = NULL;
	int r;

	pen_dev = input_allocate_device();
	if (!pen_dev) {
		ts_err("Failed to allocated pen device");
		return -ENOMEM;
	}
	core_data->pen_dev = pen_dev;
	input_set_drvdata(pen_dev, core_data);

	pen_dev->name = GOODIX_PEN_DRIVER_NAME;
	pen_dev->id.product = 0xDEAD;
	pen_dev->id.vendor = 0xBEEF;
	pen_dev->id.version = 10427;

	pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(ABS_X, pen_dev->absbit);
	__set_bit(ABS_Y, pen_dev->absbit);
	__set_bit(BTN_STYLUS, pen_dev->keybit);
	__set_bit(BTN_STYLUS2, pen_dev->keybit);
	__set_bit(BTN_TOUCH, pen_dev->keybit);
	__set_bit(BTN_TOOL_PEN, pen_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, 0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(pen_dev, ABS_PRESSURE, 0,
			     GOODIX_PEN_MAX_PRESSURE, 0, 0);

	r = input_register_device(pen_dev);
	if (r < 0) {
		ts_err("Unable to register pen device");
		input_free_device(pen_dev);
		return r;
	}

	return 0;
}

void goodix_ts_input_dev_remove_gt9886(struct goodix_ts_core *core_data)
{
	input_unregister_device(core_data->input_dev);
	input_free_device(core_data->input_dev);
	core_data->input_dev = NULL;
}

void goodix_ts_pen_dev_remove_gt9886(struct goodix_ts_core *core_data)
{
	input_unregister_device(core_data->pen_dev);
	input_free_device(core_data->pen_dev);
	core_data->pen_dev = NULL;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd = container_of(dwork,
			struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *core = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(core);
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;
	int r = 0;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	if (hw_ops->check_hw)
		r = hw_ops->check_hw(core->ts_dev);
	if (r < 0) {
		goodix_ts_power_off_gt9886(core);
		goodix_ts_power_on_gt9886(core);
		if (hw_ops->reset)
			hw_ops->reset(core->ts_dev);

		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev, core->ts_dev->reg.esd,
					&data, 1);
		if (r < 0)
			ts_err("failed init dynamic esd");
	} else {
		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			ts_err("failed init watch dog");
	}

	if (atomic_read(&ts_esd->esd_on))
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
static void goodix_ts_esd_on(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	if (core->ts_dev->reg.esd == 0)
		return;

	atomic_set(&ts_esd->esd_on, 1);
	if (!schedule_delayed_work(&ts_esd->esd_work, 2 * HZ)) {
		ts_info("esd work already in workqueue");
	}
	ts_info("esd on");
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
static void goodix_ts_esd_off(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	int ret;

	atomic_set(&ts_esd->esd_on, 0);
	ret = cancel_delayed_work_sync(&ts_esd->esd_work);
	ts_info("Esd off, esd work state %d", ret);
}

/**
 * goodix_esd_notifier_callback - notification callback
 *  under certain condition, we need to turn off/on the esd
 *  protector, we use kernel notify call chain to achieve this.
 *
 *  for example: before firmware update we need to turn off the
 *  esd protector and after firmware update finished, we should
 *  turn on the esd protector.
 */
static int goodix_esd_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_esd *ts_esd = container_of(nb,
			struct goodix_ts_esd, esd_notifier);

	switch (action) {
	case NOTIFY_FWUPDATE_START:
	case NOTIFY_SUSPEND:
	case NOTIFY_ESD_OFF:
		goodix_ts_esd_off(ts_esd->ts_core);
		break;
	case NOTIFY_FWUPDATE_FAILED:
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_RESUME:
	case NOTIFY_ESD_ON:
		goodix_ts_esd_on(ts_esd->ts_core);
		break;
	default:
		break;
	}

	return 0;
}

/**
 * goodix_ts_esd_init_gt9886 - initialize esd protection
 */
int goodix_ts_esd_init_gt9886(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	struct goodix_ts_device *dev = core->ts_dev;
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;
	int r;

	if (!dev->hw_ops->check_hw || !dev->reg.esd) {
		ts_info("missing key info for esd check");
		return 0;
	}

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	ts_esd->ts_core = core;
	atomic_set(&ts_esd->esd_on, 0);
	ts_esd->esd_notifier.notifier_call = goodix_esd_notifier_callback;
	goodix_ts_register_notifier_gt9886(&ts_esd->esd_notifier);

	/*init dynamic esd*/
	r = dev->hw_ops->write_trans(core->ts_dev, core->ts_dev->reg.esd,
				     &data, 1);
	if (r < 0)
		ts_err("failed init dynamic esd[ignore]");

	goodix_ts_esd_on(core);

	return 0;
}

static void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
	struct input_dev *input_dev = core_data->input_dev;
	struct input_mt *mt = input_dev->mt;
	int i;

	if (mt) {
		for (i = 0; i < mt->num_slots; i++) {
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER,
					false);
#if defined(CONFIG_TPD_UFP_MAC) && defined(ZTE_ONE_KEY)
			one_key_report(false, -1, -1, i);
#endif
			point_up_log(i);
		}
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_mt_sync_frame(input_dev);
		input_sync(input_dev);
	}

	report_ufp_uevent(UFP_FP_UP);
}

/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to  sleep
 */
/*
static int goodix_ts_suspend(struct goodix_ts_core *core_data)
*/
static int goodix_ts_suspend(void *_core_data)
{
	struct goodix_ts_core *core_data =
				(struct goodix_ts_core *)_core_data;
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int r;

	if (atomic_read(&core_data->suspended)) {
		ts_info("already in suspend!\n");
		goto exit;
	}

	mutex_lock(&(core_data->zc.rawdata_read_lock));
	ts_info("Suspend start");

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify_gt9886(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;

			r = ext_module->funcs->before_suspend(core_data,
							      ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_gt9886.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);

				tp_irq_wake(core_data, 1);

				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

	/* disable irq */
	goodix_ts_irq_enable(core_data, false);

	/* let touch ic work in sleep mode */
	if (ts_dev && ts_dev->hw_ops->suspend)
		ts_dev->hw_ops->suspend(ts_dev);

	atomic_set(&core_data->suspended, 1);

#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
				core_data->pin_sta_suspend);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* inform exteranl modules */
	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (!ext_module->funcs->after_suspend)
				continue;

			r = ext_module->funcs->after_suspend(core_data,
							     ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_gt9886.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

	goodix_ts_release_connects(core_data);
out:
	mutex_unlock(&(core_data->zc.rawdata_read_lock));
	ts_info("Suspend end");
exit:
	return 0;
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
/*
static int goodix_ts_resume(struct goodix_ts_core *core_data)
*/
static int goodix_ts_resume(void *_core_data)
{
	struct goodix_ts_core *core_data =
				(struct goodix_ts_core *)_core_data;
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_device *ts_dev =
				core_data->ts_dev;
	int r;
	char status;

	if (!atomic_read(&core_data->suspended)) {
		ts_info("not in suspend!\n");
		goto exit;
	}

	mutex_lock(&(core_data->zc.rawdata_read_lock));
	ts_info("Resume start");
	goodix_ts_release_connects(core_data);

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (!ext_module->funcs->before_resume)
				continue;

			r = ext_module->funcs->before_resume(core_data,
							     ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_gt9886.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					 core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	atomic_set(&core_data->suspended, 0);

	status = (core_data->zc.is_wakeup_gesture << 1) | core_data->zc.is_single_tap;

	/* resume device */
	if (ts_dev && ts_dev->hw_ops->resume)
		ts_dev->hw_ops->resume(ts_dev);

	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data,
							    ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_gt9886.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

	if (status)
		tp_irq_wake(core_data, 0);
	else
		goodix_ts_irq_enable(core_data, true);

	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	ts_info("try notify resume");
	goodix_ts_blocking_notify_gt9886(NOTIFY_RESUME, NULL);

	core_data->zc.is_single_tap = core_data->zc.is_set_single_in_suspend;
	core_data->zc.is_wakeup_gesture = core_data->zc.is_set_wakeup_in_suspend;
	core_data->zc.is_one_key = core_data->zc.is_set_onekey_in_suspend;

	if (core_data->zc.is_smart_cover) {
		/* send highsense_cfg to firmware */
		r = ts_dev->hw_ops->send_config(ts_dev, &(ts_dev->highsense_cfg));
		if (r < 0) {
			ts_info("failed send highsense config[ignore]");
		}
	}

	goodix_set_edge_suppress(core_data->zc.level, 0, core_data);

out:
	mutex_unlock(&(core_data->zc.rawdata_read_lock));
	ts_debug("Resume end");
exit:
	return 0;
}

#ifdef CONFIG_DRM_PANEL_NOTIFIER
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
int goodix_ts_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;

	if (fb_event && fb_event->data && core_data) {
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			/* before fb blank */
		} else if (event == DRM_PANEL_EVENT_BLANK) {
			int *blank = fb_event->data;

			if (*blank == DRM_PANEL_BLANK_UNBLANK)
				change_tp_state(ON);
			else if (*blank == DRM_PANEL_BLANK_POWERDOWN)
				change_tp_state(OFF);
		}
	}

	return 0;
}
#elif defined(CONFIG_FB)
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
int goodix_ts_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;

	if (fb_event && fb_event->data && core_data) {
		if (event == FB_EARLY_EVENT_BLANK) {
			/* before fb blank */
		} else if (event == FB_EVENT_BLANK) {
			int *blank = fb_event->data;

			if (*blank == FB_BLANK_UNBLANK)
				goodix_ts_resume(core_data);
			else if (*blank == FB_BLANK_POWERDOWN)
				goodix_ts_suspend(core_data);
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * goodix_ts_earlysuspend - Early suspend function
 * Called by kernel during system suspend phrase
 */
static void goodix_ts_earlysuspend(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_lateresume - Late resume function
 * Called by kernel during system wakeup
 */
static void goodix_ts_lateresume(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_resume(core_data);
}
#endif

#ifdef CONFIG_PM
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_resume(core_data);
}
#endif
#endif

/**
 * goodix_generic_noti_callback - generic notifier callback
 *  for goodix touch notification event.
 */
static int goodix_generic_noti_callback(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct goodix_ts_core *ts_core = container_of(self,
			struct goodix_ts_core, ts_notifier);
	struct goodix_ts_device *ts_dev = ts_device(ts_core);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(ts_core);
	int r;

	ts_info("notify event type 0x%x", (unsigned int)action);
	switch (action) {
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_FWUPDATE_FAILED:
		r = hw_ops->read_version(ts_dev, &ts_dev->chip_version);
		if (r < 0)
			ts_info("failed read fw version info[ignore]");
		break;
	default:
		break;
	}

	return 0;
}

int __goodix_set_edge_suppress(int *cmd_start, struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module, *next;
	int ret = -1;

	/* inform external module */
	mutex_lock(&goodix_modules_gt9886.mutex);
	if (!list_empty(&goodix_modules_gt9886.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9886.head, list) {
			if (!ext_module->funcs->set_edge_suppress)
				continue;

			ret = ext_module->funcs->set_edge_suppress(cmd_start, core_data, ext_module);
		}
	}
	mutex_unlock(&goodix_modules_gt9886.mutex);

	return ret;
}

static int goodix_set_edge_suppress(int level, int is_hor, void *priv_data)
{
	int ret = -1;
	static in_hor = 1;
	struct goodix_ts_core *core_data = priv_data;
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);

	if (is_hor & 0x02) {
		if (atomic_read(&core_data->suspended)) {
			ts_info("%s in suspend!\n", __func__);
			return 0;
		}

		ret = __goodix_set_edge_suppress(
				&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * GOODIX_HORIZONTAL_RIGHT]),
				core_data);
		in_hor = 1;
	} else if (is_hor & 0x01) {
		if (atomic_read(&core_data->suspended)) {
			ts_info("%s in suspend!\n", __func__);
			return 0;
		}

		ret = __goodix_set_edge_suppress(
				&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * GOODIX_HORIZONTAL_LEFT]),
				core_data);
		in_hor = 1;
	} else if (level < GOODIX_MAX_EDGE_LEVEL) {
		core_data->zc.level = level;

		if (atomic_read(&core_data->suspended)) {
			ts_info("%s in suspend!\n", __func__);
			in_hor = 0;
			return 0;
		}

		if (in_hor) {
			ret = __goodix_set_edge_suppress(
				&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * GOODIX_MAX_EDGE_LEVEL]),
				core_data);
			if (ret) {
				ts_info("vertical set is wrong!\n");
				goto exit;
			}

			in_hor = 0;
		}
		ret = __goodix_set_edge_suppress(&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * level]),
									core_data);
	} else {
		core_data->zc.level = DEFAULT_SUPPRESS_LEVEL;

		if (atomic_read(&core_data->suspended)) {
			ts_info("%s in suspend!\n", __func__);
			in_hor = 0;
			return 0;
		}

		if (in_hor) {
			ret = __goodix_set_edge_suppress(
					&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * GOODIX_MAX_EDGE_LEVEL]),
					core_data);
			if (ret) {
				ts_info("vertical set is wrong!\n");
				goto exit;
			}

			in_hor = 0;
		}
		__goodix_set_edge_suppress(&(ts_bdata->edge_suppress_cmd[GOODIX_CMD_LEN * DEFAULT_SUPPRESS_LEVEL]),
									core_data);
	}

exit:
	return ret;
}

static void setup_suppress_func(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);

	zte_edge_suppress_set(ts_bdata->panel_max_x, ts_bdata->panel_max_y,
						ZTE_SUPPRESS_LENGTH, DEFAULT_SUPPRESS_LEVEL, core_data);
	zte_fef = goodix_set_edge_suppress;

	goodix_set_edge_suppress(DEFAULT_SUPPRESS_LEVEL, 0, core_data);
}

#ifdef CONFIG_TPD_UFP_MAC
static void tpd_ufp_register(struct goodix_ts_core *core_data)
{
	ufp_tp_ops.tp_data = core_data;
	ufp_tp_ops.tp_resume_func = goodix_ts_resume;
	ufp_tp_ops.tp_suspend_func = goodix_ts_suspend;
}
#endif

int goodix_ts_stage2_init_gt9886(struct goodix_ts_core *core_data)
{
	int r;
	struct goodix_ts_device *ts_dev = ts_device(core_data);

	/* send normal-cfg to firmware */
	r = ts_dev->hw_ops->send_config(ts_dev, &(ts_dev->normal_cfg));
	if (r < 0) {
		ts_info("failed send normal config[ignore]");
	}

	r = ts_dev->hw_ops->read_version(ts_dev, &ts_dev->chip_version);
	if (r < 0)
		ts_info("failed read fw version info[ignore]");

	/* alloc/config/register input device */
	r = goodix_ts_input_dev_config(core_data);
	if (r < 0) {
		ts_err("failed set input device");
		return r;
	}

	if (ts_dev->board_data.pen_enable) {
		r = goodix_ts_pen_dev_config(core_data);
		if (r < 0) {
			ts_err("failed set pen device");
			goto err_finger;
		}
	}
	/* request irq line */
	r = goodix_ts_irq_setup(core_data);
	if (r < 0) {
		ts_info("failed set irq");
		goto exit;
	}
	ts_info("success register irq");

	device_init_wakeup(&core_data->pdev->dev, 1);

#ifdef CONFIG_DRM_PANEL_NOTIFIER
	ts_info("CONFIG_DRM_PANEL_NOTIFIER");
	core_data->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback;
	if (goodix_drm_register(core_data->ts_dev->dev) == 0) {
		if (drm_panel_notifier_register(active_panel, &core_data->fb_notifier) < 0)
			ts_err("Failed to register FB notifier client\n");
	}
#elif defined(CONFIG_FB)
	ts_info("CONFIG_FB");
	core_data->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback;
	if (fb_register_client(&core_data->fb_notifier))
		ts_err("Failed to register fb notifier client:%d", r);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	core_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	core_data->early_suspend.resume = goodix_ts_lateresume;
	core_data->early_suspend.suspend = goodix_ts_earlysuspend;
	register_early_suspend(&core_data->early_suspend);
#endif
	/*create sysfs files*/
	goodix_ts_sysfs_init(core_data);

	/* esd protector */
	goodix_ts_esd_init_gt9886(core_data);

#ifdef CONFIG_TPD_UFP_MAC
	tpd_ufp_register(core_data);
#endif

	return 0;
exit:
	if (ts_dev->board_data.pen_enable) {
		goodix_ts_pen_dev_remove_gt9886(core_data);
	}
err_finger:
	goodix_ts_input_dev_remove_gt9886(core_data);
	return r;
}

/**
 * goodix_ts_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(void *dev)
{
	struct goodix_ts_core *core_data = NULL;
	struct goodix_ts_device *ts_device;
	struct platform_device *pdev = dev;
	struct i2c_client *client;
	int r = 0;

	ts_info("goodix_ts_probe IN");

	ts_device = pdev->dev.platform_data;
	if (!ts_device || !ts_device->hw_ops) {
		ts_err("Invalid touch device");
		return -ENOMEM;
	}

	core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_ts_core),
				 GFP_KERNEL);
	if (!core_data) {
		ts_err("Failed to allocate memory for core data");
		return -ENOMEM;
	}

	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->ts_dev = ts_device;
	ts_device->core_data = core_data;
	platform_set_drvdata(pdev, core_data);

	r = goodix_ts_power_init(core_data);
	if (r < 0)
		goto out;

	r = goodix_ts_power_on_gt9886(core_data);
	if (r < 0)
		goto out;

#ifdef CONFIG_PINCTRL
	/* Pinctrl handle is optional. */
	r = goodix_ts_pinctrl_init(core_data);
	if (!r && core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					 core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* get GPIO resource */
	r = goodix_ts_gpio_setup(core_data);
	if (r < 0)
		goto out;

	/* confirm it's goodix touch dev or not */
	r = ts_device->hw_ops->dev_confirm(ts_device);
	if (r) {
		ts_err("goodix device confirm failed[skip]");
		goto out;
	}
	msleep(100);

	/* Try start a thread to get config-bin info */
	r = goodix_start_later_init(core_data);
	if (r) {
		ts_info("Failed start cfg_bin_proc");
		goto out;
	}

	/* generic notifier callback */
	core_data->ts_notifier.notifier_call = goodix_generic_noti_callback;
	goodix_ts_register_notifier_gt9886(&core_data->ts_notifier);

	core_data->zc.is_single_tap = 0;
	core_data->zc.is_set_single_in_suspend = 0;
	core_data->zc.is_smart_cover = 0;
	core_data->zc.is_set_wakeup_in_suspend = 0;
	core_data->zc.is_wakeup_gesture = 0;
	core_data->zc.is_wake_irq = 0;
	core_data->zc.is_one_key = 0;
	core_data->zc.is_set_onekey_in_suspend = 0;
	core_data->zc.is_play_game = 0;
	core_data->zc.level = DEFAULT_SUPPRESS_LEVEL;
	mutex_init(&(core_data->zc.rawdata_read_lock));

	/* interface */
	goodix_tpd_register_fw_class(core_data);
	setup_suppress_func(core_data);

out:

	if (r) {
		core_data->initialized = 0;
		client = container_of(ts_device->dev, struct i2c_client, dev);
		client->addr = 1;
	} else
		core_data->initialized = 1;
	goodix_modules_gt9886.core_data = core_data;
	ts_info("goodix_ts_probe OUT, r:%d", r);
	/* wakeup ext module register work */
	complete_all(&goodix_modules_gt9886.core_comp);

	return r;
}

static int goodix_ts_workqueue_probe(struct platform_device *pdev)
{
	return tp_probe_on_queue(pdev, goodix_ts_probe, goodix_ts_core_release);
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = platform_get_drvdata(pdev);

	core_data->initialized = 0;
	if (atomic_read(&core_data->ts_esd.esd_on))
		goodix_ts_esd_off(core_data);
	goodix_remove_all_ext_modules();
	goodix_ts_power_off_gt9886(core_data);
	goodix_debugfs_exit();
	goodix_ts_sysfs_exit(core_data);
	device_init_wakeup(&core_data->pdev->dev, 0);
	/* can't free the memory for tools or gesture module
	kfree(core_data); */

	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
#endif
};
#endif

static const struct platform_device_id ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &dev_pm_ops,
#endif
	},
	.probe = goodix_ts_workqueue_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};

int goodix_ts_core_init(void)
{
	ts_info("Core layer init");

	/*goodix_tools_init_gt9886();
	goodix_gsx_gesture_init_gt9886();
	goodix_fwu_module_init();*/

	if (!goodix_modules_gt9886.initilized) {
		/* this may init by outer modules register event */
		ts_info("init modules struct");
		goodix_modules_gt9886.initilized = true;
		INIT_LIST_HEAD(&goodix_modules_gt9886.head);
		mutex_init(&goodix_modules_gt9886.mutex);
		init_completion(&goodix_modules_gt9886.core_comp);
	}

	goodix_debugfs_init();
	return platform_driver_register(&goodix_ts_driver);
}

/* uninit module manually */
int goodix_ts_core_release(void)
{
	ts_info("goodix core module removed");

	platform_driver_unregister(&goodix_ts_driver);
	goodix_ts_dev_release();
	/*goodix_gsx_gesture_exit_gt9886();
	goodix_fwu_module_exit_gt9886();
	goodix_tools_exit_gt9886();*/
	return 0;
}
