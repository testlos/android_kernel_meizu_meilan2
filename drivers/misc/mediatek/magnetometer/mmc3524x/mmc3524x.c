/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/time.h>
#include <linux/hrtimer.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>

#include <cust_mag.h>
#include "mmc3524x.h"
#include <hwmsen_helper.h>
#include "mag.h"

/* #define DEBUG 1 */
#define MMC3524X_DEV_NAME         "mmc3524x"
#define DRIVER_VERSION          "1.0.0"

/* #define MMC3524X_DEBUG		1 */
/* #define MMC3524X_DEBUG_MSG	1 */
/* #define MMC3524X_DEBUG_FUNC	1 */
/* #define MMC3524X_DEBUG_DATA	1 */
#define MAX_FAILURE_COUNT	3
#define MMC3524X_RETRY_COUNT	3
#define MMC3524X_DEFAULT_DELAY	20
#define	MMC3524X_BUFSIZE  0x20
#define MMC3524X_DELAY_RM	10	/* ms */
#define READMD			0

char *msensor_name = NULL;

#define MMCTAG                  "mmc3524x"
#define MMCFUN(f)               pr_debug(MMCTAG " %s\n", __func__)
#define MMCERR(fmt, args...) \
	pr_err(MMCTAG " %s %d :" fmt "\n", __func__, __LINE__, ##args)
#define MMCINFO(fmt, args...)   pr_info(MMCTAG fmt, ##args)
#define MMCDBG(fmt, args...)    pr_debug(MMCTAG fmt, ##args)

static struct i2c_client *this_client;
struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/* calibration msensor and orientation data */
static int sensor_data[CALIBRATION_DATA_SIZE];/* 16]; CALIBRATION_DATA_SIZE]; */
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC3524X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static const struct i2c_device_id mmc3524x_i2c_id[] = {
	{MMC3524X_DEV_NAME, 0}, {}
};
/* static struct i2c_board_info __initdata i2c_mmc3524x =
{ I2C_BOARD_INFO("mmc3524x", (MMC3524X_I2C_ADDR>>1))}; */

static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc3524x_i2c_remove(struct i2c_client *client);
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc3524x_resume(struct i2c_client *client);

enum MMC_TRC {
	MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
};

#define MMC3524X_DELAY_TM	10	/* ms */
#define MMC3524X_DELAY_SET	50	/* ms */
#define MMC3524X_DELAY_RST	50	/* ms */
#define MMC3524X_DELAY_STDN	1	/* ms */

#define MMC3524X_RESET_INTV	250

static u32 read_idx;

struct mmc3524x_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif
};

#ifdef CONFIG_OF
static const struct of_device_id mmc_of_match[] = {
	{ .compatible = "mediatek,mmc3524x", },
	{},
};
#endif

static struct i2c_driver mmc3524x_i2c_driver = {
	.driver = {
		.name  = MMC3524X_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mmc_of_match,
#endif
	},
	.probe  = mmc3524x_i2c_probe,
	.remove = mmc3524x_i2c_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = mmc3524x_suspend,
	.resume     = mmc3524x_resume,
#endif
	.id_table = mmc3524x_i2c_id,
};

static int mmc3524x_local_init(void);
static int mmc3524x_remove(void);
static int mmc3524x_init_flag = -1; /* 0<==>OK -1 <==> fail */
static struct mag_init_info mmc3524x_init_info = {
	.name = "mmc3524x",
	.init = mmc3524x_local_init,
	.uninit = mmc3524x_remove,
};

static atomic_t dev_open_count;

#if 0
static int mmc3524x_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = MMC3524X_REG_CTRL;
	/* struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client); */

	if (hwmsen_read_byte(client, addr, databuf)) {
		MMCDBG("mmc3524x: read power ctl register err and retry!\n");
		if (hwmsen_read_byte(client, addr, databuf)) {
			MMCDBG("mmc3524x: read power ctl register retry err!\n");
			return -1;
		}
	}

	databuf[0] &= ~MMC3524X_CTRL_TM;

	if (enable == true)
		databuf[0] |= MMC3524X_CTRL_TM;

	databuf[1] = databuf[0];
	databuf[0] = MMC3524X_REG_CTRL;

	res = i2c_master_send(client, databuf, 0x2);

	if (res <= 0) {
		MMCDBG("mmc3524x: set power mode failed!\n");
		return -1;
	}
	MMCDBG("mmc3524x: set power mode ok %x!\n", databuf[1]);

	return 0;
}
#endif

static void mmc3524x_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on;

#ifndef CONFIG_OF
	if (hw->power_id != MT65XX_POWER_NONE) {
		MMCDBG("power %s\n", on ? "on" : "off");
		if (power_on == on)
			MMCDBG("ignore power control: %d\n", on);
		else if (on) {
			if (!hwPowerOn(hw->power_id, hw->power_vol, "mmc3524x"))
				MMCERR("power on fails!!\n");
		} else {
			if (!hwPowerDown(hw->power_id, "mmc3524x"))
				MMCERR("power off fail!!\n");
		}
	}
#endif
	power_on = on;
}
static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;

#ifdef DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif

	/* Caller should check parameter validity.*/
	if ((rxData == NULL) || (length < 1))
		return -EINVAL;

	for (loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++) {
		this_client->addr = (this_client->addr & I2C_MASK_FLAG) | (I2C_WR_FLAG);
		if (i2c_master_send(this_client, (const char *)rxData, ((length<<0X08) | 0X01)))
			break;

		MMCDBG("I2C_RxData delay!\n");
		mdelay(10);
	}

	if (loop_i >= MMC3524X_RETRY_COUNT) {
		MMCERR("%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_I2C_DEBUG) {
		MMCDBG(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for (i = 0; i < length; i++)
			MMCDBG(KERN_INFO " %02x", rxData[i]);

		MMCDBG(KERN_INFO "\n");
	}
#endif
	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

#ifdef DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
		return -EINVAL;

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for (loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++) {
		if (i2c_master_send(this_client, (const char *)txData, length) > 0)
			break;

		MMCDBG("I2C_TxData delay!\n");
		mdelay(10);
	}

	if (loop_i >= MMC3524X_RETRY_COUNT) {
		MMCERR("%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_I2C_DEBUG) {
		MMCDBG(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for (i = 0; i < (length-1); i++)
			MMCDBG(KERN_INFO " %02x", txData[i + 1]);

		MMCDBG(KERN_INFO "\n");
	}
#endif
	return 0;
}

/*static int mmc3524x_dev_init(struct i2c_client *client)
{       char tmp[2];

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_SET);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_SET;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = 0;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_SET);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_RST);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_RESET;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = 0;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);

	tmp[0] = MMC3524X_REG_BITS;
	tmp[1] = MMC3524X_BITS_SLOW_16;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_TM);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_TM;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_TM);

}*/

/* Daemon application save the data */
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{
#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

/*#ifdef DEBUG
	pr_info("Get daemon data: acc %d, %d, %d, %d\n",
		sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]);
	pr_info("Get daemon data: mag %d, %d, %d, %d\n",
		sensor_data[4], sensor_data[5], sensor_data[6], sensor_data[7]);
	pr_info("Get daemon data: ypr %d, %d, %d, %d\n",
		sensor_data[8], sensor_data[9], sensor_data[10], sensor_data[11]);
	pr_info("Get daemon data: gyr %d, %d, %d, %d\n",
		sensor_data[12], sensor_data[13], sensor_data[14], sensor_data[15]);
	pr_info("Get daemon data: rot %d, %d, %d, %d\n",
		sensor_data[16], sensor_data[17], sensor_data[18], sensor_data[19]);
	pr_info("Get daemon data: grv %d, %d, %d, %d\n",
		sensor_data[20], sensor_data[21], sensor_data[22], sensor_data[23]);
	pr_info("Get daemon data: lac %d, %d, %d, %d\n",
		sensor_data[24], sensor_data[25], sensor_data[26], sensor_data[27]);
	if (atomic_read(&data->trace) & MMC_HWM_DEBUG) {

	}
#endif*/

	return 0;

}
static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0, 0, 0, 0, 0, 0};
	/* ktime_t expires; */
	/* int wait_n = 0; */
	/* int MD_times = 0; */
	static int last_data[3];
	struct timespec time1, time2, time3;/* ,time4,delay,aa; */
#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
#endif
/* set_current_state(TASK_INTERRUPTIBLE); */
time1 = current_kernel_time();

	if (size < 3)
		return -1;

	mutex_lock(&read_i2c_xyz);
#if 0
	/* do RESET/SET every MMC3524X_RESET_INTV times read */
	if (!(read_idx % MMC3524X_RESET_INTV)) {
		/* SET */
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_SET;
		/* not check return value here, assume it always OK */
		I2C_TxData(data, 2);
		/* delay.tv_sec = 0; */
		/* delay.tv_nsec = MMC3524X_DELAY_STDN *1000; */
		/* hrtimer_nanosleep(&delay, &aa, HRTIMER_MODE_REL, CLOCK_MONOTONIC); */
		msleep(MMC3524X_DELAY_STDN);
	}

	/* wait TM done for coming data read */
/* time2 = current_kernel_time(); */

	time3 = current_kernel_time();
	/* read xyz raw data */
	read_idx++;
	data[0] = MMC3524X_REG_DATA;
	if (I2C_RxData(data, 6) < 0) {
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	vec[0] = data[0] << 8 | data[1];
	vec[1] = data[2] << 8 | data[3];
	vec[2] = data[4] << 8 | data[5];
	for (wait_n = 0; wait_n < 10; wait_n++) {
		if ((vec[0] != 0) && (vec[1] != 0) && (vec[2] != 0))
			break;
		data[0] = MMC3524X_REG_DATA;
		if (I2C_RxData(data, 6) < 0) {
			mutex_unlock(&read_i2c_xyz);
			return -EFAULT;
		}
		vec[0] = data[0] << 8 | data[1];
		vec[1] = data[2] << 8 | data[3];
		vec[2] = data[4] << 8 | data[5];
	}
	time4 = current_kernel_time();
	/* MMCDBG("start time %d - %d, sleep %d - %d , stop %d !\n",
		 time1.tv_sec, time1.tv_nsec, time2.tv_nsec, time3.tv_nsec, time4.tv_nsec); */
#ifdef DEBUG
	if (atomic_read(&clientdata->trace) & MMC_DATA_DEBUG) {
		MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
		if ((vec[0]-last_data[0] > 100) || (last_data[0]-vec[0] > 100) ||
			(vec[1]-last_data[1] > 100) || (last_data[1]-vec[1] > 100) ||
			(vec[2]-last_data[2] > 100) || (last_data[2]-vec[2] > 100)) {
			msleep(3000);
			MMCDBG("data error!\n");
		}
	}
#endif
	/* send TM cmd before read */
	data[0] = MMC3524X_REG_CTRL;
	data[1] = MMC3524X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
#endif
	time2 = current_kernel_time();

	if (!(read_idx % MMC3524X_RESET_INTV)) {
		/* RM */
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		/* not check return value here, assume it always OK */
		I2C_TxData(data, 2);
		/* wait external capacitor charging done for next RM */
		msleep(MMC3524X_DELAY_SET);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_SET;
		I2C_TxData(data, 2);
		msleep(1);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		I2C_TxData(data, 2);
		msleep(1);

		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		I2C_TxData(data, 2);
		msleep(MMC3524X_DELAY_RST);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_RESET;
		I2C_TxData(data, 2);
		msleep(1);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		I2C_TxData(data, 2);
		msleep(1);
	}
	time3 = current_kernel_time();
	/* send TM cmd before read */
	data[0] = MMC3524X_REG_CTRL;
	data[1] = MMC3524X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3524X_DELAY_TM);

#if READMD
	/* Read MD */
		data[0] = MMC3524X_REG_DS;
		I2C_RxData(data, 1);
		while (!(data[0] & 0x01)) {
			msleep(1);
			/* Read MD again*/
			data[0] = MMC3524X_REG_DS;
			I2C_RxData(data, 1);
			if (data[0] & 0x01)
				break;
			MD_times++;
			if (MD_times > 3) {
				MMCDBG("TM not work!!");
				mutex_unlock(&read_i2c_xyz);
				return -EFAULT;
			}
		}
#endif
	/* read xyz raw data */
	read_idx++;
	data[0] = MMC3524X_REG_DATA;
	if (I2C_RxData(data, 6) < 0) {
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];
	vec[2] = 65536 - vec[2];
#ifdef DEBUG
	if (atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
		MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);

#endif
	mutex_unlock(&read_i2c_xyz);
	last_data[0] = vec[0];
	last_data[1] = vec[1];
	last_data[2] = vec[2];
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;

	err = ECS_ReadXYZData(data, 3);
	if (err != 0) {
		MMCERR("MMC3524X_IOC_TM failed\n");
		return -1;
	}

	/* sensitivity 2048 count = 1 Guass = 100uT */
	data[0] = (data[0] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[1] = (data[1] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[2] = (data[2] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;

	return err;
}
static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int mmc3524x_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= MMC3524X_BUFSIZE - 1))
		return -1;

	if (!this_client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "mmc3524x Chip");
	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3524X_BUFSIZE];

	mmc3524x_ReadChipInfo(strbuf, MMC3524X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];
	char strbuf[MMC3524X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0], sensordata[1], sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[MMC3524X_BUFSIZE];

	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0], tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if (!kstrtoint(buf, 0, &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt))
			MMCERR("HWMSEN_GET_CONVERT function error!\n");
		else if (!hwmsen_get_convert(data->hw->direction, &data->cvt))
			MMCERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		else {
			MMCERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else
		MMCERR("invalid format = '%s'\n", buf);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if (data->hw) {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	} else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);

	if (NULL == obj) {
		MMCERR("mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;

	if (NULL == obj) {
		MMCERR("mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	if (!kstrtoint(buf, 0, &trace))
		atomic_set(&obj->trace, trace);
	else
		MMCERR("invalid content: '%s', length = %lu\n", buf, count);

	return count;
}
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%s", "memsicd3524x");
}

/* for msesensor engineer mode auto test */
#define abs_msensor(a) (((a) < 0) ? -(a) : (a))
int data_initial[3] = {0, 0, 0};
int data_magnet_close[3] = {0, 0, 0};
int data_magnet_leave[3] = {0, 0, 0};

#if 0
static ssize_t show_autotest_testID(struct device_driver *ddri, char *buf)
{
	/* mmc3524x_dev_init(NULL); */

	/*Read initial data*/
	ECS_ReadXYZData(data_initial, 3);

	/*Data available??*/
	if ((65535 == data_initial[0])
			|| (65535 == data_initial[1])
			|| (65535 == data_initial[2]))
		goto READ_DATA_FAIL;

	MMCDBG("%s : data_initial[x] = %d, data_initial[y] = %d, data_initial[z] = %d\n",
			__func__, data_initial[0], data_initial[1], data_initial[2]);

	return 0;

/*READ_ID_FAIL:
	MMCERR("Auto Test: read id error!!");
	return -1;
RESET_DEV_FAIL:
	MMCERR("Auto Test: reset device error!!");
	return -1;*/
READ_DATA_FAIL:
	MMCERR("Auto Test: read data error!!");
	return -1;
}
#endif

static ssize_t show_autotest_magnetclose(struct device_driver *ddri, char *buf)
{
	int i;

	/*50 loops which take about 5 seconds*/
	for (i = 0; i < 50; i++) {
		if (ECS_ReadXYZData(data_magnet_close, 3)) {
			MMCERR("Auto Test:close read data fail");
			break;
		}

		/*Data available??*/
		if ((65535 == data_magnet_close[0])
			|| (65535 == data_magnet_close[1])
			|| (65535 == data_magnet_close[2])) {
			MMCERR("Auto Test:close read data fail");
			break;
		}

		MMCERR("%d, %d, %d\n", data_magnet_close[0], data_magnet_close[1], data_magnet_close[2]);

		if ((abs_msensor(data_magnet_close[0] - data_initial[0]) >= 50)
				&& (abs_msensor(data_magnet_close[1] - data_initial[1]) >= 50)
				&& (abs_msensor(data_magnet_close[2] - data_initial[2]) >= 50))
			return 0;

		msleep(100);
	}
	return -1;
}

static ssize_t show_autotest_magnetleave(struct device_driver *ddri, char *buf)
{
	return 0;
}

static ssize_t show_autotest_get_ic_mode(struct device_driver *ddri, char *buf)
{
	MMCDBG("msensor is mmc416x\n");
	return 0;
}

static DRIVER_ATTR(daemon, S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace, S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);

/* add for msensor engineer auto test */
/* static DRIVER_ATTR(test_id,       S_IRUGO, show_autotest_testID, NULL); */
static DRIVER_ATTR(magnet_close, S_IRUGO, show_autotest_magnetclose, NULL);
static DRIVER_ATTR(magnet_leave, S_IRUGO, show_autotest_magnetleave, NULL);
static DRIVER_ATTR(get_ic_modle, S_IRUGO, show_autotest_get_ic_mode, NULL);

static struct driver_attribute *mmc3524x_attr_list[] = {
	&driver_attr_daemon,
	&driver_attr_magnet_close,
	&driver_attr_magnet_leave,
	&driver_attr_get_ic_modle,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	/* add for msensor engineer auto test */
	/* &driver_attr_test_id, */
};

static int mmc3524x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, mmc3524x_attr_list[idx]);
		if (err) {
			MMCERR("driver_create_file (%s) = %d\n", mmc3524x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int mmc3524x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, mmc3524x_attr_list[idx]);

	return 0;
}

static int mmc3524x_open(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if (atomic_read(&obj->trace) & MMC_CTR_DEBUG)
		MMCDBG("Open device node:mmc3524x\n");

	ret = nonseekable_open(inode, file);

	return ret;
}

static int mmc3524x_release(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);

	atomic_dec(&dev_open_count);
	if (atomic_read(&obj->trace) & MMC_CTR_DEBUG)
		MMCDBG("Release device node:mmc3524x\n");

	return 0;
}

/* static int mmc3524x_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) */
static long mmc3524x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[MMC3524X_BUFSIZE]; /* for chip information */
	int value[CALIBRATION_DATA_SIZE]; /* 16]; for SET_YPR */
	int delay; /* for GET_DELAY */
	int status; /* for OPEN/CLOSE_STATUS */
	short sensor_status; /* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	unsigned char reg_addr;
	unsigned char reg_value;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
	struct hwm_sensor_data *osensor_data;
	uint32_t enable;

	switch (cmd) {
	case MMC31XX_IOC_TM:
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_TM;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_TM failed\n");
			return -EFAULT;
		}
		/* wait TM done for coming data read */
		/* msleep(MMC3524X_DELAY_TM); */
		break;

	case MMC31XX_IOC_SET:
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_SET failed\n");
			return -EFAULT;
		}
		/* wait external capacitor charging done for next SET/RESET */
		msleep(MMC3524X_DELAY_SET);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_SET;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_SET failed2\n");
			return -EFAULT;
		}
		msleep(1);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_SET failed3\n");
			return -EFAULT;
		}
		msleep(MMC3524X_DELAY_SET);
		break;

	case MMC31XX_IOC_RESET:
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_RESET failed\n");
			return -EFAULT;
		}
		/* wait external capacitor charging done for next SET/RESET */
		msleep(MMC3524X_DELAY_RST);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_RESET;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_RESET failed2\n");
			return -EFAULT;
		}
		msleep(1);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		if (I2C_TxData(data, 2) < 0) {
			MMCERR("MMC3524X_IOC_RESET failed3\n");
			return -EFAULT;
		}
		msleep(1);
		break;

	case MMC31XX_IOC_READ:
		data[0] = MMC3524X_REG_DATA;
		if (I2C_RxData(data, 6) < 0) {
			MMCERR("MMC3524X_IOC_READ failed\n");
			return -EFAULT;
		}
		vec[0] = data[1] << 8 | data[0];
		vec[1] = data[3] << 8 | data[2];
		vec[2] = data[5] << 8 | data[4];
		/* vec[2] = 65536 - vec[2]; */
		#ifdef DEBUG
		if (atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
			MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
		#endif
		if (copy_to_user(argp, vec, sizeof(vec))) {
			MMCERR("MMC3524X_IOC_READ: copy to user failed\n");
			return -EFAULT;
		}
		break;

	case MMC31XX_IOC_READXYZ:
		ECS_ReadXYZData(vec, 3);
		if (copy_to_user(argp, vec, sizeof(vec))) {
			MMCERR("MMC3524X_IOC_READXYZ: copy to user failed\n");
			return -EFAULT;
		}
		break;

	case ECOMPASS_IOC_GET_DELAY:
		delay = mmcd_delay;
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			MMCERR("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECOMPASS_IOC_SET_YPR:
		if (argp == NULL) {
			MMCDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(value, argp, sizeof(int)*12)) {
			MMCDBG("copy_from_user failed.");
			return -EFAULT;
		}
		ECS_SaveData(value);
		break;

	case ECOMPASS_IOC_GET_OPEN_STATUS:
		status = ECS_GetOpenStatus();
		if (copy_to_user(argp, &status, sizeof(status))) {
			MMCDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECOMPASS_IOC_GET_MFLAG:
		sensor_status = atomic_read(&m_flag);
		if (copy_to_user(argp, &sensor_status, sizeof(sensor_status))) {
			MMCDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECOMPASS_IOC_GET_OFLAG:
		sensor_status = atomic_read(&o_flag);
		if (copy_to_user(argp, &sensor_status, sizeof(sensor_status))) {
			MMCDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_READ_CHIPINFO:
		if (argp == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}

		mmc3524x_ReadChipInfo(buff, MMC3524X_BUFSIZE);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;

	case MSENSOR_IOCTL_READ_SENSORDATA:
		if (argp == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		ECS_GetRawData(vec);
		sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;

	case ECOMPASS_IOC_GET_LAYOUT:
		status = atomic_read(&clientdata->layout);
		if (copy_to_user(argp, &status, sizeof(status))) {
			MMCDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_SENSOR_ENABLE:
		if (argp == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		if (copy_from_user(&enable, argp, sizeof(enable))) {
			MMCDBG("copy_from_user failed.");
			return -EFAULT;
		}
		MMCDBG("MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\n", enable);
		if (1 == enable) {
			atomic_set(&o_flag, 1);
			atomic_set(&open_flag, 1);
		} else {
			atomic_set(&o_flag, 0);
			if (atomic_read(&m_flag) == 0)
				atomic_set(&open_flag, 0);
			}
		wake_up(&open_wq);
		break;

	case MMC3524X_IOC_READ_REG:
		MMCDBG("MMC3524X_IOC_READ_REG\n");
		if (copy_from_user(&reg_addr, argp, sizeof(delay)))
			return -EFAULT;
		data[0] = reg_addr;
		if (I2C_TxData(data, 1) < 0) {
			/* mutex_unlock(&ecompass_lock); */
			return -EFAULT;
		}
		MMCDBG("planar Read register No. 0x%0x\n", data[0]);
		reg_value = data[0];
		if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
			/* mutex_unlock(&ecompass_lock); */
			return -EFAULT;
		}
		break;
	case MMC3524X_IOC_WRITE_REG:
		MMCDBG("MMC3524X_IOC_WRITE_REG\n");
		if (copy_from_user(&data, argp, sizeof(data)))
			return -EFAULT;
		if (I2C_TxData(data, 2) < 0) {
			/* mutex_unlock(&ecompass_lock); */
			return -EFAULT;
		}
		MMCDBG("planar Write '0x%0x' to  register No. 0x%0x\n", data[0], data[1]);
		break;

	case MMC3524X_IOC_READ_REGS:
		MMCDBG("MMC3524X_IOC_READ_REGS\n");
		if (copy_from_user(&data, argp, sizeof(data)))
			return -EFAULT;
		MMCDBG("planar Read %d registers from 0x%0x\n", data[1], data[0]);
		if (I2C_RxData(data, data[1]) < 0) {
			/* mutex_unlock(&ecompass_lock); */
			return -EFAULT;
		}
		MMCDBG("data: %x %x %x\n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
		if (copy_to_user(argp, data, sizeof(data))) {
			/* mutex_unlock(&ecompass_lock); */
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (argp == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}

		/* AKECS_GetRawData(buff, AKM8975_BUFSIZE); */
		osensor_data = (struct hwm_sensor_data *)buff;
		mutex_lock(&sensor_data_mutex);

		osensor_data->values[0] = sensor_data[8] * CONVERT_O;
		osensor_data->values[1] = sensor_data[9] * CONVERT_O;
		osensor_data->values[2] = sensor_data[10] * CONVERT_O;
		osensor_data->status = sensor_data[11];
		osensor_data->value_divide = CONVERT_O_DIV;

		mutex_unlock(&sensor_data_mutex);

		sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
			osensor_data->values[2], osensor_data->status, osensor_data->value_divide);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;

	default:
		MMCERR(" not supported = 0x%04x", cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}
#ifdef CONFIG_COMPAT
static long mmc3524x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *arg64 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		MMCERR("file->f_op OR file->f_op->unlocked_ioctl is null!\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_MMC31XX_IOC_TM:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_TM, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_TM is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_SET:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_SET, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_SET is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_RM:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RM, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_RM is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_RESET:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RESET, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_RESET is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_RRM:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RRM, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_RRM is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_READ:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READ, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_READ is failed!\n");
		break;

	case COMPAT_MMC31XX_IOC_READXYZ:
		ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READXYZ, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC31XX_IOC_READXYZ is failed!\n");
		break;

	case COMPAT_MMC3524X_IOC_READ_REG:
		ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REG, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC3524X_IOC_READ_REG is failed!\n");
		break;

	case COMPAT_MMC3524X_IOC_WRITE_REG:
		ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_WRITE_REG, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC3524X_IOC_WRITE_REG is failed!\n");
		break;

	case COMPAT_MMC3524X_IOC_READ_REGS:
		ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REGS, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MMC3524X_IOC_READ_REGS is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_GET_DELAY:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_DELAY, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_GET_DELAY is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_SET_YPR:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_SET_YPR, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_SET_YPR is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OPEN_STATUS, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_GET_MFLAG:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_MFLAG, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_GET_MFLAG is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_GET_OFLAG:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OFLAG, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_GET_OFLAG is failed!\n");
		break;

	case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
		if (arg64 == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
		break;

	case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
		if (arg64 == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MSENSOR_IOCTL_READ_SENSORDATA is failed!\n");
		break;

	case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
		ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_LAYOUT, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_ECOMPASS_IOC_GET_LAYOUT is failed!\n");
		break;

	case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
		if (arg64 == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE is failed!\n");
		break;

	case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (arg64 == NULL) {
			MMCERR("IO parameter pointer is NULL!\n");
			break;
		}
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA, (unsigned long)arg64);
		if (ret < 0)
			MMCERR("COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA is failed!\n");
		break;

	default:
		MMCERR(" not supported = 0x%04x", cmd);
		return -ENOIOCTLCMD;
	}

	return ret;
}

#endif

static const struct file_operations mmc3524x_fops = {
	/* .owner = THIS_MODULE, */
	.open = mmc3524x_open,
	.release = mmc3524x_release,
	.unlocked_ioctl = mmc3524x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc3524x_compat_ioctl,
#endif
};

static struct miscdevice mmc3524x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &mmc3524x_fops,
};

int mmc3524x_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *msensor_data;

#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_operate");

#endif
	MMCERR("command %d!\n", command);
	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&m_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
				if (atomic_read(&o_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
			/* TODO: turn device into standby or normal mode */
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			msensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			msensor_data->values[0] = sensor_data[4] * CONVERT_M;
			msensor_data->values[1] = sensor_data[5] * CONVERT_M;
			msensor_data->values[2] = sensor_data[6] * CONVERT_M;
			msensor_data->status = sensor_data[7];
			msensor_data->value_divide = CONVERT_M_DIV;

			mutex_unlock(&sensor_data_mutex);
			MMCINFO("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
								msensor_data->values[0], msensor_data->values[1], msensor_data->values[2],
								msensor_data->value_divide, msensor_data->status);
#ifdef DEBUG
			if (atomic_read(&data->trace) & MMC_HWM_DEBUG) {
				MMCDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
					msensor_data->values[0], msensor_data->values[1], msensor_data->values[2],
					msensor_data->value_divide, msensor_data->status);
			}
#endif
		}
		break;

	default:
		MMCERR("msensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

int mmc3524x_orientation_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *osensor_data;
#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_orientation_operate");

#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if (atomic_read(&m_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			osensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);
#ifdef DEBUG
			if (atomic_read(&data->trace) & MMC_HWM_DEBUG) {
				MMCDBG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
					osensor_data->values[0], osensor_data->values[1], osensor_data->values[2],
					osensor_data->value_divide, osensor_data->status);
			}
#endif
		}
		break;

	default:
		MMCERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}
int mmc3524x_gyro_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *gysensor_data;
#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_gyro_operate");

#endif

	switch (command) {
	case SENSOR_DELAY:
		MMCFUN("mmc3524x_gyro_delay");
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		MMCFUN("mmc3524x_gyro_enable");
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if (atomic_read(&m_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
		}
		break;

	case SENSOR_GET_DATA:
		MMCFUN("mmc3524x_gyro_data");
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gysensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			gysensor_data->values[0] = sensor_data[12] * CONVERT_GY;
			gysensor_data->values[1] = sensor_data[13] * CONVERT_GY;
			gysensor_data->values[2] = sensor_data[14] * CONVERT_GY;
			gysensor_data->status = sensor_data[15];
			gysensor_data->value_divide = CONVERT_GY_DIV;

			mutex_unlock(&sensor_data_mutex);
#ifdef DEBUG
			if (atomic_read(&data->trace) & MMC_HWM_DEBUG) {
				MMCDBG("Hwm get gyr-sensor data: %d, %d, %d. divide %d, status %d!\n",
					gysensor_data->values[0], gysensor_data->values[1], gysensor_data->values[2],
					gysensor_data->value_divide, gysensor_data->status);
			}
#endif
		}
		break;

	default:
		MMCERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

int mmc3524x_roation_vector(void *self, uint32_t command, void *buff_in,
		int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *vector_data;

#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);

	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_roation_vector");
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if (atomic_read(&m_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			vector_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			vector_data->values[0] = sensor_data[16] * CONVERT_VEC;
			vector_data->values[1] = sensor_data[17] * CONVERT_VEC;
			vector_data->values[2] = sensor_data[18] * CONVERT_VEC;
			vector_data->status = sensor_data[19];
			vector_data->value_divide = CONVERT_VECTOR_DIV;

			mutex_unlock(&sensor_data_mutex);
#ifdef DEBUG
			/* if (atomic_read(&data->trace) & MMC_HWM_DEBUG) */
			{
				pr_info("Hwm get rat-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0], vector_data->values[1], vector_data->values[2],
					vector_data->value_divide, vector_data->status);
			}
#endif
		}
		break;

	default:
		MMCERR("rat operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

int mmc3524x_gravity(void *self, uint32_t command, void *buff_in,
		int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *vector_data;

#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_gravity");
#endif

	switch (command) {
	case SENSOR_DELAY:
		MMCFUN("mmc3524x_gravity\n");
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if (atomic_read(&m_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			vector_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			vector_data->values[0] = sensor_data[20] * GRAVITY_FACTOR / 8192;
			vector_data->values[1] = sensor_data[21] * GRAVITY_FACTOR / 8192;
			vector_data->values[2] = sensor_data[22] * GRAVITY_FACTOR / 8192;
			vector_data->status = sensor_data[23];
			vector_data->value_divide = 1000;

			mutex_unlock(&sensor_data_mutex);
#ifdef DEBUG
			/* if (atomic_read(&data->trace) & MMC_HWM_DEBUG) */
			{
				pr_info("Hwm get grv-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0], vector_data->values[1], vector_data->values[2],
					vector_data->value_divide, vector_data->status);
			}
#endif
		}
		break;

	default:
		MMCERR("grv operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

int mmc3524x_linear_acc(void *self, uint32_t command, void *buff_in,
		int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *vector_data;

#ifdef DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#ifdef DEBUG
	if (atomic_read(&data->trace) & MMC_FUN_DEBUG)
		MMCFUN("mmc3524x_linear_acc");
#endif

	switch (command) {
	case SENSOR_DELAY:
		MMCFUN("mmc3524x_linear_acc\n");
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 20)
				mmcd_delay = 20;
			mmcd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MMCERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value == 1) {
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if (atomic_read(&m_flag) == 0)
					atomic_set(&open_flag, 0);
			}
			wake_up(&open_wq);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MMCERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			vector_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			vector_data->values[0] = sensor_data[24] * GRAVITY_FACTOR / 8192;
			vector_data->values[1] = sensor_data[25] * GRAVITY_FACTOR / 8192;
			vector_data->values[2] = sensor_data[26] * GRAVITY_FACTOR / 8192;
			vector_data->status = sensor_data[27];
			vector_data->value_divide = 1000;

			mutex_unlock(&sensor_data_mutex);
#ifdef DEBUG
			/* if (atomic_read(&data->trace) & MMC_HWM_DEBUG) */
			{
				pr_info("Hwm get lineacc-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0], vector_data->values[1], vector_data->values[2],
					vector_data->value_divide, vector_data->status);
			}
#endif
		}
		break;
	default:
		MMCERR("lineacc operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

#ifndef	CONFIG_HAS_EARLYSUSPEND

static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);

	if (msg.event == PM_EVENT_SUSPEND)
		mmc3524x_power(obj->hw, 0);
	return 0;
}

static int mmc3524x_resume(struct i2c_client *client)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);

	mmc3524x_power(obj->hw, 1);
	return 0;
}

#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/

static void mmc3524x_early_suspend(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);

	if (NULL == obj)
		MMCERR("null pointer!!\n");
	else if (mmc3524x_SetPowerMode(obj->client, false))
		MMCDBG("mmc3524x: write power control fail!!\n");
}

static void mmc3524x_late_resume(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);

	if (NULL == obj)
		MMCERR("null pointer!!\n");
	else
		mmc3524x_power(obj->hw, 1);
}

#endif /*CONFIG_HAS_EARLYSUSPEND*/

static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mmc3524x_i2c_data *data;
	char tmp[2];
	int err = 0;
	struct hwmsen_object sobj_m, sobj_o, sobj_gy, sobj_vec, sobj_grv, sobj_linacc;

	MMCDBG("%s: ++++\n", __func__);
	MMCINFO("%s +\n", __func__);
	data = kmalloc(sizeof(struct mmc3524x_i2c_data), GFP_KERNEL);
	if (!data) {
		MMCERR("Cannot allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}

	memset(data, 0, sizeof(struct mmc3524x_i2c_data));

	data->hw = hw;

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);
	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	data->client = client;
	i2c_set_clientdata(client, data);
	this_client = client;
	/* this_client->timing = 400; */

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(tmp, 2) < 0) {
		MMCERR("mmc3416x_device I2C_TxData failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
	msleep(MMC3524X_DELAY_SET);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_SET;
	if (I2C_TxData(tmp, 2) < 0) {
		MMCERR("mmc3416x_device I2C_TxData failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
	msleep(1);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = 0;
	if (I2C_TxData(tmp, 2) < 0) {
		MMCDBG("mmc3416x_device I2C_TxData failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
	msleep(MMC3524X_DELAY_SET);

	tmp[0] = MMC3524X_REG_BITS;
	tmp[1] = MMC3524X_BITS_SLOW_16;
	if (I2C_TxData(tmp, 2) < 0) {
		MMCERR("mmc3416x_device I2C_TxData failed\n");
		err = -ENXIO;
		goto exit_kfree;
	}
	msleep(MMC3524X_DELAY_TM);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_TM;
	if (I2C_TxData(tmp, 2) < 0) {
		MMCDBG("mmc3416x_device I2C_TxData failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
	msleep(MMC3524X_DELAY_TM);

	err = mmc3524x_create_attr(&(mmc3524x_init_info.platform_diver_addr->driver));
	/* Register sysfs attribute */
	if (err) {
		MMCERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&mmc3524x_device);
	if (err) {
		MMCERR("mmc3524x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	sobj_m.self = data;
	sobj_m.polling = 1;
	sobj_m.sensor_operate = mmc3524x_operate;
	err = hwmsen_attach(ID_MAGNETIC, &sobj_m);
	if (err) {
		MMCERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_o.self = data;
	sobj_o.polling = 1;
	sobj_o.sensor_operate = mmc3524x_orientation_operate;
	err = hwmsen_attach(ID_ORIENTATION, &sobj_o);
	if (err) {
		MMCERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_gy.self = data;
	sobj_gy.polling = 1;
	sobj_gy.sensor_operate = mmc3524x_gyro_operate;
	err = hwmsen_attach(ID_GYROSCOPE, &sobj_gy);
	if (err) {
		MMCERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_vec.self = data;
	sobj_vec.polling = 1;
	sobj_vec.sensor_operate = mmc3524x_roation_vector;
	err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_vec);
	if (err) {
		MMCERR("vector attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_grv.self = data;
	sobj_grv.polling = 1;
	sobj_grv.sensor_operate = mmc3524x_gravity;
	err = hwmsen_attach(ID_GRAVITY, &sobj_grv);
	if (err) {
		MMCERR("gravity attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_linacc.self = data;
	sobj_linacc.polling = 1;
	sobj_linacc.sensor_operate = mmc3524x_linear_acc;
	err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_linacc);
	if (err) {
		MMCERR("linear acc attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = mmc3524x_early_suspend,
	data->early_drv.resume   = mmc3524x_late_resume,
	register_early_suspend(&data->early_drv);
#endif
	msensor_name = "mmc3524x";

	mmc3524x_init_flag = 1;

	MMCDBG("%s: OK\n", __func__);
	MMCINFO("%s - OK\n", __func__);
	return 0;

exit_sysfs_create_group_failed:
exit_misc_device_register_failed:
exit_kfree:
	kfree(data);
exit:
	MMCERR("err = %d\n", err);
	return err;
}

static int mmc3524x_i2c_remove(struct i2c_client *client)
{
	int err;

	err = mmc3524x_delete_attr(&(mmc3524x_init_info.platform_diver_addr->driver));
	if (err)
		MMCDBG("mmc3524x_delete_attr fail: %d\n", err);

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&mmc3524x_device);
	return 0;
}

static int mmc3524x_local_init(void)
{
	MMCINFO("%s +\n", __func__);
	mmc3524x_power(hw, 1);

	atomic_set(&dev_open_count, 0);

	if (i2c_add_driver(&mmc3524x_i2c_driver)) {
		MMCERR("add driver error\n");
		return -1;
	}

	if (-1 == mmc3524x_init_flag) {
		MMCERR("%s failed!\n", __func__);
		return -1;
	}

	return 0;
}

static int mmc3524x_remove(void)
{
	mmc3524x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}

static int __init mmc3524x_init(void)
{
	MMCINFO("%s +\n", __func__);
	hw = get_mag_dts_func(mmc_of_match[0].compatible, hw);
	if (!hw) {
		MMCERR("Cannot get hw info\n");
		return -ENODEV;
	}
	MMCDBG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	mag_driver_add(&mmc3524x_init_info);
	return 0;
}

static void __exit mmc3524x_exit(void)
{
}

module_init(mmc3524x_init);
module_exit(mmc3524x_exit);

MODULE_DESCRIPTION("mmc3524x compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

