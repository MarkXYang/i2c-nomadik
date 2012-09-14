/*
 * ST LIS3LV02DL accelerometer driver (I2C) for Nomadik NHK8815 platform
 *
 * (2012) Written by Fabrizio Ghiringhelli <fghiro@gmail.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 */

/* Debug switch (comment to compile out the debug code) */
#define LIS3LV02D_DEBUG

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>			/* kzalloc */
#include <linux/delay.h>		/* msleep */
#include <linux/platform_device.h>	/* sysfs support */
#include <linux/err.h>			/* IS_ERR, PTR_ERR */
#include <linux/input-polldev.h>	/* input subsystem */

#include <mach/lis3lv02d-nhk8815.h>

/* LIS3LV02D register map */
#define	WHO_AM_I		0x0F	/* R Physical Address */
#define	OFFSET_X		0x16	/* R/W Offset Trimming for X-Axis */
#define	OFFSET_Y		0x17	/* R/W Offset Trimming for Y-Axis */
#define	OFFSET_Z		0x18	/* R/W Offset Trimming for Z-Axis */
#define	GAIN_X			0x19	/* R/W Gain Trimming for X-Axis */
#define	GAIN_Y			0x1A	/* R/W Gain Trimming for Y-Axis */
#define	GAIN_Z			0x1B	/* R/W Gain Trimming for Z-Axis */
#define	CTRL_REG1		0x20	/* R/W Control Register 1 */
#define	CTRL_REG2		0x21	/* R/W Control Register 2 */
#define	CTRL_REG3		0x22	/* R/W Control Register 3 */
#define	HP_FILTER_RESET		0x23	/* R Internal high pass-filter reset */
#define	STATUS_REG		0x27	/* R/W Status Register */
#define	OUTX_L			0x28	/* R Acceleration along X-Axis (Low) */
#define	OUTX_H			0x29	/* R Acceleration along X-Axis (High) */
#define	OUTY_L			0x2A	/* R Acceleration along Y-Axis (Low) */
#define	OUTY_H			0x2B	/* R Acceleration along Y-Axis (High) */
#define	OUTZ_L			0x2C	/* R Acceleration along Z-Axis (Low) */
#define	OUTZ_H			0x2D	/* R Acceleration along Z-Axis (High) */
#define	FF_WU_CFG		0x30	/* R/W Free-fall/Wake-up Config. */
#define	FF_WU_SRC		0x31	/* R/W FF/WU Interrupt Source */
#define	FF_WU_ACK		0x32	/* R FF/WU Interrupt Acknowledge */
#define	FF_WU_THS_L		0x34	/* R/W FF/WU threshold LSB */
#define	FF_WU_THS_H		0x35	/* R/W FF/WU threshold MSB*/
#define	FF_WU_DURATION		0x36	/* R/W FF/WU event minimum duration */
#define	DD_CFG			0x38	/* R/W Direction-Detection Config. */
#define	DD_SRC			0x39	/* R/W DD interrupt source */
#define	DD_ACK			0x3A	/* R DD Interrupt Acknowledge */
#define	DD_THSI_L		0x3C	/* R/W DD Internal Threshold LSB */
#define	DD_THSI_H		0x3D	/* R/W DD Internal Threshold MSB */
#define	DD_THSE_L		0x3E	/* R/W DD External Threshold LSB */
#define	DD_THSE_H		0x3F	/* R/W DD External Threshold MSB */

/* WHO_AM_I */
#define	WAI_LIS3LV02D		0x3A

/* CTRL_REG1 */
#define	CTRL1_Xen		0x01
#define	CTRL1_Yen		0x02
#define	CTRL1_Zen		0x04
#define	CTRL1_ST		0x08
#define	CTRL1_DF0		0x10
#define	CTRL1_DF1		0x20
#define	CTRL1_PD0		0x40
#define	CTRL1_PD1		0x80

/* CTRL_REG2 */
#define	CTRL2_DAS		0x01
#define	CTRL2_SIM		0x02
#define	CTRL2_DRDY		0x04
#define	CTRL2_IEN		0x08
#define	CTRL2_BOOT		0x10
#define	CTRL2_BLE		0x20
#define	CTRL2_BDU		0x40	/* Block Data Update */
#define	CTRL2_FS		0x80	/* Full Scale selection */

/* CTRL_REG3 */
#define	CTRL3_CFS0		0x01
#define	CTRL3_CFS1		0x02
#define	CTRL3_FDS		0x10
#define	CTRL3_HPFF		0x20
#define	CTRL3_HPDD		0x40
#define	CTRL3_ECK		0x80

/* STATUS_REG */
#define	STATUS_XDA		0x01
#define	STATUS_YDA		0x02
#define	STATUS_ZDA		0x04
#define	STATUS_XYZDA		0x08
#define	STATUS_XOR		0x10
#define	STATUS_YOR		0x20
#define	STATUS_ZOR		0x40
#define	STATUS_XYZOR		0x80

/* FF_WU_CFG */
#define	FF_WU_CFG_XLIE		0x01
#define	FF_WU_CFG_XHIE		0x02
#define	FF_WU_CFG_YLIE		0x04
#define	FF_WU_CFG_YHIE		0x08
#define	FF_WU_CFG_ZLIE		0x10
#define	FF_WU_CFG_ZHIE		0x20
#define	FF_WU_CFG_LIR		0x40
#define	FF_WU_CFG_AOI		0x80

/* FF_WU_SRC */
#define	FF_WU_SRC_XL		0x01
#define	FF_WU_SRC_XH		0x02
#define	FF_WU_SRC_YL		0x04
#define	FF_WU_SRC_YH		0x08
#define	FF_WU_SRC_ZL		0x10
#define	FF_WU_SRC_ZH		0x20
#define	FF_WU_SRC_IA		0x40

/* DD_CFG */
#define	DD_CFG_XLIE		0x01
#define	DD_CFG_XHIE		0x02
#define	DD_CFG_YLIE		0x04
#define	DD_CFG_YHIE		0x08
#define	DD_CFG_ZLIE		0x10
#define	DD_CFG_ZHIE		0x20
#define	DD_CFG_LIR		0x40
#define	DD_CFG_IEND		0x80

/* DD_SRC */
#define	DD_SRC_XL		0x01
#define	DD_SRC_XH		0x02
#define	DD_SRC_YL		0x04
#define	DD_SRC_YH		0x08
#define	DD_SRC_ZL		0x10
#define	DD_SRC_ZH		0x20
#define	DD_SRC_IA		0x40

/* Device and driver names */
#define	DEVICE_NAME		"lis3lv02d"
#define	DRIVER_NAME		"lis3lv02d-nhk8815"	/* Driver's name (must
							 * match module name) */
/* LIS3LV02D sensitivity [LSB/g] */
#define SENSITIVITY_2G		1024	/* range = +-2g */
#define	SENSITIVITY_6G		340	/* range = +-6g */
#define FULLRES_MAX_VAL		2048	/* 11 bit (12th bit is used for sign) */

/* Free-Fall EV-KEY code (must be less than KEY_MAX and should not overlap with
 * existing codes - see 'linux/input.h')
 */
#define KEY_FREE_FALL		KEY_UNKNOWN	/* FIXME: temporary set to unknown */

/* Ouput Data Rates */
static const unsigned int odrs[] = { 40, 160, 640, 2560 };

/* LIS3LV02D default configuration */
static const struct lis3lv02d_nhk8815_platform_data lis3lv02d_default_init = {
	.device_cfg = LIS3_ODR_40HZ,	/* currently the scale is not supported :-( */
	.poll_interval = 500,
	.free_fall_cfg = LIS3_FF_XL | LIS3_FF_YL | LIS3_FF_ZL,
	.free_fall_threshold = 600,
	.free_fall_duration = 5,	/* 1/ODR [s] */
};

/* LIS3LV02D private structures */
struct lis3lv02d_axis {
	int x;
	int y;
	int z;
};

struct lis3lv02d_priv {
	struct i2c_client *client;
	struct platform_device *pdev;	/* sysfs */
	struct mutex mutex;		/* reentrant protection for struct */
	const struct lis3lv02d_nhk8815_platform_data *pdata;  /* board config */
	struct input_polled_dev *input_polled;

	bool powered;
	bool ff_enabled;

#ifdef LIS3LV02D_DEBUG
	u8 reg_val;
#endif
};

/* Write a byte to the specified offset */
static int __lis3lv02d_byte_write(struct i2c_client *client, u8 offset, u8 reg)
{
	u8 buf[2] = { offset, reg};

	return i2c_master_send(client, buf, 2);
}

/* Read a byte from the specified offset */
static int __lis3lv02d_byte_read(struct i2c_client *client, u8 offset)
{
	u8 buf;
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2] = {
		{ client->addr, client->flags & I2C_M_TEN, 1, &offset },
		{ client->addr, (client->flags & I2C_M_TEN) | I2C_M_RD, 1, &buf }
	};

	ret = i2c_transfer(adap, msg, 2);
	return (ret == 2 ? buf : ret);
}

/* Read a block of bytes from the specified offset */
static int __lis3lv02d_blk_read(struct i2c_client *client, u8 offset,
				void *buf, u8 nbytes)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2] = {
		{ client->addr, client->flags & I2C_M_TEN, 1, &offset },
		{ client->addr, (client->flags & I2C_M_TEN) | I2C_M_RD, nbytes, buf }
	};

	/* Enable address auto incrementation */
	offset |= 0x80;

	ret = i2c_transfer(adap, msg, 2);
	return (ret == 2 ? 0 : ret);
}


/* Power on */
static int __lis3lv02d_poweron(struct lis3lv02d_priv *priv)
{
	struct i2c_client *client = priv->client;
	u8 reg, odrbit;
	int err;

	/* Turn on the sensor and enable XYZ axis */
	odrbit = priv->pdata->device_cfg & LIS3_ODR_MASK;
	reg = odrbit | CTRL1_PD0 | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen;
	err = __lis3lv02d_byte_write(client, CTRL_REG1, reg);
	if (err < 0)
		return err;

	/* BDU | BOOT, event interrupts disabled */
	reg = CTRL2_BDU | CTRL2_BOOT;
	err = __lis3lv02d_byte_write(client, CTRL_REG2, reg);
	if (err < 0)
		return err;

	/* Power on delay
	 * As stated in the lis3lv02d data sheet, the device turn on time is
	 * given by the ratio 5/ODR [sec].
	 */
	msleep(5000 / odrs[odrbit >> 4]);

	priv->powered = true;

	return 0;
}

/* Power off */
static int __lis3lv02d_poweroff(struct lis3lv02d_priv *priv)
{
	int err;

	/* Disable XYZ axis and power down */
	err = __lis3lv02d_byte_write(priv->client, CTRL_REG1, 0x00);
	if (err < 0)
		return err;

	priv->powered = false;

	return 0;
}

/* Interrupt enable */
static int __lis3lv02d_int_enable(struct lis3lv02d_priv *priv, bool enable)
{
	struct i2c_client *client = priv->client;
	u8 reg;
	int err;

	err = __lis3lv02d_byte_read(client, CTRL_REG2);
	if (err < 0)
		return err;

	/* Update IEND bit. To enable interrupts IEND bit must be set */
	reg = enable ? (u8) (err | CTRL2_IEN) : (u8) (err & ~CTRL2_IEN);

	return __lis3lv02d_byte_write(client, CTRL_REG2, reg);
}

/*
 * Read the acceleration along the xyz axis.
 * @axis may be null if the only aim is to ack interrupt.
 */
static int __lis3lv02d_read_axis(struct lis3lv02d_priv *priv,
			struct lis3lv02d_axis *axis)
{
	u16 databuf[3];
	int err;

	/* Read xyz axis */
	err = __lis3lv02d_blk_read(priv->client, OUTX_L, databuf, 6);
	if (err < 0)
		return err;

	if (axis) {
		axis->x = (s16) le16_to_cpu(databuf[0]);
		axis->y = (s16) le16_to_cpu(databuf[1]);
		axis->z = (s16) le16_to_cpu(databuf[2]);
	}
	return 0;
}

/* == sysfs == */
static int lis3lv02d_poll_enable(struct lis3lv02d_priv *priv);
static void lis3lv02d_poll_disable(struct lis3lv02d_priv *priv);
/* Show acceleration along x,y,z axis */
static ssize_t lis3lv02d_position_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);
	struct lis3lv02d_axis axis = { 0, 0, 0};
	ssize_t ret_val;

	mutex_lock(&priv->mutex);
	if (!priv->powered) {
		ret_val = 0;
		goto out;
	}

	if (__lis3lv02d_read_axis(priv, &axis) < 0)
		ret_val = 0;
	else
		ret_val = sprintf(buf, "%d %d %d\n", axis.x, axis.y, axis.z);

 out:
	mutex_unlock(&priv->mutex);
	return ret_val;
}

static DEVICE_ATTR(position, S_IRUGO, lis3lv02d_position_show, NULL);

/* Show the device enable/disable status */
static ssize_t lis3lv02d_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf,"%c\n", priv->powered ? 'y' : 'n');
}

/* Write the device enable/disable status */
static ssize_t lis3lv02d_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	mutex_lock(&priv->mutex);
	if (val) {
		if (!priv->powered)
			err = __lis3lv02d_poweron(priv);
	} else {
		if (priv->powered)
			err = __lis3lv02d_poweroff(priv);
	}
	mutex_unlock(&priv->mutex);

	return err < 0 ? err : count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, lis3lv02d_enable_show,
		lis3lv02d_enable_store);

/* Show the Free-Fall enable/disable status */
static ssize_t lis3lv02d_ff_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf,"%c\n", priv->ff_enabled ? 'y' : 'n');
}

static DEVICE_ATTR(ff_enable, S_IRUGO, lis3lv02d_ff_enable_show, NULL);

/* Show the polling enable/disable status */
static ssize_t lis3lv02d_poll_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf,"%c\n", priv->input_polled ? 'y' : 'n');
}

/* Write the polling enable/disable status */
static ssize_t lis3lv02d_poll_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	if (val)
		err = lis3lv02d_poll_enable(priv);
	else
		lis3lv02d_poll_disable(priv);

	return err < 0 ? err : count;
}

static DEVICE_ATTR(poll_enable, S_IRUGO | S_IWUSR, lis3lv02d_poll_enable_show,
		lis3lv02d_poll_enable_store);

#ifdef LIS3LV02D_DEBUG
/* Show the LIS3LV02D register last read */
static ssize_t lis3lv02d_read_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%02X\n", priv->reg_val);
}

/* Read a LIS3LV02D register.
 * The register value is available to sysfs through 'lis3lv02d_read_show()'.
 */
static ssize_t lis3lv02d_read_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int err;

	err = kstrtouint(buf, 16, &val);
	if (err)
		return err;

	err = __lis3lv02d_byte_read(priv->client, val);
	if (err < 0)
		return err;

	mutex_lock(&priv->mutex);
	priv->reg_val = err;
	mutex_unlock(&priv->mutex);

	return count;
}

static DEVICE_ATTR(read, S_IRUGO | S_IWUSR, lis3lv02d_read_show,
		lis3lv02d_read_store);

/* Write a LIS3LV02D register */
static ssize_t lis3lv02d_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lis3lv02d_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int err;

	err = kstrtouint(buf, 16, &val);
	if (err)
		return err;

	err = __lis3lv02d_byte_write(priv->client, val >> 8, val & 0xff);

	return err < 0 ? err : count;
}

static DEVICE_ATTR(write, S_IWUSR, NULL, lis3lv02d_write_store);
#endif

/* Define attributes */
static struct attribute *lis3lv02d_attributes[] = {
	&dev_attr_position.attr,
	&dev_attr_enable.attr,
	&dev_attr_ff_enable.attr,
	&dev_attr_poll_enable.attr,
#ifdef LIS3LV02D_DEBUG
	&dev_attr_read.attr,
	&dev_attr_write.attr,
#endif
	NULL
};

static struct attribute_group lis3lv02d_attr_group = {
	.attrs = lis3lv02d_attributes
};

/* Add device in sysfs */
static int lis3lv02d_sysfs_add(struct lis3lv02d_priv *priv)
{
	/* Register the device and save its pointer in the lis3lv02d private
	 * structure
	 */
	priv->pdev = platform_device_register_simple(DEVICE_NAME, -1, NULL, 0);
	if (IS_ERR(priv->pdev))
		return PTR_ERR(priv->pdev);

	/* Set platform data */
	platform_set_drvdata(priv->pdev, priv);

	/* Create sysfs file */
	return sysfs_create_group(&priv->pdev->dev.kobj, &lis3lv02d_attr_group);
}

/* Remove device from sysfs */
static int lis3lv02d_sysfs_remove(struct lis3lv02d_priv *priv)
{
	sysfs_remove_group(&priv->pdev->dev.kobj, &lis3lv02d_attr_group);
	platform_device_unregister(priv->pdev);
	return 0;
}

/* Sensor configuration */
static int lis3lv02d_configure(struct lis3lv02d_priv *priv)
{
	const struct lis3lv02d_nhk8815_platform_data *pdata = priv->pdata;
	int err;
	u8 reg;

	/* Setup the Free-Fall threshold */
	err = __lis3lv02d_byte_write(priv->client, FF_WU_THS_L,
				pdata->free_fall_threshold);
	if (err < 0)
		return err;

	err = __lis3lv02d_byte_write(priv->client, FF_WU_THS_H,
				pdata->free_fall_threshold >> 8);
	if (err < 0)
		return err;

	/* Setup the Free-Fall duration */
	err = __lis3lv02d_byte_write(priv->client, FF_WU_DURATION,
				pdata->free_fall_duration);
	if (err < 0)
		return err;

	/* Program the interrupt sources */
	reg = pdata->free_fall_cfg & (LIS3_FF_ALL | LIS3_FF_AOI);
	err = __lis3lv02d_byte_write(priv->client, FF_WU_CFG, reg);

	return err;
}

/* Open */
static void lis3lv02d_open(struct input_polled_dev *dev)
{
	struct lis3lv02d_priv *priv = dev->private;

	mutex_lock(&priv->mutex);
	if (!priv->powered)
		__lis3lv02d_poweron(priv);

	if (priv->ff_enabled) {

		/* FIXME: needed? as soon as int is enabled we get irq anyway
		 * Perform a dummy read to ack any pending interrupt */
		__lis3lv02d_read_axis(priv, NULL);

		/* The driver uses the accelerometer interrupt to report
		 * Free-Fall events to the input subsystem.
		 */
		__lis3lv02d_int_enable(priv, true);
	}
 	mutex_unlock(&priv->mutex);
}

/* Close */
static void lis3lv02d_close(struct input_polled_dev *dev)
{
	struct lis3lv02d_priv *priv = dev->private;

	mutex_lock(&priv->mutex);
	__lis3lv02d_int_enable(priv, false);
	mutex_unlock(&priv->mutex);
}

/* Poll */
static void lis3lv02d_poll(struct input_polled_dev *dev)
{
	struct lis3lv02d_priv *priv = dev->private;
	struct input_dev *input = dev->input;
	struct lis3lv02d_axis axis;

	/* Read xyz axis */
	__lis3lv02d_read_axis(priv, &axis);

	/* Report event to the input subsystem */
	input_report_abs(input, ABS_X, axis.x);
	input_report_abs(input, ABS_Y, axis.y);
	input_report_abs(input, ABS_Z, axis.z);
	input_sync(input);
}

/*
 * Threaded IRQ handler
 *
 * The inertial sensor is hooked up to the CPU using i2c and a single chip
 * interrupt line. Deasserting the interrupt requires communicating with the
 * sensor over the i2c bus.
 * Since i2c bus controller may sleep we need to use threaded IRQ here.
 */
static irqreturn_t lis3lv02d_isr_thread(int irq, void *dev)
{
	struct lis3lv02d_priv *priv = dev;
	struct input_dev *input = priv->input_polled->input;
	int err;

	/* Test int source */
	err = __lis3lv02d_byte_read(priv->client, FF_WU_SRC);
	if ((err < 0) || !((u8) err & FF_WU_SRC_IA ))
		goto out;

	/* Report free-fall (KEY) event */
	input_report_key(input, KEY_FREE_FALL, true);
	input_report_key(input, KEY_FREE_FALL, false);
	input_sync(input);

	/* Poll device and report ABS events (this also ack interrupt) */
	lis3lv02d_poll(priv->input_polled);

out:
	return IRQ_HANDLED;
}

/* Enable the device polling.
 * Register the device as a polled input with the input subsystem
 */
static int lis3lv02d_poll_enable(struct lis3lv02d_priv *priv)
{
	struct input_polled_dev *input_polled;
	struct input_dev *input;
	int err;

	if (priv->input_polled)
		return -EINVAL;		/* polling is already enable */

	/* Allocate memory for the input device */
	input_polled = input_allocate_polled_device();
	if (!input_polled) {
		dev_err(&priv->client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Setup input parameters */
	input_polled->open = lis3lv02d_open;
	input_polled->close = lis3lv02d_close;
	input_polled->poll = lis3lv02d_poll;
	input_polled->poll_interval = priv->pdata->poll_interval;
	input_polled->poll_interval_min = 0;
	input_polled->poll_interval_max = 2 * priv->pdata->poll_interval;
	input_polled->private = priv;

	input = input_polled->input;
	input->name = "LIS3LV02D accelerometer";
	input->phys = DEVICE_NAME "/input0";
	input->dev.parent = &priv->client->dev;
	input->id.bustype = BUS_I2C;
	input->id.product = 0;

	/* Setup ABS input events */
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);
	set_bit(ABS_Y, input->absbit);
	set_bit(ABS_Z, input->absbit);

	input_set_abs_params(input, ABS_X, -FULLRES_MAX_VAL, FULLRES_MAX_VAL, 3, 3);
	input_set_abs_params(input, ABS_Y, -FULLRES_MAX_VAL, FULLRES_MAX_VAL, 3, 3);
	input_set_abs_params(input, ABS_Z, -FULLRES_MAX_VAL, FULLRES_MAX_VAL, 3, 3);

	/* Setup KEY event for free-fall (only if enabled) */
	if (priv->ff_enabled) {
		set_bit(EV_KEY, input->evbit);
		set_bit(KEY_FREE_FALL, input->keybit);
	}

	/* Register input polled device */
	err = input_register_polled_device(input_polled);
	mutex_lock(&priv->mutex);
	if (err) {
		input_free_polled_device(input_polled);
		priv->input_polled = NULL;
	}
	else {
		priv->input_polled = input_polled;
	}
	mutex_unlock(&priv->mutex);

	return err;
}

/* Disable the device polling.
 * Unregister the device with the input subsystem
 */
static void lis3lv02d_poll_disable(struct lis3lv02d_priv *priv)
{
	if (!priv->input_polled)
		return;		/* polling is already disabled */

	/* Unregister input device */
	input_unregister_polled_device(priv->input_polled);
	input_free_polled_device(priv->input_polled);

	mutex_lock(&priv->mutex);
	priv->input_polled = NULL;
	mutex_unlock(&priv->mutex);
}

/* Probe function */
static int lis3lv02d_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	struct lis3lv02d_priv *priv;
	const struct lis3lv02d_nhk8815_platform_data *pdata;
	int err;

	/* Allocate memory for the LIS3LV02D device */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	priv->client = client;

	/* Save pointer to platform data */
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_info(&client->dev,
			"no platform data, using default initialization\n");
		pdata = &lis3lv02d_default_init;
	}
	priv->pdata = pdata;

	/* Look for li3lv02d inertial sensor */
	if (__lis3lv02d_byte_read(client, WHO_AM_I) == WAI_LIS3LV02D) {
		dev_info(&client->dev, "found LIS3LV02DL inertial sensor\n");
	} else {
		dev_err(&client->dev,
			"unable to find LIS3LV02DL inertial sensor\n");
		err = -ENODEV;
		goto err_free_mem;
	}

	/* Register I2C client data with the I2C core */
	i2c_set_clientdata(client, priv);

	/* Initialize mutex */
	mutex_init(&priv->mutex);

	/* Add device in sysfs */
	err = lis3lv02d_sysfs_add(priv);
	if (err)
		goto err_free_mem;

	/* Power on the sensor */
	err = __lis3lv02d_poweron(priv);
	if (err < 0)
		goto err_sysfs_remove;

	/* Configure the sensor (mostly interrupt stuff) */
	err = lis3lv02d_configure(priv);
	if (err < 0)
		goto err_sysfs_remove;

	/* Get IRQ */
	if (pdata->free_fall_cfg & LIS3_FF_ALL) {

		/* Verify IRQ */
		if (!client->irq) {
			dev_err(&client->dev, "no IRQ?\n");
			err = -ENODEV;
			goto err_sysfs_remove;
		}

		/* Register IRQ.
		 * There is no primary IRQ handler since int ack involves doing
		 * I2C ransactions which will likely sleep, so cannot be done by
		 * the primary handler which runs in interrupt context
		 */
		err = request_threaded_irq(client->irq, NULL,
					lis3lv02d_isr_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					DEVICE_NAME, priv);
		if (err < 0) {
			dev_err(&client->dev, "can't register IRQ %d\n",
				 client->irq);
			goto err_sysfs_remove;
		}

		/* At this point we have a valid irq that will be used to detect
		 * the Free-Fall, thus Free-Fall detection is enabled.
		 */
		priv->ff_enabled = true;
		dev_info(&client->dev, "Free-Fall detection enabled\n");
	}

	/* Enable input polling */
	if (pdata->device_cfg & LIS3_EPOLL) {
		err = lis3lv02d_poll_enable(priv);
		if (err)
			goto err_free_irq;
	}

	return 0;

 err_free_irq:
	if (priv->ff_enabled)
		free_irq(client->irq, priv);

 err_sysfs_remove:
	lis3lv02d_sysfs_remove(priv);

 err_free_mem:
	kfree(priv);

	return err;
}

/* Remove function */
static int __devexit lis3lv02d_remove(struct i2c_client *client)
{
	struct lis3lv02d_priv *priv = i2c_get_clientdata(client);

	/* Power off the sensor*/	__lis3lv02d_poweroff(priv);

	/* Remove device from sysfs */
	lis3lv02d_sysfs_remove(priv);

	/* Free IRQ */
	if (priv->ff_enabled)
		free_irq(client->irq, priv);

	/* Unregister input device */
	lis3lv02d_poll_disable(priv);

	/* Free memory */
	kfree(priv);

	return 0;
}

/* I2C client structure */
static struct i2c_device_id lis3lv02d_idtable[] = {
	{ DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, lis3lv02d_idtable);

static struct i2c_driver lis3lv02d_driver = {
	.driver   = {
		.name = DRIVER_NAME
	},
	.probe    = lis3lv02d_probe,
	.remove   = __devexit_p(lis3lv02d_remove),
	.id_table = lis3lv02d_idtable,
};

/* Module init */
static int __init lis3lv02d_init(void)
{
	return i2c_add_driver(&lis3lv02d_driver);
}

/* Module exit */
static void __exit lis3lv02d_exit(void)
{
	i2c_del_driver(&lis3lv02d_driver);
}

MODULE_DESCRIPTION("ST LIS3LV02DL inertial sensor driver");
MODULE_AUTHOR("Fabrizio Ghiringhelli <fghiro@gmail.com>");
MODULE_LICENSE("GPL");


module_init(lis3lv02d_init);
module_exit(lis3lv02d_exit);
