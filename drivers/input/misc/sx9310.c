/*! \file sx9310.c
 * \brief  SX9310 Driver
 *
 * Driver for the SX9310
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
/* #define DEBUG */
#define DRIVER_NAME "sx9310"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/input/sx9310.h>	/* main struct, interrupt,init,pointers */
#include <linux/proc_fs.h>

#define IDLE 0
#define ACTIVE 1

#define MAX_WRITE_ARRAY_SIZE 32
#define INVALID_GPIO        (-1)

/*power supply VDD 3.3V, VIO 1.8 */

/* Huaqin modify for SAR VDD by chenyijun5 at 2018/02/23 start */
#define SX9310_VDD_MIN_UV       2800000
#define SX9310_VDD_MAX_UV       2800000
/* Huaqin modify for SAR VDD by chenyijun5 at 2018/02/23 end */
#define SX9310_VDD1_MIN_UV       1800000	/* modify by zch */
#define SX9310_VDD1_MAX_UV       1800000	/* modify by zch */
#define SX9310_VIO_MIN_UV       1800000
#define SX9310_VIO_MAX_UV       1800000

/* Huaqin add sar switcher by chenyijun5 at 2018/03/20 start*/
bool sar_switcher = 0;//Indonesia set sar_switcher to 1
module_param(sar_switcher, bool, 0644);
MODULE_PARM_DESC(sar_switcher, "Control sarsensor open or close.");
/* Huaqin add sar switcher by chenyijun5 at 2018/03/20 end*/

/* Huaqin add for check hw by zhuqiang at 2018/06/22 start */
#define SX9310_ID_ERROR 	1
#define SX9310_I2C_ERROR	2
#define SX9310_WHOAMI_REG       0x42
#define SX9310_WHOAMI_VALUE     0x1
static bool err_flag = 0;
/* Huaqin add for check hw by zhuqiang at 2018/06/22 end */

/*! \struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9310 {
	pbuttonInformation_t pbuttonInformation;
	psx9310_platform_data_t hw;	/* specific platform data settings */
} sx9310_t, *psx9310_t;
static int write_register(psx93XX_t this, u8 address, u8 value);
static int sx9310_get_nirq_state(void);
static int sx9310_power_ctl(sx93XX_t *data, bool on);
static int sx9310_power_vdd1_ctl(sx93XX_t *data, bool on);

static psx9310_t PSX9310Device;

static unsigned int sx9310_enable = 1;
#define PROC_CAPSENSOR_FILE "capsensor_enable"
psx93XX_t psx93XX_this;
static ssize_t capsensor_config_read_proc(struct file *, char __user *, size_t,
					  loff_t *);
static ssize_t capsensor_config_write_proc(struct file *, const char __user *,
					   size_t, loff_t *);
static struct proc_dir_entry *capsensor_proc;

static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = capsensor_config_read_proc,
	.write = capsensor_config_write_proc,
};

static void ForcetoTouched(psx93XX_t this)
{
	psx9310_t pDevice = NULL;
	struct input_dev *input = NULL;
	struct _buttonInfo *pCurrentButton = NULL;

	if (!this)
		dev_err(this->pdev, "ForcetoTouched,this is null\n");

	pDevice = this->pDevice;
	if (pDevice) {
		dev_dbg(this->pdev, "ForcetoTouched()\n");

		pCurrentButton = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		dev_dbg(this->pdev, "sar---report keycode = %d\n",
			pCurrentButton->keycode0);
		//input_report_key(input, pCurrentButton->keycode, 1);
		input_report_key(input, pCurrentButton->keycode0, 1);
		input_report_key(input, pCurrentButton->keycode0, 0);

		pCurrentButton->state = ACTIVE;

		input_sync(input);

		dev_dbg(this->pdev, "Leaving ForcetoTouched()\n");
	}
}

#ifdef USE_THREADED_IRQ
static void sx93XX_process_interrupt(psx93XX_t this, u8 nirqlow)
{
	int status = 0;
	int counter = 0;

	if (!this) {
		dev_err(this->pdev, "sx93XX_worker_func, NULL sx93XX_t\n");
		return;
	}
	/* since we are not in an interrupt don't need to disable irq. */
	status = this->refreshStatus(this);
	counter = -1;
	dev_dbg(this->pdev, "Worker - Refresh Status %d\n", status);

	while ((++counter) < MAX_NUM_STATUS_BITS) {	/* counter start from MSB */
		dev_dbg(this->pdev, "Looping Counter %d\n", counter);
		if (((status >> counter) & 0x01) && (this->statusFunc[counter])) {
			dev_dbg(this->pdev,
				"Function Pointer Found. Calling\n");
			this->statusFunc[counter] (this);
		}
	}
	if (unlikely(this->useIrqTimer && nirqlow)) {
		/* In case we need to send a timer for example on a touchscreen
		 * checking penup, perform this here
		 */
		cancel_delayed_work(&this->dworker);
		queue_delayed_work(system_power_efficient_wq,
				   &this->dworker,
				   msecs_to_jiffies(this->irqTimeout));
		dev_info(this->pdev, "Schedule Irq timer");
	}
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;

	if (work) {
		this = container_of(work, sx93XX_t, dworker.work);
		if (!this) {
			dev_err(this->pdev,
				"sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		if ((!this->get_nirq_low) || (!this->get_nirq_low())) {
			/* only run if nirq is high */
			sx93XX_process_interrupt(this, 0);
		}
	} else {
		dev_err(this->pdev, "sx93XX_worker_func, NULL work_struct\n");
	}
}

static irqreturn_t sx93XX_interrupt_thread(int irq, void *data)
{
	psx93XX_t this = 0;

	this = data;
	//mutex_lock(&this->mutex);
	dev_dbg(this->pdev, "sx93XX_irq\n");
	if ((!this->get_nirq_low) || this->get_nirq_low()) {
		sx93XX_process_interrupt(this, 1);
	} else
		dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
	//mutex_unlock(&this->mutex);

	return IRQ_HANDLED;
}
#else
static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
	unsigned long flags;

	if (this) {
		dev_dbg(this->pdev, "sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock, flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		/* after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue. */
		queue_delayed_work(system_power_efficient_wq,
				   &this->dworker, delay);
		spin_unlock_irqrestore(&this->lock, flags);
	} else
		dev_err(this->pdev, "sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
	psx93XX_t this = 0;

	if (pvoid) {
		this = (psx93XX_t) pvoid;
		dev_dbg(this->pdev, "sx93XX_irq\n");
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
			dev_dbg(this->pdev, "sx93XX_irq - Schedule Work\n");
			sx93XX_schedule_work(this, 0);
		} else
			dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
	} else
		dev_err(this->pdev, "sx93XX_irq, NULL pvoid\n");

	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;

	if (work) {
		this = container_of(work, sx93XX_t, dworker.work);

		if (!this) {
			dev_err(this->pdev,
				"sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low()) {
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
		dev_dbg(this->pdev, "Worker - Refresh Status %d\n", status);
		while ((++counter) < MAX_NUM_STATUS_BITS) {	/* counter start from MSB */
			dev_dbg(this->pdev, "Looping Counter %d\n", counter);
			if (((status >> counter) & 0x01)
			    && (this->statusFunc[counter])) {
				dev_dbg(this->pdev,
					"Function Pointer Found. Calling\n");
				this->statusFunc[counter] (this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow)) {
			/* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			sx93XX_schedule_work(this,
					     msecs_to_jiffies
					     (this->irqTimeout));
		}
	} else {
		dev_err(this->pdev, "sx93XX_worker_func, NULL work_struct\n");
	}
}
#endif

void sx93XX_suspend(psx93XX_t this)
{
	/* int err =0; */

	if (this) {
		disable_irq(this->irq);
		write_register(this, 0x10, 0x50);
		write_register(this, 0x00, 0xff);
		/* err = sx9310_power_ctl(this, false); */
		/* if (err) { */
		/* dev_err(this->pdev, "Failed to disable Capacitive Touch Controller power\n"); */
		/* } */
	}
}

void sx93XX_resume(psx93XX_t this)
{
	/* int err = 0; */
	if (this) {
		/* err = sx9310_power_ctl(this, true); */
		/* if (err) { */
		/* dev_err(this->pdev, "Failed to enable Capacitive Touch Controller power\n"); */
		/* } */
#ifdef USE_THREADED_IRQ
		mutex_lock(&this->mutex);
		/* Just in case need to reset any uncaught interrupts */
		sx93XX_process_interrupt(this, 0);
		mutex_unlock(&this->mutex);
#else
		sx93XX_schedule_work(this, 0);
#endif
		/* if (this->init) */
		/* this->init(this); */
/* write_register(this,0x10,0x56); // modify by zch for cs0 */
		write_register(this, 0x10, 0x51);

		enable_irq(this->irq);
		write_register(this, 0x00, 0xff);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*TODO: Should actually call the device specific suspend/resume
 * As long as the kernel suspend/resume is setup, the device
 * specific ones will be called anyways
 */
extern suspend_state_t get_suspend_state(void);
void sx93XX_early_suspend(struct early_suspend *h)
{
	psx93XX_t this = 0;

	dev_dbg(this->pdev, "inside sx93XX_early_suspend()\n");
	this = container_of(h, sx93XX_t, early_suspend);
	sx93XX_suspend(this);
	dev_dbg(this->pdev, "exit sx93XX_early_suspend()\n");
}

void sx93XX_late_resume(struct early_suspend *h)
{
	psx93XX_t this = 0;

	dev_dbg(this->pdev, "inside sx93XX_late_resume()\n");
	this = container_of(h, sx93XX_t, early_suspend);
	sx93XX_resume(this);
	dev_dbg(this->pdev, "exit sx93XX_late_resume()\n");
}
#endif

int sx93XX_init(psx93XX_t this)
{
	int err = 0;

	if (this && this->pDevice) {
#ifdef USE_THREADED_IRQ
		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);

		/* initialize mutex */
		mutex_init(&this->mutex);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;

		err =
		    request_threaded_irq(this->irq, NULL,
					 sx93XX_interrupt_thread,
					 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					 this->pdev->driver->name, this);
#else
		/* initialize spin lock */
		spin_lock_init(&this->lock);

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);

		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
				  this->pdev->driver->name, this);
#endif

		if (err) {
			dev_err(this->pdev, "irq request failed %d, error %d\n",
				this->irq, err);
			return err;
		}
#ifdef USE_THREADED_IRQ
		dev_info(this->pdev, "registered with threaded irq (%d)\n",
			 this->irq);
#else
		dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
		this->early_suspend.level =
		    EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		this->early_suspend.suspend = sx93XX_early_suspend;
		this->early_suspend.resume = sx93XX_late_resume;
		register_early_suspend(&this->early_suspend);
		if (has_wake_lock(WAKE_LOCK_SUSPEND) == 0 &&
		    get_suspend_state() == PM_SUSPEND_ON)
			sx93XX_early_suspend(&this->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

		/* call init function pointer (this should initialize all registers */
		if (this->init)
			return this->init(this);
		dev_err(this->pdev, "No init function!!!!\n");
	}

	return -ENOMEM;
}

int sx93XX_remove(psx93XX_t this)
{
	if (this) {
		cancel_delayed_work_sync(&this->dworker);	/* Cancel the Worker Func */
		/*destroy_workqueue(this->workq); */

#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&this->early_suspend);
#endif

		free_irq(this->irq, this);
		kfree(this);
		return 0;
	}

	return -ENOMEM;
}

/*! \fn static int write_register(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = (struct i2c_client *)this->bus;

		returnValue = i2c_master_send(i2c, buffer, 2);
		dev_dbg(&i2c->dev,
			"write_register Address: 0x%x Value: 0x%x Return: %d\n",
			address, value, returnValue);
	}
	if (returnValue < 0) {
		ForcetoTouched(this);
		dev_info(this->pdev, "Write_register-ForcetoTouched()\n");
	}

	return returnValue;
}

/*! \fn static int read_register(psx93XX_t this, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (this && value && this->bus) {
		i2c = (struct i2c_client *)this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c, address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",
			address, returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}

	ForcetoTouched(this);
	dev_info(this->pdev, "read_register-ForcetoTouched()\n");
	return -ENOMEM;
}

void sar_int_config(int state, struct pinctrl * pin)
{
	struct pinctrl_state *set_state;
	int ret = 0;

	if (state) {
		set_state = pinctrl_lookup_state(pin, "sarsensor_active");
		if (IS_ERR(set_state)) {
			pr_err
			    ("[sx9310 error]  sarsensor find sarsensor_active error!");
			return;
		}
	} else {
		set_state = pinctrl_lookup_state(pin, "sarsensor_suspend");
		if (IS_ERR(set_state)) {
			pr_err
			    ("[sx9310 error]  sarsensor find sarsensor_active error!");
			return;
		}
	}
	ret = pinctrl_select_state(pin, set_state);
	if (ret) {
		pr_err("[sx9310 error]  sarsensor pinctrl set  state error!");
		return;
	}
	pr_info("[sx9301 ]  sarsensor pinctrl set  state sucess!");
}

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;

	returnValue = write_register(this, SX9310_IRQSTAT_REG, 0xFF);
	return returnValue;
}

/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this, SX9310_IRQSTAT_REG, &reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val = 0;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val) {
		dev_info(this->pdev,
			 "Performing manual_offset_calibration()\n");
		if (this->init)
			this->init(this);
		manual_offset_calibration(this);
	}
	return count;
}

static DEVICE_ATTR(calibrate, 0664, manual_offset_calibration_show,
		   manual_offset_calibration_store);

static ssize_t sx9310_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);

	dev_dbg(this->pdev, "Reading sx9310 enable status.\n");
	return sprintf(buf, "%u\n", sx9310_enable);
}

static ssize_t sx9310_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned int val;
	int err = 0;

	if (count >= sizeof(sx9310_enable))
		return -EINVAL;

	if (sscanf(buf, "%u", &val) != 1)
		return -EINVAL;

	if (val) {
		if (!sx9310_enable && this) {
			dev_info(this->pdev, "going to enable sx9310.\n");
			err = sx9310_power_ctl(this, true);
			err = sx9310_power_vdd1_ctl(this, true);
			if (err) {
				dev_err(this->pdev,
					"Failed to enable Capacitive Touch Controller power\n");
			}
#ifdef USE_THREADED_IRQ
			mutex_lock(&this->mutex);
			/* Just in case need to reset any uncaught interrupts */
			sx93XX_process_interrupt(this, 0);
			mutex_unlock(&this->mutex);
#else
			sx93XX_schedule_work(this, 0);
#endif
			if (this->init)
				this->init(this);

			enable_irq(this->irq);
		} else {
			if (this)
				dev_info(this->pdev,
					 "sx9310 has already enabled.\n");
		}
	} else {
		if (sx9310_enable && this) {
			dev_info(this->pdev, "going to disable sx9310.\n");
			disable_irq(this->irq);

			err = sx9310_power_ctl(this, false);
			err = sx9310_power_vdd1_ctl(this, false);
			if (err) {
				dev_err(this->pdev,
					"Failed to disable Capacitive Touch Controller power\n");
			}
		} else {
			if (this)
				dev_info(this->pdev,
					 "sx9310 has already disabled.\n");
		}
	}

	sx9310_enable = val;

	return count;
}

static DEVICE_ATTR(enable, 0664, sx9310_enable_show, sx9310_enable_store);

static ssize_t sx9310_register_write_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int regist = 0, val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf, "%x,%x", &regist, &val) != 2) {
		pr_err("[SX9310]: %s - The number of data are wrong\n",
		       __func__);
		return -EINVAL;
	}

	write_register(this, (unsigned char)regist, (unsigned char)val);
	pr_info("[SX9310]: %s - Register(0x%x) data(0x%x)\n",
		__func__, regist, val);

	return count;
}

static ssize_t sx9310_register_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0, i = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	int reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup);
	int err = 0;
	ssize_t len = 0;

	while (i < reg_num) {
		err =
		    read_register(this, sx9310_i2c_reg_setup[i].reg,
				  &reg_value);
		len +=
		    sprintf(buf + len, "%x , %x \r\n",
			    sx9310_i2c_reg_setup[i].reg, reg_value);
		i++;
	}
	err = read_register(this, 0x30, &reg_value);

	len += sprintf(buf + len, "0x30, %x \r\n", reg_value);

	err = read_register(this, 0x35, &reg_value);

	len += sprintf(buf + len, "0x35 , %x \r\n", reg_value);

	err = read_register(this, 0x36, &reg_value);

	len += sprintf(buf + len, "0x36, %x\r\n", reg_value);

	err = read_register(this, 0x37, &reg_value);

	len += sprintf(buf + len, "0x37 , %x \r\n", reg_value);

	err = read_register(this, 0x38, &reg_value);

	len += sprintf(buf + len, "0x38 , %x \r\n", reg_value);

	err = read_register(this, 0x00, &reg_value);

	len += sprintf(buf + len, "0x00, %x \r\n", reg_value);

	err = read_register(this, 0x01, &reg_value);

	len += sprintf(buf + len, "0x01, %x \r\n", reg_value);

	return len;
}

static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR | S_IWGRP,
		   sx9310_register_show, sx9310_register_write_store);

static void read_rawData(psx93XX_t this)
{
	u8 msb = 0, lsb = 0;

	if (this) {
		write_register(this, SX9310_CPSRD, 0);
		msleep(100);
		read_register(this, SX9310_USEMSB, &msb);
		read_register(this, SX9310_USELSB, &lsb);
		dev_dbg(this->pdev,
			"sx9310 raw data USEFUL msb = 0x%x, lsb = 0x%x\n", msb,
			lsb);
		read_register(this, SX9310_AVGMSB, &msb);
		read_register(this, SX9310_AVGLSB, &lsb);
		dev_dbg(this->pdev,
			"sx9310 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n", msb,
			lsb);
		read_register(this, SX9310_DIFFMSB, &msb);
		read_register(this, SX9310_DIFFLSB, &lsb);
		dev_dbg(this->pdev,
			"sx9310 raw data DIFF msb = 0x%x, lsb = 0x%x\n", msb,
			lsb);
		read_register(this, SX9310_OFFSETMSB, &msb);
		read_register(this, SX9310_OFFSETLSB, &lsb);
		dev_dbg(this->pdev,
			"sx9310 raw data OFFSET msb = 0x%x, lsb = 0x%x\n", msb,
			lsb);
	}
}

static struct attribute *sx9310_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_enable.attr,
	&dev_attr_reg.attr,
	NULL,
};

static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};

/* Huaqin add sar switcher by chenyijun5 at 2018/03/20 start*/
void sar_switch(bool switcher)
{
	/* Huaqin add to report near event when sar switche off by chenyijun5 at 2018/03/22 start*/
	psx9310_t pDevice = NULL;
	struct input_dev *input = NULL;
	struct _buttonInfo *pCurrentButton = NULL;
    /* Huaqin add for check hw by zhuqiang at 2018/06/22 start */
	if(err_flag)
		return;
    /* Huaqin add for check hw by zhuqiang at 2018/06/22 end */

	pDevice = psx93XX_this->pDevice;
	pCurrentButton = pDevice->pbuttonInformation->buttons;
	input = pDevice->pbuttonInformation->input;
	/* Huaqin add to report near event when sar switche off by chenyijun5 at 2018/03/22 end*/

	if (sar_switcher) {//sar_switcher is 1 for Indonesia, open sarsensor
		if (switcher) {//switch on
			dev_dbg(psx93XX_this->pdev, "Indonesia: going to enable SAR!\n");
			write_register(psx93XX_this, SX9310_IRQ_ENABLE_REG, 0x70);
			write_register(psx93XX_this, SX9310_CPS_CTRL0_REG, 0x51);
			enable_irq(psx93XX_this->irq);
		} else {//switch off
			dev_dbg(psx93XX_this->pdev, "Indonesia: going to disable SAR!\n");
			disable_irq(psx93XX_this->irq);
			write_register(psx93XX_this, SX9310_IRQ_ENABLE_REG, 0);
			write_register(psx93XX_this, SX9310_CPS_CTRL0_REG, 0);
			/* Huaqin add to report near event when sar switche off by chenyijun5 at 2018/03/22 start*/
			if (IDLE == pCurrentButton->state) {
				input_report_key(input, pCurrentButton->keycode0, 1);
				input_report_key(input, pCurrentButton->keycode0, 0);
				input_sync(input);
				pCurrentButton->state = ACTIVE;
			} else {
				return;
			}
			/* Huaqin add to report near event when sar switche off by chenyijun5 at 2018/03/22 end*/
		}
	} else {//other countries, do nothing
		dev_dbg(psx93XX_this->pdev, "not Indonesia: do nothing about SAR!\n");
		return;
	}

	return;
}
EXPORT_SYMBOL(sar_switch);
/* Huaqin add sar switcher by chenyijun5 at 2018/03/20 end*/

/*********************************************************************/

/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u8 data = 0;

	if (this) {
		if (read_register(this, SX9310_IRQSTAT_REG, &data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */
static void hw_init(psx93XX_t this)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		while (i < pdata->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			dev_dbg(this->pdev,
				"Going to Write Reg: 0x%x Value: 0x%x\n",
				pdata->pi2c_reg[i].reg, pdata->pi2c_reg[i].val);
			/* msleep(3); */
			write_register(this, pdata->pi2c_reg[i].reg,
				       pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n", pDevice->hw);
		/* Force to touched if error */
		ForcetoTouched(this);
		dev_info(this->pdev, "Hardware_init-ForcetoTouched()\n");
	}
}

/*********************************************************************/

/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
	if (this) {
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this, SX9310_SOFTRESET_REG, SX9310_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		dev_dbg(this->pdev,
			"Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(300);
		/* while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); } */
		dev_dbg(this->pdev,
			"Device is back from the reset, continuing. NIRQ = %d\n",
			this->get_nirq_low());
		hw_init(this);
		msleep(100);	/* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);
		this->irq_disabled = 0;

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		read_regStat(this);
		dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",
			this->get_nirq_low());
		return 0;
	}
	return -ENOMEM;
}

/*!
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct
 */
static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u8 i = 0;
	int numberOfButtons = 0;
	psx9310_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;

	struct _buttonInfo *pCurrentButton = NULL;

	if (this && (pDevice = this->pDevice)) {
		dev_dbg(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX9310_STAT0_REG, &i);

		buttons = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		if (unlikely((buttons == NULL) || (input == NULL))) {
			dev_err(this->pdev,
				"ERROR!! buttons or input NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton == NULL) {
				dev_err(this->pdev,
					"ERROR!! current button at index: %d NULL!!!\n",
					counter);
				return;	/* ERRORR!!!! */
			}

			switch (pCurrentButton->state) {
			case IDLE:	/* Button is not being touched! */
				if (((i & pCurrentButton->mask) ==
				     pCurrentButton->mask)) {
					/* User pressed button */
					dev_info(this->pdev,
						 "cap button %d touched\n",
						 counter);
					/*
					input_report_key(input,
							 pCurrentButton->
							 keycode, 1);
					*/
					input_report_key(input, pCurrentButton->keycode0, 1);
					input_report_key(input, pCurrentButton->keycode0, 0);
					pCurrentButton->state = ACTIVE;
				} else {
					dev_dbg(this->pdev,
						"Button %d already released.\n",
						counter);
				}
				break;
			case ACTIVE:	/* Button is being touched! */
				if (((i & pCurrentButton->mask) !=
				     pCurrentButton->mask)) {
					/* User released button */
					dev_info(this->pdev,
						 "cap button %d released\n",
						 counter);
					/*
					input_report_key(input,
							 pCurrentButton->
							 keycode, 0);
					*/
					input_report_key(input, pCurrentButton->keycode1, 1);
					input_report_key(input, pCurrentButton->keycode1, 0);
					pCurrentButton->state = IDLE;
				} else {
					dev_dbg(this->pdev,
						"Button %d still touched.\n",
						counter);
				}
				break;
			default:	/* Shouldn't be here, device only allowed ACTIVE or IDLE */
				break;
			};
		}
		input_sync(input);

		dev_dbg(this->pdev, "Leaving touchProcess()\n");
	}
}

static int sx9310_get_nirq_state(void)
{
	int value;

	if (!gpio_is_valid(PSX9310Device->hw->irq_gpio)) {
		pr_err("sx9310 irq_gpio was not assigned properly");
	}
	value = gpio_get_value(PSX9310Device->hw->irq_gpio);
	pr_info("sx9310 irq gpio status(%d)", value);
	return !value;
}

static int sx9310_power_vdd1_ctl(sx93XX_t *data, bool on)
{
	int ret = 0;

	if (!on) {
		ret = regulator_disable(data->vdd1);
		if (ret) {
			pr_err("Regulator vdd1 disable failed ret=%d\n", ret);
		}
	} else {
		ret = regulator_enable(data->vdd1);
		if (ret) {
			pr_err("Regulator vdd1 enable failed ret=%d\n", ret);
		}
		data->power_enabled = on;
	}
	return ret;
}

static int sx9310_power_ctl(sx93XX_t *data, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			pr_err("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			pr_err("Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			pr_err("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		msleep(8);	/* //>=5ms OK. */
		ret = regulator_enable(data->vio);
		if (ret) {
			pr_err("Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		msleep(10);	/* wait 10ms */
		data->power_enabled = on;
	} else {
		pr_info("Power on=%d. enabled=%d\n", on, data->power_enabled);
	}

	return ret;
}

static int sx9310_power_init(sx93XX_t *data)
{
	int ret;
	/* struct i2c_client *client =(struct i2c_client *)data->bus; */

	data->vdd =
	    regulator_get(&((struct i2c_client *)data->bus)->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		pr_err("Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
					    SX9310_VDD_MIN_UV,
					    SX9310_VDD_MAX_UV);
		if (ret) {
			pr_err("Regulator set failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	data->vdd1 =
	    regulator_get(&((struct i2c_client *)data->bus)->dev, "vdd1");
	if (IS_ERR(data->vdd1)) {
		ret = PTR_ERR(data->vdd1);
		pr_err("Regulator get failed vdd1 ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vdd1) > 0) {
		ret = regulator_set_voltage(data->vdd1,
					    SX9310_VDD1_MIN_UV,
					    SX9310_VDD1_MAX_UV);
		if (ret) {
			pr_err("Regulator set failed vdd1 ret=%d\n", ret);
			goto reg_vdd1_put;
		}
	}

	data->vio =
	    regulator_get(&((struct i2c_client *)data->bus)->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		pr_err("Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd1_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
					    SX9310_VIO_MIN_UV,
					    SX9310_VIO_MAX_UV);
		if (ret) {
			pr_err("Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd1_set:
	if (regulator_count_voltages(data->vdd1) > 0)
		regulator_set_voltage(data->vdd1, 0, SX9310_VDD1_MAX_UV);
reg_vdd1_put:
	regulator_put(data->vdd1);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, SX9310_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int sx9310_parse_dt(struct device *dev, sx9310_platform_data_t *pdata)
{
	/* struct device_node *np = dev->of_node; */
	/* struct i2c_client *client = to_i2c_client(dev); */

	pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
						  "Semtech,gpio-irq", 0, NULL);
	if (pdata->irq_gpio < 0)
		pr_debug("sx910 irq gpio is not available\n");
	return 0;
}

static ssize_t capsensor_config_read_proc(struct file *file, char __user *page,
					  size_t size, loff_t *ppos)
{
	char *ptr = page;

	if (*ppos) {
		return 0;
	}

	pr_debug("Reading sx9310 enable status sx9310_enable = %u\n",
		 sx9310_enable);
	ptr += sprintf(ptr, "%u\n", sx9310_enable);

	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t capsensor_config_write_proc(struct file *filp,
					   const char __user *buffer,
					   size_t count, loff_t *off)
{
	unsigned int val = 0;

	if (sscanf(buffer, "%u", &val) != 1)
		return -EINVAL;

	pr_info("%s val = %u\n", __func__, val);
	if (val) {
		if (!sx9310_enable && psx93XX_this) {
#ifdef USE_THREADED_IRQ
			mutex_lock(&psx93XX_this->mutex);
			/* Just in case need to reset any uncaught interrupts */
			sx93XX_process_interrupt(psx93XX_this, 0);
			mutex_unlock(&psx93XX_this->mutex);
#else
			sx93XX_schedule_work(psx93XX_this, 0);
#endif
			if (psx93XX_this->init)
				psx93XX_this->init(psx93XX_this);

			enable_irq(psx93XX_this->irq);
			/* write_register(psx93XX_this,0x10,0x56); // modify by zch for cs0 */
			write_register(psx93XX_this, 0x10, 0x51);

			write_register(psx93XX_this, 0x00, 0xff);
		} else {
			if (psx93XX_this)
				pr_info("sx9310 has already enabled.\n");
		}
	} else {
		if (sx9310_enable && psx93XX_this) {
			disable_irq(psx93XX_this->irq);
			write_register(psx93XX_this, 0x10, 0x50);
		} else {
			if (psx93XX_this)
				pr_info("sx9310 has already disabled.\n");
		}
	}

	sx9310_enable = val;

	return count;
}

/* Huaqin add for check hw by zhuqiang at 2018/06/22 start */
/* Failer Index */
static int sx9310_Hardware_Check(psx93XX_t this)
{
	int ret;
	u8 failcode;
	u8 failStatusCode = 0;

	//Check I2C Connection
	ret = read_register(this, SX9310_WHOAMI_REG, &failcode);
	if(ret < 0){
		failStatusCode = SX9310_I2C_ERROR;
	}

	if(failcode!= SX9310_WHOAMI_VALUE){
		failStatusCode = SX9310_ID_ERROR;
	}

	dev_info(this->pdev, "sx9310 failcode = 0x%x\n",failStatusCode);
	return failStatusCode;
}
/* Huaqin add for check hw by zhuqiang at 2018/06/22 end */

/*! \fn static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9310_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int i = 0, err = 0;
	psx93XX_t this = 0;
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pplatData = 0;
	/* u8 reg_value = 0; */
	/* s32 returnValue = 0; */
	struct input_dev *input = NULL;

	printk("zch sar---in sx9310_probe\n");
	dev_dbg(&client->dev, "sx9310_probe()\n");

	/* pplatData = &sx9310_config; */
	/* (1) allocation memory for psx9310_platform_data */
	pplatData = kzalloc(sizeof(sx9310_platform_data_t), GFP_KERNEL);
	if (!pplatData) {
		dev_err(&client->dev, "memory alocation was failed");
		err = -ENOMEM;
		return err;
	}
	/* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
	pplatData->get_is_nirq_low = sx9310_get_nirq_state;
	/*  pointer to an initializer function. Here in case needed in the future */
	/* pplatData->init_platform_hw = sx9310_init_ts; */
	pplatData->init_platform_hw = NULL;
	/*  pointer to an exit function. Here in case needed in the future */
	/* pplatData->exit_platform_hw = sx9310_exit_ts; */
	pplatData->exit_platform_hw = NULL;
	pplatData->irq_gpio = INVALID_GPIO;

	pplatData->pi2c_reg = sx9310_i2c_reg_setup;
	pplatData->i2c_reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup);

	pplatData->pbuttonInformation = &smtcButtonInformation;

	if (client->dev.of_node) {
		err = sx9310_parse_dt(&client->dev, pplatData);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto error_0;
		}
		dev_dbg(&client->dev, "sx9310 use gpio interrupt is %d\n",
			pplatData->irq_gpio);
	} else {
		pplatData = client->dev.platform_data;
		dev_err(&client->dev, "Use platform data\n");
		if (!pplatData) {
			dev_err(&client->dev, "platform data is required!\n");
			return -EINVAL;
		}
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "i2c_check_functionality was failed");
		err = -EIO;
		goto error_0;
	}

	if (gpio_is_valid(pplatData->irq_gpio)) {
		/* sx9310 interrupt gpio request */
		err = gpio_request(pplatData->irq_gpio, "SX9310_NIRQ");
		if (err < 0) {
			dev_err(&client->dev,
				"gpio req failed for sx9310 interrupt\n");
			pplatData->irq_gpio = INVALID_GPIO;
			goto error_0;
		}
		err = gpio_direction_input(pplatData->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"unable to set direction for gpio " "[%d]\n",
				pplatData->irq_gpio);
			goto error_0;
		}
	}

	this = kzalloc(sizeof(sx93XX_t), GFP_KERNEL);	/* create memory for main struct */
	dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%p\n", this);

	if (this) {
		/* In case we need to reinitialize data
		 * (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown
		 * (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		/* this->irq = client->irq; */
		this->irq = client->irq = gpio_to_irq(pplatData->irq_gpio);
		dev_dbg(&client->dev, "sx9310 gpio interrupt id is %d\n",
			this->irq);
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS >= 8) {
			this->statusFunc[0] = 0;	/* TXEN_STAT */
			this->statusFunc[1] = 0;	/* UNUSED */
			this->statusFunc[2] = 0;	/* UNUSED */
			this->statusFunc[3] = read_rawData;	/* CONV_STAT */
			this->statusFunc[4] = 0;	/* COMP_STAT */
			this->statusFunc[5] = touchProcess;	/* RELEASE_STAT */
			this->statusFunc[6] = touchProcess;	/* TOUCH_STAT  */
			this->statusFunc[7] = 0;	/* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);
		/* record device struct */
		this->pdev = &client->dev;
		psx93XX_this = this;

		/* create memory for device specific struct */
		this->pDevice = pDevice = kzalloc(sizeof(sx9310_t), GFP_KERNEL);
		dev_dbg(&client->dev,
			"\t Initialized Device Specific Memory: 0x%p\n",
			pDevice);
	
		/* Huaqin add for check hw by zhuqiang at 2018/06/22 start */
		if (sx9310_Hardware_Check(this) != 0)
		{
			goto error_1;
		}
		/* Huaqin add for check hw by zhuqiang at 2018/06/21 end */


		if (pDevice) {
			/* for accessing items in user data (e.g. calibrate) */
			/* sysfs_create_group(&client->dev.kobj, &sx9310_attr_group); */
			err =
			    sysfs_create_group(&client->dev.kobj,
					       &sx9310_attr_group);
			if (err) {
				dev_err(&client->dev,
					"failed to register sysfs. err: %d\n",
					err);
				goto error_1;
			}

			/* Check if we hava a platform initialization function to call */
			if (pplatData->init_platform_hw)
				pplatData->init_platform_hw();

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation =
			    pplatData->pbuttonInformation;

			/* err = sx9310_power_init(client, pplatData); */
			err = sx9310_power_init(this);
			if (err) {
				dev_err(&client->dev,
					"Failed to get Capacitive Touch Controller regulators\n");
				err = -EINVAL;
				goto error_1;
			}

			/* err = sx9310_power_ctl(pplatData, true); */
			err = sx9310_power_ctl(this, true);

			err |= sx9310_power_vdd1_ctl(this, true);
			if (err) {
				dev_err(&client->dev,
					"Failed to enable Capacitive Touch Controller power\n");
				err = -EINVAL;
				goto error_1;
			}
			PSX9310Device = pDevice;

			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				err = -ENOMEM;
				goto error_1;
			}

			/* Set all the keycodes */
			__set_bit(EV_KEY, input->evbit);
			for (i = 0; i < pDevice->pbuttonInformation->buttonSize;
			     i++) {
				__set_bit(pDevice->pbuttonInformation->
					  buttons[i].keycode0, input->keybit);
				__set_bit(pDevice->pbuttonInformation->
					  buttons[i].keycode1, input->keybit);
				pDevice->pbuttonInformation->buttons[i].state =
				    IDLE;
			}
			/* save the input pointer and finish initialization */
			pDevice->pbuttonInformation->input = input;
			input->name = "SX9310 Cap Touch";
			input->id.bustype = BUS_I2C;
			if (input_register_device(input)) {
				err = -ENOMEM;
				goto error_1;
			}
		}
#if 0
		read_register(this, SX9310_IRQSTAT_TOUCH_FLAG, &reg_value);
		pr_info("read SX9310_IRQSTAT_TOUCH_FLAG is 0x%x\n", reg_value);

		returnValue = write_register(this, SX9310_CPS_CTRL18_REG, 0x20);
		read_register(this, SX9310_CPS_CTRL18_REG, &reg_value);
		pr_info("read SX9310_CPS_CTRL18_REG is 0x%x\n", reg_value);

		returnValue = write_register(this, SX9310_CPS_CTRL19_REG, 0xFF);
		read_register(this, SX9310_CPS_CTRL19_REG, &reg_value);
		pr_info("read SX9310_CPS_CTRL19_REG is 0x%x\n", reg_value);
#endif
		sx93XX_init(this);

	pplatData->sar_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pplatData->sar_pinctrl)) {
		pr_err("[sx9310 error] pinctrl get error!");
	} else {
		sar_int_config(1, pplatData->sar_pinctrl);
	}

		/* Create proc file system */
		capsensor_proc =
		    proc_create(PROC_CAPSENSOR_FILE, 0660, NULL,
				&config_proc_ops);
		if (capsensor_proc == NULL) {
			pr_info("create_proc_entry %s failed\n",
				PROC_CAPSENSOR_FILE);
		} else {
			pr_info("create proc entry %s success",
				PROC_CAPSENSOR_FILE);
		}
		printk("zch sar---sx9310_probe success\n");

		return 0;
	}
error_1:
	kfree(pDevice);
	PSX9310Device = NULL;
	kfree(this);
error_0:
	kfree(pplatData);
	/* Huaqin add for check hw by zhuqiang at 2018/06/22 start */
	err_flag = 1;
	/* Huaqin add for check hw by zhuqiang at 2018/06/22 end */
	return err;
}

/*! \fn static int sx9310_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
static int sx9310_remove(struct i2c_client *client)
{
	psx9310_platform_data_t pplatData = 0;
	psx9310_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);

	if (this && (pDevice = this->pDevice)) {
		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw();
		sx9310_power_ctl(this, false);
		sx9310_power_vdd1_ctl(this, false);
		kfree(this->pDevice);
	}
	sx9310_enable = 0;
	return sx93XX_remove(this);
}

#if defined(USE_KERNEL_SUSPEND)
/*====================================================*/
/***** Kernel Suspend *****/
static int sx9310_suspend(struct i2c_client *client, pm_message_t mesg)
{
	psx93XX_t this = i2c_get_clientdata(client);

	if (sx9310_enable)
		sx93XX_suspend(this);

	pr_info("%s\n", __func__);
	return 0;
}

/***** Kernel Resume *****/
static int sx9310_resume(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);

	if (sx9310_enable)
		sx93XX_resume(this);
	pr_info("%s\n", __func__);
	return 0;
}

/*====================================================*/
#endif

static struct i2c_device_id sx9310_idtable[] = {
	{DRIVER_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sx9310_idtable);

static const struct of_device_id sx9310_of_match[] = {
	{.compatible = "Semtech,sx9310",},
	{},
};

static struct i2c_driver sx9310_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRIVER_NAME,
		   .of_match_table = sx9310_of_match,
		   },
	.id_table = sx9310_idtable,
	.probe = sx9310_probe,
	.remove = sx9310_remove,
#if defined(USE_KERNEL_SUSPEND)
	.suspend = sx9310_suspend,
	.resume = sx9310_resume,
#endif
};

static int __init sx9310_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&sx9310_driver);
}

static void __exit sx9310_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&sx9310_driver);
}

module_init(sx9310_init);
module_exit(sx9310_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9310 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
