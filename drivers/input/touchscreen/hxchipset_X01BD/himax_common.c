/*  Himax Android Driver Sample Code for common functions

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#include "himax_common.h"
#include "himax_ic_core.h"
/* Huaqin add by zhangxiude for ITO test start */
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/device.h>
/* Huaqin add by zhangxiude for ITO test end */
#ifdef HX_SMART_WAKEUP
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#define GESTURE_EVENT_C 		KEY_TP_GESTURE_C
#define GESTURE_EVENT_E 		KEY_TP_GESTURE_E
#define GESTURE_EVENT_S 		KEY_TP_GESTURE_S
#define GESTURE_EVENT_V 		KEY_TP_GESTURE_V
#define GESTURE_EVENT_W 		KEY_TP_GESTURE_W
#define GESTURE_EVENT_Z 		KEY_TP_GESTURE_Z
#define GESTURE_EVENT_SWIPE_UP KEY_TP_GESTURE_SWIPE_UP
#define GESTURE_EVENT_DOUBLE_CLICK 	KEY_WAKEUP

/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
#define GEST_SUP_NUM 26
/*Setting cust key define (DF = double finger)*/
/*{Double Tap, Up, Down, Left, Rright, C, Z, M,
	O, S, V, W, e, m, @, (reserve),
	Finger gesture, ^, >, <, f(R), f(L), Up(DF), Down(DF),
	Left(DF), Right(DF)}*/

	uint8_t gest_event[GEST_SUP_NUM] = {
	0x80, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
	0x08, 0x79, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
	0x81, 0x1D, 0x2D, 0x3D, 0x1F, 0x2F, 0x51, 0x52,
	0x53, 0x54};

/*gest_event mapping to gest_key_def*/
	uint16_t gest_key_def[GEST_SUP_NUM] = {
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
	GESTURE_EVENT_DOUBLE_CLICK/*KEY_POWER*/, GESTURE_EVENT_SWIPE_UP, 252, 253, 254, GESTURE_EVENT_C, GESTURE_EVENT_Z, 257,
	258, GESTURE_EVENT_S, GESTURE_EVENT_V, GESTURE_EVENT_W, GESTURE_EVENT_E, 263, 264, 265,
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
	266, 267, 268, 269, 270, 271, 272, 273,
	274, 275};
#endif

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)
#define FRAME_COUNT 5

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
#if defined(HX_EN_DYNAMIC_NAME)
char *i_CTPM_firmware_name = NULL;
#else
char *i_CTPM_firmware_name = "Himax_firmware.bin";
#endif
bool g_auto_update_flag = false;
#endif
#if defined(HX_AUTO_UPDATE_FW)
unsigned char *i_CTPM_FW = NULL;
int i_CTPM_FW_len;
int g_i_FW_VER = 0;
int g_i_CFG_VER = 0;
int g_i_CID_MAJ = 0; /*GUEST ID*/
int g_i_CID_MIN = 0; /*VER for GUEST*/
#endif
#ifdef HX_ZERO_FLASH
int g_f_0f_updat = 0;
#endif

struct himax_ts_data *private_ts;
struct himax_ic_data *ic_data;
struct himax_report_data *hx_touch_data;
struct himax_core_fp g_core_fp;
struct himax_debug *debug_data;

struct proc_dir_entry *himax_touch_proc_dir;

uint8_t g_hx_ic_dt_num = 0;
struct himax_chip_detect *g_core_chip_dt = NULL;

// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  start
extern int tp_status_fun(void);
// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  end
#define HIMAX_PROC_TOUCH_FOLDER 	"android_touch"
#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
	extern int himax_debug_init(void);
	extern int himax_debug_remove(void);
#endif
/*ts_work about start*/
struct himax_target_report_data *g_target_report_data = NULL;
int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status);
static void himax_report_all_leave_event(struct himax_ts_data *ts);
/*ts_work about end*/
static int		HX_TOUCH_INFO_POINT_CNT;

unsigned long	FW_VER_MAJ_FLASH_ADDR;
unsigned long 	FW_VER_MIN_FLASH_ADDR;
unsigned long 	CFG_VER_MAJ_FLASH_ADDR;
unsigned long 	CFG_VER_MIN_FLASH_ADDR;
unsigned long 	CID_VER_MAJ_FLASH_ADDR;
unsigned long 	CID_VER_MIN_FLASH_ADDR;
/*unsigned long	PANEL_VERSION_ADDR;*/

unsigned long 	FW_VER_MAJ_FLASH_LENG;
unsigned long 	FW_VER_MIN_FLASH_LENG;
unsigned long 	CFG_VER_MAJ_FLASH_LENG;
unsigned long 	CFG_VER_MIN_FLASH_LENG;
unsigned long 	CID_VER_MAJ_FLASH_LENG;
unsigned long 	CID_VER_MIN_FLASH_LENG;
/*unsigned long	PANEL_VERSION_LENG;*/

unsigned long 	FW_CFG_VER_FLASH_ADDR;

unsigned char	IC_CHECKSUM = 0;

#ifdef HX_ESD_RECOVERY
	u8 HX_ESD_RESET_ACTIVATE = 0;
	int hx_EB_event_flag = 0;
	int hx_EC_event_flag = 0;
	int hx_ED_event_flag = 0;
	int g_zero_event_count = 0;
#endif
u8 	HX_HW_RESET_ACTIVATE = 0;

#if defined(HX_PLATFOME_DEFINE_KEY)
	extern void	himax_platform_key(void);
#endif

extern int himax_parse_dt(struct himax_ts_data *ts,
						struct himax_i2c_platform_data *pdata);

static uint8_t 	AA_press = 0x00;
static uint8_t 	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;

static int	p_point_num	= 0xFFFF;
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
static uint8_t 	vk_press = 0x00;
static int	tpd_key	   	= 0x00;
static int	tpd_key_old	= 0x00;
#endif
static int	probe_fail_flag;
#ifdef HX_USB_DETECT_GLOBAL
	bool USB_detect_flag;
#endif

#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
						unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#ifdef HX_GESTURE_TRACK
	static int gest_pt_cnt;
	static int gest_pt_x[GEST_PT_MAX_NUM];
	static int gest_pt_y[GEST_PT_MAX_NUM];
	static int gest_start_x, gest_start_y, gest_end_x, gest_end_y;
	static int gest_width, gest_height, gest_mid_x, gest_mid_y;
	static int hx_gesture_coor[16];
#endif

int himax_report_data_init(void);

extern int himax_dev_set(struct himax_ts_data *ts);
extern int himax_input_register_device(struct input_dev *input_dev);

int g_ts_dbg = 0;

/* File node for Selftest, SMWP and HSEN - Start*/
#define HIMAX_PROC_SELF_TEST_FILE	"self_test"
struct proc_dir_entry *himax_proc_self_test_file;

uint8_t HX_PROC_SEND_FLAG;
#ifdef HX_SMART_WAKEUP
	#define HIMAX_PROC_SMWP_FILE "SMWP"
	struct proc_dir_entry *himax_proc_SMWP_file = NULL;
	#define HIMAX_PROC_GESTURE_FILE "GESTURE"
	struct proc_dir_entry *himax_proc_GESTURE_file = NULL;
	uint8_t HX_SMWP_EN = 0;
#endif
#if defined(HX_SMART_WAKEUP) || defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
bool FAKE_POWER_KEY_SEND = true;
#endif

#ifdef HX_HIGH_SENSE
	#define HIMAX_PROC_HSEN_FILE "HSEN"
	struct proc_dir_entry *himax_proc_HSEN_file = NULL;
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#if defined(HX_SMART_WAKEUP)
#define NVT_GESTURE_MODE "tpd_gesture"

static long gesture_mode = 0;

static ssize_t nvt_gesture_mode_get_proc(struct file *file,
                        char __user *buffer, size_t size, loff_t *ppos)
{
	char ptr[64] = {0};
	unsigned int len = 0;
	unsigned int ret = 0;

	if (gesture_mode == 0) {
		len = sprintf(ptr, "0\n");
	} else {
		len = sprintf(ptr, "1\n");
	}
	ret = simple_read_from_buffer(buffer, size, ppos, ptr, (size_t)len);
	return ret;
}

static ssize_t nvt_gesture_mode_set_proc(struct file *filp,
                        const char __user *buffer, size_t count, loff_t *off)
{
	char msg[20] = {0};
	int ret = 0;
	struct himax_ts_data *ts = private_ts;
	if (private_ts->suspended) {
		I("Touch is already sleep cant modify gesture node\n");
		return count;
	}
	ret = copy_from_user(msg, buffer, count);
	I("msg = %s\n", msg);
	if (ret) {
		return -EFAULT;
	}

	ret = kstrtol(msg, 0, &gesture_mode);
	if (!ret) {
		if (gesture_mode == 0) {
			gesture_mode = 0;
			ts->SMWP_enable = 0;
		} else {
			gesture_mode = 0x1FF;
			ts->SMWP_enable = 1;
		}
	}
	else {
		E("set gesture mode failed\n");
	}
	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("gesture_mode = 0x%x\n", (unsigned int)gesture_mode);
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);

	return count;
}

static struct proc_dir_entry *nvt_gesture_mode_proc = NULL;
static const struct file_operations gesture_mode_proc_ops = {
	.owner = THIS_MODULE,
	.read = nvt_gesture_mode_get_proc,
	.write = nvt_gesture_mode_set_proc,
};
#endif

#if NVT_POWER_SOURCE_CUST_EN

static int nvt_lcm_bias_power_init(struct himax_ts_data *data)
{
	int ret;
	data->lcm_lab = regulator_get(&data->client->dev, "lcm_lab");
	if (IS_ERR(data->lcm_lab)){
		ret = PTR_ERR(data->lcm_lab);
		E("Regulator get failed lcm_lab ret=%d", ret);
		goto _end;
	}
	if (regulator_count_voltages(data->lcm_lab)>0){
		ret = regulator_set_voltage(data->lcm_lab, LCM_LAB_MIN_UV, LCM_LAB_MAX_UV);
		if (ret){
			E("Regulator set_vtg failed lcm_lab ret=%d", ret);
			goto reg_lcm_lab_put;
		}
	}
	data->lcm_ibb = regulator_get(&data->client->dev, "lcm_ibb");
	if (IS_ERR(data->lcm_ibb)){
		ret = PTR_ERR(data->lcm_ibb);
		E("Regulator get failed lcm_ibb ret=%d", ret);
		goto reg_set_lcm_lab_vtg;
	}
	if (regulator_count_voltages(data->lcm_ibb)>0){
		ret = regulator_set_voltage(data->lcm_ibb, LCM_IBB_MIN_UV, LCM_IBB_MAX_UV);
		if (ret){
			E("Regulator set_vtg failed lcm_lab ret=%d", ret);
			goto reg_lcm_ibb_put;
		}
	}
	return 0;
reg_lcm_ibb_put:
	if (regulator_count_voltages(data->lcm_ibb) > 0){
		regulator_set_voltage(data->lcm_ibb, 0, LCM_IBB_MAX_UV);
	}
reg_set_lcm_lab_vtg:
	data->lcm_ibb = NULL;
	regulator_put(data->lcm_ibb);
reg_lcm_lab_put:
	if (regulator_count_voltages(data->lcm_lab) > 0){
		regulator_set_voltage(data->lcm_lab, 0, LCM_LAB_MAX_UV);
	}
_end:
	data->lcm_lab = NULL;
	regulator_put(data->lcm_lab);
	return ret;
}

static int nvt_lcm_bias_power_deinit(struct himax_ts_data *data)
{
	if (data-> lcm_ibb != NULL){
		if (regulator_count_voltages(data->lcm_ibb) > 0){
			regulator_set_voltage(data->lcm_ibb, 0, LCM_IBB_MAX_UV);
		}
		regulator_put(data->lcm_ibb);
	}
	if (data-> lcm_lab != NULL){
		if (regulator_count_voltages(data->lcm_lab) > 0){
			regulator_set_voltage(data->lcm_lab, 0, LCM_LAB_MAX_UV);
		}
		regulator_put(data->lcm_lab);
	}
	return 0;

}


static int nvt_lcm_power_source_ctrl(struct himax_ts_data *data, int enable)
{
	int rc;

	if (data->lcm_lab!= NULL && data->lcm_ibb!= NULL){
		if (enable){
			if (atomic_inc_return(&(data->lcm_lab_power)) == 1) {
				rc = regulator_enable(data->lcm_lab);
				if (rc) {
					atomic_dec(&(data->lcm_lab_power));
					E("Regulator lcm_lab enable failed rc=%d", rc);
				}
			}
			else {
				atomic_dec(&(data->lcm_lab_power));
			}
			if (atomic_inc_return(&(data->lcm_ibb_power)) == 1) {
				rc = regulator_enable(data->lcm_ibb);
				if (rc) {
					atomic_dec(&(data->lcm_ibb_power));
					E("Regulator lcm_ibb enable failed rc=%d", rc);
				}
			}
			else {
				atomic_dec(&(data->lcm_ibb_power));
			}
		}
		else {
			if (atomic_dec_return(&(data->lcm_lab_power)) == 0) {
				rc = regulator_disable(data->lcm_lab);
				if (rc)
				{
					atomic_inc(&(data->lcm_lab_power));
					E("Regulator lcm_lab disable failed rc=%d", rc);
				}
			}
			else{
				atomic_inc(&(data->lcm_lab_power));
			}
			if (atomic_dec_return(&(data->lcm_ibb_power)) == 0) {
				rc = regulator_disable(data->lcm_ibb);
				if (rc)	{
					atomic_inc(&(data->lcm_ibb_power));
					E("Regulator lcm_ibb disable failed rc=%d", rc);
				}
			}
			else{
				atomic_inc(&(data->lcm_ibb_power));
			}
		}
	}
	else
		E("Regulator lcm_ibb or lcm_lab is invalid");
	return 0;
}

#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
/* Huaqin add by zhangxiude for ITO test start */
#define HIMAX_INFO_PROC_FILE "nvt_info"
static struct proc_dir_entry *himax_info_proc_entry;
static int32_t firmware_id = 0;

static void *himax_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}


static void *himax_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}


static void himax_stop(struct seq_file *m, void *v)
{
	return;
}

static int32_t himax_info_show(struct seq_file *m, void *v)
{
	/* Huaqin modify for nvt_info by limengxia at 2018/11/05 start */
        seq_printf(m, "IC=HIMAX HX83112 module=TianMa fw_ver= 0x%2.2X\n",ic_data->vendor_touch_cfg_ver);
	/* Huaqin modify for nvt_info by limengxia at 2018/11/05 end */
	return 0;
}
const struct seq_operations himax_info_seq_ops = {
	.start  = himax_start,
	.next   = himax_next,
	.stop   = himax_stop,
	.show   = himax_info_show
};
/*******************************************************
Description:
	nvt touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t himax_info_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_info_seq_ops);
}

static const struct file_operations himax_info_proc_fops = {
	.owner = THIS_MODULE,
	.open = himax_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
/*******************************************************
Description:
	nvt touchscreen info function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t himax_tp_info_proc_init(void)
{
	himax_info_proc_entry = proc_create(HIMAX_INFO_PROC_FILE, 0777, NULL, &himax_info_proc_fops);
	if (NULL == himax_info_proc_entry)
	{
           printk( "[himax] %s() Couldn't create proc entry!",  __func__);
           return -ENOMEM;
	}
	else
	{
           printk( "[himax] %s() Create proc entry success!",  __func__);
	}
	return 0;
}
/**********add ito test mode function  *******************/
int himax_TestResultLen=0;
static struct platform_device hwinfo_device= {
	.name = HWINFO_NAME,
	.id = -1,
};
extern int32_t himax_selftest_open(void);
static ssize_t himax_test_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	int count;
	himax_selftest_open();
	count = sprintf(buf, "%d\n", himax_TestResultLen);
	return count;
}

static ssize_t himax_test_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(factory_check, 0644, himax_test_show, himax_test_store);

static struct attribute *ito_test_attributes[] ={

	&dev_attr_factory_check.attr,
	NULL
};
static struct attribute_group ito_test_attribute_group = {

.attrs = ito_test_attributes

};
int himax_test_node_init(struct platform_device *tpinfo_device)
{
	int err=0;
    err = sysfs_create_group(&tpinfo_device->dev.kobj, &ito_test_attribute_group);
    if (0 != err)
    {
        printk( "[himax-ito] %s() - ERROR: sysfs_create_group() failed.",  __func__);
        sysfs_remove_group(&tpinfo_device->dev.kobj, &ito_test_attribute_group);
        return -EIO;
    }
    else
    {
        printk("[himax-ito] %s() - sysfs_create_group() succeeded.", __func__);
    }
    return err;
}
/*************************************************/
/* Huaqin add by zhangxiude for ITO test end */
#if defined(HX_PALM_REPORT)
static int himax_palm_detect(uint8_t *buf)
{
	struct himax_ts_data *ts = private_ts;
	int32_t loop_i;
	int base = 0;
	int x = 0, y = 0, w = 0;

	loop_i = 0;
	base = loop_i * 4;
	x = buf[base] << 8 | buf[base + 1];
	y = (buf[base + 2] << 8 | buf[base + 3]);
	w = buf[(ts->nFinger_support * 4) + loop_i];
	I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x \n", __func__, loop_i, base, x, y, w);
	if ((!atomic_read(&ts->suspend_mode)) && (x == 0xFA5A) && (y == 0xFA5A) && (w == 0x00))
		return PALM_REPORT;
	else
		return NOT_REPORT;
}
#endif

static ssize_t himax_self_test_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	int val = 0x00;
	size_t ret = 0;
	char *temp_buf;
	I("%s: enter, %d \n", __func__, __LINE__);

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		himax_int_enable(0);/* disable irq */
		private_ts->in_self_test = 1;
		val = g_core_fp.fp_chip_self_test();
#ifdef HX_ESD_RECOVERY
		HX_ESD_RESET_ACTIVATE = 1;
#endif
		himax_int_enable(1);/* enable irq */

		if (val == 0x00) {
			ret += snprintf(temp_buf + ret, len - ret, "Self_Test Pass\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "Self_Test Fail\n");
		}

		private_ts->in_self_test = 0;

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_self_test_ops = {
	.owner = THIS_MODULE,
	.read = himax_self_test_read,
};

#ifdef HX_HIGH_SENSE
static ssize_t himax_HSEN_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->HSEN_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_HSEN_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0')
		ts->HSEN_enable = 0;
	else if (buf[0] == '1')
		ts->HSEN_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_HSEN_enable(ts->HSEN_enable, ts->suspended);
	I("%s: HSEN_enable = %d.\n", __func__, ts->HSEN_enable);
	return len;
}

static struct file_operations himax_proc_HSEN_ops = {
	.owner = THIS_MODULE,
	.read = himax_HSEN_read,
	.write = himax_HSEN_write,
};
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_SMWP_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0')
		ts->SMWP_enable = 0;
	else if (buf[0] == '1')
		ts->SMWP_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);
	return len;
}

static struct file_operations himax_proc_SMWP_ops = {
	.owner = THIS_MODULE,
	.read = himax_SMWP_read,
	.write = himax_SMWP_write,
};

static ssize_t himax_GESTURE_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		for (i = 0; i < GEST_SUP_NUM; i++)
			ret += snprintf(temp_buf + ret, len - ret, "ges_en[%d]=%d \n", i, ts->gesture_cust_en[i]);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}

	return ret;
}

static ssize_t himax_GESTURE_write(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	int j = 0;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	I("himax_GESTURE_store= %s, len = %d \n", buf, (int)len);

	for (i = 0; i < len; i++) {
		if (buf[i] == '0' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 0;
			I("gesture en[%d]=%d \n", j, ts->gesture_cust_en[j]);
			j++;
		}	else if (buf[i] == '1' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 1;
			I("gesture en[%d]=%d \n", j, ts->gesture_cust_en[j]);
			j++;
		}	else
			I("Not 0/1 or >=GEST_SUP_NUM : buf[%d] = %c\n", i, buf[i]);
	}

	return len;
}

static struct file_operations himax_proc_Gesture_ops = {
	.owner = THIS_MODULE,
	.read = himax_GESTURE_read,
	.write = himax_GESTURE_write,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
extern void (*fp_himax_self_test_init)(void);
#endif

int himax_common_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir(HIMAX_PROC_TOUCH_FOLDER, NULL);

	if (himax_touch_proc_dir == NULL) {
		E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
	if (fp_himax_self_test_init != NULL)
		fp_himax_self_test_init();
#endif

	himax_proc_self_test_file = proc_create(HIMAX_PROC_SELF_TEST_FILE, (S_IRUGO), himax_touch_proc_dir, &himax_proc_self_test_ops);
	if (himax_proc_self_test_file == NULL) {
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_1;
	}

#ifdef HX_HIGH_SENSE
	himax_proc_HSEN_file = proc_create(HIMAX_PROC_HSEN_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_HSEN_ops);

	if (himax_proc_HSEN_file == NULL) {
		E(" %s: proc HSEN file create failed!\n", __func__);
		goto fail_2;
	}

#endif
#ifdef HX_SMART_WAKEUP
	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_SMWP_ops);

	if (himax_proc_SMWP_file == NULL) {
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_GESTURE_file = proc_create(HIMAX_PROC_GESTURE_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										  himax_touch_proc_dir, &himax_proc_Gesture_ops);

	if (himax_proc_GESTURE_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_4;
	}
#endif
	return 0;
#ifdef HX_SMART_WAKEUP
	remove_proc_entry(HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir);
fail_4:
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
fail_3:
#endif
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
fail_2:
#endif
	remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
fail_1:
	return -ENOMEM;
}

void himax_common_proc_deinit(void)
{
	remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
#ifdef HX_SMART_WAKEUP
	remove_proc_entry(HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
#endif
}

/* File node for SMWP and HSEN - End*/

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;
#if defined(HX_SMART_WAKEUP)
	int i = 0;
#endif
	ret = himax_dev_set(ts);

	if (ret < 0) {
		goto input_device_fail;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
#if defined(HX_PLATFOME_DEFINE_KEY)
	himax_platform_key();
#else
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP) || defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP)
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
	for (i = 0; i < GEST_SUP_NUM; i++) {
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
		set_bit(gest_key_def[i], ts->input_dev->keybit);
	}
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#ifdef	HX_PROTOCOL_A
	/*ts->input_dev->mtsize = ts->nFinger_support;*/
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 3, 0, 0);
#else
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if defined(HX_PROTOCOL_B_3PA)
	input_mt_init_slots(ts->input_dev, ts->nFinger_support, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif
	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
	  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
#ifndef	HX_PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);
#endif
/*	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);*/

	if (himax_input_register_device(ts->input_dev) == 0) {
		return NO_ERR;
	} else {
		ret = INPUT_REGISTER_FAIL;
	}

input_device_fail:
	I("%s, input device register fail!\n", __func__);
	return ret;
}

static void calcDataSize(void)
{
	struct himax_ts_data *ts_data = private_ts;
	ts_data->x_channel = ic_data->HX_RX_NUM;
	ts_data->y_channel = ic_data->HX_TX_NUM;
	ts_data->nFinger_support = ic_data->HX_MAX_PT;

	ts_data->coord_data_size = 4 * ts_data->nFinger_support;
	ts_data->area_data_size = ((ts_data->nFinger_support / 4) + (ts_data->nFinger_support % 4 ? 1 : 0)) * 4;
	ts_data->coordInfoSize = ts_data->coord_data_size + ts_data->area_data_size + 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;

	if (ts_data->raw_data_frame_size == 0) {
		E("%s: could NOT calculate! \n", __func__);
		return;
	}

	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
									(((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size) ? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d\n", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4;

	if ((ic_data->HX_MAX_PT % 4) == 0) {
		HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4;
	} else {
		HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) + 1) * 4;
	}
}
#if defined(HX_AUTO_UPDATE_FW)
static int himax_auto_update_check(void)
{
	I("%s:Entering!\n", __func__);
	if (g_core_fp.fp_fw_ver_bin() == 0) {
		if (((ic_data->vendor_fw_ver < g_i_FW_VER) || (ic_data->vendor_config_ver < g_i_CFG_VER))) {
			I("Need to update!\n");
			return NO_ERR;
		}	else {
			E("No need to update!\n");
			return 1;
		}
	} else {
		E("FW bin fail!\n");
		return 1;
	}
}
static int i_get_FW(void)
{
	int ret = 0;
	const struct firmware *image = NULL;

	I("file name = %s\n", i_CTPM_firmware_name);
	ret = request_firmware(&image, i_CTPM_firmware_name, private_ts->dev);
	if (ret < 0) {
		E("%s,fail in line%d error code=%d\n", __func__, __LINE__, ret);
		return OPEN_FILE_FAIL;
	}

	if (image != NULL) {
		i_CTPM_FW_len = image->size;
		i_CTPM_FW = kzalloc(sizeof(char)*i_CTPM_FW_len, GFP_KERNEL);
		memcpy(i_CTPM_FW, image->data, sizeof(char)*i_CTPM_FW_len);
	} else {
		I("%s: i_CTPM_FW = NULL\n", __func__);
		return OPEN_FILE_FAIL;
	}

	release_firmware(image);
	ret = NO_ERR;
	return ret;
}
static int i_update_FW(void)
{
	int upgrade_times = 0;

	uint8_t ret = 0, result = 0;

	himax_int_enable(0);


update_retry:

	if (i_CTPM_FW_len == FW_SIZE_32k) {
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k(i_CTPM_FW, i_CTPM_FW_len, false);
	} else if (i_CTPM_FW_len == FW_SIZE_60k) {
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k(i_CTPM_FW, i_CTPM_FW_len, false);
	} else if (i_CTPM_FW_len == FW_SIZE_64k) {
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k(i_CTPM_FW, i_CTPM_FW_len, false);
	} else if (i_CTPM_FW_len == FW_SIZE_124k) {
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k(i_CTPM_FW, i_CTPM_FW_len, false);
	} else if (i_CTPM_FW_len == FW_SIZE_128k) {
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k(i_CTPM_FW, i_CTPM_FW_len, false);
	}

	if (ret == 0) {
		upgrade_times++;
		E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);

		if (upgrade_times < 3) {
			goto update_retry;
		}	else {
			result = -1;
		} /*upgrade fail*/
	} else {
		g_core_fp.fp_read_FW_ver();
		g_core_fp.fp_touch_information();
		result = 1;/*upgrade success*/
		I("%s: TP upgrade OK\n", __func__);
	}

	kfree(i_CTPM_FW);

#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#else
	g_core_fp.fp_sense_on(0x00);
#endif
	himax_int_enable(1);
	return result;
}
#endif


static int himax_loadSensorConfig(struct himax_i2c_platform_data *pdata)
{
	I("%s: initialization complete\n", __func__);
	return NO_ERR;
}

#ifdef HX_ESD_RECOVERY
static void himax_esd_hw_reset(void)
{
#ifdef HX_ZERO_FLASH
	int result = 0;
#endif
	if (g_ts_dbg != 0)
		I("%s: Entering \n", __func__);

	I("START_Himax TP: ESD - Reset\n");

	if (private_ts->in_self_test == 1) {
		I("In self test , not  TP: ESD - Reset\n");
		return;
	}

	g_core_fp.fp_esd_ic_reset();
#ifdef HX_ZERO_FLASH
	I("It will update fw after esd event in zero flash mode!\n");
	result = g_core_fp.fp_0f_operation_dirly();
	if (result) {
		E("Something is wrong! Skip Update with zero flash!\n");
		goto ESCAPE_0F_UPDATE;
	}
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_sense_on(0x00);
	himax_report_all_leave_event(private_ts);
	himax_int_enable(1);
ESCAPE_0F_UPDATE:
#endif
	I("END_Himax TP: ESD - Reset\n");
}
#endif

#ifdef HX_SMART_WAKEUP
#ifdef HX_GESTURE_TRACK
static void gest_pt_log_coordinate(int rx, int tx)
{
	/*driver report x y with range 0 - 255 , we scale it up to x/y pixel*/
	gest_pt_x[gest_pt_cnt] = rx * (ic_data->HX_X_RES) / 255;
	gest_pt_y[gest_pt_cnt] = tx * (ic_data->HX_Y_RES) / 255;
}
#endif
static int himax_wake_event_parse(struct himax_ts_data *ts, int ts_status)
{
	uint8_t *buf;
#ifdef HX_GESTURE_TRACK
	int tmp_max_x = 0x00, tmp_min_x = 0xFFFF, tmp_max_y = 0x00, tmp_min_y = 0xFFFF;
	int gest_len;
#endif
	int i = 0, check_FC = 0;
	int j = 0, gesture_pos = 0, gesture_flag = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering!, ts_status=%d\n", __func__, ts_status);

	buf = kzalloc(hx_touch_data->event_size * sizeof(uint8_t), GFP_KERNEL);
	if (buf == NULL) {
		return -ENOMEM;
	}

	memcpy(buf, hx_touch_data->hx_event_buf, hx_touch_data->event_size);

	for (i = 0; i < GEST_PTLG_ID_LEN; i++) {
		for (j = 0; j < GEST_SUP_NUM; j++) {
			if (buf[i] == gest_event[j]) {
				gesture_flag = buf[i];
				gesture_pos = j;
				break;
			}
		}
		I("0x%2.2X ", buf[i]);
		if (buf[i] == gesture_flag) {
			check_FC++;
		} else {
			I("ID START at %x , value = 0x%2X skip the event\n", i, buf[i]);
			break;
		}
	}

	I("Himax gesture_flag= %x\n", gesture_flag);
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC != GEST_PTLG_ID_LEN) {
		kfree(buf);
		return 0;
	}

	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
		buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		kfree(buf);
		return 0;
	}

#ifdef HX_GESTURE_TRACK

	if (buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
		buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = buf[GEST_PTLG_ID_LEN + 2];
		I("gest_len = %d \n", gest_len);
		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start \n %s", __func__);

		while (i < (gest_len + 1) / 2) {
			gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN + 4 + i * 2], buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1]);
			i++;
			I("gest_pt_x[%d]=%d \n", gest_pt_cnt, gest_pt_x[gest_pt_cnt]);
			I("gest_pt_y[%d]=%d \n", gest_pt_cnt, gest_pt_y[gest_pt_cnt]);
			gest_pt_cnt += 1;
		}

		if (gest_pt_cnt) {
			for (i = 0; i < gest_pt_cnt; i++) {
				if (tmp_max_x < gest_pt_x[i])
					tmp_max_x = gest_pt_x[i];
				if (tmp_min_x > gest_pt_x[i])
					tmp_min_x = gest_pt_x[i];
				if (tmp_max_y < gest_pt_y[i])
					tmp_max_y = gest_pt_y[i];
				if (tmp_min_y > gest_pt_y[i])
					tmp_min_y = gest_pt_y[i];
			}

			I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n", tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);
			gest_start_x = gest_pt_x[0];
			hx_gesture_coor[0] = gest_start_x;
			gest_start_y = gest_pt_y[0];
			hx_gesture_coor[1] = gest_start_y;
			gest_end_x = gest_pt_x[gest_pt_cnt - 1];
			hx_gesture_coor[2] = gest_end_x;
			gest_end_y = gest_pt_y[gest_pt_cnt - 1];
			hx_gesture_coor[3] = gest_end_y;
			gest_width = tmp_max_x - tmp_min_x;
			hx_gesture_coor[4] = gest_width;
			gest_height = tmp_max_y - tmp_min_y;
			hx_gesture_coor[5] = gest_height;
			gest_mid_x = (tmp_max_x + tmp_min_x) / 2;
			hx_gesture_coor[6] = gest_mid_x;
			gest_mid_y = (tmp_max_y + tmp_min_y) / 2;
			hx_gesture_coor[7] = gest_mid_y;
			hx_gesture_coor[8] = gest_mid_x;/*gest_up_x*/
			hx_gesture_coor[9] = gest_mid_y - gest_height / 2; /*gest_up_y*/
			hx_gesture_coor[10] = gest_mid_x;/*gest_down_x*/
			hx_gesture_coor[11] = gest_mid_y + gest_height / 2;	/*gest_down_y*/
			hx_gesture_coor[12] = gest_mid_x - gest_width / 2;	/*gest_left_x*/
			hx_gesture_coor[13] = gest_mid_y;	/*gest_left_y*/
			hx_gesture_coor[14] = gest_mid_x + gest_width / 2;	/*gest_right_x*/
			hx_gesture_coor[15] = gest_mid_y; /*gest_right_y*/
		}
	}

#endif

	kfree(buf);
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#if 0
	if (!ts->gesture_cust_en[gesture_pos]) {
		I("%s NOT report key [%d] = %d \n", __func__, gesture_pos, gest_key_def[gesture_pos]);
		g_target_report_data->SMWP_event_chk = 0;
		return 0;
	} else
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
	{
		g_target_report_data->SMWP_event_chk = gest_key_def[gesture_pos];
		return gesture_pos;
	}
}

static void himax_wake_event_report(void)
{
	int KEY_EVENT = g_target_report_data->SMWP_event_chk;

	if (g_ts_dbg != 0)
		I("%s: Entering! \n", __func__);

	if (KEY_EVENT) {
		I(" %s SMART WAKEUP KEY event %d press\n", __func__, KEY_EVENT);
		input_report_key(private_ts->input_dev, KEY_EVENT, 1);
		input_sync(private_ts->input_dev);
		I(" %s SMART WAKEUP KEY event %d release\n", __func__, KEY_EVENT);
		input_report_key(private_ts->input_dev, KEY_EVENT, 0);
		input_sync(private_ts->input_dev);
		FAKE_POWER_KEY_SEND = true;
#ifdef HX_GESTURE_TRACK
		I("gest_start_x= %d, gest_start_y= %d, gest_end_x= %d, gest_end_y= %d\n", gest_start_x, gest_start_y,
		  gest_end_x, gest_end_y);
		I("gest_width= %d, gest_height= %d, gest_mid_x= %d, gest_mid_y= %d\n", gest_width, gest_height,
		  gest_mid_x, gest_mid_y);
		I("gest_up_x= %d, gest_up_y= %d, gest_down_x= %d, gest_down_y= %d\n", hx_gesture_coor[8], hx_gesture_coor[9],
		  hx_gesture_coor[10], hx_gesture_coor[11]);
		I("gest_left_x= %d, gest_left_y= %d, gest_right_x= %d, gest_right_y= %d\n", hx_gesture_coor[12], hx_gesture_coor[13],
		  hx_gesture_coor[14], hx_gesture_coor[15]);
#endif
		g_target_report_data->SMWP_event_chk = 0;
	}
}

#endif

int himax_report_data_init(void)
{
	if (hx_touch_data->hx_coord_buf != NULL) {
		kfree(hx_touch_data->hx_coord_buf);
	}

	if (hx_touch_data->hx_rawdata_buf != NULL) {
		kfree(hx_touch_data->hx_rawdata_buf);
	}

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->event_size = g_core_fp.fp_get_touch_data_size();

	if (hx_touch_data->hx_event_buf != NULL) {
		kfree(hx_touch_data->hx_event_buf);
	}

#endif
	hx_touch_data->touch_all_size = g_core_fp.fp_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT / 4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 2) * 4;
	} else { /* less than 4 fingers */
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 1) * 4;
	}

	if ((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) % hx_touch_data->rawdata_size == 0) {
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size;
	} else {
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size + 1;
	}

	I("%s: rawdata_frame_size = %d\n", __func__, hx_touch_data->rawdata_frame_size);
	I("%s: ic_data->HX_MAX_PT:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, hx_touch_data->touch_info_size:%d\n", __func__, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size, hx_touch_data->touch_info_size);
	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_coord_buf == NULL) {
		goto mem_alloc_fail;
	}

	if (g_target_report_data == NULL) {
		g_target_report_data = kzalloc(sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail;
		g_target_report_data->x = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->y = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->w = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->w == NULL)
			goto mem_alloc_fail;
		g_target_report_data->finger_id = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->finger_id == NULL)
			goto mem_alloc_fail;
	}
#ifdef HX_SMART_WAKEUP
	g_target_report_data->SMWP_event_chk = 0;
#endif

	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_rawdata_buf == NULL) {
		goto mem_alloc_fail;
	}

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->event_size), GFP_KERNEL);

	if (hx_touch_data->hx_event_buf == NULL) {
		goto mem_alloc_fail;
	}

#endif
	return NO_ERR;
mem_alloc_fail:
	kfree(g_target_report_data->x);
	kfree(g_target_report_data->y);
	kfree(g_target_report_data->w);
	kfree(g_target_report_data->finger_id);
	kfree(g_target_report_data);
	g_target_report_data = NULL;
	kfree(hx_touch_data->hx_coord_buf);
	kfree(hx_touch_data->hx_rawdata_buf);
#if defined(HX_SMART_WAKEUP)
	kfree(hx_touch_data->hx_event_buf);
#endif
	I("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}

/*start ts_work*/
#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew)
{
	struct himax_ts_data *ts;
	u32 connect_status = 0;
	/* Huaqin modify for ZQL1820-973 by limengxia at 2018/10/19 start */
	USB_detect_flag=custom_usb_presence;
	/* Huaqin modify for ZQL1820-973 by limengxia at 2018/10/19 end */
	connect_status = USB_detect_flag;/* upmu_is_chr_det(); */
	ts = private_ts;

	/* I("Touch: cable status=%d, cable_config=%p, usb_connected=%d \n", connect_status, ts->cable_config, ts->usb_connected); */
	if (ts->cable_config) {
		if (((!!connect_status) != ts->usb_connected) || force_renew) {
			if (!!connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}

			g_core_fp.fp_usb_detect_set(ts->cable_config);
			I("%s: Cable status change: 0x%2.2X\n", __func__, ts->usb_connected);
		}

		/* else */
		/* 	I("%s: Cable status is the same as previous one, ignore.\n", __func__); */
	}
}
#endif

static int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	hx_touch_data->diag_cmd = ts->diag_cmd;
	if (hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

#ifdef HX_SMART_WAKEUP
/* Huaqin modify for HX by limengxia at 2018/11/07 start */
	if (atomic_read(&ts->suspend_mode) && ! (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) && (!hx_touch_data->diag_cmd))
/* Huaqin modify for HX by limengxia at 2018/11/07 end */
	        result = HX_REPORT_SMWP_EVENT;
#endif
	/* I("Now Status is %d\n", result); */
	return result;
}

static int himax_touch_get(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
			|| (HX_ESD_RESET_ACTIVATE)
#endif
			) {
			if (!g_core_fp.fp_read_event_stack(buf, 128)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
				goto END_FUNCTION;
			}
			break;
		} else {
			if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->touch_info_size)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
				goto END_FUNCTION;
			}
			break;
		}
#if defined(HX_SMART_WAKEUP)

	/*SMWP*/
	case HX_REPORT_SMWP_EVENT:
		g_core_fp.fp_burst_enable(0);

		if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->event_size)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		if (!g_core_fp.fp_read_event_stack(buf, 128)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;
	default:
		break;
	}

END_FUNCTION:
	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t	i = 0;
	int length = 0;
	int zero_cnt = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		I("[HIMAX TP MSG] All Zero event\n");
		ret_val = HX_CHKSUM_FAIL;
//Huaqin add for ESD test bu xudongfang at 2018/11/05 start
	}else{
		private_ts->is_esd_event = false;
//Huaqin add for ESD test bu xudongfang at 2018/11/05 end
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d! \n", __func__, ret_val);
	return ret_val;
}

#ifdef HX_ESD_RECOVERY
#ifdef HX_ZERO_FLASH
void hx_update_dirly_0f(void)
{
	I("It will update fw after esd event in zero flash mode!\n");
	g_core_fp.fp_0f_operation_dirly();
}
#endif
static int himax_ts_event_check(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	int hx_EB_event = 0;
	int hx_EC_event = 0;
	int hx_ED_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	int32_t	loop_i = 0;
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		I("Now Path=%d, Now status=%d, length=%d\n", ts_path, ts_status, length);

	for (loop_i = 0; loop_i < length; loop_i++) {
		if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_EB_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_EC_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ED_event++;
			} else if (buf[loop_i] == 0x00) { /* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_EB_event = 0;
				hx_EC_event = 0;
				hx_ED_event = 0;
				hx_zero_event = 0;
				g_zero_event_count = 0;
			}
		}
	}

	if (hx_EB_event == length) {
		hx_esd_event = length;
		hx_EB_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
	} else if (hx_EC_event == length) {
		hx_esd_event = length;
		hx_EC_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
	} else if (hx_ED_event == length) {
		hx_esd_event = length;
		hx_ED_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
	}
#ifdef HX_ZERO_FLASH
		else if (hx_zero_event == length) {
		/*check zero flash status*/
		if (g_core_fp.fp_0f_esd_check() < 0) {
			g_zero_event_count = 6;
			I("[HIMAX TP MSG]: ESD event checked - ALL Zero in ZF.\n");
		} else {
			I("[HIMAX TP MSG]: Status check pass in ZF.\n");
		}
	}
#endif
	else {
		hx_esd_event = 0;
	}
//Huaqin add for ESD test bu xudongfang at 2018/11/05 start
	if(hx_esd_event == length || hx_zero_event == length)
		private_ts->is_esd_event = true;
//Huaqin add for ESD test bu xudongfang at 2018/11/05 end
	if ((hx_esd_event == length || hx_zero_event == length)
		&& (HX_HW_RESET_ACTIVATE == 0)
		&& (HX_ESD_RESET_ACTIVATE == 0)
		&& (hx_touch_data->diag_cmd == 0)
		&& (ts->in_self_test == 0)) {
		shaking_ret = g_core_fp.fp_ic_esd_recovery(hx_esd_event, hx_zero_event, length);

		if (shaking_ret == HX_ESD_EVENT) {
			himax_esd_hw_reset();
			ret_val = HX_ESD_EVENT;
		} else if (shaking_ret == HX_ZERO_EVENT_COUNT) {
			ret_val = HX_ZERO_EVENT_COUNT;
		} else {
			I("I2C running. Nothing to be done!\n");
			ret_val = HX_IC_RUNNING;
		}
	} else if (HX_ESD_RESET_ACTIVATE) { /* drop 1st interrupts after chip reset */
		HX_ESD_RESET_ACTIVATE = 0;
		I("[HX_ESD_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ret_val = HX_ESD_REC_OK;
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d! \n", __func__, ret_val);

	return ret_val;
}
#endif

static int himax_err_ctrl(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
#ifdef HX_RST_PIN_FUNC
	if (HX_HW_RESET_ACTIVATE) {
		/* drop 1st interrupts after chip reset */
		HX_HW_RESET_ACTIVATE = 0;
		I("[HX_HW_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ts_status = HX_RST_OK;
		goto END_FUNCTION;
	}
#endif

	ts_status = himax_checksum_cal(ts, buf, ts_path, ts_status);
//Huaqin add for ESD test bu xudongfang at 2018/11/05 start
	if (ts_status == HX_CHKSUM_FAIL){
		goto CHK_FAIL;
	} else {
#ifdef HX_ESD_RECOVERY
		/* continuous N times record, not total N times. */
		g_zero_event_count = 0;
#endif
		goto END_FUNCTION;
	}
//Huaqin add for ESD test bu xudongfang at 2018/11/05 end

CHK_FAIL:
#ifdef HX_ESD_RECOVERY
	ts_status = himax_ts_event_check(ts, buf, ts_path, ts_status);
#endif


END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(uint8_t *buf, int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}

		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
		|| (HX_ESD_RESET_ACTIVATE)
#endif
		) {
			memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}

		memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
#if defined(HX_SMART_WAKEUP)
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		memcpy(hx_touch_data->hx_event_buf, buf, hx_touch_data->event_size);
#endif
	} else {
		E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: End, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/

/* start parse_report_data*/
int himax_parse_report_points(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	int x = 0;
	int y = 0;
	int w = 0;
	int base = 0;
	int32_t	loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start! \n", __func__);


	ts->old_finger = ts->pre_finger_mask;
	// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30  start
		if (ts->hx_point_num == 0){
		//I("%s: hx_point_num = 0! \n", __func__);
		return ts_status;
	}
	// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30  end
	ts->pre_finger_mask = 0;
	hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
	hx_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = hx_touch_data->finger_num;
	g_target_report_data->finger_on = hx_touch_data->finger_on;

	if (g_ts_dbg != 0)
		I("%s:finger_num = 0x%2X, finger_on = %d \n", __func__, g_target_report_data->finger_num, g_target_report_data->finger_on);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
		y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
		w = hx_touch_data->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__, loop_i, x, y, w);

		if (x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 && y <= ts->pdata->abs_y_max) {
			hx_touch_data->finger_num--;

			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 1;

			/* I("%s: g_target_report_data->x[loop_i]=%d, g_target_report_data->y[loop_i]=%d, g_target_report_data->w[loop_i]=%d", */
			/* 	__func__, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]); */


			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				I("S1@%d, %d\n", x, y);
			}

			ts->pre_finger_data[loop_i][0] = x;
			ts->pre_finger_data[loop_i][1] = y;

			ts->pre_finger_mask = ts->pre_finger_mask + (1 << loop_i);
		} else {/* report coordinates */
			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			D("DBG X=%d  Y=%d ID=%d\n", g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->finger_id[loop_i]);
		}
		D("DBG finger number %d\n", g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		I("%s: end! \n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{

	if (g_ts_dbg != 0)
		I("%s: start now_status=%d! \n", __func__, ts_status);


	EN_NoiseFilter = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
	/* I("EN_NoiseFilter=%d\n", EN_NoiseFilter); */
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	/* I("EN_NoiseFilter2=%d\n", EN_NoiseFilter); */
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	tpd_key = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 4);

	/* All (VK+AA)leave */
	if (tpd_key == 0x0F) {
		tpd_key = 0x00;
	}
#endif
	p_point_num = ts->hx_point_num;
	// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
	if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
		ts->hx_point_num = 0;
	} else {
		ts->hx_point_num = hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
	}
	// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		if (debug_data != NULL) {
			if (debug_data->fp_set_diag_cmd(ic_data, hx_touch_data))
				I("%s: coordinate dump fail and bypass with checksum err\n", __func__);
		} else {
			E("%s,There is no init set_diag_cmd\n", __func__);
		}
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
#ifdef HX_SMART_WAKEUP
	case HX_REPORT_SMWP_EVENT:
		himax_wake_event_parse(ts, ts_status);
		break;
#endif
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		I("%s: end now_status=%d! \n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

static void himax_report_all_leave_event(struct himax_ts_data *ts)
{
	int loop_i = 0;

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
#ifndef	HX_PROTOCOL_A
		input_mt_slot(ts->input_dev, loop_i);
		// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

/* start report_data */
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
static void himax_key_report_operation(int tp_key_index, struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering \n", __func__);

	if (tp_key_index != 0x00) {
		I("virtual key index =%x\n", tp_key_index);

		if (tp_key_index == 0x01) {
			vk_press = 1;
			I("back key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[0].index) {
					x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
					y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
				}

#ifdef	HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			}	else {
				input_report_key(ts->input_dev, KEY_BACK, 1);
			}
		} else if (tp_key_index == 0x02) {
			vk_press = 1;
			I("home key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[1].index) {
					x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
					y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
				}

#ifdef	HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			} else {
				input_report_key(ts->input_dev, KEY_HOME, 1);
			}
		} else if (tp_key_index == 0x04) {
			vk_press = 1;
			I("APP_switch key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[2].index) {
					x_position = (ts->button[2].x_range_min + ts->button[2].x_range_max) / 2;
					y_position = (ts->button[2].y_range_min + ts->button[2].y_range_max) / 2;
				}

#ifdef HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			} else {
				input_report_key(ts->input_dev, KEY_APPSELECT, 1);
			}
		}
		input_sync(ts->input_dev);
	} else { /*tp_key_index =0x00*/
		I("virtual key released\n");
		vk_press = 0;
#ifndef	HX_PROTOCOL_A
		input_mt_slot(ts->input_dev, 0);
		// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#else
		input_mt_sync(ts->input_dev);
#endif
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		input_report_key(ts->input_dev, KEY_APPSELECT, 0);
#ifndef	HX_PROTOCOL_A
		input_sync(ts->input_dev);
#endif
	}
}

void himax_finger_report_key(struct himax_ts_data *ts)
{
	if (ts->hx_point_num != 0) {
		/*Touch KEY*/
		if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
		 /* temp_x[0] = 0xFFFF;
			temp_y[0] = 0xFFFF;
			temp_x[1] = 0xFFFF;
			temp_y[1] = 0xFFFF; */
			hx_touch_data->finger_on = 0;
#ifdef HX_PROTOCOL_A
			input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
			himax_key_report_operation(tpd_key, ts);
		}

#ifndef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
		input_sync(ts->input_dev);
	}
}

void himax_finger_leave_key(struct himax_ts_data *ts)
{
	if (tpd_key != 0x00) {
		hx_touch_data->finger_on = 1;
#ifdef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
		himax_key_report_operation(tpd_key, ts);
	} else if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
		hx_touch_data->finger_on = 0;
#ifdef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
		himax_key_report_operation(tpd_key, ts);
	}

#ifndef HX_PROTOCOL_A
	input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
	input_sync(ts->input_dev);
}

static void himax_report_key(struct himax_ts_data *ts)
{
	if (ts->hx_point_num != 0) { /* Touch KEY */
		himax_finger_report_key(ts);
	} else { /* Key */
		himax_finger_leave_key(ts);
	}

	tpd_key_old = tpd_key;
	Last_EN_NoiseFilter = EN_NoiseFilter;
}
#endif

/* start report_point*/
static void himax_finger_report(struct himax_ts_data *ts)
{
	int i = 0;
	bool valid = false;
	if (g_ts_dbg != 0) {
		I("%s:start\n", __func__);
		I("hx_touch_data->finger_num=%d\n", hx_touch_data->finger_num);
	}
	for (i = 0; i < ts->nFinger_support; i++) {
		if (g_target_report_data->x[i] >= 0 && g_target_report_data->x[i] <= ts->pdata->abs_x_max && g_target_report_data->y[i] >= 0 && g_target_report_data->y[i] <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			I("valid=%d\n", valid);
		if (valid) {
			if (g_ts_dbg != 0)
				I("g_target_report_data->x[i]=%d, g_target_report_data->y[i]=%d, g_target_report_data->w[i]=%d\n", g_target_report_data->x[i], g_target_report_data->y[i], g_target_report_data->w[i]);
#ifndef	HX_PROTOCOL_A
			input_mt_slot(ts->input_dev, i);
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, g_target_report_data->finger_on);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_target_report_data->w[i]);
			// Huaqin del for HX_PROTOCOL_B by limengxia at 2018/10/30 start
			//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			// Huaqin del for HX_PROTOCOL_B by limengxia at 2018/10/30 end
#ifndef	HX_PROTOCOL_A
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_target_report_data->w[i]);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_target_report_data->w[i]);
// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
#else
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end

#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_target_report_data->x[i]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_target_report_data->y[i]);
#ifndef	HX_PROTOCOL_A
			ts->last_slot = i;
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#else
			input_mt_sync(ts->input_dev);
#endif

		} else {
			input_mt_slot(ts->input_dev, i);
			// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, g_target_report_data->finger_on);
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}

static void himax_finger_leave(struct himax_ts_data *ts)
{
	int32_t loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start! \n", __func__);
#if defined(HX_PALM_REPORT)
	if (himax_palm_detect(hx_touch_data->hx_coord_buf) == PALM_REPORT) {
		I(" %s HX_PALM_REPORT KEY power event press\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 1);
		input_sync(ts->input_dev);
		msleep(100);

		I(" %s HX_PALM_REPORT KEY power event release\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 0);
		input_sync(ts->input_dev);
		return;
	}
#endif

	hx_touch_data->finger_on = 0;
	AA_press = 0;

#ifdef HX_PROTOCOL_A
	input_mt_sync(ts->input_dev);
#endif

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		// Huaqin modify for HX_PROTOCOL_B by limengxia at 2018/10/30 start
		if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
		// Huaqin modify for HX_PROTOCOL_B by limengxia at 2018/10/30 end
			input_mt_slot(ts->input_dev, loop_i);
			// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 start
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			// Huaqin add for HX_PROTOCOL_B by limengxia at 2018/10/30 end
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (ts->pre_finger_mask > 0) {
		ts->pre_finger_mask = 0;
	}

	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
	}

	/* if (ts->debug_log_level & BIT(1)) */
	/* 	himax_log_touch_event(x, y, w, loop_i, EN_NoiseFilter, HX_FINGER_LEAVE); */

	input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s: end! \n", __func__);
	return;
}

static void himax_report_points(struct himax_ts_data *ts)
{
	if (g_ts_dbg != 0)
		I("%s: start! \n", __func__);

	if (ts->hx_point_num != 0) {
		himax_finger_report(ts);
	} else {
		himax_finger_leave(ts);
	}

	Last_EN_NoiseFilter = EN_NoiseFilter;

	if (g_ts_dbg != 0)
		I("%s: end! \n", __func__);
}
/* end report_points*/

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		// Huaqin del for HX_PROTOCOL_B by limengxia at 2018/10/30 start
		/*if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
			ts->hx_point_num = 0;
		} else {
			ts->hx_point_num = hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
		}*/
		// Huaqin del for HX_PROTOCOL_B by limengxia at 2018/10/30 end

		/* Touch Point information */
		himax_report_points(ts);

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
		/*report key(question mark)*/
		if (tpd_key && tpd_key_old) {
			himax_report_key(ts);
		}
#endif
#ifdef HX_SMART_WAKEUP
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
	/* Huaqin add for ZQL1820-861 by zhangxiude at 2018/10/16 start */
		//wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
	/* Huaqin add for ZQL1820-861 by zhangxiude at 2018/10/16 end */
		himax_wake_event_report();
#endif
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int himax_ts_operation(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	uint8_t hw_reset_check[2];
	uint8_t buf[128];

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	ts_status = himax_touch_get(ts, buf, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status = himax_err_ctrl(ts, buf, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END) {
		ts_status = himax_distribute_touch_data(buf, ts_path, ts_status);
		ts_status = himax_parse_report_data(ts, ts_path, ts_status);
	} else {
		goto END_FUNCTION;
	}
	ts_status = himax_report_data(ts, ts_path, ts_status);


END_FUNCTION:
	return ts_status;
}

void himax_ts_work(struct himax_ts_data *ts)
{

	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;
	// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  start
	int falg_Psenser = 0;
	// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  end

	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_ON);

#if defined(HX_USB_DETECT_GLOBAL)
	himax_cable_detect_func(false);
#endif

	// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  start
		falg_Psenser = tp_status_fun();
		if (falg_Psenser) {
			goto END_FUNCTION;
		}
	// Huaqin add for ZQL1820-952 by limengxia at 2018/10/18  end
	ts_path = himax_ts_work_status(ts);
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_SMWP_EVENT:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	default:
		E("%s:Path Fault! value=%d\n", __func__, ts_path);
		goto END_FUNCTION;
		break;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	I("%s: Now reset the Touch chip.\n", __func__);
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, true);
#endif
END_FUNCTION:
	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_LEAVE);
	/* Huaqin add for ZQL1820-861 by zhangxiude at 2018/10/16 start */
	enable_irq(private_ts->client->irq);// wdd modify for irq timeout
	/* Huaqin add for ZQL1820-861 by zhangxiude at 2018/10/16 end */
	return;
}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;
	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#if defined(HX_USB_DETECT_CALLBACK)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	I("Touch: cable change to %d\n", connect_status);
	ts = private_ts;

	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if (connect_status != ts->usb_connected) {
				if (connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				himax_bus_master_write(ts->cable_config,
										sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);
				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else {
					I("%s: Cable status is the same as previous one, ignore.\n", __func__);
			}
		} else {
			if (connect_status) {
				ts->usb_connected = 0x01;
			} else {
				ts->usb_connected = 0x00;
			}

			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

#endif

#ifdef HX_AUTO_UPDATE_FW
static void himax_update_register(struct work_struct *work)
{
	I(" %s in\n", __func__);

	if (i_get_FW() != 0)
		return ;

	if (g_auto_update_flag == true) {
		I("Update FW Directly");
		goto UPDATE_FW;
	}

	if (himax_auto_update_check() != 0) {
		return ;
	}

UPDATE_FW:
	if (i_update_FW() <= 0) {
		I("Auto update FW fail!\n");
	}	else {
		I("It have Updated\n");
	}
}
#endif


#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work_att.work);
	I(" %s in\n", __func__);
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);

	if (ret) {
		E(" Unable to register fb_notifier: %d\n", ret);
	}
}
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
void himax_platform_shutdown(void)
{
	printk("HQ add for himax tp shut down\n");
	nvt_lcm_power_source_ctrl(private_ts, 0);
}
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */

int himax_chip_common_init(void)
{

	int i = 0, ret = 0, err = 0;
	struct himax_ts_data *ts = private_ts;
	struct himax_i2c_platform_data *pdata;
//Huaqin add for ESD test bu xudongfang at 2018/11/05 start
	private_ts->is_esd_event = true;
//Huaqin add for ESD test bu xudongfang at 2018/11/05 end
	I("PDATA START\n");
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

	I("ic_data START\n");
	ic_data = kzalloc(sizeof(*ic_data), GFP_KERNEL);
	if (ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto err_dt_ic_data_fail;
	}

	/* allocate report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_touch_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

	if (himax_parse_dt(ts, pdata) < 0) {
		I(" pdata is NULL for DT\n");
		goto err_alloc_dt_pdata_failed;
	}

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif
/* Huaqin modify for power time long by zhangxiude at 2018/10/16 start*/
	ret = himax_gpio_power_config(pdata);
	if ( ret != 0){
		E("%s:gpio config failed\n",__func__);
		goto err_gpio_config_failed;
	}
/* Huaqin modify for power time long by zhangxiude at 2018/10/16 end*/
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#if NVT_POWER_SOURCE_CUST_EN
		atomic_set(&(ts->lcm_lab_power), 0);
		atomic_set(&(ts->lcm_ibb_power), 0);
		ret = nvt_lcm_bias_power_init(ts);
		if (ret) {
			E("power resource init error!\n");
			goto err_power_resource_init_fail;
		}
		nvt_lcm_power_source_ctrl(private_ts, 1);
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */

#ifndef CONFIG_OF

	if (pdata->power) {
		ret = pdata->power(1);

		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}

#endif
	
	for (i = 0; i < g_hx_ic_dt_num; i++) {
	if (g_core_chip_dt[i].fp_chip_detect != NULL) {
			if (g_core_chip_dt[i].fp_chip_detect() == true) {
				I("%s: chip detect found! list_num=%d \n", __func__, i);
				goto found_hx_chip;
			}
			else {
				I("%s: num=%d chip detect NOT found! goto Next \n", __func__, i);
				continue;
			}
		}
	}

	if(i == g_hx_ic_dt_num) {
			E("%s: chip detect failed!\n", __func__);
			goto error_ic_detect_failed;
		}
found_hx_chip:	
	if (g_core_fp.fp_chip_init != NULL) {
		g_core_fp.fp_chip_init();
	} else {
		E("%s: function point of chip_init is NULL!\n", __func__);
		goto error_ic_detect_failed;
	}


	if (pdata->virtual_key) {
		ts->button = pdata->virtual_key;
	}

#ifdef HX_AUTO_UPDATE_FW
	g_auto_update_flag = (!g_core_fp.fp_calculateChecksum(false));
	g_auto_update_flag |= g_core_fp.fp_flash_lastdata_check();
	if (g_auto_update_flag)
		goto FW_force_upgrade;
#endif
#ifndef HX_ZERO_FLASH
	g_core_fp.fp_read_FW_ver();
#endif

#ifdef HX_AUTO_UPDATE_FW
FW_force_upgrade:
	ts->himax_update_wq = create_singlethread_workqueue("HMX_update_reuqest");
	if (!ts->himax_update_wq) {
		E(" allocate syn_update_wq failed\n");
		err = -ENOMEM;
		goto err_update_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->work_update, himax_update_register);
	queue_delayed_work(ts->himax_update_wq, &ts->work_update, msecs_to_jiffies(2000));
#endif
#ifdef HX_ZERO_FLASH
	g_auto_update_flag = true;
	ts->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
	INIT_DELAYED_WORK(&ts->work_0f_update, g_core_fp.fp_0f_operation);
	queue_delayed_work(ts->himax_0f_update_wq, &ts->work_0f_update, msecs_to_jiffies(2000));
#endif

	/*Himax Power On and Load Config*/
	if (himax_loadSensorConfig(pdata)) {
		E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
		goto err_detect_failed;
	}

	g_core_fp.fp_power_on_init();
	calculate_point_number();

#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;
	/*calculate the i2c data size*/
	calcDataSize();
	I("%s: calcDataSize complete\n", __func__);
#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif
	ts->suspended                      = false;
#if defined(HX_USB_DETECT_CALLBACK) || defined(HX_USB_DETECT_GLOBAL)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif
#ifdef	HX_PROTOCOL_A
	ts->protocol_type = PROTOCOL_TYPE_A;
#else
	ts->protocol_type = PROTOCOL_TYPE_B;
#endif
	I("%s: Use Protocol Type %c\n", __func__,
	  ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');
	ret = himax_input_register(ts);

	if (ret) {
		E("%s: Unable to register %s input device\n",
		  __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_FB
	ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");

	if (!ts->himax_att_wq) {
		E(" allocate syn_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}

	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#endif
	/* Huaqin add by zhangxiude for ITO test start */
	//--------add ito node
	platform_device_register(&hwinfo_device);
	himax_test_node_init(&hwinfo_device);
	firmware_id=ic_data->vendor_config_ver;
	himax_tp_info_proc_init();
	/* Huaqin add by zhangxiude for ITO test end */
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#ifdef HX_SMART_WAKEUP
	nvt_gesture_mode_proc = proc_create(NVT_GESTURE_MODE, 0666, NULL,
				&gesture_mode_proc_ops);
	if (!nvt_gesture_mode_proc) {
		E("create proc tpd_gesture failed\n");
	}
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable = 0;
	wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX_common_NAME);
#endif
#ifdef HX_HIGH_SENSE
	ts->HSEN_enable = 0;
#endif

	/*touch data init*/
	err = himax_report_data_init();

	if (err) {
		goto err_report_data_init_failed;
	}

	if (himax_common_proc_init()) {
		E(" %s: himax_common proc_init failed!\n", __func__);
		goto err_creat_proc_file_failed;
	}

#if defined(HX_USB_DETECT_CALLBACK)

	if (ts->cable_config)	{
		cable_detect_register_notifier(&himax_cable_status_handler);
	}

#endif
	err = himax_ts_register_interrupt();

	if (err) {
		goto err_register_interrupt_failed;
	}

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
	if (himax_debug_init())
		E(" %s: debug initial failed!\n", __func__);
#endif

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)

	if (g_auto_update_flag) {
		himax_int_enable(0);
	}

#endif
	return 0;
err_register_interrupt_failed:
	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
err_creat_proc_file_failed:
err_report_data_init_failed:
#ifdef HX_SMART_WAKEUP
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef CONFIG_FB
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
err_get_intr_bit_failed:
#endif
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_detect_failed:
#ifdef HX_AUTO_UPDATE_FW
	if (g_auto_update_flag) {
		cancel_delayed_work_sync(&ts->work_update);
		destroy_workqueue(ts->himax_update_wq);
	}
err_update_wq_failed:
#endif
error_ic_detect_failed:
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
		nvt_lcm_power_source_ctrl(private_ts, 0);
		nvt_lcm_bias_power_deinit(ts);
err_power_resource_init_fail:
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */


	if (gpio_is_valid(pdata->gpio_irq)) {
		gpio_free(pdata->gpio_irq);
	}
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(pdata->gpio_reset))	{
		gpio_free(pdata->gpio_reset);
	}
#endif
#ifndef CONFIG_OF
err_power_failed:
#endif
/* Huaqin modify for power time long by zhangxiude at 2018/10/16 start*/
err_gpio_config_failed:
/* Huaqin modify for power time long by zhangxiude at 2018/10/16 end*/
err_alloc_dt_pdata_failed:
	kfree(hx_touch_data);
err_alloc_touch_data_failed:
	kfree(ic_data);
err_dt_ic_data_fail:
	kfree(pdata);
err_dt_platform_data_fail:
	kfree(ts);
	probe_fail_flag = 1;
	return err;
}

void himax_chip_common_deinit(void)
{
	struct himax_ts_data *ts = private_ts;

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
	himax_debug_remove();
#endif

	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
	himax_common_proc_deinit();

	if (!ts->use_irq) {
		hrtimer_cancel(&ts->timer);
		destroy_workqueue(ts->himax_wq);
	}

#ifdef HX_SMART_WAKEUP
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		E("Error occurred while unregistering fb_notifier.\n");
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
#endif
	input_free_device(ts->input_dev);
#ifdef HX_ZERO_FLASH
	cancel_delayed_work_sync(&ts->work_0f_update);
	destroy_workqueue(ts->himax_0f_update_wq);
#endif
#ifdef HX_AUTO_UPDATE_FW
	cancel_delayed_work_sync(&ts->work_update);
	destroy_workqueue(ts->himax_update_wq);
#endif
	if (gpio_is_valid(ts->pdata->gpio_irq))
		gpio_free(ts->pdata->gpio_irq);
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(ts->pdata->gpio_reset))
		gpio_free(ts->pdata->gpio_reset);
#endif
	//Huaqin add for VSN/VSP by zhangxiude at 2018/9/17 start
		nvt_lcm_bias_power_deinit(ts);
	//Huaqin add for VSN/VSP by zhangxiude at 2018/9/17 end


	kfree(hx_touch_data);
	kfree(ic_data);
	kfree(ts->pdata);
	kfree(ts);
	probe_fail_flag = 0;

	return;
}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	int ret;

	if (ts->suspended) {
		I("%s: Already suspended. Skipped. \n", __func__);
		return 0;
	} else {
		ts->suspended = true;
		I("%s: enter \n", __func__);
	}

	if (debug_data != NULL && debug_data->flash_dump_going == true) {
		I("[himax] %s: Flash dump is going, reject suspend\n", __func__);
		return 0;
	}

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) || defined(HX_USB_DETECT_GLOBAL)
#ifndef HX_RESUME_SEND_CMD
	g_core_fp.fp_resend_cmd_func(ts->suspended);
#endif
#endif
#ifdef HX_SMART_WAKEUP
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
	if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) {
	}else{
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND = false;
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n", __func__);
		return 0;
	}
#endif
	himax_int_enable(0);
	g_core_fp.fp_suspend_ic_action();

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);

		if (ret) {
			himax_int_enable(1);
		}
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	if (ts->pdata->powerOff3V3 && ts->pdata->power) {
		ts->pdata->power(0);
	}
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
#if NVT_POWER_SOURCE_CUST_EN
		if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) {
		nvt_lcm_power_source_ctrl(private_ts, 0);//disable vsp/vsn
		I("sleep suspend end	disable vsp/vsn\n");
		}
		else{
		I("gesture suspend end not disable vsp/vsn\n");
		}
#endif
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */


	I("%s: END \n", __func__);
	return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{
#if defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET) && defined(HX_ZERO_FLASH)
	int result = 0;
#endif
	I("%s: enter \n", __func__);
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
		nvt_lcm_power_source_ctrl(private_ts, 1);//enable vsp/vsn
	/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */

	if (ts->suspended == false) {
		I("%s: It had entered resume, skip this step \n", __func__);
		return 0;
	} else {
		ts->suspended = false;
	}
//Huaqin add for ESD test bu xudongfang at 2018/11/05 start
#ifdef HX_ESD_RECOVERY
		/* continuous N times record, not total N times. */
		g_zero_event_count = 0;
#endif
//Huaqin add for ESD test bu xudongfang at 2018/11/05 end
	atomic_set(&ts->suspend_mode, 0);

	if (ts->pdata->powerOff3V3 && ts->pdata->power) {
		ts->pdata->power(1);
	}

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) || defined(HX_USB_DETECT_GLOBAL)
	g_core_fp.fp_resend_cmd_func(ts->suspended);
#elif defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET)
	g_core_fp.fp_ic_reset(false, false);
#ifdef HX_ZERO_FLASH
	I("It will update fw after esd event in zero flash mode!\n");
	result = g_core_fp.fp_0f_operation_dirly();
	if (result) {
		E("Something is wrong! Skip Update with zero flash!\n");
		goto ESCAPE_0F_UPDATE;
	}
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_sense_on(0x00);
#endif
#endif
	himax_report_all_leave_event(ts);

	g_core_fp.fp_resume_ic_action();

/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 start */
if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) {
	himax_int_enable(1);
}
/* Huaqin add for ZQL1820-701 by zhangxiude at 2018/9/19 end */
#if defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET) && defined(HX_ZERO_FLASH)
ESCAPE_0F_UPDATE:
#endif
	I("%s: END \n", __func__);
	return 0;
}

