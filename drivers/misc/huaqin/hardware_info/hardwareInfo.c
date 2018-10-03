/*
 * file create by liunianliang for show hardware info, 2018/02/09
 */

#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/seq_file.h>

/* dir under proc */
#define HARDWARE_ROOT_DIR "hardwareinfo"

/* for BT/WLAN/FM/GPS chipset model */
#define FOUR_IN_ONE_DIR "wcnss"
#define FOUR_IN_ONE_MODEL "WCN3980"
#define GPS_BASEBAND_VERSION "MPSS.AT.3.1-00484-SDM660_GEN_PACK-1"
#define BTFM_VERSION "BTFM.CHE.2.1.1.c2-00099-QCACHROMZ-1"
#define WLAN_VERSION "WLAN.HL.1.0.1.c2-00308-QCAHLSWMTPLZ-1"
static char wcnsses[][5] = {"gps", "bt", "wifi", "fm"};
static char wcnss_type[][8] = {"model", "version"};

/* show gps/bt/fm/wlan model begin */
static int wcnss_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", FOUR_IN_ONE_MODEL);
	return 0;
}

static int wcnss_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcnss_show, inode->i_private);
}

static const struct file_operations wcnss_fops = {
	.open = wcnss_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* show gps/bt/fm/wlan model end  */

/* show gps version begin */
static int wcnss_gps_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", GPS_BASEBAND_VERSION);
	return 0;
}

static int wcnss_gps_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcnss_gps_show, inode->i_private);
}

static const struct file_operations wcnss_gps_fops = {
	.open = wcnss_gps_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* show gps version end */

/* show btfm version begin */
static int wcnss_bt_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", BTFM_VERSION);
	return 0;
}

static int wcnss_bt_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcnss_bt_show, inode->i_private);
}

static const struct file_operations wcnss_bt_fops = {
	.open = wcnss_bt_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wcnss_fm_fops = {
	.open = wcnss_bt_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* show btfm version end */

/* show wlan version begin */
static int wcnss_wifi_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", WLAN_VERSION);
	return 0;
}

static int wcnss_wifi_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcnss_wifi_show, inode->i_private);
}

static const struct file_operations wcnss_wifi_fops = {
	.open = wcnss_wifi_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* show wlan version end */

static int __init hardware_info_init(void)
{
	struct proc_dir_entry *ret;
	struct proc_dir_entry *tmp, *dst;
	int i;

	ret = proc_mkdir(HARDWARE_ROOT_DIR, NULL);
	if(!ret) {
		pr_err("init proc/hardwareinfo failed!!!\n");
		return -ENOMEM;
	}

	ret = proc_mkdir(FOUR_IN_ONE_DIR, ret);
	if(!ret) {
		pr_err("init proc/hardwareinfo/wcnss failed!!!\n");
		return -ENOMEM;
	}

	for (i=0; i<4; i++) {
		tmp = proc_mkdir(wcnsses[i], ret);
		if (tmp) {
			dst = proc_create(wcnss_type[0], 0444, tmp, &wcnss_fops);
			if (!dst) pr_err("create %s failed\n", wcnss_type[0]);
			switch(i) {
				case 0:
					dst = proc_create(wcnss_type[1], 0444, tmp, &wcnss_gps_fops);
					break;
                                case 1:
                                	dst = proc_create(wcnss_type[1], 0444, tmp, &wcnss_bt_fops);
                                	break;
                                case 2:
                                	dst = proc_create(wcnss_type[1], 0444, tmp, &wcnss_wifi_fops);
                                	break;
                                case 3:
                                	dst = proc_create(wcnss_type[1], 0444, tmp, &wcnss_fm_fops);
                                	break;
			}
			if (!dst) {
				pr_err("create %s failed\n", wcnss_type[1]);
			}
		} else {
			pr_err("create %s failed !!!\n", wcnsses[i]);
		}
	}

	return 0;
}

device_initcall(hardware_info_init);
