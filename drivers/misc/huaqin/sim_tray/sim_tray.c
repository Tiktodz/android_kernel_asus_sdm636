/*
 * file create by liunianliang for check sim tray status, 2018/04/20
 */

#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/printk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/seq_file.h>

#define SIM_TRAY_HOT_DETECH_GPIO 86

/* show wlan version begin */
static int sim_tray_status_show(struct seq_file *m, void *v)
{
	seq_printf(m, "STATUS=%d\n", gpio_get_value(SIM_TRAY_HOT_DETECH_GPIO));
	return 0;
}

static int sim_tray_file_open(struct inode *inode, struct file *file)
{
	return single_open(file, sim_tray_status_show, inode->i_private);
}

static const struct file_operations sim_tray_fops = {
	.open = sim_tray_file_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* show wlan version end */

static int __init sim_tray_init(void)
{
	struct proc_dir_entry *ret;

	ret = proc_mkdir("class", NULL);
	if(!ret) {
		pr_err("init proc/class failed!!!\n");
		return -ENOMEM;
	}

	ret = proc_mkdir("sd-hotplug", ret);
	if(!ret) {
		pr_err("init proc/class/sd-hotplug failed!!!\n");
		return -ENOMEM;
	}

	ret = proc_mkdir("status", ret);
	if(!ret) {
		pr_err("init proc/class/sd-hotplug/status failed!!!\n");
		return -ENOMEM;
	}

	ret = proc_create("hot-status", 0444, ret, &sim_tray_fops);

	return 0;
}

late_initcall(sim_tray_init);
