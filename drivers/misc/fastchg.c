/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 *
 * Ported by: engstk <eng.stk@sapo.pt>
 * Refactored by: Edrick Vince Sinsuan <sedrickvince@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Possible values for "force_fast_charge" are :
 *
 *   0 - Disabled (default)
 *   1 - Force faster charge
*/

#include <linux/module.h>

#include <linux/fastchg.h>

int force_fast_charge = 1;

static int __init get_fastcharge_opt(char *ffc)
{
	if (!strcmp(ffc, "1"))
		force_fast_charge = 1;
	else
		force_fast_charge = 0;

	return 1;
}

__setup("ffc=", get_fastcharge_opt);

static ssize_t force_fast_charge_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", force_fast_charge);
	return count;
}

static ssize_t force_fast_charge_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int temp;
	sscanf(buf, "%d ", &temp);
	force_fast_charge = (temp != 1) ? 0 : 1;
	return count;
}

static struct kobj_attribute force_fast_charge_attribute =
	__ATTR(force_fast_charge, 0664, force_fast_charge_show, force_fast_charge_store);

static struct attribute *force_fast_charge_attrs[] = {
	&force_fast_charge_attribute.attr,
	NULL
};

static struct attribute_group force_fast_charge_attr_group = {
	.attrs = force_fast_charge_attrs
};

/* Initialize fast charge sysfs folder */
static struct kobject *force_fast_charge_kobj;

static int force_fast_charge_init(void)
{
	int ret = 0;

	force_fast_charge_kobj = kobject_create_and_add("fast_charge", kernel_kobj);
	if (!force_fast_charge_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(force_fast_charge_kobj, &force_fast_charge_attr_group);
	if (ret)
		kobject_put(force_fast_charge_kobj);

	return ret;
}

static void force_fast_charge_exit(void)
{
	kobject_put(force_fast_charge_kobj);
}

module_init(force_fast_charge_init);
module_exit(force_fast_charge_exit);
