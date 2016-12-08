/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/pm_opp.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <soc/qcom/msm_cpu_voltage.h>

#define MAX_FREQ_GAP_KHZ 200000

static unsigned int cls0_max_freq;

static unsigned int get_max_freq(struct device *dev)
{
	struct dev_pm_opp *opp = NULL;
	unsigned long freq = ULONG_MAX;

	if (!dev)
		return 0;

	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return 0;

	freq /= 1000;

	return freq;
}

/**
  * msm_match_cpu_voltage_btol(): match voltage from a big to little cpu
  *
  * @input_freq - big cluster frequency (in kHz)
  *
  * Returns voltage equivalent little cluster frequency (in kHz)
  * or 0 on any error.
  *
  * Function to get a voltage equivalent little cluster frequency
  * for a particular big cluster frequency. Note that input_freq will
  * be rounded up to nearest big cluster frequency. Returned frequency
  * has some restrictions (e.g. can't be more than MAX_FREQ_GAP_KHZ
  * kHz less than input_freq).
  */
unsigned int msm_match_cpu_voltage_btol(unsigned int input_freq)
{
	struct dev_pm_opp *opp = NULL;
	struct device *cls0_dev = NULL;
	struct device *cls1_dev = NULL;
	unsigned long freq = 0;
	unsigned long input_volt = 0;
	unsigned int input_freq_actual = 0;
	unsigned int target_freq = 0;
	unsigned int temp_freq = 0;
	unsigned long temp_volt = 0;
	int i, max_opps_cls0 = 0;

	cls0_dev = get_cpu_device(0);
	if (!cls0_dev) {
		pr_err("Error getting CPU0 device\n");
		return 0;
	}
	cls1_dev = get_cpu_device(2);
	if (!cls1_dev) {
		pr_err("Error getting CPU2 device\n");
		return 0;
	}

	if (cls0_max_freq == 0)
		cls0_max_freq = get_max_freq(cls0_dev);

	/*
	 * Parse Cluster 1 OPP table for closest freq to input_freq
	 * and save its voltage
	 */
	freq = input_freq * 1000;
	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cls1_dev, &freq);
	if (IS_ERR(opp)) {
		pr_err("Error: could not find freq close to input_freq: %u\n",
				input_freq);
		goto exit;
	}
	input_freq_actual = freq / 1000;
	input_volt = dev_pm_opp_get_voltage(opp);
	if (input_volt == 0) {
		pr_err("Error getting OPP voltage on cluster 1\n");
		goto exit;
	}

	/*
	 * Parse Cluster 0 OPP table for closest voltage to input_volt
	 * and iterate to end to save max freq of Cluster 0
	 */
	max_opps_cls0 = dev_pm_opp_get_opp_count(cls0_dev);
	if (max_opps_cls0 <= 0)
		pr_err("Error getting OPP count for CPU0\n");
	for (i = 0, freq = 0; i < max_opps_cls0; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(cls0_dev, &freq);
		if (IS_ERR(opp)) {
			pr_err("Error getting OPP freq on cluster 0\n");
			goto exit;
		}
		temp_freq = freq / 1000;
		temp_volt = dev_pm_opp_get_voltage(opp);
		if (temp_volt >= input_volt) {
			/* Continue to next frequency if gap is too large */
			if ((temp_freq + MAX_FREQ_GAP_KHZ) < input_freq_actual)
				continue;
			/* block freq higher than input_freq */
			if (temp_freq > input_freq)
				target_freq = input_freq;
			else
				target_freq = temp_freq;
			break;
		}
	}

	/*
	 * If we didn't find a frequency, either input voltage is too high
	 * or gap was too large (input frequency was too high). In either
	 * case, return Cluster 0 Fmax as this is the best possible freq.
	 * Also, if input_freq is > Cluster 0 Fmax, return Cluster 0 Fmax.
	 */
	if (target_freq == 0 || input_freq > cls0_max_freq)
		target_freq = cls0_max_freq;
exit:
	rcu_read_unlock();
	return target_freq;
}
EXPORT_SYMBOL(msm_match_cpu_voltage_btol);

/**
 * get_cpu_available_volts - show frequency voltages for the specified CPU
 */
static int get_cpu_available_volts(int cpu, char *buf)
{
	int cnt = 0;
	struct dev_pm_opp *opp = NULL;
	struct device *cls0_dev = NULL;
	unsigned long freq = 0;
	unsigned long temp_volt = 0;
	int i, max_opps_cls0 = 0;

	cls0_dev = get_cpu_device(cpu);
	if (!cls0_dev) {
		pr_err("Error getting CPU[%d] device\n", cpu);
		return 0;
	}
	rcu_read_lock();
	max_opps_cls0 = dev_pm_opp_get_opp_count(cls0_dev);
	if (max_opps_cls0 <= 0)
		pr_err("Error getting OPP count for CPU[%d]\n", cpu);
	for (i = 0, freq = 0; i < max_opps_cls0; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(cls0_dev, &freq);
		if (IS_ERR(opp)) {
			pr_err("Error getting OPP freq on cluster [%d]\n", cpu);
			goto exit;
		}
		temp_volt = dev_pm_opp_get_voltage(opp);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%lu ", temp_volt);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");

exit:
	rcu_read_unlock();
	return cnt;
}

static int get_cpu0_available_volts(char *buf, const struct kernel_param *kp)
{
	return get_cpu_available_volts(0, buf);
}

static const struct kernel_param_ops param_ops_available_volts_cpu0 = {
	.get = get_cpu0_available_volts,
};
module_param_cb(cpu0_available_volts, &param_ops_available_volts_cpu0,
							NULL, 0444);

static int get_cpu1_available_volts(char *buf, const struct kernel_param *kp)
{
	return get_cpu_available_volts(1, buf);
}

static const struct kernel_param_ops param_ops_available_volts_cpu1 = {
	.get = get_cpu1_available_volts,
};
module_param_cb(cpu1_available_volts, &param_ops_available_volts_cpu1,
							NULL, 0444);

static int get_cpu2_available_volts(char *buf, const struct kernel_param *kp)
{
	return get_cpu_available_volts(2, buf);
}

static const struct kernel_param_ops param_ops_available_volts_cpu2 = {
	.get = get_cpu2_available_volts,
};
module_param_cb(cpu2_available_volts, &param_ops_available_volts_cpu2,
							NULL, 0444);

static int get_cpu3_available_volts(char *buf, const struct kernel_param *kp)
{
	return get_cpu_available_volts(3, buf);
}

static const struct kernel_param_ops param_ops_available_volts_cpu3 = {
	.get = get_cpu3_available_volts,
};
module_param_cb(cpu3_available_volts, &param_ops_available_volts_cpu3,
							NULL, 0444);
