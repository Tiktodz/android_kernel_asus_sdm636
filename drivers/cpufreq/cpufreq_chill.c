/*
 *  drivers/cpufreq/cpufreq_chill.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *            (C)  2016 Joe Maples <joe@frap129.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include "cpufreq_governor.h"

/* Chill version macros */
#define CHILL_VERSION_MAJOR			(2)
#define CHILL_VERSION_MINOR			(9)

/* Chill governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD		        (85)
#define DEF_FREQUENCY_DOWN_THRESHOLD		    (35)
#define DEF_FREQUENCY_DOWN_THRESHOLD_SUSPENDED	(45)
#define DEF_FREQUENCY_STEP			            (5)
#define DEF_SAMPLING_RATE			            (20000)
#define DEF_BOOST_ENABLED			            (1)
#define DEF_BOOST_COUNT				            (8)

static DEFINE_PER_CPU(struct cs_cpu_dbs_info_s, cs_cpu_dbs_info);

static int cs_cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event);

unsigned int boost_counter = 0;

static inline unsigned int get_freq_target(struct cs_dbs_tuners *cs_tuners,
					   struct cpufreq_policy *policy)
{
	unsigned int freq_target = (cs_tuners->freq_step * policy->max) / 100;

	/* max freq cannot be less than 100. But who knows... */
	if (unlikely(freq_target == 0))
		freq_target = DEF_FREQUENCY_STEP;

	return freq_target;
}

/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Every sampling_rate,
 *  we check, if current idle time is more than 80%
 * (default), then we try to decrease frequency
 *
 * Any frequency increase takes it to the maximum frequency. Frequency reduction
 * happens at minimum steps of 5% (default) of maximum frequency
 */
static void cs_check_cpu(int cpu, unsigned int load)
{
	struct cs_cpu_dbs_info_s *dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.shared->policy;
	struct dbs_data *dbs_data = policy->governor_data;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;

	/* Once min frequency is reached while screen off, stop taking load samples*/
	if (policy->cur == policy->min)
		return;

	/*
	 * break out if we 'cannot' reduce the speed as the user might
	 * want freq_step to be zero
	 */
	if (cs_tuners->freq_step == 0)
		return;

	/* Check for frequency decrease */
	if (load < cs_tuners->down_threshold) {
		unsigned int freq_target;
		/*
		 * if we cannot reduce the frequency anymore, break out early
		 */
		if (policy->cur == policy->min)
			return;

		/* reduce boost count with frequency */
		if (boost_counter > 0)
			boost_counter--;

		freq_target = get_freq_target(cs_tuners, policy);
		if (dbs_info->requested_freq > freq_target)
			dbs_info->requested_freq -= freq_target;
		else {
			dbs_info->requested_freq = policy->min;
			boost_counter = 0;
		}
		__cpufreq_driver_target(policy, dbs_info->requested_freq,
				CPUFREQ_RELATION_L);
		return;
	} else if (load <= cs_tuners->down_threshold_suspended) {
		unsigned int freq_target;
		/*
		 * if we cannot reduce the frequency anymore, break out early
		 */
		if (policy->cur == policy->min)
			return;

		freq_target = get_freq_target(cs_tuners, policy);
		if (dbs_info->requested_freq > freq_target)
			dbs_info->requested_freq -= freq_target;
		else {
			dbs_info->requested_freq = policy->min;
			boost_counter = 0;
		}

		__cpufreq_driver_target(policy, dbs_info->requested_freq,
				CPUFREQ_RELATION_L);
		return;
	}

	/* Check for frequency increase */
	if (load > cs_tuners->up_threshold) {

		/* if we are already at full speed then break out early */
		if (dbs_info->requested_freq == policy->max)
			return;

		/* Boost if count is reached, otherwise increase freq */
		if (cs_tuners->boost_enabled && boost_counter >= cs_tuners->boost_count) {
			dbs_info->requested_freq = policy->max;
			boost_counter = 0;
		} else {
			dbs_info->requested_freq += get_freq_target(cs_tuners, policy);
			boost_counter++;
		};

		__cpufreq_driver_target(policy, dbs_info->requested_freq,
			CPUFREQ_RELATION_H);
		return;
	}
}

static unsigned int cs_dbs_timer(struct cpu_dbs_info *cdbs,
				 struct dbs_data *dbs_data, bool modify_all)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;

	if (modify_all)
		dbs_check_cpu(dbs_data, cdbs->shared->policy->cpu);

	return delay_for_sampling_rate(cs_tuners->sampling_rate);
}

static int dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cs_cpu_dbs_info_s *dbs_info =
					&per_cpu(cs_cpu_dbs_info, freq->cpu);
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(freq->cpu);

	if (!policy)
		return 0;

	/*
	 * we only care if our internally tracked freq moves outside the 'valid'
	 * ranges of frequency available to us otherwise we do not change it
	*/
	if (dbs_info->requested_freq > policy->max
			|| dbs_info->requested_freq < policy->min)
		dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block cs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier,
};

/************************** sysfs interface ************************/
static struct common_dbs_data cs_dbs_cdata;

static ssize_t store_sampling_rate(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	cs_tuners->sampling_rate = max(input, dbs_data->min_sampling_rate);
	return count;
}

static ssize_t store_up_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 || input <= cs_tuners->down_threshold)
		return -EINVAL;

	cs_tuners->up_threshold = input;
	return count;
}

static ssize_t store_down_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= cs_tuners->up_threshold)
		return -EINVAL;

	cs_tuners->down_threshold = input;
	return count;
}

static ssize_t store_down_threshold_suspended(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= cs_tuners->up_threshold)
		return -EINVAL;

	cs_tuners->down_threshold_suspended = input;
	return count;
}

static ssize_t store_ignore_nice_load(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input, j;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == cs_tuners->ignore_nice_load) /* nothing to do */
		return count;

	cs_tuners->ignore_nice_load = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cs_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
					&dbs_info->cdbs.prev_cpu_wall, 0);
		if (cs_tuners->ignore_nice_load)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_freq_step(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/*
	 * no need to test here if freq_step is zero as the user might actually
	 * want this, they would be crazy though :)
	 */
	cs_tuners->freq_step = input;
	return count;
}

static ssize_t store_boost_enabled(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input >= 1)
		input = 1;
	else
		input = 0;

	cs_tuners->boost_enabled = input;
	return count;
}

static ssize_t store_boost_count(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input < 1)
		input = 0;

	cs_tuners->boost_count = input;
	return count;
}

show_store_one(cs, sampling_rate);
show_store_one(cs, up_threshold);
show_store_one(cs, down_threshold);
show_store_one(cs, down_threshold_suspended);
show_store_one(cs, ignore_nice_load);
show_store_one(cs, freq_step);
show_store_one(cs, boost_enabled);
show_store_one(cs, boost_count);

gov_sys_pol_attr_rw(sampling_rate);
gov_sys_pol_attr_rw(up_threshold);
gov_sys_pol_attr_rw(down_threshold);
gov_sys_pol_attr_rw(down_threshold_suspended);
gov_sys_pol_attr_rw(ignore_nice_load);
gov_sys_pol_attr_rw(freq_step);
gov_sys_pol_attr_rw(boost_enabled);
gov_sys_pol_attr_rw(boost_count);

static struct attribute *dbs_attributes_gov_sys[] = {
	&sampling_rate_gov_sys.attr,
	&up_threshold_gov_sys.attr,
	&down_threshold_gov_sys.attr,
	&down_threshold_suspended_gov_sys.attr,
	&ignore_nice_load_gov_sys.attr,
	&freq_step_gov_sys.attr,
	&boost_enabled_gov_sys.attr,
	&boost_count_gov_sys.attr,
	NULL
};

static struct attribute_group cs_attr_group_gov_sys = {
	.attrs = dbs_attributes_gov_sys,
	.name = "chill",
};

static struct attribute *dbs_attributes_gov_pol[] = {
	&sampling_rate_gov_pol.attr,
	&up_threshold_gov_pol.attr,
	&down_threshold_gov_pol.attr,
	&down_threshold_suspended_gov_pol.attr,
	&ignore_nice_load_gov_pol.attr,
	&freq_step_gov_pol.attr,
	&boost_enabled_gov_pol.attr,
	&boost_count_gov_pol.attr,
	NULL
};

static struct attribute_group cs_attr_group_gov_pol = {
	.attrs = dbs_attributes_gov_pol,
	.name = "chill",
};

/************************** sysfs end ************************/

static int cs_init(struct dbs_data *dbs_data, bool notify)
{
	struct cs_dbs_tuners *tuners;

	tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	tuners->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
	tuners->down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD;
	tuners->down_threshold_suspended = DEF_FREQUENCY_DOWN_THRESHOLD_SUSPENDED;
	tuners->ignore_nice_load = 0;
	tuners->freq_step = DEF_FREQUENCY_STEP;
	tuners->boost_enabled = DEF_BOOST_ENABLED;
	tuners->boost_count = DEF_BOOST_COUNT;

	dbs_data->tuners = tuners;
	dbs_data->min_sampling_rate = DEF_SAMPLING_RATE;
	return 0;
}

static void cs_exit(struct dbs_data *dbs_data, bool notify)
{
	if (notify)
		cpufreq_unregister_notifier(&cs_cpufreq_notifier_block,
					    CPUFREQ_TRANSITION_NOTIFIER);

	kfree(dbs_data->tuners);
}

define_get_cpu_dbs_routines(cs_cpu_dbs_info);

static struct common_dbs_data cs_dbs_cdata = {
	.governor = 1,
	.attr_group_gov_sys = &cs_attr_group_gov_sys,
	.attr_group_gov_pol = &cs_attr_group_gov_pol,
	.get_cpu_cdbs = get_cpu_cdbs,
	.get_cpu_dbs_info_s = get_cpu_dbs_info_s,
	.gov_dbs_timer = cs_dbs_timer,
	.gov_check_cpu = cs_check_cpu,
	.init = cs_init,
	.exit = cs_exit,
	.mutex = __MUTEX_INITIALIZER(cs_dbs_cdata.mutex),
};

static int cs_cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	return cpufreq_governor_dbs(policy, &cs_dbs_cdata, event);
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_CHILL
static
#endif
struct cpufreq_governor cpufreq_gov_chill = {
	.name			= "chill",
	.governor		= cs_cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_chill);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_chill);
}

MODULE_AUTHOR("Alexander Clouter <alex@digriz.org.uk>");
MODULE_AUTHOR("Joe Maples <joe@frap129.org>");
MODULE_DESCRIPTION("'cpufreq_chill' - A dynamic cpufreq governor for "
		"Low Latency Frequency Transition capable processors "
		"optimised for use in a battery environment");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_CHILL
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
