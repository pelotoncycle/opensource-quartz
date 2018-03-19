/*
 *  OMAP2PLUS cpufreq driver
 *
 *  CPU frequency scaling for OMAP using OPP information
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Tony Lindgren <tony@atomide.com>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * Copyright (C) 2007-2011 Texas Instruments, Inc.
 * Updated to support OMAP3
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/opp.h>
#include <linux/cpu.h>
#include <linux/thermal_framework.h>
#include <linux/platform_device.h>
#include <linux/omap4_duty_cycle.h>

#include <asm/system.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>

#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/common.h>

#include <mach/hardware.h>

#ifdef CONFIG_OMAP4_DPLL_CASCADING
#include <mach/omap4-common.h>
#endif

#include "dvfs.h"

/**
 * Remove OMAP4 codes to adjust loops_per_jiffy for smp because we use the below patch instead.
 * "ARM: add cpufreq transiton notifier to adjust loops_per_jiffy for smp"
 */
#if 0
#ifdef CONFIG_SMP
struct lpj_info {
	unsigned long	ref;
	unsigned int	freq;
};

static DEFINE_PER_CPU(struct lpj_info, lpj_ref);
static struct lpj_info global_lpj_ref;
#endif
#endif

static struct cpufreq_frequency_table *freq_table;
static atomic_t freq_table_users = ATOMIC_INIT(0);
static struct clk *mpu_clk;
static char *mpu_clk_name;
static struct device *mpu_dev;
static DEFINE_MUTEX(omap_cpufreq_lock);

#ifdef CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ
static unsigned int max_emergency_shutdown;
#endif /* CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ */
static bool omap_cpufreq_locked;
static unsigned int safe_suspend_freq;
static unsigned int min_thermal;
static unsigned int max_duty_cycle;
static unsigned int max_thermal;
static unsigned int max_freq;
static unsigned int current_target_freq;
static unsigned int current_cooling_level;
static bool omap_cpufreq_ready;
static bool omap_cpufreq_suspended;

static unsigned int omap_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NR_CPUS)
		return 0;

	rate = clk_get_rate(mpu_clk) / 1000;
	return rate;
}

/**
 * Remove OMAP4 codes to adjust loops_per_jiffy for smp because we use the below patch instead.
 * "ARM: add cpufreq transiton notifier to adjust loops_per_jiffy for smp"
 */
#if 0
static void omap_cpufreq_lpj_recalculate(unsigned int target_freq,
					 unsigned int cur_freq)
{
 #ifdef CONFIG_SMP
	unsigned int i;

	/*
	 * Note that loops_per_jiffy is not updated on SMP systems in
	 * cpufreq driver. So, update the per-CPU loops_per_jiffy value
	 * on frequency transition. We need to update all dependent CPUs.
	 */
	for_each_possible_cpu(i) {
		struct lpj_info *lpj = &per_cpu(lpj_ref, i);
		if (!lpj->freq) {
			lpj->ref = per_cpu(cpu_data, i).loops_per_jiffy;
			lpj->freq = cur_freq;
		}

		per_cpu(cpu_data, i).loops_per_jiffy =
			cpufreq_scale(lpj->ref, lpj->freq, target_freq);
	}

	/* And don't forget to adjust the global one */
	if (!global_lpj_ref.freq) {
		global_lpj_ref.ref = loops_per_jiffy;
		global_lpj_ref.freq = cur_freq;
	}
	loops_per_jiffy = cpufreq_scale(global_lpj_ref.ref, global_lpj_ref.freq,
					target_freq);
#endif
}
#endif

static int omap_cpufreq_scale(unsigned int target_freq, unsigned int cur_freq)
{
	int ret;
	struct cpufreq_freqs freqs;

	if (omap_cpufreq_locked) {
		pr_err("%s: cpufreq is locked during suspend (target=%u, cur=%u<->%u)\n",
			__func__, target_freq, cur_freq, omap_getspeed(0));
		return -EINVAL;
	}

	freqs.new = target_freq;
	freqs.old = omap_getspeed(0);

	/*
	 * If the new frequency is more than the thermal max allowed
	 * frequency, go ahead and scale the mpu device to proper frequency.
	 */
	if (freqs.new > max_thermal)
		freqs.new = max_thermal;
	if (freqs.new > max_duty_cycle)
		freqs.new = max_duty_cycle;

#ifdef CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ
	if (max_emergency_shutdown && freqs.new > max_emergency_shutdown)
		freqs.new = max_emergency_shutdown;
#endif /* CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ */

	if ((freqs.old == freqs.new) && (cur_freq = freqs.new))
		return 0;

	get_online_cpus();

	/* notifiers */
	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("cpufreq-omap: transition: %u --> %u\n", freqs.old, freqs.new);
#endif

/**
 * Remove OMAP4 codes to adjust loops_per_jiffy for smp because we use the below patch instead.
 * "ARM: add cpufreq transiton notifier to adjust loops_per_jiffy for smp"
 */
#if 0
	if (target_freq > cur_freq)
		omap_cpufreq_lpj_recalculate(freqs.new, freqs.old);
#endif

	ret = omap_device_scale(mpu_dev, mpu_dev, freqs.new * 1000);

	freqs.new = omap_getspeed(0);

/**
 * Remove OMAP4 codes to adjust loops_per_jiffy for smp because we use the below patch instead.
 * "ARM: add cpufreq transiton notifier to adjust loops_per_jiffy for smp"
 */
#if 0
	if (target_freq < cur_freq)
		omap_cpufreq_lpj_recalculate(freqs.new, freqs.old);
#endif

	/* notifiers */
	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	put_online_cpus();

	return ret;
}

static unsigned int omap_thermal_lower_speed(void)
{
	unsigned int max = min_thermal;
	unsigned int curr;
	int i;

	curr = max_thermal;

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
		if (freq_table[i].frequency > max &&
		    freq_table[i].frequency < curr)
			max = freq_table[i].frequency;

	if (max == min_thermal)
		return curr;

	return max;
}

void omap_thermal_throttle(void)
{
	unsigned int cur;
	unsigned int new_max_thermal;

	if (!omap_cpufreq_ready) {
		pr_err_once("%s: Thermal throttle prior to CPUFREQ ready\n",
			     __func__);
		return;
	}

	mutex_lock(&omap_cpufreq_lock);

	new_max_thermal = omap_thermal_lower_speed();
	if (new_max_thermal == max_thermal)
		goto out;

	max_thermal = new_max_thermal;

#ifdef CONFIG_OMAP_THERMAL_DEBUG
	pr_info("%s: cpu throttle at max %u (duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif

	if (!omap_cpufreq_suspended) {
		cur = omap_getspeed(0);
		if (cur > max_thermal)
			omap_cpufreq_scale(max_thermal, cur);
	}

out:
	mutex_unlock(&omap_cpufreq_lock);
}

void omap_thermal_unthrottle(void)
{
	unsigned int cur;

	if (!omap_cpufreq_ready)
		return;

	mutex_lock(&omap_cpufreq_lock);

	if (max_thermal == max_freq) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG_VERBOSE
		pr_info("%s: not throttling (thermal %u; duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif
		goto out;
	}

	max_thermal = max_freq;

#ifdef CONFIG_OMAP_THERMAL_DEBUG
	pr_info("%s: ending cpu throttling (thermal %u; duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif

	if (!omap_cpufreq_suspended) {
		cur = omap_getspeed(0);
		omap_cpufreq_scale(current_target_freq, cur);
	}

out:
	mutex_unlock(&omap_cpufreq_lock);
}

static int omap_verify_speed(struct cpufreq_policy *policy)
{
	if (!freq_table)
		return -EINVAL;
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static int omap_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	unsigned int i;
	int ret = 0;

	if (!freq_table) {
		dev_err(mpu_dev, "%s: cpu%d: no freq table!\n", __func__,
				policy->cpu);
		return -EINVAL;
	}

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
			relation, &i);
	if (ret) {
		dev_dbg(mpu_dev, "%s: cpu%d: no freq match for %d(ret=%d)\n",
			__func__, policy->cpu, target_freq, ret);
		return ret;
	}

	mutex_lock(&omap_cpufreq_lock);

	current_target_freq = freq_table[i].frequency;

	if (!omap_cpufreq_suspended) {
#ifdef CONFIG_OMAP4_DPLL_CASCADING
		if (cpu_is_omap44xx() && target_freq > policy->min)
			omap4_dpll_cascading_blocker_hold(mpu_dev);
#endif
		ret = omap_cpufreq_scale(current_target_freq, policy->cur);
#ifdef CONFIG_OMAP4_DPLL_CASCADING
		if (cpu_is_omap44xx() && target_freq == policy->min)
			omap4_dpll_cascading_blocker_release(mpu_dev);
#endif
	}

	mutex_unlock(&omap_cpufreq_lock);

	return ret;
}

static inline void freq_table_free(void)
{
	if (atomic_dec_and_test(&freq_table_users))
		opp_free_cpufreq_table(mpu_dev, &freq_table);
}

#ifdef CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ
void omap_emergency_shutdown_max_cpufreq(unsigned int max)
{
	unsigned int cur;

	if (!omap_cpufreq_ready) {
		printk(KERN_EMERG "CPUFREQ not ready\n");
		return;
	}

	mutex_lock(&omap_cpufreq_lock);

	max_emergency_shutdown = max;

	if (!omap_cpufreq_suspended) {
		cur = omap_getspeed(0);
		if (cur > max_emergency_shutdown)
			omap_cpufreq_scale(max_emergency_shutdown, cur);
	}

	mutex_unlock(&omap_cpufreq_lock);
}
#endif /* CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ */

#if defined(CONFIG_OMAP_THERMAL) || defined(CONFIG_OMAP4_DUTY_CYCLE)
void omap_thermal_step_freq_down(void)
{
	unsigned int cur;
	unsigned int new_max_thermal;

	if (!omap_cpufreq_ready) {
		pr_err_once("%s: Thermal throttle prior to CPUFREQ ready\n",
			     __func__);
		return;
	}

	mutex_lock(&omap_cpufreq_lock);

	new_max_thermal = omap_thermal_lower_speed();
	if (new_max_thermal == max_thermal) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG_VERBOSE
		pr_info("%s: not throttling (thermal %u; duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif
		goto out;
	}

	max_thermal = new_max_thermal;

#ifdef CONFIG_OMAP_THERMAL_DEBUG
	pr_info("%s: starting cpu throttling at max %u (duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif

	if (!omap_cpufreq_suspended) {
		cur = omap_getspeed(0);
		if (cur > max_thermal)
			omap_cpufreq_scale(max_thermal, cur);
	}

out:
	mutex_unlock(&omap_cpufreq_lock);
}

void omap_thermal_step_freq_up(void)
{
	unsigned int cur;

	if (!omap_cpufreq_ready)
		return;

	mutex_lock(&omap_cpufreq_lock);

	if (max_thermal == max_freq) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG_VERBOSE
		pr_info("%s: not throttling (thermal %u; duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif
		goto out;
	}

	max_thermal = max_freq;

#ifdef CONFIG_OMAP_THERMAL_DEBUG
	pr_info("%s: stepping up to %u (thermal %u; duty cycle %u)\n", __func__, current_target_freq, max_thermal, max_duty_cycle);
#endif

	if (!omap_cpufreq_suspended) {
		cur = omap_getspeed(0);
		omap_cpufreq_scale(current_target_freq, cur);
	}

out:
	mutex_unlock(&omap_cpufreq_lock);
}

/*
 * cpufreq_apply_cooling: based on requested cooling level, throttle the cpu
 * @param cooling_level: percentage of required cooling at the moment
 *
 * The maximum cpu frequency will be readjusted based on the required
 * cooling_level.
*/
static int cpufreq_apply_cooling(struct thermal_dev *dev,
				int cooling_level)
{
	if (cooling_level < current_cooling_level) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG
		pr_info("%s: unthrottle cool level %i curr cool %i\n",
			__func__, cooling_level, current_cooling_level);
#endif
		omap_thermal_step_freq_up();
	} else if (cooling_level > current_cooling_level) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG
		pr_info("%s: throttle cool level %i curr cool %i\n",
			__func__, cooling_level, current_cooling_level);
#endif
		omap_thermal_step_freq_down();
	}

	current_cooling_level = cooling_level;

	return 0;
}
#endif

#ifdef CONFIG_OMAP4_DUTY_CYCLE

/*
 * duty_cycle_apply_cooling: set the max thermal
 * @param max: max cpu frequency allowed by duty cycle
 *
 * The maximum cpu frequency will be readjusted based on the required
 * max frequency
*/
static int duty_cycle_apply_cooling(struct thermal_dev *dev, int max)
{
	if (!omap_cpufreq_ready)
		return 0;

	if (max <= 0)
		max = max_freq;

	mutex_lock(&omap_cpufreq_lock);

	if (max_duty_cycle == max) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG_VERBOSE
		pr_info("%s: not throttling (thermal %u; duty cycle %u)\n", __func__, max_thermal, max_duty_cycle);
#endif
		goto out;
	}

	max_duty_cycle = max;

#ifdef CONFIG_OMAP_THERMAL_DEBUG
	pr_info("%s: setting max to %u (thermal %u)\n", __func__, max_duty_cycle, max_thermal);
#endif

	if (!omap_cpufreq_suspended) {
		unsigned int cur = omap_getspeed(0);
		if (cur > max_duty_cycle)
			omap_cpufreq_scale(current_target_freq, cur);
	}

out:
	mutex_unlock(&omap_cpufreq_lock);

	return 0;
}

static struct duty_cycle_dev duty_dev = {
	.cool_device = duty_cycle_apply_cooling,
};

static int __init omap_duty_cooling_init(void)
{
	return duty_cooling_dev_register(&duty_dev);
}

static void __exit omap_duty_cooling_exit(void)
{
	duty_cooling_dev_unregister();
}


#else

static int __init omap_duty_cooling_init(void) { return 0; }
static void __exit omap_duty_cooling_exit(void) { }

#endif

#ifdef CONFIG_OMAP_THERMAL

static struct thermal_dev_ops cpufreq_cooling_ops = {
	.cool_device = cpufreq_apply_cooling,
};

static struct thermal_dev thermal_dev = {
	.name		= "cpufreq_cooling",
	.domain_name	= "cpu",
	.dev_ops	= &cpufreq_cooling_ops,
};

static int __init omap_cpufreq_cooling_init(void)
{
	return thermal_cooling_dev_register(&thermal_dev);
}

static void __exit omap_cpufreq_cooling_exit(void)
{
	thermal_governor_dev_unregister(&thermal_dev);
}
#else
static int __init omap_cpufreq_cooling_init(void) { return 0; }
static void __exit omap_cpufreq_cooling_exit(void) { }
#endif

static int __cpuinit omap_cpu_init(struct cpufreq_policy *policy)
{
	int result = 0;
	int i;

	mpu_clk = clk_get(NULL, mpu_clk_name);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if (policy->cpu >= NR_CPUS) {
		result = -EINVAL;
		goto fail_ck;
	}

	policy->cur = policy->min = policy->max = omap_getspeed(policy->cpu);

	if (atomic_inc_return(&freq_table_users) == 1)
		result = opp_init_cpufreq_table(mpu_dev, &freq_table);

	if (result) {
		dev_err(mpu_dev, "%s: cpu%d: failed creating freq table[%d]\n",
				__func__, policy->cpu, result);
		goto fail_ck;
	}

	result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (result)
		goto fail_table;

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = omap_getspeed(policy->cpu);

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
		max_freq = max(freq_table[i].frequency, max_freq);

	/*
	 * On OMAP SMP configuartion, both processors share the voltage
	 * and clock. So both CPUs needs to be scaled together and hence
	 * needs software co-ordination. Use cpufreq affected_cpus
	 * interface to handle this scenario. Additional is_smp() check
	 * is to keep SMP_ON_UP build working.
	 */
	if (is_smp()) {
		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->cpus);
	}

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	return 0;

fail_table:
	freq_table_free();
fail_ck:
	clk_put(mpu_clk);
	return result;
}

static int omap_cpu_exit(struct cpufreq_policy *policy)
{
	freq_table_free();
	clk_put(mpu_clk);
	return 0;
}

static struct freq_attr *omap_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver omap_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= omap_verify_speed,
	.target		= omap_target,
	.get		= omap_getspeed,
	.init		= omap_cpu_init,
	.exit		= omap_cpu_exit,
	.name		= "omap2plus",
	.attr		= omap_cpufreq_attr,
};

static int omap_cpufreq_suspend_noirq(struct device *dev)
{
	unsigned int cur;

	mutex_lock(&omap_cpufreq_lock);
	omap_cpufreq_suspended = true;
	cur = omap_getspeed(0);
	if (cur != safe_suspend_freq) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG
		pr_info("%s: cpu suspend freq %u\n", __func__, safe_suspend_freq);
#endif
		omap_cpufreq_scale(safe_suspend_freq, cur);
	}
	omap_cpufreq_locked = true;
	mutex_unlock(&omap_cpufreq_lock);
	return 0;
}

static int omap_cpufreq_resume_noirq(struct device *dev)
{
	unsigned int cur;

	mutex_lock(&omap_cpufreq_lock);
	omap_cpufreq_locked = false;
	cur = omap_getspeed(0);
	if (cur != current_target_freq) {
#ifdef CONFIG_OMAP_THERMAL_DEBUG
		pr_info("%s: cpu resume freq %u\n", __func__, current_target_freq);
#endif
		omap_cpufreq_scale(current_target_freq, cur);
	}
	omap_cpufreq_suspended = false;
	mutex_unlock(&omap_cpufreq_lock);
	return 0;
}

static struct dev_pm_ops omap_cpufreq_driver_pm_ops = {
	.suspend_noirq = omap_cpufreq_suspend_noirq,
	.resume_noirq = omap_cpufreq_resume_noirq,
};

static struct platform_driver omap_cpufreq_platform_driver = {
	.driver.name = "omap_cpufreq",
	.driver.pm = &omap_cpufreq_driver_pm_ops,
};
static struct platform_device omap_cpufreq_device = {
	.name = "omap_cpufreq",
};

static int __init omap_cpufreq_init(void)
{
	int ret;

	if (cpu_is_omap24xx())
		mpu_clk_name = "virt_prcm_set";
	else if (cpu_is_omap34xx())
		mpu_clk_name = "dpll1_ck";
	else if (cpu_is_omap443x())
		mpu_clk_name = "dpll_mpu_ck";
	else if (cpu_is_omap446x() || cpu_is_omap447x())
		mpu_clk_name = "virt_dpll_mpu_ck";

	if (!mpu_clk_name) {
		pr_err("%s: unsupported Silicon?\n", __func__);
		return -EINVAL;
	}

	mpu_dev = omap2_get_mpuss_device();
	if (!mpu_dev) {
		pr_warning("%s: unable to get the mpu device\n", __func__);
		return -EINVAL;
	}

	ret = cpufreq_register_driver(&omap_driver);
	omap_cpufreq_ready = !ret;

	if (cpu_is_omap443x()) {
		min_thermal = CONFIG_OMAP4430_THERMAL_MINIMAL_CPU_FREQUENCY; /* > min_thermal */
		safe_suspend_freq = 300000;
	} else if (cpu_is_omap446x()) {
		min_thermal = CONFIG_OMAP4460_THERMAL_MINIMAL_CPU_FREQUENCY; /* > min_thermal */
		safe_suspend_freq = 350000;
	} else { /* cpu_is_omap447x() */
		min_thermal = CONFIG_OMAP4470_THERMAL_MINIMAL_CPU_FREQUENCY; /* > min_thermal */
		safe_suspend_freq = 396800;
	}

	max_thermal = max_duty_cycle = max_freq;
	current_cooling_level = 0;

	if (!ret) {
		int t;

		t = platform_device_register(&omap_cpufreq_device);
		if (t)
			pr_warn("%s_init: platform_device_register failed\n",
				__func__);
		t = platform_driver_register(&omap_cpufreq_platform_driver);
		if (t)
			pr_warn("%s_init: platform_driver_register failed\n",
				__func__);
		ret = omap_cpufreq_cooling_init();

		if (ret)
			return ret;

		ret = omap_duty_cooling_init();
		if (ret)
			pr_warn("%s: omap_duty_cooling_init failed\n",
				__func__);
	}

	return ret;
}

static void __exit omap_cpufreq_exit(void)
{
	omap_cpufreq_cooling_exit();
	omap_duty_cooling_exit();
	cpufreq_unregister_driver(&omap_driver);
	platform_driver_unregister(&omap_cpufreq_platform_driver);
	platform_device_unregister(&omap_cpufreq_device);
}

MODULE_DESCRIPTION("cpufreq driver for OMAP2PLUS SOCs");
MODULE_LICENSE("GPL");
late_initcall(omap_cpufreq_init);
module_exit(omap_cpufreq_exit);
