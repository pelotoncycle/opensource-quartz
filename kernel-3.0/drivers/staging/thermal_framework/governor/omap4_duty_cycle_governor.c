/*
 * Duty cycle governor
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Eugene Mandrenko <Ievgen.mandrenko@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/suspend.h>
#include <plat/omap_device.h>

#define NORMAL_TEMP_MONITORING_RATE 1000
#define NORMAL_MONITORING_RATE 10000
#define DEFAULT_TEMPERATURE 65000
#define TEMP_THRESHOLD 1
#define INIT_SECTION -1

struct duty_governor {
	struct pcb_sens *tpcb;
	struct duty_cycle *tduty;
	struct pcb_section *tpcb_sections;
	int period;
	int previous_temp;
	int curr_pcb_temp;
	int previous_pcb_temp;
	int working_section;
	int npcb_sections;
	struct delayed_work duty_cycle_governor_work;
};

/* protect global data */
static DEFINE_MUTEX(mutex_duty_governor);

static struct duty_governor *t_governor;
static struct pcb_section *pcb_sections;
static int pcb_sections_size;

static void omap4_duty_schedule(struct duty_governor *t_gov)
{
	if (!IS_ERR_OR_NULL(t_gov) &&
	    !IS_ERR_OR_NULL(t_gov->tpcb) &&
	    !IS_ERR_OR_NULL(t_gov->tduty))
		schedule_delayed_work(&t_governor->duty_cycle_governor_work,
				      msecs_to_jiffies(0));
}

int omap4_duty_pcb_register(struct pcb_sens *tpcb)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor)) {
		if (t_governor->tpcb == NULL) {
			t_governor->tpcb = tpcb;
			t_governor->period = NORMAL_TEMP_MONITORING_RATE;
		} else {
			pr_err("%s:dublicate of pcb registration\n", __func__);
			mutex_unlock(&mutex_duty_governor);

			return -EBUSY;
		}
	}
	omap4_duty_schedule(t_governor);

	mutex_unlock(&mutex_duty_governor);

	return 0;
}
static bool is_treshold(struct duty_governor *tgov)
{
	int delta;

	delta = abs(tgov->previous_pcb_temp - tgov->curr_pcb_temp);

	if (delta > TEMP_THRESHOLD)
		return true;

	return false;
}

static int omap4_duty_apply_constraint(struct duty_governor *tgov,
					int sect_num)
{
	struct pcb_section *t_pcb_sections = &tgov->tpcb_sections[sect_num];
	struct duty_cycle_params *tduty_params = &t_pcb_sections->tduty_params;
	int dc_enabled = t_pcb_sections->duty_cycle_enabled;
	struct duty_cycle *t_duty = tgov->tduty;
	int ret = true;

	if (tgov->working_section != sect_num) {
		ret = tgov->tduty->enable(false, false);

		if (ret)
			return ret;

		if (dc_enabled) {
			if (t_duty->update_params(tduty_params))
				return ret;

			tgov->tduty->enable(dc_enabled, true);
		}
		tgov->working_section = sect_num;
	}

	return ret;
}

static void omap4_duty_update(struct duty_governor *tgov)
{
	int sect_num;

	if (IS_ERR_OR_NULL(tgov) ||
	    IS_ERR_OR_NULL(tgov->tduty) ||
	    IS_ERR_OR_NULL(tgov->tpcb_sections))
		return;

	for (sect_num = 0; sect_num < tgov->npcb_sections; sect_num++)
		if (tgov->tpcb_sections[sect_num].pcb_temp_level >
		    tgov->curr_pcb_temp)
			break;

	if (sect_num >= tgov->npcb_sections)
		sect_num = tgov->npcb_sections - 1;

	if (omap4_duty_apply_constraint(tgov, sect_num))
		tgov->previous_pcb_temp = tgov->curr_pcb_temp;
}

static void omap4_duty_governor_delayed_work_fn(struct work_struct *work)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor->tpcb)) {
		if (!IS_ERR_OR_NULL(t_governor->tpcb->update_temp)) {
			t_governor->curr_pcb_temp =
					t_governor->tpcb->update_temp();
			if (is_treshold(t_governor))
				omap4_duty_update(t_governor);
		} else {
			pr_err("%s:update_temp() isn't defined\n", __func__);
		}
	}
	schedule_delayed_work(&t_governor->duty_cycle_governor_work,
			      msecs_to_jiffies(t_governor->period));

	mutex_unlock(&mutex_duty_governor);
}

static int omap4_duty_pm_notifier_cb(struct notifier_block *notifier,
				     unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&t_governor->duty_cycle_governor_work);
		break;
	case PM_POST_SUSPEND:
		omap4_duty_schedule(t_governor);
		break;
	}

	return NOTIFY_DONE;
}

int omap4_duty_cycle_register(struct duty_cycle *tduty)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor)) {
		if (t_governor->tduty == NULL) {
			t_governor->tduty = tduty;
		} else {
			pr_err("%s:dublicate of duty cycle registration\n",
			       __func__);
			mutex_unlock(&mutex_duty_governor);

			return -EBUSY;
		}
		/* Setup initial parameters for duty cycle */
		omap4_duty_update(t_governor);
	}

	omap4_duty_schedule(t_governor);

	mutex_unlock(&mutex_duty_governor);

	return 0;
}

void omap4_duty_pcb_section_reg(struct pcb_section *pcb_sect, int sect_size)
{
	mutex_lock(&mutex_duty_governor);

	pcb_sections = pcb_sect;
	pcb_sections_size = sect_size;

	if (!IS_ERR_OR_NULL(t_governor)) {
		t_governor->tpcb_sections = pcb_sections;
		t_governor->npcb_sections = pcb_sections_size;
		omap4_duty_update(t_governor);
	}

	mutex_unlock(&mutex_duty_governor);
}

static struct notifier_block omap4_duty_pm_notifier = {
	.notifier_call = omap4_duty_pm_notifier_cb,
};

/**
   * Comes From Blaze Tablete which may reuse by other projects
**/
static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= DEFAULT_TEMPERATURE,
		.max_opp			= 1500000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1300000,
			.cooling_rate		= 1100000,
			.nitro_interval		= 20000, /* ms */
			.nitro_percentage	= 50,
		},
	},
};

static int __init omap4_duty_governor_init(void)
{
	
	if ((!cpu_is_omap443x())&&(!cpu_is_omap446x())&&(!cpu_is_omap447x()))
		return 0;
	

	if (cpu_is_omap443x()) {
		omap4_duty_governor_pcb_sections[0].tduty_params.nitro_rate		= 1200000;
		omap4_duty_governor_pcb_sections[0].tduty_params.cooling_rate	= 1008000;
	} else if (cpu_is_omap446x()) {
		omap4_duty_governor_pcb_sections[0].tduty_params.nitro_rate		= 1500000;
		omap4_duty_governor_pcb_sections[0].tduty_params.cooling_rate	= 1200000;
	} else { /* cpu_is_omap447x() */
/* Remember to modify the condition in omap4_duty_frequency_change() for OMAP4_DUTY_HEATING state */
#if !defined(CONFIG_MACH_OMAP4_PB1ICOM) && !defined(CONFIG_MACH_OMAP4_QUARTZ)
		omap4_duty_governor_pcb_sections[0].tduty_params.nitro_rate		= 1500000;
		omap4_duty_governor_pcb_sections[0].tduty_params.cooling_rate	= 1300000;
#endif
	}
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
		ARRAY_SIZE(omap4_duty_governor_pcb_sections));

	t_governor = kzalloc(sizeof(struct duty_governor), GFP_KERNEL);
	if (IS_ERR_OR_NULL(t_governor)) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}
	t_governor->period = NORMAL_MONITORING_RATE;
	t_governor->curr_pcb_temp = DEFAULT_TEMPERATURE;
	t_governor->previous_temp = DEFAULT_TEMPERATURE;
	t_governor->tpcb_sections = pcb_sections;
	t_governor->npcb_sections = pcb_sections_size;
	t_governor->working_section = INIT_SECTION;

	if (register_pm_notifier(&omap4_duty_pm_notifier))
		pr_err("%s:omap4_duty_gov pm registration failed!\n", __func__);

	INIT_DELAYED_WORK(&t_governor->duty_cycle_governor_work,
			  omap4_duty_governor_delayed_work_fn);

	return 0;
}

static void __exit omap4_duty_governor_exit(void)
{
	cancel_delayed_work_sync(&t_governor->duty_cycle_governor_work);
	kfree(t_governor);
}

early_initcall(omap4_duty_governor_init);
module_exit(omap4_duty_governor_exit);

MODULE_AUTHOR("Euvgen Mandrenko <ievgen.mandrenko@ti.com>");
MODULE_DESCRIPTION("OMAP on-die thermal governor");
MODULE_LICENSE("GPL");

