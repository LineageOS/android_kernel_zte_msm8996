/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "power.h"
#include <linux/fb.h>//ZTE_PM to record how long LCD keeps on

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count) ||
		system_state != SYSTEM_RUNNING) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(autosleep_state);

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

#ifndef CONFIG_ZTE_PLATFORM_LCD_ON_TIME
#define CONFIG_ZTE_PLATFORM_LCD_ON_TIME
#endif

#ifdef CONFIG_ZTE_PLATFORM_LCD_ON_TIME
extern void zte_update_lateresume_2_earlysuspend_time(bool resume_or_earlysuspend);	//LHX_PM_20110411_01 resume_or_earlysuspend? lateresume : earlysuspend
#endif
int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif
	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}

#ifdef CONFIG_ZTE_PLATFORM_LCD_ON_TIME
//ZTE_PM begin
static int lcd_fb_callback(struct notifier_block *nfb,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	//pr_info("ZTE_PM %s enter , event=%lu\n", __func__, event);
	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
        pr_info("ZTE_PM %s enter , blank=%d\n", __func__, *blank);
		if (*blank == FB_BLANK_UNBLANK){
			zte_update_lateresume_2_earlysuspend_time(true);//LHX_PM_20110411_01 update resume time,indicate the LCD will turn on.
		}
		else if ((*blank == FB_BLANK_POWERDOWN)||(*blank == FB_BLANK_NORMAL)){
			zte_update_lateresume_2_earlysuspend_time(false);//LHX_PM_20110411_01 update suspend time,indicate the LCD will turn off.
		}
	}

	return 0;
}

static struct notifier_block __refdata lcd_fb_notifier = {
	.notifier_call = lcd_fb_callback,
};
//ZTE_PM end
#endif
int __init pm_autosleep_init(void)
{
#ifdef CONFIG_ZTE_PLATFORM_LCD_ON_TIME
	fb_register_client(&lcd_fb_notifier);//ZTE_PM
#endif
	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
