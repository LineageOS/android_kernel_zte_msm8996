/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mfd/ak49xx/core-resource.h>


static enum ak49xx_intf_status ak49xx_intf = -1;

int ak49xx_core_irq_init(
	struct ak49xx_core_resource *ak49xx_core_res)
{
	int ret = 0;

	if (ak49xx_core_res->irq != -1) {
		ret = ak49xx_irq_init(ak49xx_core_res);
		if (ret)
			pr_err("IRQ initialization failed\n");
	}

	return ret;
}
EXPORT_SYMBOL(ak49xx_core_irq_init);

int ak49xx_initialize_irq(
	struct ak49xx_core_resource *ak49xx_core_res,
	unsigned int irq,
	unsigned int irq_base)
{
	ak49xx_core_res->irq = irq;
	ak49xx_core_res->irq_base = irq_base;

	return 0;
}
EXPORT_SYMBOL(ak49xx_initialize_irq);

int ak49xx_core_res_init(
	struct ak49xx_core_resource *ak49xx_core_res,
	int num_irqs, int num_irq_regs,
	int (*codec_read)(struct ak49xx_core_resource*, unsigned short),
	int (*codec_write)(struct ak49xx_core_resource*, unsigned short, u8),
	int (*codec_bulk_read) (struct ak49xx_core_resource*, unsigned short,
							int, u8*),
	int (*codec_bulk_write) (struct ak49xx_core_resource*, unsigned short,
							int, u8*))
{
	mutex_init(&ak49xx_core_res->pm_lock);
	ak49xx_core_res->wlock_holders = 0;
	ak49xx_core_res->pm_state = AK49XX_PM_SLEEPABLE;
	init_waitqueue_head(&ak49xx_core_res->pm_wq);
	pm_qos_add_request(&ak49xx_core_res->pm_qos_req,
				PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	ak49xx_core_res->codec_reg_read = codec_read;
	ak49xx_core_res->codec_reg_write = codec_write;
	ak49xx_core_res->codec_bulk_read = codec_bulk_read;
	ak49xx_core_res->codec_bulk_write = codec_bulk_write;
	ak49xx_core_res->num_irqs = num_irqs;
	ak49xx_core_res->num_irq_regs = num_irq_regs;

	pr_info("%s: num_irqs = %d, num_irq_regs = %d\n",
			__func__, ak49xx_core_res->num_irqs,
			ak49xx_core_res->num_irq_regs);

	return 0;
}
EXPORT_SYMBOL(ak49xx_core_res_init);

void ak49xx_core_res_deinit(struct ak49xx_core_resource *ak49xx_core_res)
{
	pm_qos_remove_request(&ak49xx_core_res->pm_qos_req);
	mutex_destroy(&ak49xx_core_res->pm_lock);
	ak49xx_core_res->codec_reg_read = NULL;
	ak49xx_core_res->codec_reg_write = NULL;
	ak49xx_core_res->codec_bulk_read = NULL;
	ak49xx_core_res->codec_bulk_write = NULL;
}
EXPORT_SYMBOL(ak49xx_core_res_deinit);

enum ak49xx_pm_state ak49xx_pm_cmpxchg(
		struct ak49xx_core_resource *ak49xx_core_res,
		enum ak49xx_pm_state o,
		enum ak49xx_pm_state n)
{
	enum ak49xx_pm_state old;
	mutex_lock(&ak49xx_core_res->pm_lock);
	old = ak49xx_core_res->pm_state;
	if (old == o)
		ak49xx_core_res->pm_state = n;
	mutex_unlock(&ak49xx_core_res->pm_lock);
	return old;
}
EXPORT_SYMBOL(ak49xx_pm_cmpxchg);

int ak49xx_core_res_suspend(
	struct ak49xx_core_resource *ak49xx_core_res,
	pm_message_t pmesg)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);
	/*
	 * pm_qos_update_request() can be called after this suspend chain call
	 * started. thus suspend can be called while lock is being held
	 */
	mutex_lock(&ak49xx_core_res->pm_lock);
	if (ak49xx_core_res->pm_state == AK49XX_PM_SLEEPABLE) {
		pr_err("%s: suspending system, state %d, wlock %d\n",
			 __func__, ak49xx_core_res->pm_state,
			 ak49xx_core_res->wlock_holders);
		ak49xx_core_res->pm_state = AK49XX_PM_ASLEEP;
	} else if (ak49xx_core_res->pm_state == AK49XX_PM_AWAKE) {
		/*
		 * unlock to wait for pm_state == AK49XX_PM_SLEEPABLE
		 * then set to AK49XX_PM_ASLEEP
		 */
		pr_err("%s: waiting to suspend system, state %d, wlock %d\n",
			 __func__, ak49xx_core_res->pm_state,
			 ak49xx_core_res->wlock_holders);
		mutex_unlock(&ak49xx_core_res->pm_lock);
		if (!(wait_event_timeout(ak49xx_core_res->pm_wq,
					 ak49xx_pm_cmpxchg(ak49xx_core_res,
						  AK49XX_PM_SLEEPABLE,
						  AK49XX_PM_ASLEEP) ==
							AK49XX_PM_SLEEPABLE,
					 HZ))) {
			pr_err("%s: suspend failed state %d, wlock %d\n",
				 __func__, ak49xx_core_res->pm_state,
				 ak49xx_core_res->wlock_holders);
			ret = -EBUSY;
		} else {
			pr_err("%s: done, state %d, wlock %d\n", __func__,
				 ak49xx_core_res->pm_state,
				 ak49xx_core_res->wlock_holders);
		}
		mutex_lock(&ak49xx_core_res->pm_lock);
	} else if (ak49xx_core_res->pm_state == AK49XX_PM_ASLEEP) {
		pr_err("%s: system is already suspended, state %d, wlock %dn",
			__func__, ak49xx_core_res->pm_state,
			ak49xx_core_res->wlock_holders);
	}
	mutex_unlock(&ak49xx_core_res->pm_lock);

	return ret;
}
EXPORT_SYMBOL(ak49xx_core_res_suspend);

int ak49xx_core_res_resume(
	struct ak49xx_core_resource *ak49xx_core_res)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);
	mutex_lock(&ak49xx_core_res->pm_lock);
	if (ak49xx_core_res->pm_state == AK49XX_PM_ASLEEP) {
		pr_err("[LHS]%s: resuming system, state %d, wlock %d\n", __func__,
				ak49xx_core_res->pm_state,
				ak49xx_core_res->wlock_holders);
		ak49xx_core_res->pm_state = AK49XX_PM_SLEEPABLE;
	} else {
		pr_err("%s: system is already awake, state %d wlock %d\n",
				__func__, ak49xx_core_res->pm_state,
				ak49xx_core_res->wlock_holders);
	}
	mutex_unlock(&ak49xx_core_res->pm_lock);
	wake_up_all(&ak49xx_core_res->pm_wq);

	return ret;
}
EXPORT_SYMBOL(ak49xx_core_res_resume);

enum ak49xx_intf_status ak49xx_get_intf_type(void)
{
	return ak49xx_intf;
}
EXPORT_SYMBOL(ak49xx_get_intf_type);

void ak49xx_set_intf_type(enum ak49xx_intf_status intf_status)
{
	ak49xx_intf = intf_status;
}
EXPORT_SYMBOL(ak49xx_set_intf_type);

