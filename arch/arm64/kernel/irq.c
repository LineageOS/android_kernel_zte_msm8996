/*
 * Based on arch/arm/kernel/irq.c
 *
 * Copyright (C) 1992 Linus Torvalds
 * Modifications for ARM processor Copyright (C) 1995-2000 Russell King.
 * Support for Dynamic Tick Timer Copyright (C) 2004-2005 Nokia Corporation.
 * Dynamic Tick Timer written by Tony Lindgren <tony@atomide.com> and
 * Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>.
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel_stat.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/seq_file.h>
#include <linux/ratelimit.h>

unsigned long irq_err_count;

//zte_pm_liyf_20151010 cp 8952
int zte_smd_wakeup	=	0;
void print_irq_info(int i)
{
    struct irqaction * action;
    struct irq_desc* zte_irq_desc;
    unsigned long flags;
    zte_irq_desc = irq_to_desc(i);
    if (zte_irq_desc) {
        raw_spin_lock_irqsave(&zte_irq_desc->lock, flags);
        action = zte_irq_desc->action;
        if (!action)
            goto unlock;
            printk(KERN_ERR "[IRQ] IRQ-NUM=%d\n",i);
            printk(KERN_ERR "      chip->name=%10s\n",zte_irq_desc->irq_data.chip->name ? : "-");
            printk(KERN_ERR "      action->name=%s\n",action->name);

            //zte jiangfeng add
            //notes:use ' adb shell cat /proc/interrupts | grep smd '  could find :IRQ-NUM=57
            if(!strcmp(action->name,"qcom,smd-modem"))//note: zte change smd-dev to smd-modem 20150304
                zte_smd_wakeup	=	1;

            for (action = action->next; action; action = action->next)
                printk(KERN_ERR "      action->name=%s\n",action->name);

	unlock:
            raw_spin_unlock_irqrestore(&zte_irq_desc->lock, flags);
        } else{
            printk(KERN_ERR "[IRQ] error in dump irq info for irq %d\n", i);
        }
}
//zte_pm end
int arch_show_interrupts(struct seq_file *p, int prec)
{
#ifdef CONFIG_SMP
	show_ipi_list(p, prec);
#endif
	seq_printf(p, "%*s: %10lu\n", prec, "Err", irq_err_count);
	return 0;
}

void (*handle_arch_irq)(struct pt_regs *) = NULL;

void __init set_handle_irq(void (*handle_irq)(struct pt_regs *))
{
	if (handle_arch_irq)
		return;

	handle_arch_irq = handle_irq;
}

void __init init_IRQ(void)
{
	irqchip_init();
	if (!handle_arch_irq)
		panic("No interrupt controller found.");
}

#ifdef CONFIG_HOTPLUG_CPU
static bool migrate_one_irq(struct irq_desc *desc)
{
	struct irq_data *d = irq_desc_get_irq_data(desc);
	const struct cpumask *affinity = d->affinity;

	/*
	 * If this is a per-CPU interrupt, or the affinity does not
	 * include this CPU, then we have nothing to do.
	 */
	if (irqd_is_per_cpu(d) || !cpumask_test_cpu(smp_processor_id(), affinity))
		return false;

	if (cpumask_any_and(affinity, cpu_online_mask) >= nr_cpu_ids)
		affinity = cpu_online_mask;

	return irq_set_affinity_locked(d, affinity, false) != 0;
}

/*
 * The current CPU has been marked offline.  Migrate IRQs off this CPU.
 * If the affinity settings do not allow other CPUs, force them onto any
 * available CPU.
 *
 * Note: we must iterate over all IRQs, whether they have an attached
 * action structure or not, as we need to get chained interrupts too.
 */
void migrate_irqs(void)
{
	unsigned int i;
	struct irq_desc *desc;
	unsigned long flags;

	local_irq_save(flags);

	for_each_irq_desc(i, desc) {
		bool affinity_broken;

		raw_spin_lock(&desc->lock);
		affinity_broken = migrate_one_irq(desc);
		raw_spin_unlock(&desc->lock);

		if (affinity_broken)
			pr_warn_ratelimited("IRQ%u no longer affine to CPU%u\n",
					    i, smp_processor_id());
	}

	local_irq_restore(flags);
}
#endif /* CONFIG_HOTPLUG_CPU */
