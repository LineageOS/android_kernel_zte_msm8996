/* Copyright (c) 2010-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/smp.h>
#include <linux/tick.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/msm-bus.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/scm-boot.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/system_misc.h>
#include <soc/qcom/socinfo.h>//zte_pm liyf 20151010
#ifdef CONFIG_VFP
#include <asm/vfp.h>
#endif
#include <soc/qcom/jtag.h>
#include "idle.h"
#include "pm-boot.h"
//#include "../../../arch/arm/mach-msm/clock.h"
#include "../../clk/msm/clock.h"//zte_pm liyf 20151010
#include <linux/seq_file.h>     //zte_pm liyf 20151010

#define SCM_CMD_TERMINATE_PC	(0x2)
#define SCM_CMD_CORE_HOTPLUGGED (0x10)
#define SCM_FLUSH_FLAG_MASK	(0x3)

#define SCLK_HZ (32768)

#define MAX_BUF_SIZE  1024

static int msm_pm_debug_mask = 1;
module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);
#ifndef ZTE_GPIO_DEBUG  //zte_pm liyf 20151010
#define ZTE_GPIO_DEBUG
#endif

enum {
	MSM_PM_DEBUG_SUSPEND = BIT(0),
	MSM_PM_DEBUG_POWER_COLLAPSE = BIT(1),
	MSM_PM_DEBUG_SUSPEND_LIMITS = BIT(2),
	MSM_PM_DEBUG_CLOCK = BIT(3),
	MSM_PM_DEBUG_RESET_VECTOR = BIT(4),
	MSM_PM_DEBUG_IDLE = BIT(5),
	MSM_PM_DEBUG_IDLE_LIMITS = BIT(6),
	MSM_PM_DEBUG_HOTPLUG = BIT(7),
#ifdef ZTE_GPIO_DEBUG
	MSM_PM_DEBUG_ZTE_LOGS = BIT(9),//zte's customize log,default not open
#endif
	MSM_PM_DEBUG_ZTE_IDLE_CLOCK = BIT(10),//zte's customize log,default not open
};

enum msm_pc_count_offsets {
	MSM_PC_ENTRY_COUNTER,
	MSM_PC_EXIT_COUNTER,
	MSM_PC_FALLTHRU_COUNTER,
	MSM_PC_UNUSED,
	MSM_PC_NUM_COUNTERS,
};
//zte_pm liyf 20151010
typedef struct {
	uint32_t app_suspend_state;
    uint32_t modemsleeptime; //sleeptime, zenghuipeng 20120828 add
	uint32_t modemawaketime;//lianghouxing 20121119 add to store how long modem keep awake
	uint32_t modemsleep_or_awake;//lianghouxing 20121119 add to indicate modem is sleep or awake,1 sleep,0 means never enter sleep yet, 2 awake
	uint32_t physlinktime;/////liukejing 20120606 add to count data link time
	uint32_t modemawake_timeout_crash;////ZTE_PM_liukejing 20130621
} pm_count_time;

pm_count_time  *zte_imem_ptr = NULL;///zte_liukejing add 
//zte_pm end

static bool msm_pm_ldo_retention_enabled = true;
static bool msm_pm_tz_flushes_cache;
static bool msm_pm_ret_no_pll_switch;
static bool msm_no_ramp_down_pc;
static struct msm_pm_sleep_status_data *msm_pm_slp_sts;
DEFINE_PER_CPU(struct clk *, cpu_clks);
static struct clk *l2_clk;

static long *msm_pc_debug_counters;
static int kernel_sleep_count=0;//zte_pm_20160222 check log for huangyanjun

static cpumask_t retention_cpus;
static DEFINE_SPINLOCK(retention_lock);

static bool msm_pm_is_L1_writeback(void)
{
	u32 cache_id = 0;

#if defined(CONFIG_CPU_V7)
	u32 sel = 0;
	asm volatile ("mcr p15, 2, %[ccselr], c0, c0, 0\n\t"
		      "isb\n\t"
		      "mrc p15, 1, %[ccsidr], c0, c0, 0\n\t"
		      :[ccsidr]"=r" (cache_id)
		      :[ccselr]"r" (sel)
		     );
	return cache_id & BIT(30);
#elif defined(CONFIG_ARM64)
	u32 sel = 0;
	asm volatile("msr csselr_el1, %[ccselr]\n\t"
		     "isb\n\t"
		     "mrs %[ccsidr],ccsidr_el1\n\t"
		     :[ccsidr]"=r" (cache_id)
		     :[ccselr]"r" (sel)
		    );
	return cache_id & BIT(30);
#else
#error No valid CPU arch selected
#endif
}

static bool msm_pm_swfi(bool from_idle)
{
	msm_arch_idle();
	return true;
}

static bool msm_pm_retention(bool from_idle)
{
	int ret = 0;
	unsigned int cpu = smp_processor_id();
	struct clk *cpu_clk = per_cpu(cpu_clks, cpu);

	spin_lock(&retention_lock);

	if (!msm_pm_ldo_retention_enabled)
		goto bailout;

	cpumask_set_cpu(cpu, &retention_cpus);
	spin_unlock(&retention_lock);

	if (!msm_pm_ret_no_pll_switch)
		clk_disable(cpu_clk);

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_RETENTION, false);
	WARN_ON(ret);

	msm_arch_idle();

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);

	if (!msm_pm_ret_no_pll_switch)
		if (clk_enable(cpu_clk))
			pr_err("%s(): Error restore cpu clk\n", __func__);

	spin_lock(&retention_lock);
	cpumask_clear_cpu(cpu, &retention_cpus);
bailout:
	spin_unlock(&retention_lock);
	return true;
}

static inline void msm_pc_inc_debug_count(uint32_t cpu,
		enum msm_pc_count_offsets offset)
{
	int cntr_offset;
	uint32_t cluster_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
	uint32_t cpu_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);

	if (cluster_id >= MAX_NUM_CLUSTER || cpu_id >= MAX_CPUS_PER_CLUSTER)
		BUG();

	cntr_offset = (cluster_id * MAX_CPUS_PER_CLUSTER * MSM_PC_NUM_COUNTERS)
			 + (cpu_id * MSM_PC_NUM_COUNTERS) + offset;

	if (!msm_pc_debug_counters)
		return;

	msm_pc_debug_counters[cntr_offset]++;
}

static bool msm_pm_pc_hotplug(void)
{
	uint32_t cpu = smp_processor_id();
	enum msm_pm_l2_scm_flag flag;
	struct scm_desc desc;

	flag = lpm_cpu_pre_pc_cb(cpu);

	if (!msm_pm_tz_flushes_cache) {
		if (flag == MSM_SCM_L2_OFF)
			flush_cache_all();
		else if (msm_pm_is_L1_writeback())
			flush_cache_louis();
	}

	msm_pc_inc_debug_count(cpu, MSM_PC_ENTRY_COUNTER);

	if (is_scm_armv8()) {
		desc.args[0] = SCM_CMD_CORE_HOTPLUGGED |
			       (flag & SCM_FLUSH_FLAG_MASK);
		desc.arginfo = SCM_ARGS(1);
		scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
				 SCM_CMD_TERMINATE_PC), &desc);
	} else {
		scm_call_atomic1(SCM_SVC_BOOT, SCM_CMD_TERMINATE_PC,
		SCM_CMD_CORE_HOTPLUGGED | (flag & SCM_FLUSH_FLAG_MASK));
	}

	/* Should not return here */
	msm_pc_inc_debug_count(cpu, MSM_PC_FALLTHRU_COUNTER);
	return 0;
}

static bool msm_pm_fastpc(bool from_idle)
{
	int ret = 0;
	unsigned int cpu = smp_processor_id();

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_FASTPC, false);
	WARN_ON(ret);

	if (from_idle || cpu_online(cpu))
		msm_arch_idle();
	else
		msm_pm_pc_hotplug();

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);

	return true;
}

int msm_pm_collapse(unsigned long unused)
{
	uint32_t cpu = smp_processor_id();
	enum msm_pm_l2_scm_flag flag;
	struct scm_desc desc;

	flag = lpm_cpu_pre_pc_cb(cpu);

	if (!msm_pm_tz_flushes_cache) {
		if (flag == MSM_SCM_L2_OFF)
			flush_cache_all();
		else if (msm_pm_is_L1_writeback())
			flush_cache_louis();
	}
	msm_pc_inc_debug_count(cpu, MSM_PC_ENTRY_COUNTER);

	if (is_scm_armv8()) {
		desc.args[0] = flag;
		desc.arginfo = SCM_ARGS(1);
		scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
				 SCM_CMD_TERMINATE_PC), &desc);
	} else {
		scm_call_atomic1(SCM_SVC_BOOT, SCM_CMD_TERMINATE_PC, flag);
	}

	msm_pc_inc_debug_count(cpu, MSM_PC_FALLTHRU_COUNTER);

	return 0;
}
EXPORT_SYMBOL(msm_pm_collapse);

static bool __ref msm_pm_spm_power_collapse(
	unsigned int cpu, int mode, bool from_idle, bool notify_rpm)
{
	void *entry;
	bool collapsed = 0;
	int ret;
	bool save_cpu_regs = (cpu_online(cpu) || from_idle);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: notify_rpm %d\n",
			cpu, __func__, (int) notify_rpm);

	ret = msm_spm_set_low_power_mode(mode, notify_rpm);
	WARN_ON(ret);

	entry = save_cpu_regs ?  cpu_resume : msm_secondary_startup;

	msm_pm_boot_config_before_pc(cpu, virt_to_phys(entry));

	if (MSM_PM_DEBUG_RESET_VECTOR & msm_pm_debug_mask)
		pr_info("CPU%u: %s: program vector to %p\n",
			cpu, __func__, entry);

	msm_jtag_save_state();

	collapsed = save_cpu_regs ?
		!__cpu_suspend(0, msm_pm_collapse) : msm_pm_pc_hotplug();

	msm_jtag_restore_state();

	if (collapsed)
		local_fiq_enable();

	msm_pm_boot_config_after_pc(cpu);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: msm_pm_collapse returned, collapsed %d\n",
			cpu, __func__, collapsed);

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);
	return collapsed;
}

static bool msm_pm_power_collapse_standalone(
		bool from_idle)
{
	unsigned int cpu = smp_processor_id();
	bool collapsed;

	collapsed = msm_pm_spm_power_collapse(cpu,
			MSM_SPM_MODE_STANDALONE_POWER_COLLAPSE,
			from_idle, false);

	return collapsed;
}

static int ramp_down_last_cpu(int cpu)
{
	struct clk *cpu_clk = per_cpu(cpu_clks, cpu);
	int ret = 0;

	clk_disable(cpu_clk);
	clk_disable(l2_clk);

	return ret;
}

static int ramp_up_first_cpu(int cpu, int saved_rate)
{
	struct clk *cpu_clk = per_cpu(cpu_clks, cpu);
	int rc = 0;

	if (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)
		pr_info("CPU%u: %s: restore clock rate\n",
				cpu, __func__);

	clk_enable(l2_clk);

	if (cpu_clk) {
		int ret = clk_enable(cpu_clk);

		if (ret) {
			pr_err("%s(): Error restoring cpu clk\n",
					__func__);
			return ret;
		}
	}

	return rc;
}

static bool msm_pm_power_collapse(bool from_idle)
{
	unsigned int cpu = smp_processor_id();
	unsigned long saved_acpuclk_rate = 0;
	bool collapsed;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: idle %d\n",
			cpu, __func__, (int)from_idle);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: pre power down\n", cpu, __func__);

	if (cpu_online(cpu) && !msm_no_ramp_down_pc)
		saved_acpuclk_rate = ramp_down_last_cpu(cpu);

	collapsed = msm_pm_spm_power_collapse(cpu, MSM_SPM_MODE_POWER_COLLAPSE,
			from_idle, true);

	if (cpu_online(cpu) && !msm_no_ramp_down_pc)
		ramp_up_first_cpu(cpu, saved_acpuclk_rate);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: post power up\n", cpu, __func__);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: return\n", cpu, __func__);
	return collapsed;
}
/******************************************************************************
 * External Idle/Suspend Functions
 *****************************************************************************/

void arch_idle(void)
{
	return;
}

static bool (*execute[MSM_PM_SLEEP_MODE_NR])(bool idle) = {
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = msm_pm_swfi,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] =
		msm_pm_power_collapse_standalone,
	[MSM_PM_SLEEP_MODE_RETENTION] = msm_pm_retention,
	[MSM_PM_SLEEP_MODE_FASTPC] = msm_pm_fastpc,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = msm_pm_power_collapse,
};
//zte jiangfeng add
#ifdef ZTE_GPIO_DEBUG

extern int msm_dump_gpios(struct seq_file *s, int curr_len, char *gpio_buffer);
extern int pmic_dump_pins(struct seq_file *s, int curr_len, char *gpio_buffer);
static char *gpio_sleep_status_info;

int print_gpio_buffer(struct seq_file *s)
{
	if (gpio_sleep_status_info)
		seq_printf(s, gpio_sleep_status_info);
	else
		seq_printf(s, "Device haven't suspended yet!\n");

	return 0;
}
EXPORT_SYMBOL(print_gpio_buffer);

int free_gpio_buffer(void)
{
	kfree(gpio_sleep_status_info);
	gpio_sleep_status_info = NULL;

	return 0;
}
EXPORT_SYMBOL(free_gpio_buffer);

#endif

#ifndef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
//#define ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
#endif

#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
static zte_smem_global*    zte_global  =   NULL;   //zte jiangfeng
#endif


#ifndef CONFIG_ZTE_PLATFORM_RECORD_APP_AWAKE_SUSPEND_TIME 
#define CONFIG_ZTE_PLATFORM_RECORD_APP_AWAKE_SUSPEND_TIME
#endif

#ifdef CONFIG_ZTE_PLATFORM_RECORD_APP_AWAKE_SUSPEND_TIME

#define MSM_PM_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & msm_pm_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)


long lateresume_2_earlysuspend_time_s = 0;		//LHX_PM_20110411_01 time to record how long it takes to earlysuspend after last resume. namely,to record how long the LCD keeps on.
void zte_update_lateresume_2_earlysuspend_time(bool resume_or_earlysuspend)	// LHX_PM_20110411_01 resume_or_earlysuspend? lateresume : earlysuspend
{
    pr_info("ZTE_PM turn LCD %s \n",resume_or_earlysuspend?"ON":"OFF");
	if(resume_or_earlysuspend)//lateresume,need to record when the lcd is turned on
	{
		lateresume_2_earlysuspend_time_s = current_kernel_time().tv_sec;
	}
	else	//earlysuspend,need to record when the lcd is turned off
	{
		lateresume_2_earlysuspend_time_s = current_kernel_time().tv_sec - lateresume_2_earlysuspend_time_s;	//record how long the lcd keeps on
	}
}

//#include <linux/io.h>
//#include <mach/boot_shared_imem_cookie.h>
//#include <mach/msm_iomap.h>
//#include "../../clk/qcom/clock.h"

#if 0
static ssize_t pm_monitor_amss_sleep_time_show(struct device *devp, struct device_attribute *attr, char *buf)
{
	char * echo = buf;	
	int msleeptimeinsecs=zte_imem_ptr->modemsleeptime;
	if(msleeptimeinsecs==-1)
		msleeptimeinsecs=0;
	
	echo += sprintf(echo, "%d", (int)(msleeptimeinsecs/100));
	
	return echo-buf;
}
static DEVICE_ATTR(amss_sleep_time, S_IRUGO, pm_monitor_amss_sleep_time_show, NULL);
#endif
unsigned pm_modem_sleep_time_get(void)
{
	if (!zte_imem_ptr)
	{
		pr_err("zte_imem_ptr is null,pm_modem_sleep_time_get return\n");
		return 0;
	}
	pr_info("ZTE_PM get modemsleeptime %d \n",zte_imem_ptr->modemsleeptime);
	return zte_imem_ptr->modemsleeptime;
}

unsigned pm_modem_phys_link_time_get(void)/////liukejing add 20130606 
{
	if (!zte_imem_ptr)
	{
		pr_err("zte_imem_ptr is null,pm_modem_phys_link_time_get return\n");
		return 0;
	}
	pr_info("ZTE_PM get physlinktime %d\n",zte_imem_ptr->physlinktime);
	return zte_imem_ptr->physlinktime;
}

unsigned pm_modem_awake_time_get(int *current_sleep)
{
	if (!zte_imem_ptr)
	{
		pr_err("zte_imem_ptr is null,pm_modem_awake_time_get return\n");
		return 0;
	}
	*current_sleep =  zte_imem_ptr->modemsleep_or_awake;
	pr_info("ZTE_PM get modemawaketime %d,current_sleep=%d \n",zte_imem_ptr->modemawaketime,*current_sleep);
	return zte_imem_ptr->modemawaketime;
}



/*BEGIN LHX_PM_20110324_01 add code to record how long the APP sleeps or keeps awake*/
#define AMSS_NEVER_ENTER_SLEEP 0x4//lianghouxing 20121119 add to indicate modem is sleep or awake,0 sleep, 1 awake,4 means never enter sleep yet
#define AMSS_NOW_SLEEP 0x0
#define AMSS_NOW_AWAKE 0x1
#define THRESOLD_FOR_OFFLINE_AWAKE_TIME 100 //ms,modem awake time is less than this,conside modem is set as offline.
#define THRESOLD_FOR_OFFLINE_TIME 5000 //5s

static int zte_enable2record_flag = 0;
module_param_named(zte_enable2record,
	zte_enable2record_flag, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define RECORED_TOTAL_TIME
struct timespec time_updated_when_sleep_awake;
void record_sleep_awake_time(bool record_sleep_awake)
{
	//record_sleep_awake?: true?record awake time, else record  sleep time
	struct timespec ts;
	unsigned time_updated_when_sleep_awake_s;
	#ifdef RECORED_TOTAL_TIME
	static bool record_firsttime = true;
	static bool record_firsttime_modem = true;
	static	unsigned time_modem_firsttime_awake_s=0;
	static	unsigned time_modem_firsttime_sleep_s =0;
	static	unsigned time_app_total_awake_s=0;
	static	unsigned time_app_total_sleep_s =0;
	static unsigned time_lcdon_total_s=0;
	#endif
	unsigned time_updated_when_sleep_awake_ms;
	unsigned time_updated_when_sleep_awake_ms_temp;
	static unsigned amss_sleep_time_ms = 0;
	static unsigned amss_physlink_current_total_time_s = 0;/////liukejing 20130606 add
	static unsigned amss_physlink_last_total_time_s = 0;/////liukejing 20130606 add
	unsigned amss_sleep_time_ms_temp = 0;
	unsigned deta_sleep_ms = 0;
	unsigned deta_awake_ms = 0;
	unsigned deta_physlink_s = 0;/////liukejing 20130606 add
//	bool offlinemode = false;
	unsigned amss_awake_last =0;
	
	unsigned amss_current_sleep_or_awake=0;//indicate modem is sleep or awake,1 means never enter sleep yet,2 sleep, 3 awake
	static unsigned  amss_current_sleep_or_awake_previous=0;
	
	static unsigned amss_awake_time_ms = 0;
	unsigned amss_awake_time_ms_temp = 0;
	bool get_amss_awake_ok = false;// true:get amss_awake time,false get amss_sleep_time

	unsigned percentage_amss_not_sleep_while_app_suspend = 0;	//the percentage of modem awake while app suspend in %o;
	static bool sleep_success_flag = false;  //set true while msm_pm_collapse returned 1 by passing record_sleep_awake as true;

//	if (!((MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE) & msm_pm_debug_mask) )
	//	return ;

if(0 ==zte_enable2record_flag)
{
	pr_info("[ZTE_PM]: not enable to record when app enter to suspend or resume yet! \n");
	return;
}
	ts = current_kernel_time();
	
	time_updated_when_sleep_awake_ms_temp =(unsigned) ((ts.tv_sec - time_updated_when_sleep_awake.tv_sec) * MSEC_PER_SEC + ((ts.tv_nsec / NSEC_PER_MSEC) - (time_updated_when_sleep_awake.tv_nsec / NSEC_PER_MSEC)));
	time_updated_when_sleep_awake_s = (time_updated_when_sleep_awake_ms_temp/MSEC_PER_SEC);
	time_updated_when_sleep_awake_ms = (time_updated_when_sleep_awake_ms_temp - time_updated_when_sleep_awake_s * MSEC_PER_SEC);

	if(record_sleep_awake)//record app awake time
	{
		sleep_success_flag = true;

		amss_sleep_time_ms_temp = amss_sleep_time_ms;	//backup previous total sleep time
		amss_sleep_time_ms  = pm_modem_sleep_time_get();	//get new total sleep time
		deta_sleep_ms = amss_sleep_time_ms - amss_sleep_time_ms_temp;
		//printk("  during this %s: modem sleep pre: %d  new %d s,modem sleep this time: %d ms\n",record_sleep_awake?"awake":"sleep",(int)amss_sleep_time_ms_temp ,amss_sleep_time_ms,deta_sleep_ms);

		amss_awake_time_ms_temp = amss_awake_time_ms;	//backup previous total sleep time
		amss_awake_time_ms  = pm_modem_awake_time_get(&amss_current_sleep_or_awake);	//get new total sleep time
		deta_awake_ms = amss_awake_time_ms - amss_awake_time_ms_temp;
		
		amss_physlink_current_total_time_s=pm_modem_phys_link_time_get();///liukejing 20130606 add to get total phys link time
		deta_physlink_s=amss_physlink_current_total_time_s - amss_physlink_last_total_time_s;
		//pr_info("print phys link time------awake------:amss_physlink_current_total_time_s %10d s amss_physlink_last_total_time_s %10d s\n", 
						//amss_physlink_current_total_time_s,amss_physlink_last_total_time_s);
		amss_physlink_last_total_time_s = amss_physlink_current_total_time_s;
//		printk("  during this %s: modem awake pre: %d  new %d s,modem awake this time: %d ms \n",record_sleep_awake?"awake":"sleep",(int)amss_awake_time_ms_temp ,amss_awake_time_ms,deta_awake_ms);
	//	printk(" modem total sleep: %d  ms,modem total awake: %d ms \n",amss_sleep_time_ms,amss_awake_time_ms);
/*
amss_current_sleep_or_awake_previous  amss_current_sleep_or_awake 
X 4 ---modem not enter sleep yet
0 0 ---previous is sleep,curret is sleep, modem awake time is updated,get awake deta directly.
otherwise get modem sleep time.
if modem is set to offline,print offline in the log
*/
		if((AMSS_NOW_SLEEP ==amss_current_sleep_or_awake_previous)&&(AMSS_NOW_SLEEP ==amss_current_sleep_or_awake))//00,get modem awake time
		{
			if((deta_awake_ms < THRESOLD_FOR_OFFLINE_AWAKE_TIME ) && (THRESOLD_FOR_OFFLINE_TIME < time_updated_when_sleep_awake_ms_temp))//if sleep time is 0 and awake is 0,offline mode
			{
				pr_info("[ZTE_PM] offline mode \n");
			}
			get_amss_awake_ok = true;
			amss_awake_last = deta_awake_ms;
		}
		else if(AMSS_NEVER_ENTER_SLEEP == amss_current_sleep_or_awake)
		{
			pr_info("[ZTE_PM] modem not enter sleep yet \n");
		}

		if(!get_amss_awake_ok)
		{
			amss_awake_last = time_updated_when_sleep_awake_ms_temp - deta_sleep_ms;
		}
		//printk("during this  %s: amss_awake_last: %d ms,deta_awake_ms %d ms,deta_sleep_ms this time %d ms\n",record_sleep_awake?"awake":"sleep",amss_awake_last,deta_awake_ms,deta_sleep_ms);
		percentage_amss_not_sleep_while_app_suspend = (amss_awake_last * 1000/(time_updated_when_sleep_awake_ms_temp + 1));

#ifdef RECORED_TOTAL_TIME
		if(!record_firsttime)
			{
				time_app_total_awake_s += time_updated_when_sleep_awake_s;
				time_lcdon_total_s += lateresume_2_earlysuspend_time_s;
			}
				record_firsttime = false;		
#endif
		pr_info("[ZTE_PM] APP wake for %6d.%03d s, lcd on for %6d s %3d %%,modem wake for %10d ms(%s) %4d %%o,modem sleep for %10d ----%d%d\n", 
			time_updated_when_sleep_awake_s,time_updated_when_sleep_awake_ms,(int)lateresume_2_earlysuspend_time_s,(int)(lateresume_2_earlysuspend_time_s*100/(time_updated_when_sleep_awake_s + 1))
			,amss_awake_last,get_amss_awake_ok?"get_directly ":"from sleep_time",percentage_amss_not_sleep_while_app_suspend,deta_sleep_ms,amss_current_sleep_or_awake_previous,amss_current_sleep_or_awake);//in case Division by zero, +1
		pr_info("[ZTE_PM] modem_phys_link_total_time %4d min %4d s,deta_physlink_s %4d min %4d s during app wake\n", 
				amss_physlink_current_total_time_s/60,amss_physlink_current_total_time_s%60,deta_physlink_s/60,deta_physlink_s%60);//liukejing 20130606 add to count phys link time

		time_updated_when_sleep_awake = ts; 
		lateresume_2_earlysuspend_time_s = 0;	//LHX_PM_20110411_01 clear how long the lcd keeps on

	}
	else	//record app sleep time
	{
		if(!sleep_success_flag) //only record sleep time while really resume after successfully suspend/sleep;
		{
			printk("[ZTE_PM] app resume due to fail to suspend\n");
			return;
		}
		sleep_success_flag = false;

		amss_sleep_time_ms_temp = amss_sleep_time_ms;	//backup previous total sleep time
//		amss_sleep_time_ms  = pm_modem_sleep_time_get() / MSEC_PER_SEC;	//get new total sleep time
		amss_sleep_time_ms  = pm_modem_sleep_time_get();	//get new total sleep time
		deta_sleep_ms = amss_sleep_time_ms - amss_sleep_time_ms_temp;
		//printk("  during this %s: modem sleep pre: %d  new %d s,modem sleep this time: %d ms\n",record_sleep_awake?"awake":"sleep",(int)amss_sleep_time_ms_temp ,amss_sleep_time_ms,deta_sleep_ms);
		amss_awake_time_ms_temp = amss_awake_time_ms;	//backup previous total sleep time
	//	amss_awake_time_ms  = pm_modem_awake_time_get(&amss_current_sleep_or_awake) / MSEC_PER_SEC;	//get new total sleep time
		amss_awake_time_ms  = pm_modem_awake_time_get(&amss_current_sleep_or_awake);	//get new total sleep time
		deta_awake_ms = amss_awake_time_ms - amss_awake_time_ms_temp;
		
		amss_physlink_current_total_time_s=pm_modem_phys_link_time_get();///liukejing 20130606 add to get total phys link time
		deta_physlink_s=amss_physlink_current_total_time_s - amss_physlink_last_total_time_s;
				//pr_info("print phys link time------sleep------:amss_physlink_current_total_time_s %10d s amss_physlink_last_total_time_s %10d s\n", 
						//amss_physlink_current_total_time_s,amss_physlink_last_total_time_s);
		amss_physlink_last_total_time_s = amss_physlink_current_total_time_s;
		//printk("  during this %s: modem awake pre: %d  new %d s,modem awake this time: %d ms \n",record_sleep_awake?"awake":"sleep",(int)amss_awake_time_ms_temp ,amss_awake_time_ms,deta_awake_ms);
		if((AMSS_NOW_SLEEP ==amss_current_sleep_or_awake_previous)&&(AMSS_NOW_SLEEP ==amss_current_sleep_or_awake))//00,get modem awake time
		{
			if((deta_awake_ms < THRESOLD_FOR_OFFLINE_AWAKE_TIME ) && (THRESOLD_FOR_OFFLINE_TIME < time_updated_when_sleep_awake_ms_temp))//if sleep time is 0 and awake is 0,offline mode
			{
				pr_info("[ZTE_PM] offline mode \n");
			}
			get_amss_awake_ok = true;
			amss_awake_last = deta_awake_ms;
		}
		else if(AMSS_NEVER_ENTER_SLEEP == amss_current_sleep_or_awake)
		{
			pr_info("[ZTE_PM] modem not enter sleep yet \n");
		}
		
		if(!get_amss_awake_ok)
		{
			amss_awake_last = time_updated_when_sleep_awake_ms_temp - deta_sleep_ms;
		}
#ifdef RECORED_TOTAL_TIME
		time_app_total_sleep_s += time_updated_when_sleep_awake_s;
		if(record_firsttime_modem)
		{
			time_modem_firsttime_awake_s =		amss_awake_last/1000;	
			time_modem_firsttime_sleep_s =		amss_sleep_time_ms/1000;	
			record_firsttime_modem= false;		
		}
		pr_info("[ZTE_PM] modem total sleep: %d	s,modem total awake: %d s app total sleep: %d	s,app total awake: %d s,lcdon total: %d s\n",(amss_sleep_time_ms/1000 - time_modem_firsttime_sleep_s),(amss_awake_time_ms/1000-time_modem_firsttime_awake_s),time_app_total_sleep_s,time_app_total_awake_s,time_lcdon_total_s);
#endif

        //zte_pm_20160222 check bug: disordered timestamp, and lost 2-hour kernel-log
        if(kernel_sleep_count > 10000){
            kernel_sleep_count = 1;
            pr_info("[ZTE_PM] init again,kernel_sleep_count=%d\n",kernel_sleep_count);
        }
        else{
            kernel_sleep_count = kernel_sleep_count+1;
            //pr_info("[ZTE_PM] kernel_sleep_count=%d\n",kernel_sleep_count);
            if( kernel_sleep_count%5 == 0){
                pr_info("[ZTE_PM] kernel_sleep_count=%d\n",kernel_sleep_count);
            }
        }
        //zte_pm_20160222 end

		percentage_amss_not_sleep_while_app_suspend = (amss_awake_last * 1000/(time_updated_when_sleep_awake_ms_temp + 1));
		pr_info("[ZTE_PM] APP sleep for %10d.%03d s, modem wake for %10d ms(%s) %4d %%o,modem_sleep for %10d ----%d%d\n",time_updated_when_sleep_awake_s,time_updated_when_sleep_awake_ms,amss_awake_last,get_amss_awake_ok?"get_directly ":"from sleep_time",percentage_amss_not_sleep_while_app_suspend,deta_sleep_ms,amss_current_sleep_or_awake_previous,amss_current_sleep_or_awake);
		pr_info("[ZTE_PM] modem_phys_link_total_time %4d min %4d s, deta_physlink_s %4d min %4d s during app sleep\n", 
				amss_physlink_current_total_time_s/60,amss_physlink_current_total_time_s%60,deta_physlink_s/60,deta_physlink_s%60);//liukejing 20130606 add to count phys link time
		time_updated_when_sleep_awake = ts; 
	}
	
	amss_current_sleep_or_awake_previous = amss_current_sleep_or_awake;
}
/*End LHX_PM_20110324_01 add code to record how long the APP sleeps or keeps awake*/
#endif

/*
zte_pm_before_powercollapse called before enter PowerCollapse from suspend,which will record app state ,and do gpio dump.
*/
void zte_pm_before_powercollapse(void)
{
#ifdef ZTE_GPIO_DEBUG
	int curr_len = 0;//Default not open the gpio dump,too much logs...

	do{
		if (MSM_PM_DEBUG_ZTE_LOGS & msm_pm_debug_mask) {
			if (gpio_sleep_status_info) {
				memset(gpio_sleep_status_info, 0,
					sizeof(*gpio_sleep_status_info));            
                
			} else {
				gpio_sleep_status_info = kmalloc(25000, GFP_KERNEL);
				if (!gpio_sleep_status_info) {
					pr_err("[PM] kmalloc memory failed in %s\n",__func__);
					break;
				}
			}

            //zte_pm 20160119 add for zte_gpio/dump_sleep_gpios
            //zte_notes: 1> echo 512 > sys/module/msm_pm/para*/debug_mask 2> let device sleep 3> cat dump_sleep_gpios
			curr_len = msm_dump_gpios(NULL, curr_len, gpio_sleep_status_info);
			curr_len = pmic_dump_pins(NULL, curr_len, gpio_sleep_status_info);
		}
	}
	while(0);
#endif

#if 1
#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
		//zte jiangfeng
		if(zte_global	==	NULL)
			zte_global= ioremap(ZTE_SMEM_LOG_GLOBAL_BASE,
				sizeof(zte_smem_global));
		if(zte_global)
			zte_global->app_suspend_state	=	0xAA;
		//zte jiangfeng, end
#endif

#ifdef CONFIG_ZTE_PLATFORM_RECORD_APP_AWAKE_SUSPEND_TIME
		//clock_debug_print_enabled();
		record_sleep_awake_time(true);//LHX_PM_20110324_01 add code to record how long the APP sleeps or keeps awake 
#endif		
}
/*
zte_pm_before_powercollapse called after exit PowerCollapse from suspend,which will inform modem app has exit suspend.
*/
void zte_pm_after_powercollapse(void)
{
#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
	if(zte_global)
		zte_global->app_suspend_state	=	0;		//zte jiangfeng
#endif
#endif
}
//zte jiangfeng add, end

/**
 * msm_cpu_pm_enter_sleep(): Enter a low power mode on current cpu
 *
 * @mode - sleep mode to enter
 * @from_idle - bool to indicate that the mode is exercised during idle/suspend
 *
 * returns none
 *
 * The code should be with interrupts disabled and on the core on which the
 * low power is to be executed.
 *
 */
bool msm_cpu_pm_enter_sleep(enum msm_pm_sleep_mode mode, bool from_idle)
{
	bool exit_stat = false;
	unsigned int cpu = smp_processor_id();

	if ((!from_idle  && cpu_online(cpu))
			|| (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask))
		pr_info("CPU%u:%s mode:%d during %s\n", cpu, __func__,
				mode, from_idle ? "idle" : "suspend");

	if (execute[mode])
		exit_stat = execute[mode](from_idle);

	return exit_stat;
}
bool zte_msm_cpu_pm_enter_sleep(enum msm_pm_sleep_mode mode, bool from_idle)
{
	bool exit_stat = false;
	unsigned int cpu = smp_processor_id();
	
	if ((!from_idle  && cpu_online(cpu))
			|| ((MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)&&cpu==0))
		pr_info("zte_CPU%u:%s mode:%d during %s\n", cpu, __func__,
				mode, from_idle ? "idle" : "suspend");

	if((MSM_PM_SLEEP_MODE_POWER_COLLAPSE ==mode) && (!from_idle)) //suspend -> PC,need to record
	{
		zte_pm_before_powercollapse();
	} else if ((MSM_PM_SLEEP_MODE_POWER_COLLAPSE ==mode) && (from_idle)) {
		if (MSM_PM_DEBUG_ZTE_IDLE_CLOCK & msm_pm_debug_mask)
			clock_debug_print_enabled();
	}

	if (execute[mode])
		exit_stat = execute[mode](from_idle);
	if((MSM_PM_SLEEP_MODE_POWER_COLLAPSE ==mode) && (!from_idle)) //exit PC from suspend ,need to record
	{
		zte_pm_after_powercollapse();
	}

	return exit_stat;
}

/**
 * msm_pm_wait_cpu_shutdown() - Wait for a core to be power collapsed during
 *				hotplug
 *
 * @ cpu - cpu to wait on.
 *
 * Blocking function call that waits on the core to be power collapsed. This
 * function is called from platform_cpu_die to ensure that a core is power
 * collapsed before sending the CPU_DEAD notification so the drivers could
 * remove the resource votes for this CPU(regulator and clock)
 */
int msm_pm_wait_cpu_shutdown(unsigned int cpu)
{
	int timeout = 0;

	if (!msm_pm_slp_sts)
		return 0;
	if (!msm_pm_slp_sts[cpu].base_addr)
		return 0;
	while (1) {
		/*
		 * Check for the SPM of the core being hotplugged to set
		 * its sleep state.The SPM sleep state indicates that the
		 * core has been power collapsed.
		 */
		int acc_sts = __raw_readl(msm_pm_slp_sts[cpu].base_addr);

		if (acc_sts & msm_pm_slp_sts[cpu].mask)
			return 0;

		udelay(100);
		/*
		 * Dump spm registers for debugging
		 */
		if (++timeout == 20) {
			msm_spm_dump_regs(cpu);
			__WARN_printf("CPU%u didn't collapse in 2ms, sleep status: 0x%x\n",
					cpu, acc_sts);
		}
	}

	return -EBUSY;
}

static void msm_pm_ack_retention_disable(void *data)
{
	/*
	 * This is a NULL function to ensure that the core has woken up
	 * and is safe to disable retention.
	 */
}
/**
 * msm_pm_enable_retention() - Disable/Enable retention on all cores
 * @enable: Enable/Disable retention
 *
 */
void msm_pm_enable_retention(bool enable)
{
	if (enable == msm_pm_ldo_retention_enabled)
		return;

	msm_pm_ldo_retention_enabled = enable;

	/*
	 * If retention is being disabled, wakeup all online core to ensure
	 * that it isn't executing retention. Offlined cores need not be woken
	 * up as they enter the deepest sleep mode, namely RPM assited power
	 * collapse
	 */
	if (!enable) {
		preempt_disable();
		smp_call_function_many(&retention_cpus,
				msm_pm_ack_retention_disable,
				NULL, true);
		preempt_enable();
	}
}
EXPORT_SYMBOL(msm_pm_enable_retention);

/**
 * msm_pm_retention_enabled() - Check if retention is enabled
 *
 * returns true if retention is enabled
 */
bool msm_pm_retention_enabled(void)
{
	return msm_pm_ldo_retention_enabled;
}
EXPORT_SYMBOL(msm_pm_retention_enabled);

static int msm_pm_snoc_client_probe(struct platform_device *pdev)
{
	int rc = 0;
	static struct msm_bus_scale_pdata *msm_pm_bus_pdata;
	static uint32_t msm_pm_bus_client;

	msm_pm_bus_pdata = msm_bus_cl_get_pdata(pdev);

	if (msm_pm_bus_pdata) {
		msm_pm_bus_client =
			msm_bus_scale_register_client(msm_pm_bus_pdata);

		if (!msm_pm_bus_client) {
			pr_err("%s: Failed to register SNOC client", __func__);
			rc = -ENXIO;
			goto snoc_cl_probe_done;
		}

		rc = msm_bus_scale_client_update_request(msm_pm_bus_client, 1);

		if (rc)
			pr_err("%s: Error setting bus rate", __func__);
	}

snoc_cl_probe_done:
	return rc;
}

static int msm_cpu_status_probe(struct platform_device *pdev)
{
	u32 cpu;
	int rc;

	if (!pdev | !pdev->dev.of_node)
		return -EFAULT;

	msm_pm_slp_sts = devm_kzalloc(&pdev->dev,
			sizeof(*msm_pm_slp_sts) * num_possible_cpus(),
			GFP_KERNEL);

	if (!msm_pm_slp_sts)
		return -ENOMEM;


	for_each_possible_cpu(cpu) {
		struct device_node *cpun, *node;
		char *key;

		cpun = of_get_cpu_node(cpu, NULL);

		if (!cpun) {
			__WARN();
			continue;
		}

		node = of_parse_phandle(cpun, "qcom,sleep-status", 0);
		if (!node)
			return -ENODEV;

		msm_pm_slp_sts[cpu].base_addr = of_iomap(node, 0);
		if (!msm_pm_slp_sts[cpu].base_addr) {
			pr_err("%s: Can't find base addr\n", __func__);
			return -ENODEV;
		}

		key = "qcom,sleep-status-mask";
		rc = of_property_read_u32(node, key, &msm_pm_slp_sts[cpu].mask);
		if (rc) {
			pr_err("%s: Can't find %s property\n", __func__, key);
			iounmap(msm_pm_slp_sts[cpu].base_addr);
			return rc;
		}
	}

	return 0;
};

static struct of_device_id msm_slp_sts_match_tbl[] = {
	{.compatible = "qcom,cpu-sleep-status"},
	{},
};

static struct platform_driver msm_cpu_status_driver = {
	.probe = msm_cpu_status_probe,
	.driver = {
		.name = "cpu_slp_status",
		.owner = THIS_MODULE,
		.of_match_table = msm_slp_sts_match_tbl,
	},
};

static struct of_device_id msm_snoc_clnt_match_tbl[] = {
	{.compatible = "qcom,pm-snoc-client"},
	{},
};

static struct platform_driver msm_cpu_pm_snoc_client_driver = {
	.probe = msm_pm_snoc_client_probe,
	.driver = {
		.name = "pm_snoc_client",
		.owner = THIS_MODULE,
		.of_match_table = msm_snoc_clnt_match_tbl,
	},
};

struct msm_pc_debug_counters_buffer {
	long *reg;
	u32 len;
	char buf[MAX_BUF_SIZE];
};

char *counter_name[MSM_PC_NUM_COUNTERS] = {
		"PC Entry Counter",
		"Warmboot Entry Counter",
		"PC Bailout Counter"
};

static int msm_pc_debug_counters_copy(
		struct msm_pc_debug_counters_buffer *data)
{
	int j;
	u32 stat;
	unsigned int cpu;
	unsigned int len;
	uint32_t cluster_id;
	uint32_t cpu_id;
	uint32_t offset;

	for_each_possible_cpu(cpu) {
		len = scnprintf(data->buf + data->len,
				sizeof(data->buf)-data->len,
				"CPU%d\n", cpu);
		cluster_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
		cpu_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);
		offset = (cluster_id * MAX_CPUS_PER_CLUSTER
				 * MSM_PC_NUM_COUNTERS)
				 + (cpu_id * MSM_PC_NUM_COUNTERS);

		data->len += len;

		for (j = 0; j < MSM_PC_NUM_COUNTERS - 1; j++) {
			stat = data->reg[offset + j];
			len = scnprintf(data->buf + data->len,
					 sizeof(data->buf) - data->len,
					"\t%s: %d", counter_name[j], stat);

			data->len += len;
		}
		len = scnprintf(data->buf + data->len,
			 sizeof(data->buf) - data->len,
			"\n");

		data->len += len;
	}

	return data->len;
}

static ssize_t msm_pc_debug_counters_file_read(struct file *file,
		char __user *bufu, size_t count, loff_t *ppos)
{
	struct msm_pc_debug_counters_buffer *data;

	data = file->private_data;

	if (!data)
		return -EINVAL;

	if (!bufu)
		return -EINVAL;

	if (!access_ok(VERIFY_WRITE, bufu, count))
		return -EFAULT;

	if (*ppos >= data->len && data->len == 0)
		data->len = msm_pc_debug_counters_copy(data);

	return simple_read_from_buffer(bufu, count, ppos,
			data->buf, data->len);
}

static int msm_pc_debug_counters_file_open(struct inode *inode,
		struct file *file)
{
	struct msm_pc_debug_counters_buffer *buf;


	if (!inode->i_private)
		return -EINVAL;

	file->private_data = kzalloc(
		sizeof(struct msm_pc_debug_counters_buffer), GFP_KERNEL);

	if (!file->private_data) {
		pr_err("%s: ERROR kmalloc failed to allocate %zu bytes\n",
		__func__, sizeof(struct msm_pc_debug_counters_buffer));

		return -ENOMEM;
	}

	buf = file->private_data;
	buf->reg = (long *)inode->i_private;

	return 0;
}

static int msm_pc_debug_counters_file_close(struct inode *inode,
		struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static const struct file_operations msm_pc_debug_counters_fops = {
	.open = msm_pc_debug_counters_file_open,
	.read = msm_pc_debug_counters_file_read,
	.release = msm_pc_debug_counters_file_close,
	.llseek = no_llseek,
};

static int msm_pm_clk_init(struct platform_device *pdev)
{
	bool synced_clocks;
	u32 cpu;
	char clk_name[] = "cpu??_clk";
	char *key;

	key = "qcom,saw-turns-off-pll";
	if (of_property_read_bool(pdev->dev.of_node, key))
		return 0;

	key = "qcom,synced-clocks";
	synced_clocks = of_property_read_bool(pdev->dev.of_node, key);

	for_each_possible_cpu(cpu) {
		struct clk *clk;
		snprintf(clk_name, sizeof(clk_name), "cpu%d_clk", cpu);
		clk = clk_get(&pdev->dev, clk_name);
		if (IS_ERR(clk)) {
			if (cpu && synced_clocks)
				return 0;
			else
				clk = NULL;
		}
		per_cpu(cpu_clks, cpu) = clk;
	}

	if (synced_clocks)
		return 0;

	l2_clk = clk_get(&pdev->dev, "l2_clk");
	if (IS_ERR(l2_clk))
		pr_warn("%s: Could not get l2_clk (-%ld)\n", __func__,
			PTR_ERR(l2_clk));

	return 0;
}

static int msm_cpu_pm_probe(struct platform_device *pdev)
{
	struct dentry *dent = NULL;
	struct resource *res = NULL;
	int ret = 0;
	void __iomem *msm_pc_debug_counters_imem;
	char *key;
	int alloc_size = (MAX_NUM_CLUSTER * MAX_CPUS_PER_CLUSTER
					* MSM_PC_NUM_COUNTERS
					* sizeof(*msm_pc_debug_counters));

	msm_pc_debug_counters = dma_alloc_coherent(&pdev->dev, alloc_size,
				&msm_pc_debug_counters_phys, GFP_KERNEL);

	if (msm_pc_debug_counters) {
		memset(msm_pc_debug_counters, 0, alloc_size);
		dent = debugfs_create_file("pc_debug_counter", S_IRUGO, NULL,
				msm_pc_debug_counters,
				&msm_pc_debug_counters_fops);
		if (!dent)
			pr_err("%s: ERROR debugfs_create_file failed\n",
					__func__);
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			goto skip_save_imem;
		msm_pc_debug_counters_imem = devm_ioremap(&pdev->dev,
						res->start, resource_size(res));
		if (msm_pc_debug_counters_imem) {
			writel_relaxed(msm_pc_debug_counters_phys,
					msm_pc_debug_counters_imem);
			mb();
			devm_iounmap(&pdev->dev,
					msm_pc_debug_counters_imem);
		}
	} else {
		msm_pc_debug_counters = 0;
		msm_pc_debug_counters_phys = 0;
	}
skip_save_imem:
	if (pdev->dev.of_node) {
		key = "qcom,tz-flushes-cache";
		msm_pm_tz_flushes_cache =
				of_property_read_bool(pdev->dev.of_node, key);

		key = "qcom,no-pll-switch-for-retention";
		msm_pm_ret_no_pll_switch =
				of_property_read_bool(pdev->dev.of_node, key);

		ret = msm_pm_clk_init(pdev);
		if (ret) {
			pr_info("msm_pm_clk_init returned error\n");
			return ret;
		}
	}
//zte_pm liyf 20151010
#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
	zte_global	=	ioremap(ZTE_SMEM_LOG_GLOBAL_BASE,
			sizeof(zte_smem_global));	//zte jiangfeng
	if(zte_global)
		zte_global->app_suspend_state	=	0;		//zte jiangfeng
#endif
#if 0
	zte_imem_ptr->modemsleeptime = 1000;
	zte_imem_ptr->modemawaketime = 2000;
	zte_imem_ptr->modemsleep_or_awake = 5;
#endif
#ifdef CONFIG_ZTE_BOOT_MODE
	/*
	 * Support for FTM & RECOVERY mode by ZTE_PM_LHX_20131202
	 *
	 * 0: Normal mode
	 * 1: FTM mode
	 */
	    if (socinfo_get_ftm_flag() == 1)
	    {
	        zte_enable2record_flag = 1;
			pr_info("ZTE_PM set zte_enable2record_flag to true in FTM mode");
	    }
#endif
//zte_pm end

	if (pdev->dev.of_node)
		of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);

	return ret;
}

static struct of_device_id msm_cpu_pm_table[] = {
	{.compatible = "qcom,pm"},
	{},
};

static struct platform_driver msm_cpu_pm_driver = {
	.probe = msm_cpu_pm_probe,
	.driver = {
		.name = "msm-pm",
		.owner = THIS_MODULE,
		.of_match_table = msm_cpu_pm_table,
	},
};

static int __init msm_pm_drv_init(void)
{
	int rc;

	cpumask_clear(&retention_cpus);

	rc = platform_driver_register(&msm_cpu_pm_snoc_client_driver);

	if (rc) {
		pr_err("%s(): failed to register driver %s\n", __func__,
				msm_cpu_pm_snoc_client_driver.driver.name);
		return rc;
	}

	return platform_driver_register(&msm_cpu_pm_driver);
}
late_initcall(msm_pm_drv_init);

int __init msm_pm_sleep_status_init(void)
{
	static bool registered;
    struct device_node *np; //ZTE_PM

	if (registered)
		return 0;
	registered = true;
//zte_pm liyf 20151010
     pr_info("%s: e\n", __func__);
     np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-pm-count-time");
     if (!np) {
          pr_err("unable to find DT imem msm-imem-pm-count-time node\n");
     }else {
          zte_imem_ptr =(pm_count_time  *)of_iomap(np, 0);
          if (!zte_imem_ptr)
             pr_err("unable to map imem golden copyoffset\n");
        }
//zte_pm end
	
	return platform_driver_register(&msm_cpu_status_driver);
}
arch_initcall(msm_pm_sleep_status_init);

#ifdef CONFIG_ARM
static int idle_initialize(void)
{
	arm_pm_idle = arch_idle;
	return 0;
}
early_initcall(idle_initialize);
#endif
