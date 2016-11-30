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
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/qcom/socinfo.h>//zte_pm_20151126 for pv-hall

struct boot_stats {
	uint32_t bootloader_start;
	uint32_t bootloader_end;
	uint32_t bootloader_display;
	uint32_t bootloader_load_kernel;
};

static void __iomem *mpm_counter_base;
static uint32_t mpm_counter_freq;
static struct boot_stats __iomem *boot_stats;

static int mpm_parse_dt(void)
{
	struct device_node *np;
	u32 freq;

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-boot_stats");
	if (!np) {
		pr_err("can't find qcom,msm-imem node\n");
		return -ENODEV;
	}
	boot_stats = of_iomap(np, 0);
	if (!boot_stats) {
		pr_err("boot_stats: Can't map imem\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,mpm2-sleep-counter");
	if (!np) {
		pr_err("mpm_counter: can't find DT node\n");
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "clock-frequency", &freq))
		mpm_counter_freq = freq;
	else
		return -ENODEV;

	if (of_get_address(np, 0, NULL, NULL)) {
		mpm_counter_base = of_iomap(np, 0);
		if (!mpm_counter_base) {
			pr_err("mpm_counter: cant map counter base\n");
			return -ENODEV;
		}
	}

	return 0;
}

static void print_boot_stats(void)
{
	pr_info("KPI: Bootloader start count = %u\n",
		readl_relaxed(&boot_stats->bootloader_start));
	pr_info("KPI: Bootloader end count = %u\n",
		readl_relaxed(&boot_stats->bootloader_end));
	pr_info("KPI: Bootloader display count = %u\n",
		readl_relaxed(&boot_stats->bootloader_display));
	pr_info("KPI: Bootloader load kernel count = %u\n",
		readl_relaxed(&boot_stats->bootloader_load_kernel));
	pr_info("KPI: Kernel MPM timestamp = %u\n",
		readl_relaxed(mpm_counter_base));
	pr_info("KPI: Kernel MPM Clock frequency = %u\n",
		mpm_counter_freq);
}

int boot_stats_init(void)
{
	int ret;

	ret = mpm_parse_dt();
	if (ret < 0)
		return -ENODEV;

	print_boot_stats();

	iounmap(boot_stats);
	iounmap(mpm_counter_base);

	return 0;
}

//zte_pm_20151126 add for pv-hall,set pv flag here, get in zte-hall.c
#define SOCINFO_CMDLINE_PV_FLAG "androidboot.pv-version="
#define SOCINFO_CMDLINE_PV_VERSION   "1"
#define SOCINFO_CMDLINE_NON_PV_VERSION      "0"
static int __init zte_pv_flag_init(char *ver)
{
	int is_pv_ver = 0;

	if (!strncmp(ver, SOCINFO_CMDLINE_PV_VERSION, strlen(SOCINFO_CMDLINE_PV_VERSION)))
	{
		is_pv_ver = 1;
	}
       printk(KERN_ERR "pv flag: %d ", is_pv_ver);
	socinfo_set_pv_flag(is_pv_ver);
	return 0;
}
__setup(SOCINFO_CMDLINE_PV_FLAG, zte_pv_flag_init);

#define SOCINFO_CMDLINE_FP_HW               "androidboot.fingerprinthw="
#define SOCINFO_CMDLINE_FP_HW_SYNAFP        "synafp"
#define SOCINFO_CMDLINE_FP_HW_GOODIX        "goodix"
#define FINGERPRINT_HW_UNKOWN               -1
#define FINGERPRINT_HW_GOODIX               0
#define FINGERPRINT_HW_SYNAFP               1
static int __init zte_fingerprint_hw_init(char *ver)
{
	int fp_hw = FINGERPRINT_HW_UNKOWN;

	if (!strncmp(ver, SOCINFO_CMDLINE_FP_HW_SYNAFP, strlen(SOCINFO_CMDLINE_FP_HW_SYNAFP)))
	{
		fp_hw = FINGERPRINT_HW_SYNAFP;
	}
	else if (!strncmp(ver, SOCINFO_CMDLINE_FP_HW_GOODIX, strlen(SOCINFO_CMDLINE_FP_HW_GOODIX)))
	{
		fp_hw = FINGERPRINT_HW_GOODIX;
	}  
	printk(KERN_ERR "boot_stats fingerprint_hw: %d ", fp_hw);
	socinfo_set_fp_hw(fp_hw);
	return 0;
}
__setup(SOCINFO_CMDLINE_FP_HW, zte_fingerprint_hw_init);
//zte_pm_end