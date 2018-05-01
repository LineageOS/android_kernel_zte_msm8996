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

#ifndef __MFD_AKM_CORE_RESOURCE_H__
#define __MFD_AKM_CORE_RESOURCE_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/pm_qos.h>

#define AK49XX_NUM_IRQ_REGS 1
#define AK49XX_MAX_IRQ_REGS 1

enum {
	CODEC_AK4960_ID = 0,
	CODEC_AK4961_ID,
	CODEC_AK4962_ID,
};

enum {
	AK4960_IRQ_RCE = 0,
	AK4960_IRQ_JDE,
	AK4960_NUM_IRQS,
};

enum {
	AK4961_IRQ_RCE = 0,
	AK4961_IRQ_JDE,
	AK4961_IRQ_VAD,
	AK4961_NUM_IRQS,
};

enum {
	AK4962_IRQ_JDE = 0,
	AK4962_IRQ_IDE,
	AK4962_IRQ_MICE,
	AK4962_IRQ_SARE,
	AK4962_NUM_IRQS,
};

#define MAX(X, Y) (((int)X) >= ((int)Y) ? (X) : (Y))
#ifdef CONFIG_AK4961_CODEC
#define AK49XX_MAX_NUM_IRQS MAX(AK4960_NUM_IRQS, AK4961_NUM_IRQS)
#endif
#ifdef CONFIG_AK4962_CODEC
#define AK49XX_MAX_NUM_IRQS MAX(AK4962_NUM_IRQS, AK4961_NUM_IRQS)
#endif
#if 0 /*del for compile*/
struct intr_data {
	int intr_num;
	bool clear_first;
};
#endif

enum ak49xx_pm_state {
	AK49XX_PM_SLEEPABLE,
	AK49XX_PM_AWAKE,
	AK49XX_PM_ASLEEP,
};

enum ak49xx_intf_status {
	AK49XX_INTERFACE_TYPE_PROBING,
	AK49XX_INTERFACE_TYPE_SLIMBUS,
	AK49XX_INTERFACE_TYPE_I2C,
	AK49XX_INTERFACE_TYPE_SPI,
	AK49XX_INTERFACE_TYPE_SLIMBUS_SPI,
};

struct ak49xx_core_resource {
	struct mutex irq_lock;
	struct mutex nested_irq_lock;

	enum ak49xx_pm_state pm_state;
	struct mutex pm_lock;
	/* pm_wq notifies change of pm_state */
	wait_queue_head_t pm_wq;
	struct pm_qos_request pm_qos_req;
	int wlock_holders;


	/* holds the table of interrupts per codec */
	unsigned int irq_base;
	unsigned int irq;
	u8 irq_masks_cur[AK49XX_MAX_IRQ_REGS];
	u8 irq_masks_cache[AK49XX_MAX_IRQ_REGS];
	bool irq_level_high[AK49XX_MAX_NUM_IRQS];
	int num_irqs;
	int num_irq_regs;

	/* Callback functions to read/write codec registers */
	int (*codec_reg_read) (struct ak49xx_core_resource *,
				unsigned short);
	int (*codec_reg_write) (struct ak49xx_core_resource *,
				unsigned short, u8);
	int (*codec_bulk_read) (struct ak49xx_core_resource *,
				unsigned short, int, u8 *);
	int (*codec_bulk_write) (struct ak49xx_core_resource *,
				unsigned short, int, u8 *);

	/* Pointer to parent container data structure */
	void *parent;

	struct device *dev;
};

extern int ak49xx_core_res_init(
	struct ak49xx_core_resource*,
	int, int,
	int (*codec_read)(struct ak49xx_core_resource *, unsigned short),
	int (*codec_write)(struct ak49xx_core_resource *, unsigned short, u8),
	int (*codec_bulk_read) (struct ak49xx_core_resource *, unsigned short,
							int, u8 *),
	int (*codec_bulk_write) (struct ak49xx_core_resource *, unsigned short,
							int, u8 *));

extern void ak49xx_core_res_deinit(
	struct ak49xx_core_resource *);

extern int ak49xx_core_res_suspend(
	struct ak49xx_core_resource *,
	pm_message_t);

extern int ak49xx_core_res_resume(
	struct ak49xx_core_resource *);

extern int ak49xx_core_irq_init(
	struct ak49xx_core_resource*);

extern int ak49xx_initialize_irq(
	struct ak49xx_core_resource*,
	unsigned int,
	unsigned int);

enum ak49xx_intf_status ak49xx_get_intf_type(void);
void ak49xx_set_intf_type(enum ak49xx_intf_status);

bool ak49xx_lock_sleep(struct ak49xx_core_resource *);
void ak49xx_unlock_sleep(struct ak49xx_core_resource *);
void ak49xx_nested_irq_lock(struct ak49xx_core_resource *);
void ak49xx_nested_irq_unlock(struct ak49xx_core_resource *);
enum ak49xx_pm_state ak49xx_pm_cmpxchg(
			struct ak49xx_core_resource *,
			enum ak49xx_pm_state,
			enum ak49xx_pm_state);

int ak49xx_request_irq(struct ak49xx_core_resource *, int,
			irq_handler_t, const char *, void *);

void ak49xx_free_irq(struct ak49xx_core_resource *, int, void*);
void ak49xx_enable_irq(struct ak49xx_core_resource *, int);
void ak49xx_disable_irq(struct ak49xx_core_resource *, int);
void ak49xx_disable_irq_sync(struct ak49xx_core_resource *, int);
int ak49xx_reg_read(struct ak49xx_core_resource *,
					 unsigned short);
int ak49xx_reg_write(struct ak49xx_core_resource *,
					  unsigned short, u8);
int ak49xx_bulk_read(struct ak49xx_core_resource *,
					unsigned short, int, u8 *);
int ak49xx_bulk_write(struct ak49xx_core_resource*,
					 unsigned short, int, u8*);
int ak49xx_irq_init(struct ak49xx_core_resource *);
void ak49xx_irq_exit(struct ak49xx_core_resource *);
int ak49xx_core_res_resume(
	struct ak49xx_core_resource *);
int ak49xx_core_res_suspend(
	struct ak49xx_core_resource *,
	pm_message_t);
#endif
