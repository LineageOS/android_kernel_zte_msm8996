/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ak49xx/core.h>
#include <linux/mfd/ak49xx/core-resource.h>
#include <linux/mfd/ak49xx/ak496x_registers.h>
#ifdef CONFIG_AK4960_CODEC
#include <linux/mfd/ak49xx/ak4960_registers.h>
#endif
#ifdef CONFIG_AK4961_CODEC
#include <linux/mfd/ak49xx/ak4961_registers.h>
#endif
#ifdef CONFIG_AK4962_CODEC
#include <linux/mfd/ak49xx/ak4962_registers.h>
#endif

#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <soc/qcom/pm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define BYTE_BIT_MASK(nr)		(1UL << ((nr) % BITS_PER_BYTE))
#define BIT_BYTE(nr)			((nr) / BITS_PER_BYTE)
#define AK49XX_SYSTEM_RESUME_TIMEOUT_MS 100

#ifndef NO_IRQ
#define NO_IRQ	(-1)
#endif

#ifdef CONFIG_OF
struct ak49xx_irq_drv_data {
	struct irq_domain *domain;
	int irq;
	/* ZTE_chenjun */
	struct pinctrl *pinctrl;
	struct pinctrl_state *cdc_int_cfg;
	int es804_rst_gpio;
	/*int es804_ldo_gpio;*/
};
#endif

/* ZTE_chenjun */
#if 0
extern int msm_tlmm_gp_info_get_ext(uint pin_no, unsigned int *func, unsigned int *out, unsigned int *value);
#endif

static struct ak49xx_irq_drv_data *
ak49xx_get_irq_drv_d(const struct ak49xx_core_resource *ak49xx_res);

struct ak49xx_irq {
	bool level;
};

#ifdef CONFIG_AK4961_CODEC
static struct ak49xx_irq ak49xx_irqs[AK49XX_MAX_NUM_IRQS] = {
	/* [0] = { .level = 1}, */
	/* All other ak49xx interrupts are edge triggered */
};
#endif
#ifdef CONFIG_AK4962_CODEC
static struct ak49xx_irq ak49xx_irqs[AK49XX_MAX_NUM_IRQS] = {
	/* [0] = { .level = 1}, */
	/* All other ak49xx interrupts are edge triggered */
};
#endif

static int virq_to_phyirq(
	struct ak49xx_core_resource *ak49xx_res, int virq);
static int phyirq_to_virq(
	struct ak49xx_core_resource *ak49xx_res, int irq);
static unsigned int ak49xx_irq_get_upstream_irq(
	struct ak49xx_core_resource *ak49xx_res);
static void ak49xx_irq_put_upstream_irq(
	struct ak49xx_core_resource *ak49xx_res);
static int ak49xx_map_irq(
	struct ak49xx_core_resource *ak49xx_res, int irq);

static void ak49xx_irq_lock(struct irq_data *data)
{
	struct ak49xx_core_resource *ak49xx_res =
			irq_data_get_irq_chip_data(data);
	mutex_lock(&ak49xx_res->irq_lock);
}

static void ak49xx_irq_sync_unlock(struct irq_data *data)
{
	struct ak49xx_core_resource *ak49xx_res =
			irq_data_get_irq_chip_data(data);
	mutex_unlock(&ak49xx_res->irq_lock);
}

static void ak49xx_irq_enable(struct irq_data *data)
{
	struct ak49xx_core_resource *ak49xx_res =
			irq_data_get_irq_chip_data(data);
	int ak49xx_irq = virq_to_phyirq(ak49xx_res, data->irq);
	ak49xx_res->irq_masks_cur[BIT_BYTE(ak49xx_irq)] &=
		~(BYTE_BIT_MASK(ak49xx_irq));
}

static void ak49xx_irq_disable(struct irq_data *data)
{
	struct ak49xx_core_resource *ak49xx_res =
			irq_data_get_irq_chip_data(data);
	int ak49xx_irq = virq_to_phyirq(ak49xx_res, data->irq);
	ak49xx_res->irq_masks_cur[BIT_BYTE(ak49xx_irq)]
			|= BYTE_BIT_MASK(ak49xx_irq);
}

static void ak49xx_irq_mask(struct irq_data *d)
{
	/* do nothing but required as linux calls irq_mask without NULL check */
}

static struct irq_chip ak49xx_irq_chip = {
	.name = "ak49xx",
	.irq_bus_lock = ak49xx_irq_lock,
	.irq_bus_sync_unlock = ak49xx_irq_sync_unlock,
	.irq_disable = ak49xx_irq_disable,
	.irq_enable = ak49xx_irq_enable,
	.irq_mask = ak49xx_irq_mask,
};

bool ak49xx_lock_sleep(
	struct ak49xx_core_resource *ak49xx_res)
{
	enum ak49xx_pm_state os;
/*
	mutex_lock(&ak49xx_res->pm_lock);
	if (ak49xx_res->wlock_holders++ == 0) {
		pr_debug("%s: holding wake lock\n", __func__);
		pm_qos_update_request(&ak49xx_res->pm_qos_req,
				      msm_cpuidle_get_deep_idle_latency());
	}
	mutex_unlock(&ak49xx_res->pm_lock);
*/
	if (!wait_event_timeout(ak49xx_res->pm_wq,
		((os =  ak49xx_pm_cmpxchg(ak49xx_res, AK49XX_PM_SLEEPABLE,
			AK49XX_PM_AWAKE)) ==
			AK49XX_PM_SLEEPABLE ||	(os == AK49XX_PM_AWAKE)),
			msecs_to_jiffies(AK49XX_SYSTEM_RESUME_TIMEOUT_MS))) {
		pr_warn("%s: system didn't resume within %dms, s %d\n",
			__func__,
			AK49XX_SYSTEM_RESUME_TIMEOUT_MS, ak49xx_res->pm_state);
	/* ak49xx_unlock_sleep(ak49xx_res); */
		wake_up_all(&ak49xx_res->pm_wq);
		return false;
	}
	/* wake_up_all(&ak49xx_res->pm_wq); */
	return true;
}
EXPORT_SYMBOL(ak49xx_lock_sleep);

void ak49xx_unlock_sleep(
	struct ak49xx_core_resource *ak49xx_res)
{
	mutex_lock(&ak49xx_res->pm_lock);
	/*
	if (--ak49xx_res->wlock_holders == 0) {
		pr_debug("%s: releasing wake lock pm_state %d -> %d\n",
			 __func__, ak49xx_res->pm_state, AK49XX_PM_SLEEPABLE);
	*/
	if (likely(ak49xx_res->pm_state == AK49XX_PM_AWAKE))
		ak49xx_res->pm_state = AK49XX_PM_SLEEPABLE;
	/* pm_qos_update_request(&ak49xx_res->pm_qos_req,
			PM_QOS_DEFAULT_VALUE);
	} */
	mutex_unlock(&ak49xx_res->pm_lock);
	wake_up_all(&ak49xx_res->pm_wq);
}
EXPORT_SYMBOL(ak49xx_unlock_sleep);

void ak49xx_nested_irq_lock(struct ak49xx_core_resource *ak49xx_res)
{
	mutex_lock(&ak49xx_res->nested_irq_lock);
}

void ak49xx_nested_irq_unlock(struct ak49xx_core_resource *ak49xx_res)
{
	mutex_unlock(&ak49xx_res->nested_irq_lock);
}

static void ak49xx_irq_dispatch(struct ak49xx_core_resource *ak49xx_res, int irqbit)
{
	ak49xx_nested_irq_lock(ak49xx_res);
	handle_nested_irq(phyirq_to_virq(ak49xx_res, irqbit));
	ak49xx_nested_irq_unlock(ak49xx_res);
}

static irqreturn_t ak49xx_irq_thread(int irq, void *data)
{
	int ret;
	struct ak49xx_core_resource *ak49xx_res = data;
	u8 status[AK49XX_NUM_IRQ_REGS];
	int i;

	if (unlikely(ak49xx_lock_sleep(ak49xx_res) == false)) {
/* ZTE_chenjun */
#if 0
		unsigned int func, out, value;
		struct ak49xx_irq_drv_data *data;

#define CODEC_INT_GPIO 72
	data = ak49xx_get_irq_drv_d(ak49xx_res);
	if (!data) {
		pr_warn("%s: not registered to interrupt controller\n",
			__func__);
	} else {
		msm_tlmm_gp_info_get_ext(CODEC_INT_GPIO, &func, &out, &value);
		if ((func != 0) || (out != 0)) {
			pr_err("%s:cdc int gpio err:func(%#X), out(%#X), value(%#X)\n", __func__, func, out, value);
			if (data->pinctrl) {
			ret = pinctrl_select_state(data->pinctrl, data->cdc_int_cfg);
				if (ret) {
					pr_err("%s(): error set cdc_int_cfg", __func__);
				} else {
					pr_err("%s(): set cdc_int_cfg OK", __func__);
				}
			}
		}
	}
#endif

		/* ret = ak49xx_res->codec_reg_write(ak49xx_res,DETECTION_EVENT_RESET, 0x01); */
		ret = ak49xx_res->codec_reg_read(ak49xx_res, JACK_DETECTION_EVENT);
		if (ret < 0) {
			pr_err("[LHS] %s , line%d , spi not ready !\n",__func__,__LINE__);
		} else {
			pr_err("[LHS] %s , line%d , spi is ready, JDS=%x!\n",__func__,__LINE__,ret);
		}

		dev_err(ak49xx_res->dev, "Failed to hold suspend\n");
		return IRQ_NONE;
	}

	ret = ak49xx_res->codec_reg_read(ak49xx_res, JACK_DETECTION_EVENT);
	if (ret < 0) {
		/* ak49xx_res->codec_reg_write(ak49xx_res,DETECTION_EVENT_RESET, 0x01); ZTE_chenjun */
		dev_err(ak49xx_res->dev, "[LHS]Failed to read interrupt status: %d\n", ret);
		ak49xx_unlock_sleep(ak49xx_res);
		return IRQ_NONE;
	} else {
		status[0] = ret;
	}

/* ZTE_chenjun */
	if (ret == 0) {
#ifdef CONFIG_AK4961_CODEC
	    ak49xx_res->codec_reg_write(ak49xx_res,DETECTION_EVENT_RESET, 0x01);
#endif
	    pr_err("[LHS] %s , line%d , no JDE at all !\n",__func__,__LINE__);
	    ak49xx_unlock_sleep(ak49xx_res);
	    return IRQ_NONE;
	}

	/* Apply masking */
	for (i = 0; i < AK49XX_NUM_IRQ_REGS; i++)
		status[i] &= ~ak49xx_res->irq_masks_cur[i];

	/* Find out which interrupt was triggered and call that interrupt's
	 * handler function
	 */
#ifdef CONFIG_AK4961_CODEC
	if (status[BIT_BYTE(AK4961_IRQ_JDE)] &
	    BYTE_BIT_MASK(AK4961_IRQ_JDE))
		ak49xx_irq_dispatch(ak49xx_res, AK4961_IRQ_JDE);

	if (status[BIT_BYTE(AK4961_IRQ_RCE)] &
		BYTE_BIT_MASK(AK4961_IRQ_RCE))
		ak49xx_irq_dispatch(ak49xx_res, AK4961_IRQ_RCE);

	if (status[BIT_BYTE(AK4961_IRQ_VAD)] &
		BYTE_BIT_MASK(AK4961_IRQ_VAD))
		ak49xx_irq_dispatch(ak49xx_res, AK4961_IRQ_VAD);
#endif
#ifdef CONFIG_AK4962_CODEC
	if (status[BIT_BYTE(AK4962_IRQ_JDE)] &
	    BYTE_BIT_MASK(AK4962_IRQ_JDE))
		ak49xx_irq_dispatch(ak49xx_res, AK4962_IRQ_JDE);

	if (status[BIT_BYTE(AK4962_IRQ_IDE)] &
		BYTE_BIT_MASK(AK4962_IRQ_IDE))
		ak49xx_irq_dispatch(ak49xx_res, AK4962_IRQ_IDE);

	if (status[BIT_BYTE(AK4962_IRQ_MICE)] &
		BYTE_BIT_MASK(AK4962_IRQ_MICE))
		ak49xx_irq_dispatch(ak49xx_res, AK4962_IRQ_MICE);

	if (status[BIT_BYTE(AK4962_IRQ_SARE)] &
		BYTE_BIT_MASK(AK4962_IRQ_SARE))
		ak49xx_irq_dispatch(ak49xx_res, AK4962_IRQ_SARE);
#endif
	ak49xx_unlock_sleep(ak49xx_res);
	return IRQ_HANDLED;
}

void ak49xx_free_irq(struct ak49xx_core_resource *ak49xx_res,
				int irq, void *data)
{
	free_irq(phyirq_to_virq(ak49xx_res, irq), data);
}

void ak49xx_enable_irq(struct ak49xx_core_resource *ak49xx_res, int irq)
{
	enable_irq(phyirq_to_virq(ak49xx_res, irq));
}

void ak49xx_disable_irq(struct ak49xx_core_resource *ak49xx_res, int irq)
{
	disable_irq_nosync(phyirq_to_virq(ak49xx_res, irq));
}

void ak49xx_disable_irq_sync(struct ak49xx_core_resource *ak49xx_res, int irq)
{
	disable_irq(phyirq_to_virq(ak49xx_res, irq));
}

int ak49xx_irq_init(struct ak49xx_core_resource *ak49xx_res)
{
	int ret;
	unsigned int i, virq;

	mutex_init(&ak49xx_res->irq_lock);
	mutex_init(&ak49xx_res->nested_irq_lock);

	ak49xx_res->irq = ak49xx_irq_get_upstream_irq(ak49xx_res);
	if (!ak49xx_res->irq) {
		pr_warn("%s: irq driver is not yet initialized\n", __func__);
		mutex_destroy(&ak49xx_res->irq_lock);
		mutex_destroy(&ak49xx_res->nested_irq_lock);
		return -EPROBE_DEFER;
	}
	pr_debug("%s: probed irq %d\n", __func__, ak49xx_res->irq);

	/* Mask the individual interrupt sources */
	for (i = 0; i < AK49XX_MAX_NUM_IRQS; i++) {
		/* Map OF irq */
		virq = ak49xx_map_irq(ak49xx_res, i);
		pr_debug("%s: irq %d -> %d\n", __func__, i, virq);
		if (virq == NO_IRQ) {
			pr_err("%s, No interrupt specifier for irq %d\n",
				   __func__, i);
			return NO_IRQ;
		}

		ret = irq_set_chip_data(virq, ak49xx_res);
		if (ret) {
			pr_err("%s: Failed to configure irq %d (%d)\n",
				   __func__, i, ret);
			return ret;
		}

		if (ak49xx_irqs[i].level)
			irq_set_chip_and_handler(virq, &ak49xx_irq_chip,
					 handle_level_irq);
		else
			irq_set_chip_and_handler(virq, &ak49xx_irq_chip,
					 handle_edge_irq);

		irq_set_nested_thread(virq, 1);

		ak49xx_res->irq_masks_cur[BIT_BYTE(i)] |= BYTE_BIT_MASK(i);
		ak49xx_res->irq_masks_cache[BIT_BYTE(i)] |= BYTE_BIT_MASK(i);
		ak49xx_res->irq_level_high[BIT_BYTE(i)] |= ak49xx_irqs[i].level <<
			(i % BITS_PER_BYTE);
	}
	for (i = 0; i < AK49XX_NUM_IRQ_REGS; i++) {
		/* Initialize interrupt mask and level registers */
	}

	ret = request_threaded_irq(ak49xx_res->irq, NULL, ak49xx_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"ak49xx", ak49xx_res);
	if (ret != 0)
		dev_err(ak49xx_res->dev, "Failed to request IRQ %d: %d\n",
			ak49xx_res->irq, ret);
	else {
		ret = enable_irq_wake(ak49xx_res->irq);
		if (ret)
			dev_err(ak49xx_res->dev, "Failed to set wake interrupt on"
				" IRQ %d: %d\n", ak49xx_res->irq, ret);
		if (ret)
			free_irq(ak49xx_res->irq, ak49xx_res);
	}

	if (ret) {
		pr_err("%s: Failed to init ak49xx irq\n", __func__);
		ak49xx_irq_put_upstream_irq(ak49xx_res);
		mutex_destroy(&ak49xx_res->irq_lock);
		mutex_destroy(&ak49xx_res->nested_irq_lock);
	}

	return ret;
}

int ak49xx_request_irq(struct ak49xx_core_resource *ak49xx_res,
		int irq, irq_handler_t handler,
		const char *name, void *data)
{
	int virq;

	virq = phyirq_to_virq(ak49xx_res, irq);

	pr_debug("%s: virq = %d\n", __func__, virq);
	/*
	 * ARM needs us to explicitly flag the IRQ as valid
	 * and will set them noprobe when we do so.
	 */
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	set_irq_flags(virq, IRQF_VALID);
#else
	set_irq_noprobe(virq);
#endif

	return request_threaded_irq(virq, NULL, handler, IRQF_TRIGGER_RISING,
			name, data);
}

void ak49xx_irq_exit(struct ak49xx_core_resource *ak49xx_res)
{
	if (ak49xx_res->irq) {
		disable_irq_wake(ak49xx_res->irq);
		free_irq(ak49xx_res->irq, ak49xx_res);
		/* Release parent's of node */
		ak49xx_irq_put_upstream_irq(ak49xx_res);
	}
	mutex_destroy(&ak49xx_res->irq_lock);
	mutex_destroy(&ak49xx_res->nested_irq_lock);
}

#ifndef CONFIG_OF
static int phyirq_to_virq(
	struct ak49xx_core_resource *ak49xx_res,
	int offset)
{
	return ak49xx_res->irq_base + offset;
}

static int virq_to_phyirq(
	struct ak49xx_core_resource *ak49xx_res,
	int virq)
{
	return virq - ak49xx_res->irq_base;
}

static unsigned int ak49xx_irq_get_upstream_irq(
	struct ak49xx_core_resource *ak49xx_res)
{
	return ak49xx_res->irq;
}

static void ak49xx_irq_put_upstream_irq(
	struct ak49xx_core_resource *ak49xx_res)
{
	/* Do nothing */
}

static int ak49xx_map_irq(
	struct ak49xx_core_resource *ak49xx_res, int irq)
{
	return phyirq_to_virq(ak49xx_res, irq);
}
#else
int __init ak49xx_irq_of_init(struct device_node *node,
		struct device_node *parent)
{
	struct ak49xx_irq_drv_data *data;

	pr_debug("%s: node %s, node parent %s\n", __func__,
		 node->name, node->parent->name);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * ak49xx_intc interrupt controller supports N to N irq mapping with
	 * single cell binding with irq numbers(offsets) only.
	 * Use irq_domain_simple_ops that has irq_domain_simple_map and
	 * irq_domain_xlate_onetwocell.
	 */
	data->domain = irq_domain_add_linear(node, AK49XX_MAX_NUM_IRQS,
					     &irq_domain_simple_ops, data);
	if (!data->domain) {
		kfree(data);
		return -ENOMEM;
	}

	return 0;
}

static struct ak49xx_irq_drv_data *
ak49xx_get_irq_drv_d(const struct ak49xx_core_resource *ak49xx_res)
{
	struct device_node *pnode;
	struct irq_domain *domain;

	pnode = of_irq_find_parent(ak49xx_res->dev->of_node);
	/* Shouldn't happen */
	if (unlikely(!pnode))
		return NULL;

	domain = irq_find_host(pnode);
	if (unlikely(!domain))
		return NULL;

	return (struct ak49xx_irq_drv_data *)domain->host_data;
}

static int phyirq_to_virq(struct ak49xx_core_resource *ak49xx_res, int offset)
{
	struct ak49xx_irq_drv_data *data;

	data = ak49xx_get_irq_drv_d(ak49xx_res);
	if (!data) {
		pr_warn("%s: not registered to interrupt controller\n", __func__);
		return -EINVAL;
	}
	return irq_linear_revmap(data->domain, offset);
}

static int virq_to_phyirq(struct ak49xx_core_resource *ak49xx_res, int virq)
{
	struct irq_data *irq_data = irq_get_irq_data(virq);
	if (unlikely(!irq_data)) {
		pr_err("%s: irq_data is NULL", __func__);
		return -EINVAL;
	}
	return irq_data->hwirq;
}

static unsigned int ak49xx_irq_get_upstream_irq(
		struct ak49xx_core_resource *ak49xx_res)
{
	struct ak49xx_irq_drv_data *data;

	/* Hold parent's of node */
	if (!of_node_get(of_irq_find_parent(ak49xx_res->dev->of_node)))
		return -EINVAL;

	data = ak49xx_get_irq_drv_d(ak49xx_res);
	if (!data) {
		pr_err("%s: interrupt controller is not registerd\n", __func__);
		return 0;
	}

	rmb();
	return data->irq;
}

static void ak49xx_irq_put_upstream_irq(
		struct ak49xx_core_resource *ak49xx_res)
{
	/* Hold parent's of node */
	of_node_put(of_irq_find_parent(ak49xx_res->dev->of_node));
}

static int ak49xx_map_irq(struct ak49xx_core_resource *ak49xx_res, int irq)
{
	return of_irq_to_resource(ak49xx_res->dev->of_node, irq, NULL);
}
/* ZTE_chenjun */
static void ak49xx_irq_get_pinctrl_configs(struct device *dev, struct ak49xx_irq_drv_data *data)
{
	struct pinctrl_state *set_state;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		pr_err("%s(): Pinctrl not defined", __func__);
	} else {
		pr_err("%s(): Using Pinctrl", __func__);

		set_state = pinctrl_lookup_state(data->pinctrl, "cdc_int_cfg");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for cdc_int_cfg");
			goto pinctrl_fail;
		}

		pr_err("%s(): Pinctrl cdc_int_cfg %p\n", __func__,
			set_state);
		data->cdc_int_cfg = set_state;

		return;
	}

pinctrl_fail:
	data->pinctrl = NULL;
	return;
}

/* ZTE_chenjun */
#if 0
static int ak49xx_irq_probe(struct platform_device *pdev)
{
	int irq;
	struct irq_domain *domain;
	struct ak49xx_irq_drv_data *data;
	int ret = -EINVAL;

	irq = platform_get_irq_byname(pdev, "cdc-int");
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: Couldn't find cdc-int node(%d)\n", __func__, irq);
		return -EINVAL;
	} else {
		dev_dbg(&pdev->dev, "%s: virq = %d\n", __func__, irq);
		domain = irq_find_host(pdev->dev.of_node);
		if (unlikely(!domain)) {
			pr_err("%s: domain is NULL", __func__);
			return -EINVAL;
		}
		data = (struct ak49xx_irq_drv_data *)domain->host_data;
		data->irq = irq;
		ak49xx_irq_get_pinctrl_configs(&pdev->dev, data);
		wmb();
		ret = 0;
	}

	return ret;
}
#else
/* ZTE_chenjun: for MSM8996-M */
static int ak49xx_irq_probe(struct platform_device *pdev)
{
	int irq;
	struct irq_domain *domain;
	struct ak49xx_irq_drv_data *data;
	struct device_node *node = pdev->dev.of_node;
	int ret = -EINVAL;

	pr_err("%s: Enter", __func__);

	irq = of_get_named_gpio(node, "qcom,gpio-connect", 0);
	if (!gpio_is_valid(irq)) {
		dev_err(&pdev->dev, "TLMM connect gpio not found\n");
		return -EPROBE_DEFER;
	} else {
		irq = gpio_to_irq(irq);
		if (irq < 0) {
			dev_err(&pdev->dev, "Unable to configure irq\n");
			return irq;
		}
		dev_err(&pdev->dev, "%s: virq = %d\n", __func__, irq);
		domain = irq_find_host(pdev->dev.of_node);
		if (unlikely(!domain)) {
			pr_err("%s: domain is NULL", __func__);
			return -EINVAL;
		}
		data = (struct ak49xx_irq_drv_data *)domain->host_data;
		data->irq = irq;
#if 0
        data->es804_rst_gpio= of_get_named_gpio(pdev->dev.of_node,
				"es804,cdc-reset-gpio", 0);
		pr_err("%s data->es804_rst_gpio : %d\n", __func__, data->es804_rst_gpio);
		if (data->es804_rst_gpio < 0) {
			dev_err(&pdev->dev, "Looking up %s property in node %s failed %d\n",
					"es804,cdc-reset-gpio", pdev->dev.of_node->full_name,
					data->es804_rst_gpio);
		}
#endif
#if 0
      	data->es804_ldo_gpio= of_get_named_gpio(pdev->dev.of_node,
				"es804,cdc-ldo-gpio", 0);
		pr_err("%s data->es804_ldo_gpio : %d\n", __func__, data->es804_ldo_gpio);
		if (data->es804_ldo_gpio < 0) {
			dev_err(&pdev->dev, "Looking up %s property in node %s failed %d\n",
					"es804,cdc-ldo-gpio", pdev->dev.of_node->full_name,
					data->es804_ldo_gpio);
		}
#endif

		ak49xx_irq_get_pinctrl_configs(&pdev->dev, data);

#if 0
		if (data->es804_rst_gpio > 0) {
			ret = gpio_request(data->es804_rst_gpio, "ES804_RST");
			if (ret) {
				pr_err("%s: Failed to request gpio %d\n", __func__, data->es804_rst_gpio);
				}
			}
		if (data->es804_rst_gpio > 0) {
			gpio_direction_output(data->es804_rst_gpio, 1);
			pr_err("%s(): 2: es804_rst_gpio(%d) value(%d)",
					__func__, data->es804_rst_gpio, gpio_get_value_cansleep(data->es804_rst_gpio));
			}
#endif

#if 0
		if (data->es804_ldo_gpio > 0) {
			ret = gpio_request(data->es804_ldo_gpio, "ES804_LDO");
			if (ret) {
				pr_err("%s: Failed to request gpio %d\n", __func__, data->es804_ldo_gpio);
				}
			}
		if (data->es804_ldo_gpio > 0) {
			gpio_direction_output(data->es804_ldo_gpio, 1);
			pr_err("%s(): 2: es804_ldo_gpio(%d) value(%d)",
					__func__, data->es804_ldo_gpio, gpio_get_value_cansleep(data->es804_ldo_gpio));
		}
#endif

		wmb();
		ret = 0;
	}

	return ret;
}
#endif

static int ak49xx_irq_remove(struct platform_device *pdev)
{
	struct irq_domain *domain;
	struct ak49xx_irq_drv_data *data;

	domain = irq_find_host(pdev->dev.of_node);
	if (unlikely(!domain)) {
		pr_err("%s: domain is NULL", __func__);
		return -EINVAL;
	}
	data = (struct ak49xx_irq_drv_data *)domain->host_data;
	data->irq = 0;
	wmb();

	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "akm,ak49xx-irq" },
	{ }
};

static struct platform_driver ak49xx_irq_driver = {
	.probe = ak49xx_irq_probe,
	.remove = ak49xx_irq_remove,
	.driver = {
		.name = "ak49xx_intc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_match),
	},
};

static int ak49xx_irq_drv_init(void)
{
	return platform_driver_register(&ak49xx_irq_driver);
}
subsys_initcall(ak49xx_irq_drv_init);

static void ak49xx_irq_drv_exit(void)
{
	platform_driver_unregister(&ak49xx_irq_driver);
}
module_exit(ak49xx_irq_drv_exit);
#endif /* CONFIG_OF */
