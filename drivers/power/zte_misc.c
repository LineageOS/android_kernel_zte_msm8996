/*
 * Driver for zte misc functions
 * function1: used for translate hardware GPIO to SYS GPIO number
 * function2: update fingerprint status to kernel from fingerprintd,2016/01/18
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>//for of_iomap()
#include <linux/delay.h>
#include <linux/power_supply.h>//for charging eanble/disable conrotl

int smb1351_is_good = 0;
module_param(smb1351_is_good, int, 0644);

struct zte_gpio_info {
	int sys_num;			//system pin number
	const char *name;
};

#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_gpios[MAX_SUPPORT_GPIOS];

static struct of_device_id zte_misc_of_match[] = {
	{ .compatible = "zte-misc", },
	{ },
};
MODULE_DEVICE_TABLE(of, zte_misc_of_match);

int get_sysnumber_byname(char* name)
{
	int i;
	for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
		if (zte_gpios[i].name) {
			if (!strcmp(zte_gpios[i].name,name)) 
				return zte_gpios[i].sys_num;	
		}
	}
	return -1;
}

static int get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	int count = -1;
	pr_info("zte_misc: translate hardware pin to system pin\n");
	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (!of_find_property(pp, "label", NULL)) {
			dev_warn(dev, "Found without labels\n");
			continue;
		}
		count++;
		zte_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
								GFP_KERNEL);
		zte_gpios[count].sys_num = of_get_gpio(pp, 0);
		
		pr_info("zte_misc: sys_number=%d name=%s\n",zte_gpios[count].sys_num,zte_gpios[count].name);
	}
	return 0;
}
/*battery switch functions*/
#if 0
static int poweroff_bs = 0;
static int factory_mode = 0;
static int is_factory_mode = 0;
#define BATTERY_SWITCH 		"battery_switch"
static int gpio_batt_switch_en = -1;

static int zte_batt_switch_init(void)
{
	int rc;
	
	gpio_batt_switch_en = get_sysnumber_byname(BATTERY_SWITCH);

	rc=gpio_request(gpio_batt_switch_en, BATTERY_SWITCH);
	if (rc) {
    	pr_info("unable to request battery_switch(%d)\n",rc);
    } else {
		pr_info("success\n");
	}
	//gpio_direction_output(gpio_batt_switch_en,0);
	
	return 0;
}

/*
 *hardware will poweroff after 7.3second automatically when pin from
 *low to high (rising edge triggered) and holding it high for at least 1 ms
 */
static int poweroff_bs_set(const char *val, struct kernel_param *kp)
{
	int ret;
    
    if (gpio_batt_switch_en == -1) {
        pr_info("%s:not support battery sitch",__func__); 
        return -1;
    }

	ret = param_set_int(val, kp);

	if (ret)
		return ret;
        
	if (poweroff_bs != 1)
		return -1;
	pr_info("%s:switch down",__func__);
	gpio_direction_output(gpio_batt_switch_en,0);
	msleep(10);
	gpio_direction_output(gpio_batt_switch_en,1);
	return 0;
}
module_param_call(poweroff_bs, poweroff_bs_set, NULL,
			&poweroff_bs, 0644);

static int factory_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;
	
	if (factory_mode == 0)
		is_factory_mode = 0;
	else
		is_factory_mode = 1;

	return 0;
}

static int factory_mode_get(char *buffer, struct kernel_param *kp)
{
	return	sprintf(buffer,"%d",is_factory_mode);
}
module_param_call(factory_mode, factory_mode_set, factory_mode_get,
			&factory_mode, 0644);

int battery_switch_enable(void)
{
    if (gpio_batt_switch_en == -1) {
        pr_info("%s:not support battery sitch",__func__); 
        return -1;
    }
    
	if (!is_factory_mode)
		return -1;

	gpio_direction_output(gpio_batt_switch_en,0);
	mdelay(10);
	gpio_direction_output(gpio_batt_switch_en,1);
	mdelay(10);
	return 0;
}
EXPORT_SYMBOL(battery_switch_enable);
#endif
/*
  *Emode function to enable/disable 0% shutdown
  */
int enable_to_shutdown = 1;
static int set_enable_to_shutdown(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_warn("set_enable_to_shutdown to %d\n", enable_to_shutdown);
	return 0;
}

module_param_call(enable_to_shutdown, set_enable_to_shutdown, param_get_uint,
					&enable_to_shutdown, 0644);

static int zte_misc_charging_enabled;//defined in ****-charger.c
static int zte_misc_control_charging(const char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	const union power_supply_propval enable = {1,};
	const union power_supply_propval disable = {0,};

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		if (zte_misc_charging_enabled != 0) {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &enable);
			pr_info("%s: enable charging\n",__func__);
		} else {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &disable);
			pr_info("%s: disable charging\n",__func__);
		}
		if (rc) {
			pr_err("battery does not export CHARGING_ENABLED: %d\n", rc);
		}
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return 0;
}
static int zte_misc_get_charging_enabled_node(char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	union power_supply_propval pval = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		rc = batt_psy->get_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
			pr_info("%s: enable charging status %d\n",__func__,pval.intval);
		if (rc) {
			pr_err("battery charging_enabled node is not exist: %d\n", rc);
            return	sprintf(val,"%d",-1);
		}
        zte_misc_charging_enabled = pval.intval;
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return	sprintf(val,"%d",pval.intval);
}

module_param_call(charging_enabled, zte_misc_control_charging, zte_misc_get_charging_enabled_node,
					&zte_misc_charging_enabled, 0644);


/*************************************************
haptics detector
bit(0): is_haptics 1=haptics 0=normal
**************************************************/
static uint32_t *pm_parameters = NULL;
static uint32_t is_haptics = 0;
module_param(is_haptics, uint, 0644);
static void zte_misc_parse_imem(void)
{
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-pm_parameter");
	if (!np) {
		pr_err("unable to find DT imem msm-imem-pm_parameter node\n");
	}else {
		pm_parameters =(uint32_t *)of_iomap(np, 0);
		if (!pm_parameters)
			pr_err("unable to map imem [pm_parameter]\n");
		else
			is_haptics = (*pm_parameters) & 0x1;
	}
	pr_info("is_haptics=%d\n", is_haptics);
}

bool is_haptics_zte(void)
{
    return 1;  //ZTE  msm8996 all haptic
    //return (is_haptics==1);
}

/*************************************************
fingerprint id detector
In MSM8996, Goodix and Synaptics.
Goodix id pin NO PULL;
Synaptics id pin PULL UP.
set id pin to pull up, then check the pin status:
0 for Goodix, 1 for Synaptics
**************************************************/
#if 0
struct pinctrl *fp_id_pinctrl = NULL;
struct pinctrl_state *fp_id_state = NULL;
#endif

#define FINGERPRINT_HW_UNKOWN    -1
#define FINGERPRINT_HW_GOODIX      0
#define FINGERPRINT_HW_SYNAFP      1
static uint32_t fingerprint_hw = FINGERPRINT_HW_UNKOWN;
module_param(fingerprint_hw, uint, 0644);

#if 0
static int set_fp_id_state(struct device *dev){
    int error = 0;

    fp_id_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(fp_id_pinctrl)) {
        pr_info("Can not get fp_id_pinctrl\n");
        error = PTR_ERR(fp_id_pinctrl);
        return error;
    }
    fp_id_state = pinctrl_lookup_state(fp_id_pinctrl, "ztemisc_fpid_active");
    if (IS_ERR_OR_NULL(fp_id_state)) {
        pr_info("Can not get fp_id_state\n");
        error = PTR_ERR(fp_id_state);
        return error;
    }

    error = pinctrl_select_state(fp_id_pinctrl, fp_id_state);
    if (error) {
        pr_info("can not set fp_id pins to ztemisc_fpid_active states\n");
    } else {
        pr_info("set_fp_id_state success.\n");
    }
    return error;
}

static int check_fp_id(struct device *dev){
    int rc = 0;
    int fp_id_val = -1;
    int pwr_gpio = 0;
    int id_gpio = 0;

    //prs pwr dts
    pwr_gpio = of_get_named_gpio(dev->of_node,"zte,fingerprint_gpio_pwr",0);
    if(!gpio_is_valid(pwr_gpio)) {
        pr_info("PWR GPIO is invalid.\n");
        return -1;
    }

    rc = gpio_request(pwr_gpio, "ztemisc_gpiopwr");
    if(rc) {
        pr_info("Failed to request PWR GPIO. rc = %d\n", rc);
        goto fail;
    }
    //power on fingerpint hw
    gpio_direction_output(pwr_gpio, 1);
    msleep(10);

    //chang to id gpio state to pull up
    rc = set_fp_id_state(dev);
    if(rc<0){
        printk("set_fp_id_state failed.\n");
        goto fail;
    }

    id_gpio = of_get_named_gpio(dev->of_node,"zte,fingerprint_gpio_id",0);
    if(!gpio_is_valid(id_gpio)) {
        pr_info("get fp_id  failed.\n");
        goto fail;
    }

    rc = gpio_request(id_gpio, "ztemisc_gpiopid");
    if(rc) {
        pr_info("Failed to request fp_id GPIO. rc = %d\n", rc);
        goto fail;
    }
    gpio_direction_input(id_gpio);
    fp_id_val = gpio_get_value(id_gpio);
    pr_info("fp_id_val=%d \n",fp_id_val);

    gpio_free(pwr_gpio);
    return fp_id_val;
fail:
    gpio_free(pwr_gpio);
    return -1;
}
#endif

extern int socinfo_get_fp_hw(void);
static void zte_misc_fingerprint_hw_check(struct device *dev)
{
    struct device_node *np;
    uint32_t *buf = NULL;
    uint32_t board_id = 0;
    int force_hw = socinfo_get_fp_hw();

    np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-board-id");
    if (!np) {
        pr_err("unable to find DT imem qcom,msm-imem-board-id\n");
    }else {
        buf =(uint32_t *)of_iomap(np, 0);
        if (!buf)
            pr_err("unable to map imem [board-id]\n");
        else
            board_id = *buf;
    }
    pr_info("board_id=%d\n", board_id);
#if 0
    fingerprint_hw = check_fp_id(dev);
    pr_info("fingerprint_hw=%d\n", fingerprint_hw);
#else
    if (board_id < 3)
        fingerprint_hw = FINGERPRINT_HW_SYNAFP;
    else
        fingerprint_hw = FINGERPRINT_HW_GOODIX;

    if (force_hw != FINGERPRINT_HW_UNKOWN ) {
        fingerprint_hw = force_hw;
        pr_info("fingerprint hw force set to %d %s\n",fingerprint_hw, (fingerprint_hw==FINGERPRINT_HW_SYNAFP)?"SYNAFP":"GOODIX");
    }
    pr_info("fingerprint_hw=%d %s\n", fingerprint_hw, (fingerprint_hw==FINGERPRINT_HW_SYNAFP)?"SYNAFP":"GOODIX");
#endif
}

bool is_goodix_fp(void)
{
    return (fingerprint_hw == FINGERPRINT_HW_GOODIX);
}

bool is_synafp_fp(void)
{
    return (fingerprint_hw == FINGERPRINT_HW_SYNAFP);
}

/*************************************************
 * Fingerprint Status Updater
 *************************************************/
//The same as the definitions in ap\hardware\libhardware\include\hardware\Fingerprint.h
typedef enum fingerprint_msg_type {
    FINGERPRINT_ERROR = -1,
    FINGERPRINT_ACQUIRED = 1,
    FINGERPRINT_TEMPLATE_ENROLLING = 3,
    FINGERPRINT_TEMPLATE_REMOVED = 4,
    FINGERPRINT_AUTHENTICATED = 5,
    FINGERPRINT_DETECTED = 6,
    FINGERPRINT_REMOVED = 7,
} fingerprint_msg_type_t;

//fp_msg_disable: set to 1 to disable lcd accelerator, for debug
static int fp_msg_disable = 0;
static int fp_msg_type = FINGERPRINT_ERROR;

extern void fb_blank_update_oem(void);//defined in drivers\video\fbdev\core\fbmem.c
static int fp_msg_type_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("DBG set fp_msg_type to %d, fp_msg_disable=%d\n", fp_msg_type, fp_msg_disable);
	if (fp_msg_disable == 1)
		return 0;

#if 0
	if (fp_msg_type == FINGERPRINT_ACQUIRED) {
#else
	if (fp_msg_type == FINGERPRINT_DETECTED) {
#endif
		pr_info("DBG fp_msg_set acquired\n");
		fb_blank_update_oem();
	}

	return 0;
}
module_param_call(fp_msg_type, fp_msg_type_set, param_get_int,
			&fp_msg_type, 0644);
module_param(fp_msg_disable, int, 0644);

/*************************************************/

/*ZTE_PM shipmode begin*********************************************************************/
static int shipmode_zte = -1;
static int zte_misc_control_shipmode(const char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	const union power_supply_propval enable = {0,};   //set 0 into shipmode
	//const union power_supply_propval disable = {1,};

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		if (shipmode_zte == 0) {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_SHIPMODE, &enable);
			if (rc) {
				pr_err("Warnning:ship enter error %d\n", rc);
			}
			pr_info("%s: enter into shipmode 10s later\n",__func__);
		} else {
			pr_info("%s: shipmode. Wrong value.Doing nothing\n",__func__);
		}
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return 0;
}
static int zte_misc_get_shipmode_node(char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	union power_supply_propval pval = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		rc = batt_psy->get_property(batt_psy,
					POWER_SUPPLY_PROP_SHIPMODE, &pval);
		if (rc) {
			pr_err("Get shipmode node error: %d\n", rc);
            return	sprintf(val,"%d",-1);
		}
		pr_info("%s: shipmode status %d\n",__func__,pval.intval);
        shipmode_zte = pval.intval;
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return	sprintf(val,"%d",pval.intval);
}

module_param_call(shipmode_zte, zte_misc_control_shipmode, zte_misc_get_shipmode_node,
					&shipmode_zte, 0644);

/*ZTE_PM end***************************************************************************/

//static int __devinit zte_misc_probe(struct platform_device *pdev)
static int zte_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error;
	
	pr_info("%s +++++\n",__func__);
	
	error = get_devtree_pdata(dev);
	if (error)
		return error;
    
	zte_misc_parse_imem();
       zte_misc_fingerprint_hw_check(dev);

    //zte_batt_switch_init();

	pr_info("%s ----\n",__func__);
	return 0;
}

//static int __devexit zte_misc_remove(struct platform_device *pdev)
static int  zte_misc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver zte_misc_device_driver = {
	.probe		= zte_misc_probe,
	//.remove	= __devexit_p(zte_misc_remove),  //zte jiangzhineng changed
	.remove    = zte_misc_remove,
	.driver		= {
		.name	= "zte-misc",
		.owner	= THIS_MODULE,
		.of_match_table = zte_misc_of_match,
	}
};

int __init zte_misc_init(void)
{
	return platform_driver_register(&zte_misc_device_driver);
}

static void __exit zte_misc_exit(void)
{
	platform_driver_unregister(&zte_misc_device_driver);
}
fs_initcall(zte_misc_init);
module_exit(zte_misc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Misc driver for zte");
MODULE_ALIAS("platform:zte-misc");
