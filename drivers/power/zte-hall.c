/*
 * Driver for some zte_hall functions
 */
#include <linux/module.h> 
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/interrupt.h>
//#include <linux/input/pmic8xxx-pwrkey.h>
#include <linux/gpio.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
//#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#include "zte-hall.h"

#include <linux/fb.h>//zte
#include <soc/qcom/socinfo.h>//for pv-version check

#define ENOMEM      12  /* Out of memory */
suspend_state_t new_state_backup= PM_SUSPEND_ON;/////liukejing;
static int factory_mode = 0;
int hall_is_factory_mode = 0;
int hall_current_factory_mode = 0;
struct hall_pwrkey *hallpwrkey;
struct wake_lock hall_wake_lock;
#define HALL_WAKELOCK_TIMEOUT 14
static int hall_mesc = 120;
static int hall_count=0; //zte_pm_20160324 count in PV
static struct device  *pm_hall_dev = NULL; //zte_pm_zhaolq_20151214
extern void synaptics_rmi4_smart_cover(bool enable);

static void update_hall_state_by_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(update_hall_work, update_hall_state_by_work);

struct pinctrl *hall_gpio_pinctrl = NULL;
struct pinctrl_state *hall_gpio_state = NULL;
static int set_hall_gpio_state(struct device *dev);

enum {
    DEBUG_HALL_STATE = 1U << 1,

};
static char *hall_dev_name[] = {
    "hall-wakelock",
};


#define HALL_EN         "hall_en"
static int gpio_hall_en = 0;
struct zte_gpio_info {
    int sys_num;            //system pin number
    const char *name;
};
#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_hall_gpios[MAX_SUPPORT_GPIOS];
static struct of_device_id zte_hall_of_match[] = {
    { .compatible = "zte-hall", },
    { },
};
MODULE_DEVICE_TABLE(of, zte_hall_of_match);

int get_hall_sysnumber_byname(char* name)
{
    int i;
    for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
        if (zte_hall_gpios[i].name) {
            if (!strcmp(zte_hall_gpios[i].name,name)) 
                return zte_hall_gpios[i].sys_num;   
        }
    }
    return -1;
}
unsigned int hallstate=(unsigned int)HALL_STATE_NULL;
module_param_named(hall_timer_debug, hall_mesc, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(hall_interrupt_count, hall_count, int, S_IRUGO | S_IWUSR | S_IWGRP); //zte_pm_20160324


static int factory_mode_set(const char *val, struct kernel_param *kp)
{
    int ret;

    ret = param_set_int(val, kp);

    if (ret)
        return ret;
    
    if ((factory_mode & 0x02) == 0)
        hall_current_factory_mode = 0;
    else
        hall_current_factory_mode = 1;

    return 0;
}

static int factory_mode_get(char *buffer, struct kernel_param *kp)
{               
    switch(hallstate)
    {
        case HALL_STATE_OPEN:
            hall_is_factory_mode = 1;
            break;
        case HALL_STATE_CLOSE:
            hall_is_factory_mode = 0;
            break;
        default:
            hall_is_factory_mode = 0;
        break;
    }

    return  sprintf(buffer,"%d",hall_is_factory_mode);
}
module_param_call(factory_mode, factory_mode_set, factory_mode_get,
            &factory_mode, 0644);

static void update_hall_state_by_work(struct work_struct *work)
{
    bool open;
    char *event = NULL;
    char *envp[2];
    const char *name;
    open = gpio_get_value(gpio_hall_en);////0:close;1:open
    wake_unlock(&hall_wake_lock);   

    //zte_pm_zhaolq_20151214 using uevent
    name = open?"HIGH":"LOW";
    event = kasprintf(GFP_KERNEL, "HALL_STATUS=%s", name);
    envp[0] = event;
    envp[1] = NULL;

    hallstate = open? HALL_STATE_OPEN : HALL_STATE_CLOSE;   
    if(!hall_current_factory_mode)//not in factory mode
    {
        pr_info("ZTE_PM_HALL by work hall_detect_high_interrupt: hall %s\n",open?"open":"close");
        kobject_uevent_env(&pm_hall_dev->kobj, KOBJ_CHANGE, envp);
    }
    /******add by pingzhenghai******/
    synaptics_rmi4_smart_cover( !open);
    /******add by pingzhenghai******/
}

static irqreturn_t hall_detect_interrupt(int irq, void *_pwrkey)////open
{
    cancel_delayed_work_sync(&update_hall_work);
    schedule_delayed_work(&update_hall_work,(msecs_to_jiffies(hall_mesc)));
    wake_lock_timeout(&hall_wake_lock, HALL_WAKELOCK_TIMEOUT);
    hall_count +=1; //zte_pm_20160324 count in PV

    if(hall_mesc&DEBUG_HALL_STATE) 
    {
        pr_info("ZTE_PM_HALL call hall_detect_interrupt by request_thread_irq\n");
    }
    return IRQ_HANDLED;
}
static int get_devtree_pdata(struct device *dev)
{
    struct device_node *node, *pp;
    int count = -1;
    pr_info("zte_hall: translate hardware pin to system pin\n");    
    node = dev->of_node;
    if (node == NULL)
        return -ENODEV;
    pp = NULL;
    while ((pp = of_get_next_child(node, pp))) {
        if (!of_find_property(pp, "label", NULL)) {
            dev_warn(dev, "HALL Found without labels\n");
            continue;
        }
        count++;
        zte_hall_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
                                GFP_KERNEL);
        zte_hall_gpios[count].sys_num = of_get_gpio(pp, 0);
        
        pr_info("zte_hall: sys_number=%d name=%s\n",zte_hall_gpios[count].sys_num,zte_hall_gpios[count].name);
    }
    return 0;
}

/* set hall gpio input and no pull*/
static int set_hall_gpio_state(struct device *dev){
    int error = 0;

    hall_gpio_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(hall_gpio_pinctrl)) {
        pr_info("Can not get hall_gpio_pinctrl\n");
        error = PTR_ERR(hall_gpio_pinctrl);
        return error;
    }
    hall_gpio_state = pinctrl_lookup_state(hall_gpio_pinctrl, "zte_hall_gpio_active");
    if (IS_ERR_OR_NULL(hall_gpio_state)) {
        pr_info("Can not get hall_gpio_state\n");
        error = PTR_ERR(hall_gpio_state);
        return error;
    }

    error = pinctrl_select_state(hall_gpio_pinctrl, hall_gpio_state);
    if (error) {
        pr_info("can not set hall_gpio pins to zte_hall_gpio_active states\n");
    } else {
        pr_info("set_hall_gpio_state success.\n");
    }
    return error;
}

static int  zte_hall_probe(struct platform_device *pdev)
{
    //const struct zte_hall_platform_data *pdata = pdev->dev.platform_data;
    int ret = 0;
    int err;
    int rc = 0;
    struct input_dev *hall_pwr;
    struct device *dev = &pdev->dev;
    pm_hall_dev = &pdev->dev;//zte_pm_zhaolq_20151214 change KEY_POWER to uevent
    
    pr_info("%s +++++\n",__func__);
    
    err = get_devtree_pdata(dev);
    if (err)
        return err; 

    hallpwrkey = kzalloc(sizeof(*hallpwrkey), GFP_KERNEL);
    gpio_hall_en = get_hall_sysnumber_byname(HALL_EN);;
    if (gpio_hall_en)
    {

        if (!hallpwrkey)
        {
            pr_info("alloc address of hallpwrkey failed\n");
            return -ENOMEM;
        }
    hall_pwr = input_allocate_device();
    if (!hall_pwr) {
        pr_info("alloc address of pwr failed\n");
        err = -ENOMEM;
        goto free_pwrkey;
    }

    input_set_capability(hall_pwr, EV_KEY, KEY_POWER);
    err = input_register_device(hall_pwr);
    if (err) {
        pr_info( "Can't register hall ");
        
        goto free_input_dev;
    }

    hall_pwr->name = "hall_imitate_pwrkey";
    hall_pwr->phys = "hall_imitate_pwrkey/input0";
 
    hallpwrkey->hall_pwr = hall_pwr;
    wake_lock_init(&hall_wake_lock, WAKE_LOCK_SUSPEND,
                hall_dev_name[0]);
    //gpio_direction_input(gpio_hall_en);
    rc = set_hall_gpio_state(dev);
    if(rc<0){
        printk("set_hall_gpio_state failed.\n");
    }

    ret = request_threaded_irq(gpio_to_irq(gpio_hall_en),
            NULL,hall_detect_interrupt,
            (IRQF_TRIGGER_RISING |
            IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
            "hall-detect", hallpwrkey);
    hallstate = gpio_get_value(gpio_hall_en);
    pr_info("hallstate=%d\n" , hallstate);

    if (ret) {
        pr_info("could not request IRQ HALL_DETECT_PIN for detect pin\n");
        gpio_free(gpio_hall_en);
        }
    ret = enable_irq_wake(gpio_to_irq(gpio_hall_en));
    if (ret) {
        pr_info("could not irq_set_irq_wake for detect pin\n");
        }
    }
#ifdef CONFIG_ZTE_BOOT_MODE
    if (socinfo_get_pv_flag()) {
        pr_info("%s: pv-version, set hall_current_factory_mode=1\n",__func__);
        hall_current_factory_mode = 1;
    }
#endif

    return 0;
free_input_dev:
    input_free_device(hall_pwr);
free_pwrkey:
    kfree(hallpwrkey);
    return err;

    }
static int  zte_hall_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver zte_hall_device_driver = {
    .probe      = zte_hall_probe,
    .remove     = zte_hall_remove,
    .driver     = {
        .name   = "zte-hall",
        .owner  = THIS_MODULE,
        .of_match_table = zte_hall_of_match,
    }
};

static int __init zte_hall_init(void)
{
    return platform_driver_register(&zte_hall_device_driver);
}

static void __exit zte_hall_exit(void)
{
    platform_driver_unregister(&zte_hall_device_driver);
}

late_initcall(zte_hall_init);
module_exit(zte_hall_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hall driver for zte");
MODULE_ALIAS("platform:zte-hall");

