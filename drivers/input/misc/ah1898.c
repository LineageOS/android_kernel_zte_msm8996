/***********************************
*****ah1898 hall sensor****************
************************************
************************************/
#include <linux/fs.h>
#include	<linux/gpio.h>
#include <linux/irq.h>
#include	<linux/err.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

/*#define AH1898_IRQ 11*/

static int hall_status = 0;

#if defined(CONFIG_BOARD_AILSA_II)
extern void synaptics_rmi4_smart_cover(bool enable);
#endif

module_param(hall_status, int, 0644);

struct ah1898_chip {

	/*struct mutex lock;*/
	struct input_dev *input;
	struct work_struct work;
	int irq;
	bool ah1898_enabled;
} *ah1898_chip_data;


static void ah1898_work_func(struct work_struct *work)
{

	int value;

	if (ah1898_chip_data->input == NULL || ah1898_chip_data == NULL) {
		pr_info("ah1898_work_fuc ERROR");
		return;
	}

	value = gpio_get_value(ah1898_chip_data->irq);

	pr_info("%s:hall test value=%d, enabled = %d\n", __func__, value, ah1898_chip_data->ah1898_enabled);

	if (ah1898_chip_data->ah1898_enabled == 1) {

	if (value == 1) {
		/*log for off*/
			pr_info("%s:hall ===switch is off!!the value = %d\n", __func__, value);
/* delete KEY_POWER input report
			input_report_key(ah1898_chip_data->input, KEY_POWER, 1);
			input_sync(ah1898_chip_data->input);
			input_report_key(ah1898_chip_data->input, KEY_POWER, 0);
*/
			input_report_key(ah1898_chip_data->input, KEY_HALL_SENSOR_UP, 1);
			input_sync(ah1898_chip_data->input);
			input_report_key(ah1898_chip_data->input, KEY_HALL_SENSOR_UP, 0);
			hall_status = 0;
		} else {
		/*log for on*/
			pr_info("%s:hall ===switch is on!!the value = %d\n", __func__, value);
/* delete KEY_POWER input report
			input_report_key(ah1898_chip_data->input, KEY_POWER, 1);
			input_sync(ah1898_chip_data->input);
			input_report_key(ah1898_chip_data->input, KEY_POWER, 0);
*/
			input_report_key(ah1898_chip_data->input, KEY_HALL_SENSOR_DOWN, 1);
			input_sync(ah1898_chip_data->input);
			input_report_key(ah1898_chip_data->input, KEY_HALL_SENSOR_DOWN, 0);
			hall_status = 1;
		}
		input_report_switch(ah1898_chip_data->input, SW_LID, hall_status);
		input_sync(ah1898_chip_data->input);

/* used for touchscreen mode switching */
#if defined(CONFIG_BOARD_AILSA_II)
		synaptics_rmi4_smart_cover(!value);
#endif
	}
	/*enable_irq(ah1898_chip_data->irq);*/
}


static irqreturn_t ah1898_interrupt(int irq, void *dev_id)
{
	if (ah1898_chip_data == NULL) {
       /* printk("++++++++ah1898_interrupt\n ddata = %x", (unsigned long)ah1898_chip_data);*/
	return IRQ_NONE;
	}
	pr_info("%s:chenhui ah1898_interrupt!!\n", __func__);
	/*disable_irq_nosync(irq);*/

	schedule_work(&ah1898_chip_data->work);

	return IRQ_HANDLED;
}

int ah1898_parse_dt(struct platform_device *pdev)
{

	ah1898_chip_data->irq = of_get_named_gpio(pdev->dev.of_node, "ah,gpio_irq", 0);

	if (!gpio_is_valid(ah1898_chip_data->irq)) {
		pr_info("gpio irq pin %d is invalid.\n", ah1898_chip_data->irq);
		return -EINVAL;
	}

	return 0;

}

static int  ah1898_probe(struct platform_device *pdev)
{
	int value_status;

	int error = 0;
	int irq = 0;

	if (pdev->dev.of_node == NULL) {
		dev_info(&pdev->dev, "can not find device tree node\n");
		return -ENODEV;
	}
	pr_info("++++++++ah1898_probe\n");

	ah1898_chip_data = kzalloc(sizeof(struct ah1898_chip), GFP_KERNEL);

	ah1898_chip_data->input = input_allocate_device();

	if (!ah1898_chip_data || !ah1898_chip_data->input) {
		error = -ENOMEM;
		goto fail0;
	}

/*input*/
	ah1898_chip_data->input->name = "ah1898";

	set_bit(EV_SW, ah1898_chip_data->input->evbit);
	set_bit(EV_KEY, ah1898_chip_data->input->evbit);
	/*set_bit(KEY_POWER, ah1898_chip_data->input->keybit);*/
/*hall sensor key*/
	set_bit(KEY_HALL_SENSOR_DOWN, ah1898_chip_data->input->keybit);
	set_bit(KEY_HALL_SENSOR_UP, ah1898_chip_data->input->keybit);

	input_set_capability(ah1898_chip_data->input, EV_SW, SW_LID);

	error = input_register_device(ah1898_chip_data->input);
	if (error) {
		pr_err("ah1898: Unable to register input device, error: %d\n", error);
		goto fail2;
	}

	error = ah1898_parse_dt(pdev);
	if (error) {
		goto fail2;
	}

	if (ah1898_chip_data->irq) {

		irq = gpio_to_irq(ah1898_chip_data->irq);

		error = gpio_request(ah1898_chip_data->irq, "ah1898_irq");
		if (error) {
			pr_info("%s:ah1898 error3\n", __func__);
			goto fail1;
		}

		error = gpio_direction_input(ah1898_chip_data->irq);
		if (error) {
			pr_info("%s:ah1898 error3\n", __func__);
			goto fail1;
		}
	}

	if (irq) {

	INIT_WORK(&(ah1898_chip_data->work), ah1898_work_func);

		error = request_threaded_irq(irq, NULL, ah1898_interrupt,
							   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							      "ah1898_irq", NULL);
		if (error) {
			pr_err("gpio-ah1898: Unable to claim irq %d; error %d\n", irq, error);
			goto fail1;
		}

		enable_irq_wake(irq);
	}

	ah1898_chip_data->ah1898_enabled = 1;
/*init hall status*/
	value_status = gpio_get_value(ah1898_chip_data->irq);
			pr_err("gpio-ah1898: irq %d;\n", value_status);
	if (value_status == 1) {
		hall_status = 0;
	} else {
		hall_status = 1;
	}
	pr_info("hall Init hall_state=%d\n", hall_status);
	pr_info("%s:hall Init success!\n", __func__);

	return 0;

fail2:
	pr_err("gpio-ah1898 input_allocate_device fail\n");
	input_unregister_device(ah1898_chip_data->input);
	kfree(ah1898_chip_data);

fail1:
	pr_err("ah1898 gpio irq request fail\n");
	gpio_free(ah1898_chip_data->irq);

fail0:
	pr_err("gpio-ah1898 input_register_device fail\n");
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int ah1898_remove(struct platform_device *pdev)
{
	gpio_free(ah1898_chip_data->irq);
	input_unregister_device(ah1898_chip_data->input);
	kfree(ah1898_chip_data);

	return 0;
}


static struct of_device_id ah1898_hall_of_match[] = {
	{.compatible = "ah,hall_ic", },
	{},
};


static struct platform_driver ah1898_hall_driver = {
	.probe = ah1898_probe,
	.remove = ah1898_remove,
	.driver = {
		.name = "hall_ic",
		.owner = THIS_MODULE,
		.of_match_table = ah1898_hall_of_match,
		/*.pm = &led_pm_ops,*/
	},

};


static int  ah1898_init(void)
{
	pr_info("++++++++ah1898_init\n");
	return platform_driver_register(&ah1898_hall_driver);
}


static void  ah1898_exit(void)
{
	platform_driver_unregister(&ah1898_hall_driver);
	pr_info("++++++++ah1898_exit\n");
}

module_init(ah1898_init);
module_exit(ah1898_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chenhui");
MODULE_DESCRIPTION("hall sensor driver");
MODULE_ALIAS("platform:hall-sensor");
