#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "gf3238_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/*GPIO pins reference.*/
int gf_milan_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	/*get pwr resource*/
	gf_dev->pwr_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_pwr", 0);
	if (!gpio_is_valid(gf_dev->pwr_gpio)) {
		FP_LOG(ERR, "PWR GPIO is invalid.\n");
		return -EPERM;
	}
	rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
	if (rc) {
		FP_LOG(ERR, "Failed to request PWR GPIO. rc = %d\n", rc);
		return -EPERM;
	}

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		FP_LOG(ERR, "RESET GPIO is invalid.\n");
		return -EPERM;
	}
	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		FP_LOG(ERR, "Failed to request RESET GPIO. rc = %d\n", rc);
		return -EPERM;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	FP_LOG(INFO, "gf:irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		FP_LOG(ERR, "IRQ GPIO is invalid.\n");
		return -EPERM;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		FP_LOG(ERR, "Failed to request IRQ GPIO. rc = %d\n", rc);
		return -EPERM;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	/* power on */
	gpio_direction_output(gf_dev->pwr_gpio, 1);

	/*qcom volatage end*/

	FP_LOG(INFO, "Power ok.\n");

	return 0;
}

void gf_milan_cleanup(struct gf_dev	*gf_dev)
{
	FP_LOG(INFO, "%s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		FP_LOG(INFO, "remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		FP_LOG(INFO, "remove reset_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		FP_LOG(INFO, "remove pwr_gpio success\n");
	}
}

/*power management*/
int gf_milan_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_set_value(gf_dev->pwr_gpio, 1);
	}
	usleep_range(10000, 10001);
	FP_LOG(INFO, "---- power on ok ----\n");

	return rc;
}

int gf_milan_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;
/*
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_set_value(gf_dev->pwr_gpio, 0);
		}
*/
	FP_LOG(INFO, "---- power off ----\n");
	return rc;
}

/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int gf_milan_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		FP_LOG(ERR, "Input buff is NULL.\n");
		return -EPERM;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	usleep_range(5000, 5001);
	gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(delay_ms);
	return 0;
}

int gf_milan_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		FP_LOG(ERR, "Input buff is NULL.\n");
		return -EPERM;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

