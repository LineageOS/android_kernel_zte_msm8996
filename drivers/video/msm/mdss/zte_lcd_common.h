#ifndef ZTE_LCD_COMMON_H
#define ZTE_LCD_COMMON_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include "mdss_panel.h"
/*
#define CONFIG_ZTE_LCD_COMMON_FUNCTION
#define CONFIG_ZTE_LCD_REG_DEBUG
*/
#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION

struct zte_lcd_ctrl_data {
	u32 lcd_bl_curve_mode;
	char lcd_tp_binding_resetpin;
	char lcd_reset_high_sleeping;
#ifdef CONFIG_ZTE_LCD_GPIO_CTRL_POWER
	int disp_avdd_en_gpio;
	int disp_iovdd_en_gpio;
	int disp_vsp_en_gpio;
	int disp_vsn_en_gpio;
	int (*gpio_enable_lcd_power)(struct mdss_panel_data *pdata, int enable);
#endif
#ifdef CONFIG_ZTE_LCDBL_I2C_CTRL_VSP_VSN
	u32 lcd_bl_vsp_vsn_voltage;
#endif
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	int (*zte_convert_brightness)(int level, u32 bl_max);
#endif
	char lcd_panel_name[60];
#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	struct mutex panel_sys_lock;
#endif
};

#include "mdss_dsi.h"  /*this is must in the end of struct zte_lcd_info_data,otherwise will build error*/

void zte_lcd_common_func(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node);
int zte_gpio_ctrl_lcd_power_init(struct platform_device *ctrl_pdev, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int zte_create_cabc_sys(struct mdss_dsi_ctrl_pdata *pdata);
#endif /*CONFIG_ZTE_LCD_COMMON_FUNCTION*/

#endif

