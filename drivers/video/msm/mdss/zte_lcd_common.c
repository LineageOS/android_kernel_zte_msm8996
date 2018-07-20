#include "zte_lcd_common.h"
#include "mdss_dsi_cmd.h"

#ifdef CONFIG_ZTE_LCD_REG_DEBUG
extern void zte_lcd_reg_debug_func(void);
#endif
#ifdef CONFIG_ZTE_LCDBL_I2C_CTRL_VSP_VSN
extern void tps65132b_set_vsp_vsn_level(u8 level);
#endif
struct mdss_dsi_ctrl_pdata *g_zte_ctrl_pdata;

const char *panel_name;
/********************read lcm hardware info begin****************/

/*file path: proc/driver/lcd_id/ or proc/msm_lcd*/
static int zte_lcd_proc_info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_zte_ctrl_pdata->zte_lcd_ctrl.lcd_panel_name);
	return 0;
}
static int zte_lcd_proc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, zte_lcd_proc_info_show, NULL);
}
static const struct file_operations zte_lcd_common_func_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= zte_lcd_proc_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static int zte_lcd_proc_info_display(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
	/*const char *panel_name;*/

	proc_create_data("driver/lcd_id", 0, NULL, &zte_lcd_common_func_proc_fops, NULL);
	/*proc_create_data("msm_lcd", 0, NULL, &zte_lcd_common_func_proc_fops, NULL);*/
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name) {
		pr_info("%s:%d, panel name not found!\n", __func__, __LINE__);
		strlcpy(ctrl_pdata->zte_lcd_ctrl.lcd_panel_name, "0", sizeof(ctrl_pdata->zte_lcd_ctrl.lcd_panel_name));
		return -ENODEV;
	}
	pr_info("%s: Panel Name = %s\n", __func__, panel_name);
	strlcpy(ctrl_pdata->zte_lcd_ctrl.lcd_panel_name, panel_name, sizeof(ctrl_pdata->zte_lcd_ctrl.lcd_panel_name));
	return 0;
}
/********************read lcm hardware info end***********************/

/********************extreme power save mode cabc begin*****************/
#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
static struct kobject *lcd_cabc_kobj;
struct mdss_dsi_ctrl_pdata *g_ctrl_pdata;
extern char sys_cabc[2];
extern struct dsi_cmd_desc sys_cmd_cabc;

static ssize_t lcd_cabc_show(struct device *dev, struct device_attribute *attr, char *buf)
{	int retval = 0;
	mutex_lock(&g_ctrl_pdata->zte_lcd_ctrl.panel_sys_lock);
	retval = snprintf(buf, 32, "%d,\n", sys_cabc[1]);
	pr_info("%s:ZTE_mdss_lcd_cabc: 0x%x.\n", __func__, sys_cabc[1]);
	mutex_unlock(&g_ctrl_pdata->zte_lcd_ctrl.panel_sys_lock);
	return retval;
}
static ssize_t lcd_cabc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct dcs_cmd_req cmdreq;

	if (kstrtoint(buf, 16, &input) != 0)
		return -EINVAL;
	input = input > 0 ? input : 0;
	mutex_lock(&g_ctrl_pdata->zte_lcd_ctrl.panel_sys_lock);
	sys_cabc[1] = input;
	cmdreq.cmds = &sys_cmd_cabc;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);
	mutex_unlock(&g_ctrl_pdata->zte_lcd_ctrl.panel_sys_lock);
	pr_info("%s:ZTE_mdss_lcd_cabc: 0x%x.\n", __func__, sys_cabc[1]);
	return count;
}
static DEVICE_ATTR(lcd_cabc, 0664,  lcd_cabc_show, lcd_cabc_store);
static struct attribute *lcd_cabc_attributes[] = {
	&dev_attr_lcd_cabc.attr,
		NULL
};
static struct attribute_group lcd_cabc_attribute_group = {
	.attrs = lcd_cabc_attributes
};
int zte_create_cabc_sys(struct mdss_dsi_ctrl_pdata *pdata)
{
	int err;

	g_ctrl_pdata = pdata;
	lcd_cabc_kobj = kobject_create_and_add("lcd_cabc", NULL);
	if (!lcd_cabc_kobj) {
		err = -EINVAL;
		pr_info("%s() - ERROR Unable to create lcd_cabc_kobj.\n", __func__);
		return -EIO;
	}
	err = sysfs_create_group(lcd_cabc_kobj, &lcd_cabc_attribute_group);
	if (err != 0) {
		pr_info("%s - ERROR lcd_cabc_kobj failed.\n", __func__);
		kobject_put(lcd_cabc_kobj);
		return -EIO;
	}
	pr_info("%s succeeded.\n", __func__);
	return err;
}
#endif
/********************extreme power save mode cabc end*****************/

/********************lcd gpio power ctrl***********************/
#ifdef CONFIG_ZTE_LCD_GPIO_CTRL_POWER
static int zte_gpio_ctrl_lcd_power_enable(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_info("%s:%s\n", __func__, enable ? "enable":"disable");
	if (enable) {
		usleep_range(1000, 1100);
		if (ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		#ifdef CONFIG_ZTE_LCDBL_I2C_CTRL_VSP_VSN
		if (ctrl_pdata->zte_lcd_ctrl.lcd_bl_vsp_vsn_voltage != 0x0)
			tps65132b_set_vsp_vsn_level(ctrl_pdata->zte_lcd_ctrl.lcd_bl_vsp_vsn_voltage);/*5.5v*/
		#endif
	} else {
		usleep_range(1000, 1100);
		if (ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio, 0);
			usleep_range(5000, 5100);
		}
	}
	return 0;
}
int zte_gpio_ctrl_lcd_power_init(struct platform_device *ctrl_pdev, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_avdd_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio)) {
		pr_info("%s:%d, zte,disp_avdd_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio, "disp_avdd_en_gpio")) {
			pr_info("request disp_avdd_en_gpio failed\n");
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl.disp_avdd_en_gpio, 1);
			pr_info("%s:request disp_avdd_en_gpio success\n", __func__);
		}
	}
	ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_iovdd_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio)) {
		pr_info("%s:%d, zte,disp_iovdd_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio, "disp_iovdd_en_gpio")) {
			pr_info("request disp_iovdd_en_gpio failed %d\n", ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio);
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl.disp_iovdd_en_gpio, 1);
			pr_info("%s:request disp_iovdd_en_gpio success\n", __func__);
		}
	}

	ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_vsp_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio)) {
		pr_info("%s:%d, zte,disp_vsp_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio, "disp_vsp_en_gpio")) {
			pr_info("request disp_vsp_en_gpio failed\n");
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl.disp_vsp_en_gpio, 1);
			pr_info("%s:request disp_vsp_en_gpio success\n", __func__);
		}
	}
	ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_vsn_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio)) {
		pr_info("%s:%d, zte,disp_vsn_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio, "disp_vsn_en_gpio")) {
			pr_info("request disp_vsn_en_gpio failed %d\n", ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio);
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl.disp_vsn_en_gpio, 1);
			pr_info("%s:request disp_vsn_en_gpio success\n", __func__);
		}
	}

	return 0;
}
#endif

/********************lcd backlight level curve begin*****************/
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
enum {	/* lcd curve mode */
	CURVE_MATRIX_MAX_350_LUX = 1,
	CURVE_MATRIX_MAX_400_LUX,
	CURVE_MATRIX_MAX_450_LUX,
};

int zte_backlight_curve_matrix_max_350_lux[256] = {
0, 1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18,
18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27,
28, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 36, 37,
38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48,
49, 50, 50, 51, 52, 52, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60,
61, 61, 62, 63, 64, 64, 65, 66, 67, 68, 68, 69, 70, 71, 72, 73,
74, 75, 76, 77, 77, 78, 79, 80, 81, 82, 82, 83, 84, 85, 86, 87,
88, 88, 89, 90, 91, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 146, 147,
148, 149, 151, 152, 153, 155, 156, 157, 159, 160, 162, 163, 164, 166, 167, 169,
170, 172, 173, 175, 176, 178, 179, 181, 183, 184, 186, 187, 189, 191, 192, 194,
196, 197, 199, 201, 203, 204, 206, 208, 210, 212, 214, 215, 217, 219, 221, 223,
225, 227, 229, 231, 233, 235, 237, 239, 241, 243, 245, 248, 250, 252, 254, 255
};

int zte_backlight_curve_matrix_max_400_lux[256] = {
0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8,
8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16,
16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24,
25, 25, 26, 26, 27, 27, 28, 28, 29, 30, 30, 31, 31, 32, 32, 33,
34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 42, 42, 43,
43, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53,
54, 55, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 64, 64,
65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 72, 73, 74, 74, 75, 76,
77, 78, 78, 79, 80, 81, 81, 82, 83, 84, 84, 85, 86, 87, 88, 88,
89, 90, 91, 92, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 113, 114, 115,
116, 117, 118, 119, 120, 121, 121, 122, 123, 124, 125, 126, 127, 128, 129, 129,
130, 132, 133, 134, 136, 137, 139, 140, 142, 143, 145, 147, 148, 150, 151, 153,
155, 156, 158, 160, 162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 180, 182,
184, 186, 188, 190, 192, 194, 196, 198, 200, 203, 205, 207, 209, 212, 214, 216,
219, 221, 223, 226, 228, 231, 233, 236, 238, 241, 243, 246, 249, 252, 254, 255
};

int zte_backlight_curve_matrix_max_450_lux[256] = {
0, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7,
8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14,
14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21,
22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 36, 36, 37, 37, 38,
38, 39, 39, 40, 41, 41, 42, 42, 43, 43, 44, 45, 45, 46, 46, 47,
48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 54, 54, 55, 56, 56, 57,
57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 67,
68, 69, 69, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 77, 78, 78,
79, 80, 80, 81, 82, 83, 83, 84, 85, 85, 86, 87, 88, 88, 89, 90,
91, 91, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100, 101, 101, 102,
103, 104, 105, 105, 106, 107, 108, 109, 109, 110, 111, 112, 112, 112, 113, 113,
114, 116, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131, 133, 135, 136, 138,
140, 142, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169,
171, 173, 176, 178, 180, 183, 185, 187, 190, 192, 194, 197, 199, 202, 205, 207,
210, 213, 215, 218, 221, 224, 226, 229, 232, 235, 238, 241, 244, 248, 251, 255
};
static int zte_convert_backlevel_function(int level, u32 bl_max)
{
	int bl, convert_level;

	if (level == 0)
		return 0;

	if (bl_max > 1023) {
		bl = level>>4;
	} else {
		bl = level;
	}

	if (!bl && level)
		bl = 1;/*ensure greater than 0 and less than 16 equal to 1*/

	switch (g_zte_ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode) {
	case CURVE_MATRIX_MAX_350_LUX:
		convert_level = zte_backlight_curve_matrix_max_350_lux[bl];
		break;
	case CURVE_MATRIX_MAX_400_LUX:
		convert_level = zte_backlight_curve_matrix_max_400_lux[bl];
		break;
	case CURVE_MATRIX_MAX_450_LUX:
		convert_level = zte_backlight_curve_matrix_max_450_lux[bl];
		break;
	default:
		convert_level = zte_backlight_curve_matrix_max_450_lux[bl];
		break;
	}
	if (bl_max > 1023) {
		convert_level = (convert_level >= 255) ? 4095 : (convert_level<<4);
	}

	return convert_level;
}
#endif
/********************lcd backlight level curve end*****************/

/********************lcd common function start*****************/
static void zte_lcd_panel_parse_dt(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	const char *data;
#endif

#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	data = of_get_property(node, "zte,lcm_backlight_curve_mode", NULL);
	if (data) {
		if (!strcmp(data, "lcd_brightness_max_350_lux"))
			ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode = CURVE_MATRIX_MAX_350_LUX;
		else if (!strcmp(data, "lcd_brightness_max_400_lux"))
			ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode = CURVE_MATRIX_MAX_400_LUX;
		else if (!strcmp(data, "lcd_brightness_max_450_lux"))
			ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;
		else
			ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;
	} else
		ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;

	pr_info("[MSM_LCD]%s:dtsi_mode=%s matrix_mode=%d\n", __func__, data,
			ctrl_pdata->zte_lcd_ctrl.lcd_bl_curve_mode);
#endif
#ifdef CONFIG_ZTE_LCDBL_I2C_CTRL_VSP_VSN
	of_property_read_u32(node, "zte,lcd_bl_vsp_vsn_voltage", &ctrl_pdata->zte_lcd_ctrl.lcd_bl_vsp_vsn_voltage);
	pr_info("lcd vsp_vsn=%x\n", ctrl_pdata->zte_lcd_ctrl.lcd_bl_vsp_vsn_voltage);
#endif
	ctrl_pdata->zte_lcd_ctrl.lcd_tp_binding_resetpin = of_property_read_bool(node,
															"zte,lcm_tp_incell_binding_reset_pin");
	ctrl_pdata->zte_lcd_ctrl.lcd_reset_high_sleeping = of_property_read_bool(node,
															"zte,lcm_reset_pin_keep_high_sleeping");
}

/****zte_mdss_dsi_panel_resume_ctrl_resume func lcd reset pull down only for hx83112A lcd&tp resume*******/
int zte_mdss_dsi_panel_resume_ctrl_reset(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n", __func__, __LINE__);
		return rc;
	}

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		return rc;
	}
	if (!pinfo->cont_splash_enabled) {
		if (pdata->panel_info.rst_seq_len) {
			rc = gpio_direction_output(ctrl_pdata->rst_gpio, 1);
			if (rc) {
				pr_err("%s: unable to set dir for rst gpio\n",
					__func__);
			}
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		msleep(20);
	}
	gpio_free(ctrl_pdata->rst_gpio);
	return rc;
}

void zte_lcd_common_func(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
	g_zte_ctrl_pdata = ctrl_pdata;
	zte_lcd_panel_parse_dt(ctrl_pdata, node);
#ifdef CONFIG_ZTE_LCD_GPIO_CTRL_POWER
	g_zte_ctrl_pdata->zte_lcd_ctrl.gpio_enable_lcd_power = zte_gpio_ctrl_lcd_power_enable;
#endif
	zte_lcd_proc_info_display(ctrl_pdata, node);
#ifdef CONFIG_ZTE_LCD_REG_DEBUG
	zte_lcd_reg_debug_func();
#endif
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	g_zte_ctrl_pdata->zte_lcd_ctrl.zte_convert_brightness = zte_convert_backlevel_function;
#endif

}
