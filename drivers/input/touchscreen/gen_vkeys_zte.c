/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/input/gen_vkeys_zte.h>

#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	"0x01"

#define HEIGHT_SCALE_NUM 8
#define HEIGHT_SCALE_DENOM 10


/* numerator and denomenator for border equations */
#define BORDER_ADJUST_NUM 3
#define BORDER_ADJUST_DENOM 4

static struct kobject *vkey_obj;
static char *vkey_buf;

static ssize_t vkey_show(struct kobject  *obj,
		struct kobj_attribute *attr, char *buf)
{
	strlcpy(buf, vkey_buf, MAX_BUF_SIZE);
	return strnlen(buf, MAX_BUF_SIZE);
}

static struct kobj_attribute vkey_obj_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = vkey_show,
};

static struct attribute *vkey_attr[] = {
	&vkey_obj_attr.attr,
	NULL,
};

static struct attribute_group vkey_grp = {
	.attrs = vkey_attr,
};

static int  vkey_parse_dt(struct device *dev,
			struct vkeys_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	int rc;

    rc = of_property_read_string(np, "label", &pdata->name);
    if (rc) {
    	dev_err(dev, "Failed to read label\n");
    	return -EINVAL;
    }

    rc = of_property_read_u32(np, "zte,disp-maxx_mm", &pdata->disp_maxx_mm);
    if (rc) {
    	dev_err(dev, "Failed to read display max x_mm\n");
    	return -EINVAL;
    }
    
    rc = of_property_read_u32(np, "zte,disp-maxy_mm", &pdata->disp_maxy_mm);
    if (rc) {
    	dev_err(dev, "Failed to read display max y_mm\n");
    	return -EINVAL;
    }
	
    rc = of_property_read_u32(np, "zte,disp-maxy_px", &pdata->disp_maxy_px);
    if (rc) {
    	dev_err(dev, "Failed to read display max y_px\n");
    	return -EINVAL;
    }
	
    rc = of_property_read_u32(np, "zte,x1", &pdata->x1);
    if (rc) {
    	dev_err(dev, "Failed to read  x1\n");
    	return -EINVAL;
    }

    rc = of_property_read_u32(np, "zte,x2", &pdata->x2);
    if (rc) {
    	dev_err(dev, "Failed to read  x2\n");
    	return -EINVAL;
    }

    rc = of_property_read_u32(np, "zte,x3", &pdata->x3);
    if (rc) {
    	dev_err(dev, "Failed to read  x3\n");
    	return -EINVAL;
    }

    rc = of_property_read_u32(np, "zte,lcd_offset", &pdata->lcd_offset);
    if (rc) {
    	dev_err(dev, "Failed to read lcd_offset\n");
    	return -EINVAL;
    }

    rc = of_property_read_u32(np, "zte,y_max_offset", &pdata->y_max_offset);
    if (rc) {
    	dev_err(dev, "Failed to read lcd_offset\n");
    	return -EINVAL;
    }
    rc = of_property_read_u32(np, "zte,dx", &pdata->dx);
    if (rc) {
    	dev_err(dev, "Failed to read lcd_offset\n");
    	return -EINVAL;
    }

    prop = of_find_property(np, "zte,key-codes", NULL);
    if (prop) {
    	pdata->num_keys = prop->length / sizeof(u32);
    	pdata->keycodes = devm_kzalloc(dev,
    		sizeof(u32) * pdata->num_keys, GFP_KERNEL);
    	if (!pdata->keycodes)
    		return -ENOMEM;
    	rc = of_property_read_u32_array(np, "zte,key-codes",
    			pdata->keycodes, pdata->num_keys);
    	if (rc) {
    		dev_err(dev, "Failed to read key codes\n");
    		return -EINVAL;
    	}
    }
    
	return 0;
}

static int  vkeys_probe(struct platform_device *pdev)
{
	struct vkeys_platform_data *pdata;
	int width, height,center_x[3],center_y;
	int i, c = 0, ret;
	char *name;
	
	vkey_buf = devm_kzalloc(&pdev->dev, MAX_BUF_SIZE, GFP_KERNEL);
	if (!vkey_buf) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct vkeys_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = vkey_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = pdev->dev.platform_data;

	if (!pdata || !pdata->name || !pdata->keycodes || !pdata->num_keys ||
		!pdata->disp_maxx_mm || !pdata->disp_maxy_mm ) {
		dev_err(&pdev->dev, "pdata is invalid\n");
		return -EINVAL;
	}
	
	center_x[0] = pdata->x1*pdata->disp_maxy_px/pdata->disp_maxy_mm;
	center_x[1] = pdata->x2*pdata->disp_maxy_px/pdata->disp_maxy_mm;
	center_x[2] = pdata->x3*pdata->disp_maxy_px/pdata->disp_maxy_mm;
	center_y = ((pdata->y_max_offset+pdata->lcd_offset)/2)*pdata->disp_maxy_px/pdata->disp_maxy_mm
		+pdata->disp_maxy_px;
	width = pdata->dx*pdata->disp_maxy_px/pdata->disp_maxy_mm;
	height = (pdata->y_max_offset-pdata->lcd_offset)*pdata->disp_maxy_px/pdata->disp_maxy_mm;
	for (i = 0; i < pdata->num_keys; i++) {
		c += snprintf(vkey_buf + c, MAX_BUF_SIZE - c,
				"%s:%d:%d:%d:%d:%d\n",
				VKEY_VER_CODE, pdata->keycodes[i],
				center_x[i], center_y, width, height);
	}	

	vkey_buf[c] = '\0';

	name = devm_kzalloc(&pdev->dev, sizeof(*name) * MAX_BUF_SIZE,
					GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, MAX_BUF_SIZE,
				"virtualkeys.%s", pdata->name);
	vkey_obj_attr.attr.name = name;
	
	if(!vkey_obj){
		vkey_obj = kobject_create_and_add("board_properties", NULL);
		if (!vkey_obj) {
			dev_err(&pdev->dev, "unable to create kobject\n");
			return -ENOMEM;
		}
	}
	
	ret = sysfs_create_group(vkey_obj, &vkey_grp);
	if (ret) {
		dev_err(&pdev->dev, "failed to create attributes\n");
		goto destroy_kobj;
	}
	return 0;

destroy_kobj:
	kobject_put(vkey_obj);

	return ret;
}

static int  vkeys_remove(struct platform_device *pdev)
{
	sysfs_remove_group(vkey_obj, &vkey_grp);
	kobject_put(vkey_obj);

	return 0;
}

static struct of_device_id vkey_match_table[] = {
	{ .compatible = "zte,gen-vkeys",},
	{ },
};

static struct platform_driver vkeys_zte_driver = {
	.probe = vkeys_probe,
	.remove = vkeys_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gen_vkeys_zte",
		.of_match_table = vkey_match_table,
	},
};

module_platform_driver(vkeys_zte_driver);
MODULE_LICENSE("GPL v2");
