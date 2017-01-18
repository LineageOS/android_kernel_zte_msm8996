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

#include <linux/device.h>
#include "msm_sensor.h"
#include "msm_ois.h"
#include "zte_camera_sensor_util.h"
#include "zte_camera_ois_util.h"

#define CONFIG_ZTE_CAMERA_UTIL_DEBUG
#undef CDBG
#ifdef CONFIG_ZTE_CAMERA_UTIL_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define OIS_DEBUG 1

/*
  * add  test camera sensor mipi clock interface
  *
  * by ZTE_YCM_20140710 yi.changming 000005
  */
// --->
typedef struct {
	struct msm_sensor_ctrl_t *s_ctrl;
	enum msm_camera_i2c_data_type msm_sensor_reg_data_type;
	uint64_t address;
} msm_sensor_debug_info_t;

#if OIS_DEBUG
typedef struct {
	struct msm_ois_ctrl_t *s_ctrl;
	enum msm_camera_i2c_data_type msm_ois_reg_data_type;
	uint64_t address;
} msm_ois_debug_info_t;
#endif

static int sensor_debugfs_datatype_s(void *data, u64 val)
{
	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
	
	if(val < MSM_CAMERA_I2C_DATA_TYPE_MAX 
		         && val >= MSM_CAMERA_I2C_BYTE_DATA)
		ptr->msm_sensor_reg_data_type=val;
	
	CDBG("%s:%d: msm_sensor_reg_data_type = %d \n", __func__, __LINE__,
			ptr->msm_sensor_reg_data_type);
	
	return 0;
}

#if OIS_DEBUG
static int ois_debugfs_datatype_s(void *data, u64 val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	
	if(val < MSM_CAMERA_I2C_DATA_TYPE_MAX 
		         && val >= MSM_CAMERA_I2C_BYTE_DATA)
		ptr->msm_ois_reg_data_type=val;
	
	CDBG("%s:%d: msm_ois_reg_data_type = %d \n", __func__, __LINE__,
			ptr->msm_ois_reg_data_type);
	
	return 0;
}
#endif

static int sensor_debugfs_datatype_g(void *data, u64 *val)
{
	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
		
	*val=ptr->msm_sensor_reg_data_type;
	
	CDBG("%s:%d: msm_sensor_reg_data_type = %d \n", __func__, __LINE__,
		ptr->msm_sensor_reg_data_type);	
	
	return 0;
}

#if OIS_DEBUG
static int ois_debugfs_datatype_g(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
		
	*val=ptr->msm_ois_reg_data_type;
	
	CDBG("%s:%d: msm_ois_reg_data_type = %d \n", __func__, __LINE__,
		ptr->msm_ois_reg_data_type);	
	
	return 0;
}

#endif

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_datatype, sensor_debugfs_datatype_g,
			sensor_debugfs_datatype_s, "%llx\n");

#if OIS_DEBUG
DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_datatype, ois_debugfs_datatype_g,
			ois_debugfs_datatype_s, "%llx\n");
#endif

static int sensor_debugfs_setaddr(void *data, u64 val)
{

	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
	
	ptr->address = val;
	
	CDBG("%s:%d: address = 0x%llx \n", __func__, __LINE__,
		ptr->address);
	
	return 0;
}

#if OIS_DEBUG
static int ois_debugfs_setaddr(void *data, u64 val)
{

	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	
	ptr->address = val;
	
	CDBG("%s:%d: address = 0x%llx \n", __func__, __LINE__,
		ptr->address);
	
	return 0;
}
#endif

static int sensor_debugfs_getaddr(void *data, u64 *val)
{
	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
	
	*val=ptr->address;

	CDBG("%s:%d: address = 0x%llx \n", __func__, __LINE__,
		ptr->address);
	
	return 0;
}

#if OIS_DEBUG
static int ois_debugfs_getaddr(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	
	*val=ptr->address;

	CDBG("%s:%d: address = 0x%llx \n", __func__, __LINE__,
		ptr->address);
	
	return 0;
}

#endif

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_address, sensor_debugfs_getaddr,
			sensor_debugfs_setaddr, "%llx\n");

#if OIS_DEBUG
DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_address, ois_debugfs_getaddr,
			ois_debugfs_setaddr, "%llx\n");
#endif

static int sensor_debugfs_setvalue(void *data, u64 val)
{
	
	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
	int32_t rc = 0;
	
	CDBG("%s:%d: address = 0x%llx  value = 0x%llx\n", __func__, __LINE__,
		ptr->address,val);
		
	rc = ptr->s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			ptr->s_ctrl->sensor_i2c_client,
			ptr->address, val,
			ptr->msm_sensor_reg_data_type);
	if (rc < 0) {
		pr_err("%s:%d: %s: i2c write %llx failed\n", __func__,__LINE__,
			ptr->s_ctrl->sensordata->sensor_name,val);
		return rc;
	}
	
	return 0;
}

static int sensor_debugfs_getvalue(void *data, u64 *val)
{
	
	msm_sensor_debug_info_t *ptr = (msm_sensor_debug_info_t *) data;
	int32_t rc = 0;
	uint16_t temp;

	rc = ptr->s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			ptr->s_ctrl->sensor_i2c_client,
			ptr->address, &temp,
			ptr->msm_sensor_reg_data_type);
	if (rc < 0) {
		pr_err("%s:%d: %s: i2c read %x failed\n", __func__,__LINE__,
			ptr->s_ctrl->sensordata->sensor_name,temp);
		return rc;
	}
	
	*val=temp;

	CDBG("%s:%d: address = 0x%llx  value = 0x%x\n", __func__, __LINE__,
		ptr->address,temp);
	
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_value, sensor_debugfs_getvalue,
			sensor_debugfs_setvalue, "%llx\n");

#if OIS_DEBUG
static int ois_debugfs_setvalue(void *data, u64 val)
{
	
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	struct msm_camera_i2c_client *ois_i2c_client = &(ptr->s_ctrl->i2c_client);
	int32_t rc = 0;
	
	CDBG("%s:%d: address = 0x%llx  value = 0x%llx\n", __func__, __LINE__,
		ptr->address,val);
		
	rc = ois_i2c_client->i2c_func_tbl->i2c_write(
			ois_i2c_client,
			ptr->address, val,
			ptr->msm_ois_reg_data_type);
	if (rc < 0) {
		pr_err("%s:%d: i2c write failed\n", __func__,__LINE__);
		return rc;
	}
	
	return 0;
}

static int ois_debugfs_getvalue(void *data, u64 *val)
{
	
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	struct msm_camera_i2c_client *ois_i2c_client = &(ptr->s_ctrl->i2c_client);
	int32_t rc = 0;
	uint16_t temp;

	rc = ois_i2c_client->i2c_func_tbl->i2c_read(
			ois_i2c_client,
			ptr->address, &temp,
			ptr->msm_ois_reg_data_type);
	if (rc < 0) {
		pr_err("%s:%d: i2c read failed\n", __func__,__LINE__);
		return rc;
	}
	
	*val=temp;
	
	CDBG("%s:%d: address = 0x%llx  value = 0x%x\n", __func__, __LINE__,
		ptr->address,temp);
	
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_value, ois_debugfs_getvalue,
			ois_debugfs_setvalue, "%llx\n");

#endif

struct dentry *debugfs_base=NULL;

void msm_sensor_creat_debugfs(void)
{

	if (!debugfs_base) {
		debugfs_base = debugfs_create_dir("msm_sensor", NULL);
		
		if (!debugfs_base){
			pr_err(": msm_sensor dir creat fail");
		}
	} 
}

int msm_sensor_enable_debugfs(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct dentry  *sensor_dir;
	msm_sensor_debug_info_t *debug_ptr = NULL;

	CDBG("%s:%d: %s  E\n", __func__, __LINE__,s_ctrl->sensordata->sensor_name);

	if (!debugfs_base) {
		
		debugfs_base = debugfs_create_dir("msm_sensor", NULL);
		
		if (!debugfs_base)
			return -ENOMEM;
	} 
	
	debug_ptr = kzalloc(sizeof(msm_sensor_debug_info_t), GFP_KERNEL);
	if (!debug_ptr) {
		pr_err("failed: no memory s_ctrl %p", debug_ptr);
		return -ENOMEM;
	}

	sensor_dir = debugfs_create_dir(s_ctrl->sensordata->sensor_name, debugfs_base);
	if (!sensor_dir)
		return -ENOMEM;

	debug_ptr->s_ctrl = s_ctrl;
	debug_ptr->msm_sensor_reg_data_type = MSM_CAMERA_I2C_WORD_DATA;
	

	if (!debugfs_create_file("datatype", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) debug_ptr, &sensor_debugfs_datatype))
		return -ENOMEM;
	
	if (!debugfs_create_file("address", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) debug_ptr, &sensor_debugfs_address))
		return -ENOMEM;
	
	if (!debugfs_create_file("value", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) debug_ptr, &sensor_debugfs_value))
		return -ENOMEM;
	
	CDBG("%s:%d: %s  E\n", __func__, __LINE__,s_ctrl->sensordata->sensor_name);
	
	return 0;
}
// <---

#if OIS_DEBUG
struct dentry *ois_debugfs_base=NULL;

void msm_ois_creat_debugfs(void)
{

	if (!ois_debugfs_base) {
		ois_debugfs_base = debugfs_create_dir("zte_ois", NULL);
		
		if (!ois_debugfs_base){
			pr_err(": zte_ois dir creat fail");
		}
	} 
}

int msm_ois_enable_debugfs(struct msm_ois_ctrl_t *s_ctrl)
{
	struct dentry  *ois_dir;
	msm_ois_debug_info_t *debug_ptr = NULL;

	printk("%s:%d:  E\n", __func__, __LINE__);

	if (!ois_debugfs_base) {
		
		ois_debugfs_base = debugfs_create_dir("zte_ois", NULL);
		
		if (!ois_debugfs_base){
			pr_err("%s:%d: exit \n", __func__, __LINE__);
			return -ENOMEM;
		}
	} 
	
	debug_ptr = kzalloc(sizeof(msm_ois_debug_info_t), GFP_KERNEL);
	if (!debug_ptr) {
		pr_err("failed: no memory s_ctrl %p", debug_ptr);
		return -ENOMEM;
	}

	ois_dir = debugfs_create_dir(s_ctrl->msm_sd.sd.name, ois_debugfs_base);//msm_ois
	if (!ois_dir){
		pr_err("%s:%d: exit \n", __func__, __LINE__);		
		return -ENOMEM;
	}

	debug_ptr->s_ctrl = s_ctrl;
	debug_ptr->msm_ois_reg_data_type = MSM_CAMERA_I2C_WORD_DATA;
	

	if (!debugfs_create_file("datatype", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_datatype))
	{
		pr_err("%s:%d: exit \n", __func__, __LINE__);
		return -ENOMEM;
	}
	
	if (!debugfs_create_file("address", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_address))
	{
		pr_err("%s:%d: exit \n", __func__, __LINE__);
		return -ENOMEM;
	}
	
	if (!debugfs_create_file("value", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_value))
	{
		pr_err("%s:%d: exit \n", __func__, __LINE__);
		return -ENOMEM;
	}
	
	printk("%s:%d: X\n", __func__, __LINE__);
	
	return 0;
}
#endif

/*
  * add  camera sensor engineering mode  interface
  *
  * by ZTE_YCM_20140710 yi.changming 000005
  */
// --->
typedef struct {
	uint16_t id;
	const char *name;
	uint16_t position;
	uint32_t checksum;
	uint32_t valid_flag;
} msm_sensor_sysdev_info_t;



#define MSM_SENSOR_SYSDEV_NUM_MAX  2

static msm_sensor_sysdev_info_t msm_sensor_sysdev_info[MSM_SENSOR_SYSDEV_NUM_MAX];

static struct bus_type camera_sensor_subsys = {
	.name = "camera",
	.dev_name = "camera",
};


static struct device device_camera_sensor[MSM_SENSOR_SYSDEV_NUM_MAX];


static ssize_t show_msm_sensor_id(struct device *dev,struct device_attribute *attr,char *buf)
{
	int32_t index;
	const char *ptr = NULL;
	uint16_t id;
	uint16_t len = 0;

	if (dev->kobj.name) {
		ptr = dev->kobj.name + strlen(camera_sensor_subsys.name);
//		CDBG("%s:%d   %s   %s  %lu\n", __func__, __LINE__,ptr, dev->kobj.name, strlen(camera_sensor_subsys.name));
		index = (int32_t)simple_strtol(ptr, NULL, 10);
		id = msm_sensor_sysdev_info[index].id;
	} else {
		id = -1;
	}
	
	len = snprintf(buf, PAGE_SIZE, "0x%x\n", id);
	
	CDBG("%s:%d   %s\n", __func__, __LINE__,buf);
		
	return len;
}
/*
  * modify camera sensor engineering mode node permission
  *
  * by ZTE_YCM_20140711 yi.changming 000007
  */
// --->
#if 0
static DEVICE_ATTR(id, S_IRUGO|S_IWUSR, show_msm_sensor_id,NULL);
#else
static DEVICE_ATTR(id, S_IRUGO, show_msm_sensor_id,NULL);
#endif
// <---

static ssize_t show_msm_sensor_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	int32_t index;
	const char *ptr = NULL;
	
	if (dev->kobj.name) {
		ptr = dev->kobj.name + strlen(camera_sensor_subsys.name);
		index = (int32_t)simple_strtol(ptr, NULL, 10);
	} else
		index= 0;
	
	len += sprintf(buf+len,  "%s \n", msm_sensor_sysdev_info[index].name);
	
	CDBG("%s:%d   %s\n", __func__, __LINE__,buf);
	
	return len;
}
/*
  * modify camera sensor engineering mode node permission
  *
  * by ZTE_YCM_20140711 yi.changming 000007
  */
// --->
#if 0
static DEVICE_ATTR(name, S_IRUGO|S_IWUSR, show_msm_sensor_name,NULL);
#else
static DEVICE_ATTR(name, S_IRUGO, show_msm_sensor_name,NULL);
#endif
// <---

static ssize_t show_msm_sensor_position(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	int32_t index;
	const char *ptr = NULL;
	
	if (dev->kobj.name) {
		ptr = dev->kobj.name + strlen(camera_sensor_subsys.name);
		index = (int32_t)simple_strtol(ptr, NULL, 10);
	} else
		index= 0;

	len += sprintf(buf+len,  "%d \n", msm_sensor_sysdev_info[index].position);
	
	CDBG("%s:%d   %s\n", __func__, __LINE__,buf);
	
	return len;
}
/*
  * modify camera sensor engineering mode node permission
  *
  * by ZTE_YCM_20140711 yi.changming 000007
  */
// --->
#if 0
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR, show_msm_sensor_position,NULL);
#else
static DEVICE_ATTR(position, S_IRUGO, show_msm_sensor_position,NULL);
#endif
// <---
static ssize_t show_msm_sensor_checksum(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	int32_t index;
	const char *ptr = NULL;
	
	if (dev->kobj.name) {
		ptr = dev->kobj.name + strlen(camera_sensor_subsys.name);
		index = (int32_t)simple_strtol(ptr, NULL, 10);
	} else
		index= 0;
	
	len += sprintf(buf+len,  "%x \n", msm_sensor_sysdev_info[index].checksum);
	
	CDBG("%s:%d   %s\n", __func__, __LINE__,buf);
	
	return len;
}
static DEVICE_ATTR(checksum, S_IRUGO, show_msm_sensor_checksum,NULL);

static ssize_t show_msm_sensor_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	int32_t index;
	const char *ptr = NULL;
	
	if (dev->kobj.name) {
		ptr = dev->kobj.name + strlen(camera_sensor_subsys.name);
		index = (int32_t)simple_strtol(ptr, NULL, 10);
	} else
		index= 0;
	
	len += sprintf(buf+len,  "%x \n", msm_sensor_sysdev_info[index].valid_flag);
	
	CDBG("%s:%d   %s\n", __func__, __LINE__,buf);
	
	return len;
}
static DEVICE_ATTR(flag, S_IRUGO, show_msm_sensor_flag,NULL);

const struct device_attribute *msm_sensor_dev_attrs[] = {
	&dev_attr_id,
	&dev_attr_name,
	&dev_attr_position,
	&dev_attr_checksum,
	&dev_attr_flag,
};

/*
 * MSM Camera Sensor Sys Device Register
 *
 * 1. Rear Camera Sensor:
 *    id: "/sys/devices/system/camera/camera0/id" 
 *    name: "/sys/devices/system/camera/camera0/name"
 *
 * 2. Front Camera Sensor:
 *    id: "/sys/devices/system/camera/camera1/id"
 *    name: "/sys/devices/system/camera/camera1/name"
 */
void msm_sensor_register_sysdev(struct msm_sensor_ctrl_t *s_ctrl)
{
	static int32_t sysdev_num = 0;
	int32_t index, i;
	int32_t ret;

	if (!sysdev_num) {
		ret= subsys_system_register(&camera_sensor_subsys, NULL);
		if (ret) {
			return;
		}
	}

	if (sysdev_num >= MSM_SENSOR_SYSDEV_NUM_MAX) {
		return;
	}

	index = sysdev_num++;

	msm_sensor_sysdev_info[index].id = s_ctrl->sensordata->slave_info->sensor_id;


 /*
  * by ZTE_YCM_20140728 yi.changming 400015
  */
// --->
	if(s_ctrl->sensordata->sensor_module_name)
		msm_sensor_sysdev_info[index].name = s_ctrl->sensordata->sensor_module_name;
	else
		msm_sensor_sysdev_info[index].name = s_ctrl->sensordata->sensor_name;
  // <---400015

  
	msm_sensor_sysdev_info[index].position = s_ctrl->sensordata->sensor_info->position;

	msm_sensor_sysdev_info[index].checksum= s_ctrl->sensordata->eeprom_checksum;
	msm_sensor_sysdev_info[index].valid_flag = s_ctrl->sensordata->eeprom_valid_flag;

	device_camera_sensor[index].id = index;
	device_camera_sensor[index].bus =  &camera_sensor_subsys;
	
	ret = device_register(&device_camera_sensor[index]);
	if (ret) {
		return;
	}

	for (i = 0; i < ARRAY_SIZE(msm_sensor_dev_attrs); ++i) {
		ret = device_create_file(&device_camera_sensor[index],
					msm_sensor_dev_attrs[i]);
		if (ret) {
			goto register_sysdev_error;
		}
	}

	return;

register_sysdev_error:

	while (--i >= 0) device_remove_file(&device_camera_sensor[index], msm_sensor_dev_attrs[i]);

	device_unregister(&device_camera_sensor[index]);

	return;
}
// <---
