/*
 * isl98608.c  --  audio driver for ISL98608
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "ak4490.h"

#define CONFIG_DEBUG_FS_ISL98608


// ZTE_chenjun
// #define HIFI_LDO_7V

struct isl98608_priv {
	struct i2c_client *i2c;
};
struct isl98608_priv g_l98608;


#if 1
static int isl98608_i2c_read(struct i2c_client *i2c, unsigned int reg)
{

	int ret;

	ret = i2c_smbus_read_byte_data(i2c, (u8)(reg & 0xFF));

	if (ret < 0) {
		pr_err("[LHS]%s reg=%x, ret=%d\n", __func__, reg, ret);
	}

	return ret;
}

static int isl98608_i2c_write(struct i2c_client *i2c, unsigned int reg,
	unsigned int value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(i2c, (u8)(reg & 0xFF), (u8)(value & 0xFF));
	if(ret < 0) {
		pr_err("[LHS]%s reg = %x ,ret=%d\n",__func__,reg, ret);
		return EIO;
	}
	
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS_ISL98608
static ssize_t isl98608reg_data_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int i,value,fpt,ret;
	int regs,rege;
	u8 rx[32]={0};
	ret = fpt = 0;
	regs=0x00;
	rege=0x0e;
	pr_err("[LHS] sungang%s in\n",__func__);
	isl98608_pw_enable(true);
	for(i= regs;i<rege;i++)
		{
		value=isl98608_i2c_read(g_l98608.i2c,i);
		rx[i]=(u8)value ;
		pr_err("[LHS] sungang isl98608 reg show add =%x , data =%x \n", i,value);
		if(value<0)
		break;
		}
	isl98608_pw_enable(false);
	if (i == rege) {
		for (i = regs; i < rege; i++, fpt += 6) {
			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		} 
		return ret;
        }
	else {
		return sprintf(buf, "isl98608 test error!\n");
	}
	return 0;
}

static ssize_t isl98608reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)

{
/*	char *ptr_data = (char *)buf;
	char *p;
	int i, pt_count = 0;
	unsigned short val[20];
	while((p=strsep(&ptr_data,","))){
		if(!*p)
			break;
		if(pt_count>=20)
			break;
		val[pt_count] = simple_strtoul(p,NULL,16);
		pt_count++;
	}
	for(i=0;i<pt_count;i+=2)
		{
			isl98608_i2c_write(g_l98608.i2c,val[i],val[i+1]);
			pr_err("[LHS] sungang isl98608 write reg add=%x , val =%x \n",val[i],val[i+1]);
		}
	return count;*/
	return 0;
}

static DEVICE_ATTR(isl98608reg_data, 0664, isl98608reg_data_show, isl98608reg_data_store);
#endif


void isl98608_power_down(void)
{
	//pr_err("[LHS]%s enter\n",__func__);
	isl98608_i2c_write(g_l98608.i2c,0x05,0x80);
}

void isl98608_power_up(void)
{

	int val_address = 0,val_value = 0;

	//pr_err("[LHS]%s enter\n",__func__);

	isl98608_i2c_write(g_l98608.i2c,0x00,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x01,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x02,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x03,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x04,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x05,0x87);	

#if defined(HIFI_LDO_7V)
	isl98608_i2c_write(g_l98608.i2c,0x06,0x28);
#else
	isl98608_i2c_write(g_l98608.i2c,0x06,0x0d);
#endif

	isl98608_i2c_write(g_l98608.i2c,0x07,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x08,0x00);

#if defined(HIFI_LDO_7V)
	isl98608_i2c_write(g_l98608.i2c,0x09,0x28);
#else
	isl98608_i2c_write(g_l98608.i2c,0x09,0x08);
#endif

	isl98608_i2c_write(g_l98608.i2c,0x0a,0xf0);
	isl98608_i2c_write(g_l98608.i2c,0x0b,0x0a);
	isl98608_i2c_write(g_l98608.i2c,0x0c,0x05);
	isl98608_i2c_write(g_l98608.i2c,0x0d,0x34);

for(val_address = 0x00;val_address<0x0e;val_address++)

{
		 val_value = isl98608_i2c_read(g_l98608.i2c, val_address);
		if(val_value <0 )
		{
			pr_err("[LHS]%s:i2c read failed\n",__func__);			
		}
		//pr_err("[LHS]%s:reg_addr:%x,reg_value:%x\n",__func__,val_address,val_value);
	}
    

}


static int isl98608_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct isl98608_priv *isl98608;
	int ret=0;

	pr_err("%s enter\n",__func__);

	isl98608 = kzalloc(sizeof(struct isl98608_priv), GFP_KERNEL);
	if (isl98608 == NULL) {
		pr_err("[LHS]%s Error: alloc error ! \n",__func__);
		return -ENOMEM;
	}


	if (!i2c_check_functionality(i2c->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("[LHS] %s I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE",__func__);
		return -EIO;
	}


    	isl98608->i2c = i2c;
	i2c_set_clientdata(i2c, isl98608);


	g_l98608.i2c=isl98608->i2c;
#ifdef CONFIG_DEBUG_FS_ISL98608
	ret=device_create_file(&i2c->dev,&dev_attr_isl98608reg_data);
	if(ret)
		{
			pr_err("[LHS] sungang %s error to create reg_data\n",__func__);
		}
	g_l98608.i2c= i2c;
#endif

	
	return ret;
}

static int isl98608_i2c_remove(struct i2c_client *i2c)
{
#ifdef CONFIG_DEBUG_FS_ISL98608
	device_remove_file(&i2c->dev,&dev_attr_isl98608reg_data);
#endif

	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id isl98608_i2c_id[] = {
	{ "isl98608", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, isl98608_i2c_id);

static struct i2c_driver isl98608_i2c_driver = {
	.driver = {
		.name = "isl98608",
		.owner = THIS_MODULE,
	},
	.probe = isl98608_i2c_probe,
	.remove = isl98608_i2c_remove,
	.id_table = isl98608_i2c_id,
};


static int __init isl98608_init(void)
{

	pr_info("%s \n",__func__);

	return i2c_add_driver(&isl98608_i2c_driver);
}

module_init(isl98608_init);

static void __exit isl98608_exit(void)
{
	i2c_del_driver(&isl98608_i2c_driver);
}
module_exit(isl98608_exit);

MODULE_DESCRIPTION("ASoC ISL98608 codec driver");
MODULE_LICENSE("GPL");
