/*
 * ak8157.c  --  audio driver for AK8157
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

// ZTE_chenjun
// #define MSM8996_I2S_MASTER
#define CONFIG_DEBUG_FS_AK8157



struct ak8157_priv {
	struct i2c_client *i2c;
	int rstn;
	int rstn_tmp;
};
struct ak8157_priv ak8157_i2c;
static int ak8157_i2c_read(struct i2c_client *i2c, unsigned int reg)
{

	int ret;

	ret = i2c_smbus_read_byte_data(i2c, (u8)(reg & 0xFF));

	if (ret < 0) {
		pr_err("%s reg=%x, ret=%d\n", __func__, reg, ret);
	}

	return ret;
}

static int ak8157_i2c_write(struct i2c_client *i2c, unsigned int reg,
	unsigned int value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(i2c, (u8)(reg & 0xFF), (u8)(value & 0xFF));
	if(ret < 0) {
		pr_err("[LHS]%s ret=%d\n",__func__, ret);
		return EIO;
	}
	
	return 0;
}
#ifdef CONFIG_DEBUG_FS_AK8157
static ssize_t ak8157reg_data_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int i,value,fpt,ret;
	int regs,rege;
	u8 rx[32]={0};
	ret = fpt = 0;
	regs=0x00;
	rege=0x02;
	pr_err("[LHS] sungang%s in\n",__func__);
	for(i= regs;i<rege;i++)
		{
		value=ak8157_i2c_read(ak8157_i2c.i2c,i);
			//rx[i]=value ;
		rx[i]=(u8)value ;
		pr_err("[LHS] sungang ak8157 reg show add =%x , data =%x \n", i,value);
		if(value<0)
		break;
		}
	if (i == rege) {
		for (i = regs; i < rege; i++, fpt += 6) {
			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		} 
		return ret;
        }
	else {
		return sprintf(buf, "ak8157 test error!\n");
	}
	return 0;
}

static ssize_t ak8157reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)

{
	/*char *ptr_data = (char *)buf;
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
			ak8157_i2c_write(ak8157_i2c.i2c,val[i],val[i+1]);
			pr_err("[LHS] sungang ak8157 write reg add=%x , val =%x \n",val[i],val[i+1]);
		}
	return count;*/
	return 0;
}

static DEVICE_ATTR(ak8157reg_data, 0664, ak8157reg_data_show, ak8157reg_data_store);
#endif



void ak8157_rate_set(int rate)
{

    pr_err("[LHS]%s rate=%d\n", __func__, rate);

    switch (rate) {
		case 192000: // 192K 
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x22);
			break;
		case 96000: // 96K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x12);
			break;
		case 48000: // 48K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x02);
			break;
		case 44100: // 44.1K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x01);
			break;
		default:
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x02);
			break;
	}
	msleep(5);
}


static int ak8157_config_init(struct i2c_client *client)
{

	int ret = 0,ret1=0,ret2=0;
	ret = ak8157_i2c_write(client,0x00,0xe7);

	ret1 = ak8157_i2c_read(client,0x00);
	ret = ak8157_i2c_write(client,0x01,0x00);
	
	ret2= ak8157_i2c_read(client,0x01);
	pr_err("[LHS]%s ret1 = %0x,ret2 = %0x\n",__func__,ret1,ret2);
	return 0;

}
static int ak8157_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak8157_priv *ak8157;
	int ret=0;

	pr_err("%s enter\n",__func__);

	ak8157 = kzalloc(sizeof(struct ak8157_priv), GFP_KERNEL);
	if (ak8157 == NULL) 
		return -ENOMEM;

    ak8157->i2c = i2c;
	i2c_set_clientdata(i2c, ak8157);
	
#if 1
    ak8157->rstn = of_get_named_gpio(i2c->dev.of_node, 
					      "ak8157,rstn-gpio", 0);
	if (!gpio_is_valid(ak8157->rstn)){  
		pr_err("%s error ak8157 rstn ret=%d\n", __func__, ak8157->rstn);
		return -EINVAL;
	}

	ret = gpio_request(ak8157->rstn, "ak8157_rstn_pin");
	if (ret < 0) {
		pr_err("%s(): ak8157_rstn_pin request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ak8157->rstn, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ak8157->rstn, 1);
    	pr_err("[LHS]%s(): reset success! \n",__func__);
#endif

// ZTE_chenjun:TEMP
#if 1
    ak8157->rstn_tmp = of_get_named_gpio(i2c->dev.of_node, 
					      "ak8157,rstn-gpio-tmp", 0);
	if (!gpio_is_valid(ak8157->rstn_tmp)){  
		pr_err("%s error ak8157 rstn_tmp ret=%d\n", __func__, ak8157->rstn_tmp);
		return -EINVAL;
	}

	ret = gpio_request(ak8157->rstn_tmp, "ak8157_rstn_pin_tmp");
	if (ret < 0) {
		pr_err("%s(): ak8157_rstn_pin_tmp request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ak8157->rstn_tmp, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ak8157->rstn_tmp, 1);
    	pr_err("[LHS]%s(): reset tmp success! \n",__func__);
#endif

    	ak8157_config_init(i2c);
    	ak8157_i2c.i2c= i2c;
	ak8157_i2c.rstn=ak8157->rstn;
	ak8157_i2c.rstn_tmp=ak8157->rstn_tmp;
	
#ifdef CONFIG_DEBUG_FS_AK8157
	ret=device_create_file(&i2c->dev,&dev_attr_ak8157reg_data);
	if(ret)
		{
			pr_err("[LHS] sungang %s error to create reg_data\n",__func__);
		}
	ak8157_i2c.i2c= i2c;
#endif
	return ret;
}


void ak8157_clock_open(void)
{
	int ret;
	pr_err("[LHS]%s() !\n",__func__);
#if defined(MSM8996_I2S_MASTER)
	ret=ak8157_i2c_write(ak8157_i2c.i2c,0x00,0x27); // ONLY MCLK to AK4490
#else
	ret=ak8157_i2c_write(ak8157_i2c.i2c,0x00,0xe0); 
	// pr_err("[LHS]%s sungang---0xe0\n",__func__);

	ret=ak8157_i2c_write(ak8157_i2c.i2c,0x00,0x00); // ALL I2S CLK and MCLK to AK4490
	// pr_err("[LHS]%s sungang---0x00\n",__func__);
#endif
}

void ak8157_clock_close(void)
{
	int ret;
	pr_err("[lhs]%s() !\n",__func__);
	ret = ak8157_i2c_write(ak8157_i2c.i2c,0x00,0xe7);
}


static int ak8157_i2c_remove(struct i2c_client *i2c)
{

#ifdef CONFIG_DEBUG_FS_AK8157
	device_remove_file(&i2c->dev,&dev_attr_ak8157reg_data);
#endif

	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id ak8157_i2c_id[] = {
	{ "ak8157", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak8157_i2c_id);

static struct i2c_driver ak8157_i2c_driver = {
	.driver = {
		.name = "ak8157",
		.owner = THIS_MODULE,
	},
	.probe = ak8157_i2c_probe,
	.remove = ak8157_i2c_remove,
	.id_table = ak8157_i2c_id,
};

static int __init ak8157_init(void)
{

	pr_info("%s \n",__func__);

	return i2c_add_driver(&ak8157_i2c_driver);
}

module_init(ak8157_init);

static void __exit ak8157_exit(void)
{
	i2c_del_driver(&ak8157_i2c_driver);
}
module_exit(ak8157_exit);

MODULE_DESCRIPTION("ASoC AK8157 codec driver");
MODULE_LICENSE("GPL");

