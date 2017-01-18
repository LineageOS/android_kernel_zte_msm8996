/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/input/pixcir_ts.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/checksum.h>
#include <linux/random.h>
//---------------------------------------------------------------------
#define I2C_DRIVER_NAME	 	"ndt_stress_i2c"
#define NDT_DRIVER_VERSION	"P8952A10V0.1"
//#define NDT_FW_PATH			"/etc/firmware/ZTE_NDT_0X19_20160401.nfw"
#define NDT_FW_PATH   		"/etc/firmware/NDT_5P5_2560_1440_0X21_A2_CH8.nfw"
#define NDT_CHAR_MAJOR        	103 
#define NDT_DEVICE_NAME        	"ndt_i2c_node"

#define NDT_GPIO_IRQ        		"ndt_gpio_irq"
#define NDT_GPIO_RESET        	"ndt_gpio_reset"
#define NDT_GPIO_POWER       	"ndt_gpio_power"
#define NDT_GPIO_NULL       		"ndt_gpio_unconfig"

#define NDT_IOC_MAGIC				'x'
#define NDT_IOC_HW_RESET   		_IO(NDT_IOC_MAGIC, 1)
#define NDT_IOC_READ 				_IOR(NDT_IOC_MAGIC, 2, int)
#define NDT_IOC_WRITE 				_IOW(NDT_IOC_MAGIC, 3, int)
#define NDT_IOC_INIT_UNIFORMITY	_IOW(NDT_IOC_MAGIC, 4, int)
#define NDT_IOC_READ_STATUS		_IOR(NDT_IOC_MAGIC, 5, void *)

//---------------------------------------------------------------------
#define CONFIG_NDT_USE_COMPATIBLE_TEST
#define CONFIG_NDT_USE_DEBUG
#define CONFIG_NDT_USE_PV_TEST
#define CONFIG_NDT_USE_TEST_I2C
#define NDT_PRESS_USE_PM
#define CONFIG_NDT_USE_UPDATE
#define CONFIG_NDT_USE_CHAR_DEV
//---------------------------------------------------------------------
#define NDT_DEBUG_ON 				1
#define NDT_DEBUG_FUNC_ON 		0
#define IIC_EEPROM 					0
//---------------------------------------------------------------------
// ZTE_modify add custom gain control begin
#define FORCE_MAX_VALUE 512
#define FORCE_MAX_VALUE_FW 1024
extern char forcetouch_vendor ;
static unsigned int value_temp;
// ZTE_modify end
#define NDT_X_MAX_RANGE 1440
#define NDT_Y_MAX_RANGE 2560
#define CHANNEL_NUM  	8
#define POWER_ON_DELAY  10

#define NDT_INFO(fmt,arg...)           printk("<<NDT-INF>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NDT_ERROR(fmt,arg...)          printk("<<NDT-ERR>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NDT_DEBUG(fmt,arg...)          do{\
                                         if(NDT_DEBUG_ON)\
                                         printk("<<NDT-DBG>>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
                                       }while(0)
                                       
#define NDT_DEBUG_FUNC()               do{\
                                         if(NDT_DEBUG_FUNC_ON)\
                                         printk("<<NDT-FUNC>> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
//---------------------------------------------------------------------
static const struct i2c_device_id ndt_stress_id[] = {
	{I2C_DRIVER_NAME, 0},
	{}
};

static struct of_device_id ndt_match_table[] = {
	{.compatible = "ndt,stress-i2c",},
	{ },
};

typedef struct UNIFORMITY_TYPE_T{
	unsigned short press;
	unsigned short average;
	short base1;
	short base2;
	short base3;
	short base4;
	unsigned short param;
	unsigned short x;
	unsigned short y;
	unsigned short is_mid_edge;
	unsigned short unedge_percent;
	unsigned short percent;
	unsigned short signal_sum;
	unsigned short frame_count;
	unsigned short frame_before;
} UNIFORMITY_TYPE;

typedef enum {
	DEBUG_MODE_WEIGHT	= -1,
	DEBUG_MODE_NORMAL 	= 0,
	DEBUG_MODE_DDATA 	= 1,
	DEBUG_MODE_LOG 		= 2,	
	
} DEBUG_TYPE_T;

struct ndt_stress_board_data{
	const char *pwr_reg_name;
	const char *bus_reg_name;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	int irq_gpio;
	int irq_on_state;
	int irq_flags;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int max_y_for_2d;
};

struct ndt_stress_data {
	struct i2c_client *client;
	struct ndt_stress_board_data bdata;
	struct delayed_work int_work;
	struct delayed_work debug_work;
	struct delayed_work update_work;
	struct delayed_work send_coord_work;
	struct workqueue_struct *ndt_queue;
	struct workqueue_struct *ndt_debug_queue;
	struct workqueue_struct *ndt_coord_handle;
	struct class *i2c_class;
	struct mutex stress_value_mutex;
	struct mutex i2c_mutex;
	//struct completion coord_thread_sync;
	wait_queue_head_t status_wait;
	char update_file[128];
	unsigned int custom_gain_value;
	unsigned short ratiox;// ZTE_modify add for stress calibration begin
	unsigned short ratioy;// ZTE_modify add for stress calibration begin
	unsigned short frame_count;
	unsigned short drop_frame_conut;
	unsigned short value1;
	unsigned short value2;
	unsigned short FirmwareVer;
	short sensor[8];
	short dvalue1;
	short dvalue2;
	unsigned short x1;
	unsigned short y1;
	unsigned char status;
	char dmode;
	bool appear;
	bool suspend;
	bool flashprog;
	bool force_update;
	bool power_stat;
	bool update_finished;
	bool uniformity_init;
	volatile bool status_flag;
};

struct ndt_stress_data *gNdtData = NULL;
UNIFORMITY_TYPE gpoint_info;// ZTE_modify add for stress calibration begin

void ndt_i2c_comm_lock(void);
void ndt_i2c_comm_unlock(void);
extern int get_batt_therm_globle(void);
static int ndt_i2c_test(int delay_time);

inline void print_mem(char *buffer, unsigned int len)
{
	unsigned int i = 0;

	printk("\n<<NDT-INF>>");

	for(i = 0; i < len; i++)
		{
		printk("0x%02X ", buffer[i]);

		if((i !=0) && ((i+1) % 8 == 0))	 printk("	");
		
		if((i !=0) && ((i+1) % 16 == 0)) printk("\n<<NDT-INF>>");
		}

	printk("\n");
}

static inline int ndt_Bon(void *p)
{
	if(p == NULL)
		return 1;

	return 0;
}

static int ndt_stress_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	char *buf = NULL;

	NDT_DEBUG_FUNC();

	if (config) {

		if(gpio == gNdtData->bdata.irq_gpio)
			buf = NDT_GPIO_IRQ;
		else if(gpio == gNdtData->bdata.reset_gpio)
			buf = NDT_GPIO_RESET;
		else if(gpio == gNdtData->bdata.power_gpio)
			buf = NDT_GPIO_POWER;
		else
			buf = NDT_GPIO_NULL;
		
		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	NDT_DEBUG_FUNC();

	return retval;
}

static int parse_dt(struct device *dev, struct ndt_stress_board_data *bdata)
{
	int retval;
	u32 value;
	const char *name;
	struct device_node *np = dev->of_node;

	NDT_DEBUG_FUNC();

	if (of_find_property(np, "ndt,irq-gpio", NULL)) {
		bdata->irq_gpio = of_get_named_gpio_flags(np,
				"ndt,irq-gpio", 0, NULL);
	}
	else
		bdata->irq_gpio = -1;

	NDT_INFO("irq_gpio: %d", bdata->irq_gpio);

	retval = of_property_read_u32(np, "ndt,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	NDT_INFO("irq_on_state: %d", bdata->irq_on_state);

	retval = of_property_read_u32(np, "ndt,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	NDT_INFO("irq_flags: %d", bdata->irq_flags);

	retval = of_property_read_string(np, "ndt,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	NDT_INFO("pwr_reg_name: %s", bdata->pwr_reg_name);

	retval = of_property_read_string(np, "ndt,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	NDT_INFO("bus_reg_name: %s", bdata->bus_reg_name);

	if (of_find_property(np, "ndt,power-gpio", NULL)) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"ndt,power-gpio", 0, NULL);
		retval = of_property_read_u32(np, "ndt,power-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_on_state = value;
	} else {
		bdata->power_gpio = -1;
	}

	NDT_INFO("power_gpio: %d, power_on_state %d", bdata->power_gpio, bdata->power_on_state);

	if (of_find_property(np, "ndt,power-delay-ms", NULL)) {
		retval = of_property_read_u32(np, "ndt,power-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_delay_ms = value;
	} else {
		bdata->power_delay_ms = 0;
	}

	NDT_INFO("power_delay_ms: %d", bdata->power_delay_ms);

	if (of_find_property(np, "ndt,reset-gpio", NULL)) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"ndt,reset-gpio", 0, NULL);
		retval = of_property_read_u32(np, "ndt,reset-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_on_state = value;
		retval = of_property_read_u32(np, "ndt,reset-active-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_active_ms = value;
	} else {
		bdata->reset_gpio = -1;
	}

	if (of_find_property(np, "ndt,reset-delay-ms", NULL)) {
		retval = of_property_read_u32(np, "ndt,reset-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_delay_ms = value;
	} else {
		bdata->reset_delay_ms = 0;
	}

	NDT_INFO("reset_gpio: %d, reset_on_state: %d, reset_active_ms %d, reset_delay_ms %d", 
					bdata->reset_gpio, bdata->reset_on_state, bdata->reset_active_ms, bdata->reset_delay_ms);



	NDT_DEBUG_FUNC();

	return 0;
}

/*static int ndt_hw_reset(void)
{
	const struct ndt_stress_board_data *bdata = &gNdtData->bdata;
	
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		mdelay(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		mdelay(bdata->reset_delay_ms);
		return 0;
	}

	return -1;
}
*/
static int ndt_stress_i2c_write( struct i2c_client *client, u16 addr, int len, void *txbuf )
{
	int ret;
	unsigned char* buf;
	struct i2c_msg msg;
	
	NDT_DEBUG_FUNC();
	
	if ((txbuf == NULL) || (len == 0)) {
		NDT_ERROR("txbuf is null, len is 0. write 0x%04X", addr);
		return -EINVAL;
	}

	if (in_atomic())
		buf = (unsigned char*)kmalloc(len+1, GFP_ATOMIC);
	else
		buf = (unsigned char*)kmalloc(len+1, GFP_KERNEL);
	
	if (!buf)
		return -ENOMEM;
	
	memset(buf, 0, len+1);
	buf[0] = addr & 0xff;
	memcpy(buf+1, txbuf, len);
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len+1;
	msg.buf = buf;
	
	ndt_i2c_comm_lock();
	ret = i2c_transfer(client->adapter, &msg, 1);
	ndt_i2c_comm_unlock();
	
	if (ret < 0) {
		NDT_ERROR("%s:write error! err_code:%d, addr:0x%04X", __func__, ret, addr);
		kfree(buf);
		return ret;
	}
	
	kfree(buf);
	
	NDT_DEBUG_FUNC();
	
	return len;
}

static int ndt_stress_i2c_read(struct i2c_client *client, u16 addr, u16 len, void *rxbuf)
{
	int ret = -1;
	struct i2c_msg msg[2];
	char i2c_buffer[128] = {0};

	NDT_DEBUG_FUNC();

	if (( rxbuf == NULL ) ||(len == 0))
		{
		NDT_ERROR("rxbuf is null, len is 0. read 0x%04X", addr);
		return -1;
		}

	memset(msg, 0, sizeof(msg));
	memset(i2c_buffer, 0, sizeof(i2c_buffer));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *)&addr;

	msg[1].addr = client->addr;//client->addr | I2C_DMA_FLAG;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = i2c_buffer;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, msg, 2);

	if(ret<0)
		{
		NDT_ERROR("ndt_stress_i2c_read addr 0x%04X", addr);
		ndt_i2c_comm_unlock();
		return -1;
		}

	ndt_i2c_comm_unlock();

	memcpy(rxbuf, i2c_buffer, len);

	NDT_DEBUG_FUNC();

	return len;
}

static int ndt_stress_write_eeprom(struct i2c_client *client,unsigned short reg,unsigned char* datbuf,int ByteNo)
{	
	unsigned char* buf;
	struct i2c_msg msg;
	int ret;
//	int count=0;

	//NDT_INFO("i2c_client addr: 0x%02X",client->addr);

	if(!datbuf) return -1;
	buf = (unsigned char*)kmalloc(ByteNo+2,GFP_KERNEL);
	if(!buf) return -1;
	
	memset(buf,0,ByteNo+2);
	buf[0]=(reg>>8)&0xff;
	buf[1]=reg&0xff;
	memcpy(buf+2,datbuf,ByteNo);

	//NDT_INFO("reg addr: 0x%02x,0x%02X",buf[0],buf[1]);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+2;
	msg.buf = buf;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, &msg, 1);

	ndt_i2c_comm_unlock();

	if(ret<0)
		NDT_ERROR("i2c_master_send Error ! err_code:%d, reg: 0x%04X",ret, reg);
	//else
		//NDT_INFO("i2c_master_send OK !");

	kfree(buf);

	return ret;
}

#ifdef CONFIG_NDT_USE_UPDATE
static int ndt_stress_read_eeprom(struct i2c_client *client,unsigned short reg,unsigned char* datbuf,int ByteNo)
{
	struct i2c_msg msg[2];
	int ret;
	unsigned char reg16[2];
//	int count=0;

	//NDT_INFO("i2c_client addr: 0x%02X",client->addr);

	if(!datbuf) return -1;
	
	reg16[0] = (reg>>8)&0xff;
	reg16[1] = reg&0xff;
	
	//NDT_INFO("reg addr: 0x%02X,0x%02X",reg16[0],reg16[1]);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg16;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = datbuf;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, msg, 2);

	ndt_i2c_comm_unlock();
	
	if(ret<0)
		NDT_ERROR("i2c_master_send Error ! err_code:%d, reg: 0x%04X",ret, reg);
	//else
		//NDT_INFO("i2c_transfer OK !\n");

	return ret;
}
#endif
static int ndt_stress_jump_wait(void)
{
	unsigned char erase_cmd[10];
	unsigned short reg;
	int ByteNo = 0;

	reg = IIC_EEPROM;
	//erase_cmd[ByteNo++] = 0;
	//erase_cmd[ByteNo++] = 0;
	erase_cmd[ByteNo++] = 0x7e;
	erase_cmd[ByteNo++] = 0xe7;
	erase_cmd[ByteNo++] = 0xee;
	erase_cmd[ByteNo++] = 0x77;

	if(ndt_stress_write_eeprom(gNdtData->client,reg,erase_cmd,ByteNo) < 0)
		return -1;

	return 0;	
}

static int ndt_stress_power_switch(bool SwFlag, bool ForceFlag)
{

	struct ndt_stress_board_data *bdata = NULL;
	
	if(gNdtData == NULL) 
		return -1;

	bdata = &gNdtData->bdata;

	if(gNdtData->power_stat == SwFlag)
		{
		NDT_ERROR("power mode already in power %s", SwFlag? "on":"down");
		return 0;
		}

	if(SwFlag == true)
		{
		if (bdata->power_gpio >= 0) {
			gpio_set_value(bdata->power_gpio, bdata->power_on_state);
			msleep(bdata->power_delay_ms);
			}
		
		/*if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
			mdelay(bdata->reset_active_ms);
			}*/
		
		gNdtData->power_stat = true;

		NDT_INFO("+++++++++power on");
		}
	else if((gNdtData->flashprog == false) || (ForceFlag == true))
		{
		/*if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
			mdelay(bdata->reset_active_ms);
			}*/

		if (bdata->power_gpio >= 0) {
			gpio_set_value(bdata->power_gpio, !bdata->power_on_state);
			msleep(bdata->power_delay_ms);
			}

		gNdtData->power_stat = false;

		NDT_INFO("------------power down");
		}
	
	return 0;
}


#ifdef NDT_PRESS_USE_PM
int ndt_stress_suspend(void)
{
	struct ndt_stress_board_data *bdata = NULL;
	
	NDT_INFO("start suspend!");

	if(ndt_Bon(gNdtData) || (gNdtData->appear == false))
		{
		NDT_ERROR("gNdtData is NULL & appear is false");
		return -1;
		}
	
	if(gNdtData->suspend == true)
		{
		NDT_INFO("device already in suspended!!!!");
		NDT_INFO("suspend end!");
		return -1;
		}

	if(gNdtData->flashprog == true)
		{
		NDT_INFO("NDT do not suspend in prog flash mode !!!!");
		NDT_INFO("suspend end!");
		return 0;
		}
	
	bdata = &gNdtData->bdata;

	ndt_stress_power_switch(false, false);

	gNdtData->suspend = true;
	gNdtData->dmode = DEBUG_MODE_NORMAL;
	
	NDT_INFO("suspend end!");

	return 0;
}

EXPORT_SYMBOL(ndt_stress_suspend);

int ndt_stress_resume(void)
{
	struct ndt_stress_board_data *bdata = NULL;
	
	NDT_INFO("start resume!");
	
	if(ndt_Bon(gNdtData) || (gNdtData->appear == false))
		{
		NDT_ERROR("gNdtData is NULL OR appear is false");
		return -1;
		}
	
	if(gNdtData->suspend == false)
		{
		NDT_INFO("device already in resume!!!!");
		NDT_INFO("resume end!");
		return -1;
		}
	
	bdata = &gNdtData->bdata;
		
	ndt_stress_power_switch(true, false);
	msleep(POWER_ON_DELAY);

	//ndt_hw_reset();

	ndt_stress_jump_wait();

	gNdtData->suspend = false;
	
	NDT_INFO("resume end!");

	return 0;
}

EXPORT_SYMBOL(ndt_stress_resume);
#else
int ndt_stress_suspend(void)
{
	return 0;
}

EXPORT_SYMBOL(ndt_stress_suspend);

int ndt_stress_resume(void)
{
	return 0;
}

EXPORT_SYMBOL(ndt_stress_resume);

#endif

static int ndt_stress_set_gpio(struct ndt_stress_data *ndt_data)
{
	int retval;
	const struct ndt_stress_board_data *bdata = &ndt_data->bdata;

	NDT_DEBUG_FUNC();

	retval = ndt_stress_gpio_setup(
                       	bdata->irq_gpio,
                     	true, 1, 0);
	if (retval < 0) {
		NDT_ERROR("Failed to configure attention GPIO");
		goto err_gpio_irq;
	}

	if (bdata->power_gpio >= 0) {
		retval = ndt_stress_gpio_setup(
				bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			NDT_ERROR("Failed to configure power GPIO");
			goto err_gpio_power;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = ndt_stress_gpio_setup(
				bdata->reset_gpio,
				true, 0, !bdata->reset_on_state);//input
				//true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			NDT_ERROR("Failed to configure reset GPIO");
			goto err_gpio_reset;
		}
	}

	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		mdelay(bdata->power_delay_ms);
	}

	/*if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		mdelay(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		mdelay(bdata->reset_delay_ms);
	}*/

	gNdtData->power_stat = true;

	NDT_DEBUG_FUNC();
	
	return 0;

err_gpio_irq:
	if(bdata->irq_gpio >= 0)
		ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);
err_gpio_reset:
	if(bdata->power_gpio >= 0)
		ndt_stress_gpio_setup(bdata->power_gpio, false, 0, 0);

err_gpio_power:
	ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);

	return retval;
}

void ndt_i2c_comm_lock(void)
{
	NDT_DEBUG_FUNC();
	
	if((gNdtData == NULL) || (gNdtData->appear == false))
		return;

	//NDT_INFO("lock ++++++++++++");
	
	mutex_lock(&(gNdtData->i2c_mutex));

	NDT_DEBUG_FUNC();

	return;
}

EXPORT_SYMBOL(ndt_i2c_comm_lock);

void ndt_i2c_comm_unlock(void)
{
	NDT_DEBUG_FUNC();
	
	if((gNdtData == NULL) || (gNdtData->appear == false))
		return;
	
	mutex_unlock(&(gNdtData->i2c_mutex));

	//NDT_INFO("lock ----------------");

	NDT_DEBUG_FUNC();

	return;
}

EXPORT_SYMBOL(ndt_i2c_comm_unlock);

static void ndt_update_send_coord_work(struct work_struct *work)
{
	unsigned char data_no =0;
	unsigned char data = 1;
	unsigned char buffer[5] = {0};
	int batt_temp = 0;
	unsigned short press_buf1 = 0;
	unsigned short x1 = 0, y1 = 0;
	unsigned char status = 0;

	if(gNdtData->suspend == true)
		return;

//------------------------------------------------------------------------------------
//get data
	x1 = gNdtData->x1;
	y1 = gNdtData->y1;
	status = gNdtData->status;
//------------------------------------------------------------------------------------
//send coord
	memcpy(buffer, (char *)&x1, sizeof(x1));
	
	memcpy(buffer + sizeof(x1), (char *)&y1, sizeof(y1));
	
	memcpy(buffer + sizeof(x1)+ sizeof(y1), &status, sizeof(status));

	ndt_stress_i2c_write(gNdtData->client, 0x11, sizeof(buffer), buffer);
//------------------------------------------------------------------------------------
//send coord num
	data = status ? 1 : 0;

	ndt_stress_i2c_write(gNdtData->client, 0x10, sizeof(char),  &data);

    /* Clean the press value after finger up */
	if(!status){
        gNdtData->value1 = 0;
	 gNdtData->value2 = 0;
	}

	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("###### x1:0x%04X y1:0x%04X status:%u", x1, y1, status);
//------------------------------------------------------------------------------------
	if(status)
		{
		batt_temp = 25;//get_batt_therm_globle();//send therm
		ndt_stress_i2c_write(gNdtData->client, 0x52, sizeof(batt_temp), (u8 *)&batt_temp);

		ndt_stress_i2c_read(gNdtData->client, 0x20, 1, &data_no);

		if(gNdtData->dmode == DEBUG_MODE_LOG)
			NDT_INFO("###### data ready:%u",data_no);
		
		if(data_no > 0)//get stress
			{
			ndt_stress_i2c_read(gNdtData->client, 0x21, sizeof(press_buf1), &press_buf1);
			mutex_lock(&(gNdtData->stress_value_mutex));
			gNdtData->value1 = press_buf1;
			
			if(gNdtData->dmode == DEBUG_MODE_LOG)
				NDT_INFO("###### read press value:%u", press_buf1);
	
			mutex_unlock(&(gNdtData->stress_value_mutex));

			data_no = 0;
			ndt_stress_i2c_write(gNdtData->client, 0x20, 1, &data_no);
			}
		else
			NDT_INFO("###### data ready: %u", data_no);
		}
//------------------------------------------------------------------------------------
}

int ndt_get_max_coord(unsigned short max_x,  unsigned short max_y)
{
       if(gNdtData == NULL)
	   	return -1;
	if(!max_x || !max_y)
		{
		gNdtData->ratiox = 0;
		gNdtData->ratioy = 0;
		}
	else
		{
		gNdtData->ratiox = NDT_X_MAX_RANGE * 100/max_x;
		gNdtData->ratioy = NDT_Y_MAX_RANGE * 100/max_y;
		}

	NDT_INFO("tp coord info, max x: %u, max y: %u", max_x, max_y);

	return 0;
}

EXPORT_SYMBOL(ndt_get_max_coord);

static int ndt_stress_get_base_info(unsigned short x1,  unsigned short y1, unsigned char status)
{
	unsigned char data_no =0;
	unsigned char data = 1;
	unsigned char buffer[5] = {0};
	int batt_temp=260;
	UNIFORMITY_TYPE frame;
	unsigned short x_temp, y_temp;

	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("*************x1:0x%04X y1:0x%04X status:%u", x1, y1, status);
	
	memset(&frame, 0, sizeof(frame));

       /*****modify by pzh-2016.2.20*****/
	//gNdtData->ratiox = NDT_X_MAX_RANGE * 100/max_x;
	//gNdtData->ratioy = NDT_Y_MAX_RANGE * 100/max_y;
	//x_temp =(unsigned short) ((u32)x1 * gNdtData->ratiox / 100);
	//y_temp =(unsigned short) ((u32)y1 * gNdtData->ratioy / 100);
	//x_temp =(unsigned short) ((u32)x1 * (NDT_X_MAX_RANGE * 100/1439) / 100);
	//y_temp =(unsigned short) ((u32)y1 * (NDT_X_MAX_RANGE * 100/2559) / 100);
	x_temp =(unsigned short) (x1);
	y_temp =(unsigned short) (y1);
	/******************************/
	//printk("NDT---%s,x_temp=%d,y_temp=%d",__func__,x_temp,y_temp);
	

	memcpy(buffer, (char *)&x_temp, sizeof(x_temp));
	memcpy(buffer + sizeof(x_temp), (char *)&y_temp, sizeof(y_temp));
	memcpy(buffer + sizeof(x_temp)+ sizeof(y_temp), &status, sizeof(status));
 
	ndt_stress_i2c_write(gNdtData->client, 0x11, sizeof(buffer), buffer);

	data =  status ? 1 : 0;
      	ndt_stress_i2c_write(gNdtData->client, 0x10, sizeof(char),  &data);

	//get_batt_therm_globle();//send therm
	ndt_stress_i2c_write(gNdtData->client, 0x52, sizeof(batt_temp), (u8 *)&batt_temp);

	ndt_stress_i2c_read(gNdtData->client, 0x61, 1, &data_no);

	if(data_no == 8)
		{
		ndt_stress_i2c_read(gNdtData->client, 0x62, sizeof(frame.signal_sum), &frame.signal_sum);

		ndt_stress_i2c_read(gNdtData->client, 0x63, sizeof(frame.base1), &frame.base1);

		ndt_stress_i2c_read(gNdtData->client, 0x64, sizeof(frame.base2), &frame.base2);

		ndt_stress_i2c_read(gNdtData->client, 0x65, sizeof(frame.base3), &frame.base3);

		ndt_stress_i2c_read(gNdtData->client, 0x66, sizeof(frame.base4), &frame.base4);

		ndt_stress_i2c_read(gNdtData->client, 0x67, sizeof(frame.param), &frame.param);

		ndt_stress_i2c_read(gNdtData->client, 0x68, sizeof(frame.average), &frame.average);

		ndt_stress_i2c_read(gNdtData->client, 0x69, sizeof(frame.press), &frame.press);

		data = 0;
		ndt_stress_i2c_write(gNdtData->client, 0x61, sizeof(char),  &data);

		frame.x = x_temp;

		frame.y = y_temp;

		gNdtData->frame_count = gNdtData->frame_count + 1;

		if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("touch down:%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d", frame.press, frame.signal_sum, frame.base1, frame.base2, frame.base3, frame.base4, frame.param, frame.average,
			frame.x, frame.y, frame.is_mid_edge, frame.unedge_percent, frame.percent);
		}
	else
		NDT_INFO("data_no: %d", data_no);

	if(x_temp && y_temp)
		{
		if((frame.press > gpoint_info.press) && (gNdtData->frame_count > gNdtData->drop_frame_conut))
			{
			memcpy(&gpoint_info, &frame, sizeof(UNIFORMITY_TYPE));
			gpoint_info.frame_before = gNdtData->frame_count;
			}
		}
	else
		{
		if((gNdtData->frame_count > gNdtData->drop_frame_conut))
			{
			gNdtData->status_flag = 1;
			wake_up_interruptible(&gNdtData->status_wait);
			}
		else
			{
			NDT_INFO("touch tremble found!!!frame count %d", gNdtData->frame_count);
			memset(&gpoint_info, 0, sizeof(gpoint_info));
			gNdtData->frame_count = 0;
			}
		}
	
	return data_no;
}


int ndt_stress_set_vlaue(unsigned short x1,  unsigned short y1, unsigned char status)
{
	NDT_DEBUG_FUNC();
	
	if (gNdtData == NULL) {
		NDT_ERROR("gNdtData == NULL");
		return -EINVAL;
	}

	if ((gNdtData->flashprog == true) && (gNdtData->uniformity_init == 1))//use for test
		ndt_stress_get_base_info(x1,y1,status);
	
	if ((gNdtData->appear == false) || (gNdtData->dmode == DEBUG_MODE_DDATA)
		||(gNdtData->flashprog == true))
		return -EINVAL;

	if (gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("*************x1:0x%04X y1:0x%04X status:%u", x1, y1, status);

	gNdtData->x1 = x1;
	gNdtData->y1 = y1;
	gNdtData->status = status;

	if (gNdtData->ndt_coord_handle != NULL)
		queue_delayed_work(gNdtData->ndt_coord_handle, &gNdtData->send_coord_work, 0);
	else{
		NDT_ERROR("gNdtData->ndt_coord_handle = NULL!");
		return -EINVAL;
	}

	NDT_DEBUG_FUNC();

	return 0;
}

EXPORT_SYMBOL(ndt_stress_set_vlaue);

int ndt_stress_get_vlaue(unsigned int *value)
{
	NDT_DEBUG_FUNC();

	if ((gNdtData == NULL) || (gNdtData->appear == false)
		|| (gNdtData->dmode == DEBUG_MODE_DDATA) || (gNdtData->flashprog == true))
		return -EINVAL;

	if (value == NULL)
	{
		NDT_ERROR("value == NULL");
		goto ExitLoop;
	}

	mutex_lock(&(gNdtData->stress_value_mutex));
	
    // ZTE_modify add custom gain control begin
	*value = gNdtData->custom_gain_value * (unsigned int) (gNdtData->value1) / 100;
	
	if (*value > FORCE_MAX_VALUE_FW)
		*value = value_temp;
	else
		value_temp = *value;
	
	if (*value >= FORCE_MAX_VALUE)
		*value = FORCE_MAX_VALUE - 1;

	*value = (*value) >> 1;

	*value = (*value) ? (*value) : 1;

	gNdtData->value2 = *value;
	// ZTE_modify end

	mutex_unlock(&(gNdtData->stress_value_mutex));
	
	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("*************get value: %u", *value);

ExitLoop:
	NDT_DEBUG_FUNC();

	return 0;
}

EXPORT_SYMBOL(ndt_stress_get_vlaue);

#ifdef CONFIG_NDT_USE_UPDATE
/*
static int ndt_send_update_finish_flag(void)
{
	unsigned char debug_mode = 0;
	unsigned short debug_data = 0;
	unsigned char i = 20;

	NDT_INFO("into!!!");

	debug_mode = 0x20;
	ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(debug_mode), &debug_mode);

	debug_data = 0x01;
	ndt_stress_i2c_write(gNdtData->client, 0x62, sizeof(debug_data), &debug_data);

	debug_mode = 0x01;
	ndt_stress_i2c_write(gNdtData->client, 0x61, sizeof(debug_mode), &debug_mode);

	do
		{
		ndt_stress_i2c_read(gNdtData->client, 0x61, 1, &debug_mode);
		msleep(5);
		}while(i-- && debug_mode);

	if(debug_mode !=0)
		{
		NDT_INFO("write update flag failed!!!");
		return -1;
		}

	debug_mode = 0x00;
	ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(debug_mode), &debug_mode);

	NDT_INFO("write update flag success!!!");

	return 0;
}
*/
static int ndt_fwu_burn_fw(unsigned char* buf,unsigned int len)
{
	struct i2c_client *force_key_client = gNdtData->client;
	unsigned char erase_cmd[10];
	unsigned short reg;
	int ByteNo = 0;
	int pos = 0;
	unsigned char* read_buf = NULL;
	int ret = 1;
	int i=0;
	int number=0;

	NDT_INFO("burn fw start");
	
	if(len % 128 != 0)
		{
		NDT_ERROR("burn len is not 128*");
		return -1;//no need remove device, can not return -1
		}

	read_buf = (unsigned char*)kmalloc(len,GFP_KERNEL);
	if(read_buf == NULL)
		{
		NDT_ERROR("kmalloc failed");
		return -1;//no need remove device, can not return -1
		}
	
	do{
		NDT_INFO("burn eeprom number: %d",number+1);

		ret = 0;
		ByteNo = 0;
			
		//reset
		//ndt_hw_reset();
		ndt_stress_power_switch(false, true);
		msleep(10);
		ndt_stress_power_switch(true, false);
		msleep(POWER_ON_DELAY);
		
		//erase flash
		reg = IIC_EEPROM;
		//erase_cmd[ByteNo++] = 0;
		//erase_cmd[ByteNo++] = 0;
		erase_cmd[ByteNo++] = 0xaa;
		erase_cmd[ByteNo++] = 0x55;
		erase_cmd[ByteNo++] = 0xa5;
		erase_cmd[ByteNo++] = 0x5a;

		if(ndt_stress_write_eeprom(force_key_client,reg,erase_cmd,ByteNo)>0)
			NDT_INFO("send cmd success---reg=0x%02x,ByteNo=%d",reg,ByteNo);
	  	else
	  		{
	  		ret = -2;
			NDT_ERROR("erase flash failed!");
			goto failedloop;
	  		}
		
		mdelay(2000);
		
		//write eeprom
		pos = 0;
		reg = IIC_EEPROM;
		ByteNo = 128;
		NDT_INFO("start writing eeprom!");
		while(pos < len)
			{
			if(ndt_stress_write_eeprom(force_key_client,reg,buf+pos,ByteNo) < 0)
				{
				ret = -2;
				NDT_ERROR("write eeprom failed!");
				goto failedloop;
				}
			
			pos += ByteNo;
			reg += ByteNo;
			mdelay(10);
			}

		NDT_INFO("write end! reg=0x%02x,ByteNo=%d",reg,ByteNo);

		mdelay(100);

		//read eeprom and check
		pos = 0;
		reg = IIC_EEPROM;
		ByteNo = 128;
		while(pos < len)
			{
			if(ndt_stress_read_eeprom(force_key_client,reg,read_buf+pos,ByteNo) < 0)
				{
				ret = -2;
				NDT_ERROR("read eeprom failed!\n");
				goto failedloop;
				}
			
			pos += ByteNo;
			reg += ByteNo;
			mdelay(10);
	
			}

		NDT_INFO("read reg=0x%02x,ByteNo=%d",reg,ByteNo);

		//check
		if(csum_partial(buf, len, 0) != csum_partial(read_buf, len, 0))
			{
			for(i = 0;i < len;i++)
				{
				if(buf[i] != read_buf[i])
					{
					ret = -2;
					NDT_ERROR("[0x%08X][0x%08X][0x%08X]burn check error!\n", i, buf[i], read_buf[i]);
					break;
					}
				}

			ret = -2;
			NDT_ERROR("checksum failed!!!");
			}

		number++;
	}while(number < 3 && 0 != ret);

	if(0 != ret)
		{
		NDT_ERROR("burn eeprom fail!\n");
		goto failedloop;
		}
	else
		NDT_INFO("burn eeprom succeed!\n");
	
	//exit burn
	//ndt_hw_reset();
	ndt_stress_power_switch(false, true);
	msleep(10);
	ndt_stress_power_switch(true, false);
	msleep(POWER_ON_DELAY);

	reg = IIC_EEPROM;
	ByteNo = 0;
	//erase_cmd[ByteNo++] = 0;
	//erase_cmd[ByteNo++] = 0;
	erase_cmd[ByteNo++] = 0x7e;
	erase_cmd[ByteNo++] = 0xe7;
	erase_cmd[ByteNo++] = 0xee;
	erase_cmd[ByteNo++] = 0x77;
	
	if(ndt_stress_write_eeprom(force_key_client,reg,erase_cmd,ByteNo)>0)
		NDT_INFO("reg=0x%02x,ByteNo=%d\n",reg,ByteNo);
	else
		{
		ret = -3;
		NDT_ERROR("exit burn failed!\n");
		}

	msleep(2000);

failedloop:	
	if(read_buf != NULL)
		{
		kfree(read_buf);
		read_buf = NULL;
		}

	NDT_INFO("burn fw end\n");

	return ret;
}

static int ndt_fwu_start_reflash(void)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos;
	unsigned short file_fw_ver;
	int ret = 0;
	unsigned int len=0;
	unsigned short firmware_crc = 0;
	char fw_data_len[4];
	char *fw_data = NULL;
	char *filename = NULL;
	unsigned int try_count = 1;

	if(gNdtData->force_update == true)
		filename = gNdtData->update_file;
	else
		filename = NDT_FW_PATH;

	NDT_INFO("#### force_update %u, update FW name \"%s\"",gNdtData->force_update, filename);

	do
		{
		fp = NULL;
		fp = filp_open(filename, O_RDONLY, 0);
		msleep(100 + try_count * 500);
		NDT_INFO("try to request firmware: %d", try_count);
		}while(IS_ERR(fp) && (try_count++ <= 10));

	if (IS_ERR(fp))
		{
		NDT_ERROR("open \"%s\" failed !!!", filename);
		ret = -1;
		goto ExitLoop;
		}
	
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 8;
	vfs_read(fp, (char *)&file_fw_ver, sizeof(file_fw_ver), &pos);
	NDT_INFO("read file_fw_ver: 0x%04X", ntohs(file_fw_ver));
	
	NDT_INFO("gNdtData->FirmwareVer = 0x%04X",gNdtData->FirmwareVer);
	if((gNdtData->FirmwareVer < 0x1A) && (gNdtData->force_update == false))
		{
		NDT_INFO("Old sensor,No need to update FW !!!");
		goto ExitLoop;
		}

	pos = 0x0c;
	vfs_read(fp, fw_data_len, 4, &pos);
	NDT_INFO("read fw_data_len: 0x%02x%02x%02x%02x", fw_data_len[0],fw_data_len[1],fw_data_len[2],fw_data_len[3]);
	len = (int)((unsigned int)fw_data_len[3]<<0)|((unsigned int)fw_data_len[2]<<8)|((unsigned int)fw_data_len[1]<<16)|(fw_data_len[0]<<24);

	pos = 0x10;
	vfs_read(fp, fw_data_len, 2, &pos);
	NDT_INFO("read firmware crc: 0x%02x%02x", fw_data_len[0],fw_data_len[1]);
	firmware_crc = (unsigned short)(((unsigned int)fw_data_len[1]<<16)|(fw_data_len[0]<<24));

	if((gNdtData->force_update == false) && (gNdtData->FirmwareVer < 0x100) && (ntohs(file_fw_ver) <= gNdtData->FirmwareVer))
		{
		ret = 1;
		NDT_INFO("No need to update FW");
		goto ExitLoop;
		}

         if((gNdtData->force_update == false) &&(gNdtData->FirmwareVer == 8))
		{
		ret = 1;
		NDT_INFO("fw version =8 and No need to update FW");
		goto ExitLoop;
		}

	   
	fw_data = (char*)kmalloc(len,GFP_KERNEL);
	if(fw_data == NULL)
		{
		ret = -1;
		NDT_INFO("kmalloc failed!!!");
		goto ExitLoop;
		}
	
	pos = 0x100;
	vfs_read(fp, fw_data, len, &pos);

	ret = ndt_fwu_burn_fw(fw_data,len);
	if(ret != 0)
		NDT_ERROR("Burn FW failed!!!");
	
ExitLoop:
	if (!IS_ERR(fp))
		{
		filp_close(fp,NULL);
		fp = NULL;
		set_fs(fs);
		}

	if(fw_data)
		{
		kfree(fw_data);
		fw_data = NULL;
		}

	return ret;
}

static void ndt_update_handle_work(struct work_struct *work)
{
	int ret = 0;
	
	NDT_DEBUG_FUNC();
	
	NDT_INFO("update handle work into");

	if(gNdtData->flashprog == true)
		{
		NDT_ERROR("device already in flash prog mode");
		return;
		}

	ndt_stress_power_switch(true, false);
	msleep(POWER_ON_DELAY);
	
	gNdtData->flashprog = true;

	ret = ndt_fwu_start_reflash();
	if((ret == 1) || (ret == -1))
		{
		if(gNdtData->appear == true)
			{
			NDT_INFO("do nothing!!!");
			goto finishloop;
			}
		}
	else if(ret == 0)
		{
		if(!ndt_i2c_test(0))
			{
			NDT_INFO("Update firmware success!!!");
			goto finishloop;
			}
		}

	ndt_stress_power_switch(false, true);//failed, power down

finishloop:	
	gNdtData->force_update = false;

	gNdtData->flashprog = false;

	gNdtData->update_finished = true;
	
	NDT_DEBUG_FUNC();

	return;
}

static void ndt_force_firmware_update(void)
{
	NDT_INFO("!!!-_-!!!force update firmware !!!-_-!!!");

	memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));
	strcpy(gNdtData->update_file, NDT_FW_PATH);
	
	gNdtData->dmode = 0;
	gNdtData->force_update = true;

	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(10000));
}

#endif

#ifdef CONFIG_NDT_USE_COMPATIBLE_TEST
static int compatible_proc_show(struct seq_file *m, void *v)
{
	
	NDT_DEBUG_FUNC();

	if(gNdtData->update_finished == true)
		{
		if(gNdtData->appear == true)
			seq_printf(m, "yes");
		else
			seq_printf(m, "no");
		}
	else
		seq_printf(m, "unfinish");

	NDT_DEBUG_FUNC();

	return 0;
}

static int compatible_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, compatible_proc_show, NULL);
}

static const struct file_operations compatible_proc_fops = {
	.open		= compatible_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
};

static int compatible_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/compatible_test", 0x664, NULL,
			  &compatible_proc_fops);
	if (!res)
		{
		NDT_ERROR("failed to create compatible_test!!!");
		return -ENOMEM;
		}

	NDT_INFO("created compatible_test");

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif


#ifdef CONFIG_NDT_USE_DEBUG

static void ndt_debug_handle_work(struct work_struct *work)
{
	struct i2c_client *client = gNdtData->client;
	unsigned char ReadFlag = 0, i;
	short *ptr = NULL;
	short buffer[16] = {0} ;
	int ret;
	unsigned char temp = 0;

	NDT_DEBUG_FUNC();
	
	if((gNdtData != NULL) && (gNdtData->dmode) && (gNdtData->appear == true) 
		&& (gNdtData->suspend == false) && (gNdtData->flashprog == false))
		{
		if(gNdtData->dmode == DEBUG_MODE_DDATA)
			{
			ret = ndt_stress_i2c_read(client, 0x60, 1, &temp);//save value of add 0x60
			if(ret < 0 )	goto retry_loop;
			
			ReadFlag = 16;
			ndt_stress_i2c_write(gNdtData->client, 0x60, 1, &ReadFlag);//mode 16
			
			ret = ndt_stress_i2c_read(client, 0x61, 1, &ReadFlag);
			if(ret < 0 )	goto retry_loop;

			//if((ReadFlag > 0) && (ReadFlag <= 16))
			if((ReadFlag > 0) && (ReadFlag <= (CHANNEL_NUM*2+2)))
				{
				ptr = buffer;
				/*for(i = 0; i < ReadFlag; i++)
					{
					ret = ndt_stress_i2c_read(client, 0x62 + i, sizeof(short), (u8 *)ptr);
					if(ret < 0 )	goto retry_loop;
					
					ptr++;
					}*/
				ret = ndt_stress_i2c_read(client, 0x62 , CHANNEL_NUM*2+2, ptr);
				if(ret < 0 )	goto retry_loop;
				}

				//print_mem((char *)buffer, ReadFlag * sizeof(short));

				if(ReadFlag == 1)
					{
					NDT_INFO("***************** Debug value: (%05d) - (%05d) = %d!!!", buffer[0], gNdtData->dvalue1, buffer[0] - gNdtData->dvalue1);
					mutex_lock(&(gNdtData->stress_value_mutex));
					gNdtData->dvalue2 = buffer[0];
					mutex_unlock(&(gNdtData->stress_value_mutex));
					}
				//else if(ReadFlag == 4)
				else if(ReadFlag == (CHANNEL_NUM*2+2))
					{		
					mutex_lock(&(gNdtData->stress_value_mutex));
					//memcpy((char *)gNdtData->sensor, (char *)buffer, ReadFlag * sizeof(short));
					memcpy((char *)gNdtData->sensor, (char *)buffer, CHANNEL_NUM * sizeof(short));
					mutex_unlock(&(gNdtData->stress_value_mutex));
					//temp = get_batt_therm_globle();
					printk("ndt: ");
					for(i = 0;i < CHANNEL_NUM;i++)
						{
						printk("%d\t",buffer[i]);
						}
					printk("\n");
					//printk("ndt: %d\t%d\t%d\t%d\t%d\n",temp, buffer[0], buffer[1], buffer[2], buffer[3]);
					}
								
				ndt_stress_i2c_write(gNdtData->client, 0x60, 1, &temp);//rewrite 0x60
			
				ReadFlag = 0;
				
				ndt_stress_i2c_write(gNdtData->client, 0x61, 1, &ReadFlag);
				
			}
		}
	
retry_loop:

	NDT_DEBUG_FUNC();

	if(gNdtData->dmode == DEBUG_MODE_DDATA)
		queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->debug_work, msecs_to_jiffies(10));
}

static int debug_proc_show(struct seq_file *m, void *v)
{
	NDT_DEBUG_FUNC();

	//seq_printf(m, "NDT Driver Built:	%s, %s\n",__DATE__ ,__TIME__ );
	seq_printf(m, "NDT data init: 		%s\n", gNdtData != NULL ? "yes": "no");
	
	if(gNdtData == NULL)
		return 0;
	
	seq_printf(m, "NDT firmware versions: 	0x%04X\n", gNdtData->FirmwareVer);
	seq_printf(m, "NDT appear: 		%s\n", gNdtData->appear ? "yes": "no");
	seq_printf(m, "NDT debug mode: 	%u\n", gNdtData->dmode);
	seq_printf(m, "NDT suspend: 		%s\n", gNdtData->suspend ? "yes": "no");
	seq_printf(m, "NDT flashprog: 		%s\n", gNdtData->flashprog? "yes": "no");

	mutex_lock(&(gNdtData->stress_value_mutex));
	seq_printf(m, "NDT Press value:	%u\n", gNdtData->value2);
	mutex_unlock(&(gNdtData->stress_value_mutex));
	
	seq_printf(m, "NDT gain value:		%u\n", gNdtData->custom_gain_value);

	seq_printf(m, "reset gpio:		%d\n", gpio_get_value(gNdtData->bdata.reset_gpio));

	seq_printf(m, "power gpio:		%d\n", gpio_get_value(gNdtData->bdata.power_gpio));
	

	NDT_DEBUG_FUNC();

	return 0;
}

static int debug_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, debug_proc_show, NULL);
}

static ssize_t debug_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	unsigned int input = 0;
	unsigned char debug_mode = 0;
	
	NDT_DEBUG_FUNC();

	if (sscanf(buffer, "%u", &input) != 1)
		{
		NDT_ERROR("input sscanf failed !!!");
		return -EINVAL;
		}

	NDT_INFO("input %u", input);
	
	if ((gNdtData == NULL) ||(input == gNdtData->dmode) ||(gNdtData->flashprog == true))
		{
		NDT_ERROR("dmode = input || flashprog = true");
		return count;
		}

	if(input >= DEBUG_MODE_LOG) 
		{
		mutex_lock(&(gNdtData->stress_value_mutex));

		if(input == DEBUG_MODE_LOG)
			gNdtData->dmode = DEBUG_MODE_LOG;
		else
			gNdtData->dvalue1 = gNdtData->dvalue2;
		
		mutex_unlock(&(gNdtData->stress_value_mutex));
		
		return count;
		}

	gNdtData->dmode = input;

	NDT_INFO("set dmode %u", gNdtData->dmode);

	if(gNdtData->suspend == true)
		{
		NDT_INFO("suspend is true");
		return count;
		}

	debug_mode = (gNdtData->dmode == DEBUG_MODE_NORMAL)? 0 : 1;
	ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(debug_mode), &debug_mode);

	if(gNdtData->dmode == DEBUG_MODE_DDATA)
		queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->debug_work, msecs_to_jiffies(10));

	NDT_DEBUG_FUNC();

	return count;
}


static const struct file_operations debug_proc_fops = {
	.open		= debug_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= debug_proc_write,
};

static int debug_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/ndt_debug", 0x664, NULL,
			  &debug_proc_fops);
	if (!res)
		{
		NDT_ERROR("failed to create ndt_debug!!!");
		return -ENOMEM;
		}

	NDT_INFO("created ndt_debug");

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif

#ifdef CONFIG_NDT_USE_PV_TEST
static int test_proc_show(struct seq_file *m, void *v)
{
	short *SenData = gNdtData->sensor;
	unsigned int i, count;
	
	NDT_DEBUG_FUNC();

	count = sizeof(gNdtData->sensor)/sizeof(gNdtData->sensor[0]);

	mutex_lock(&(gNdtData->stress_value_mutex));
	
	for(i = 0; i < count; i++)
		{
		seq_printf(m, "%d",SenData[i]);

		if(i != count -1)
			seq_printf(m, ",");
		}

	seq_printf(m, "\n");
	
	mutex_unlock(&(gNdtData->stress_value_mutex));

	NDT_DEBUG_FUNC();

	return 0;
}

void test_check_file_name(char *buffer, unsigned int len)
{
	unsigned int i;
	
	for(i = 0; i < len; i++)
		{
		if((buffer[i] == '\n') || (buffer[i] == '\r') || (buffer[i] == '\t'))
			buffer[i] = '\0';
		}

	return;
}

static ssize_t test_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	NDT_DEBUG_FUNC();

	NDT_INFO("write file name \"%s\"", buffer);

	if((gNdtData == NULL) ||(gNdtData->suspend == true))
		{
		NDT_ERROR("gNdtData NULL, suspend is true");
		return count;
		}

	if((count > sizeof(gNdtData->update_file)) || (count == 0))
		{
		NDT_ERROR("write count %u, update file name len %u", (unsigned int)count,(unsigned int)sizeof(gNdtData->update_file));
		return count;
		}
	
	memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));

	if(copy_from_user(gNdtData->update_file, buffer, count) != 0)
		{
		memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));
		NDT_ERROR("copy_from_user file name failed !!!");
		return count;
		}

	test_check_file_name(gNdtData->update_file, sizeof(gNdtData->update_file));
	
#ifdef CONFIG_NDT_USE_UPDATE
	gNdtData->dmode = 0;
	gNdtData->force_update = true;
	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(100));
#endif


	NDT_DEBUG_FUNC();

	return count;
}


static int test_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, test_proc_show, NULL);
}

static const struct file_operations test_proc_fops = {
	.open		= test_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= test_proc_write,
};

static int test_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/press_test", 0x664, NULL,
			  &test_proc_fops);
	if (!res)
		{
		NDT_ERROR("failed to create press_test!!!");
		return -ENOMEM;
		}

	NDT_INFO("created press_test");

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif

#ifdef CONFIG_NDT_USE_CHAR_DEV
static ssize_t dev_i2c_read(struct file *filp, char __user *usr_p,
		size_t count, loff_t *pos)
{
	char *buffer = NULL;
	int ret = 0;
	
	NDT_DEBUG_FUNC();

	if (count == 0)
		return 0;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if(buffer == NULL)
		{
		NDT_ERROR("kzalloc failed.\n");
		return -ENOMEM;
		}

	ret = ndt_stress_i2c_read(gNdtData->client, *pos, count, buffer);

	if (copy_to_user(usr_p, buffer, count))
		ret = -EFAULT;
	else
		*pos += ret;

	NDT_DEBUG_FUNC();

	if(buffer)
		{
		kfree(buffer);
		buffer = NULL;
		}

	return ret;
}

static ssize_t dev_i2c_write(struct file *file, const char __user *usr_p,
				    size_t count, loff_t *pos)
{
	char *buffer = NULL;
	int ret = 0;
	
	NDT_DEBUG_FUNC();
	
	if (count == 0)
		return 0;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if(buffer == NULL)
		{
		NDT_ERROR("kzalloc failed.\n");
		return -ENOMEM;
		}

	if (copy_from_user(buffer, usr_p, count))
		{
		NDT_ERROR("copy from user failed!!!");
		goto ExitLoop;
		}

	ret = ndt_stress_i2c_write(gNdtData->client, *pos, count, buffer);

	if(ret >= 0)
		pos += ret;

	NDT_DEBUG_FUNC();

ExitLoop:

	if(buffer)
		{
		kfree(buffer);
		buffer = NULL;
		}

	return ret;
}

static int dev_i2c_open(struct inode *inode, struct file *filp)
{
	NDT_DEBUG_FUNC();

	if (gNdtData->suspend) {
		NDT_ERROR("ndt sensor in suspend");
		return -ENOSPC;
	}

	if((gNdtData->dmode == DEBUG_MODE_DDATA) || (gNdtData->flashprog == true)){
		NDT_ERROR("ndt sensor in debug or flash prog mode");
		return -EBUSY;
	}

	gNdtData->flashprog = true;
	
	NDT_DEBUG_FUNC();
	
	return 0;
}

static loff_t dev_i2c_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = 0xFF + off;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > 0xFF) {
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:

	return newpos;
}


static int dev_i2c_release(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();

	gNdtData->flashprog = false;

	gNdtData->uniformity_init = 0;

	gNdtData->frame_count = 0;

	memset(&gpoint_info, 0, sizeof(gpoint_info));
	
	return 0;
}

static long dev_i2c_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int data = 0;
	char data8 = 0;
	short data16 = 0;

	if (_IOC_TYPE(cmd) != NDT_IOC_MAGIC)
        return -EINVAL;

	if ((_IOC_DIR(cmd) & _IOC_READ) && (!access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd))))
       	return -EFAULT;

	if((_IOC_DIR(cmd) & _IOC_WRITE) && (!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))))
        	 return -EFAULT;

	switch(cmd)
		{
		case NDT_IOC_HW_RESET:
			NDT_INFO("NDT_IOC_HW_RESET");
			ndt_stress_power_switch(false, true);
			msleep(10);
			ndt_stress_power_switch(true, false);
			msleep(POWER_ON_DELAY);
			//ndt_hw_reset();
			ndt_stress_jump_wait();
			msleep(50);
			break;
		case NDT_IOC_READ:
			NDT_INFO("NDT_IOC_READ");
			if(copy_to_user((int *)arg, &data, sizeof(data)) != 0)
				return -EINVAL;
			break;
		case NDT_IOC_WRITE:
			NDT_INFO("NDT_IOC_WRITE");
			if(copy_from_user(&data, (int *)arg, sizeof(data)) != 0)
				return -EINVAL;
			break;
		case NDT_IOC_INIT_UNIFORMITY:
			if(arg != 0)
				{
				data8 = 0x09;
				ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(data8), &data8);
				
				data8 = 0x00;
				ndt_stress_i2c_write(gNdtData->client, 0x61, sizeof(data8), &data8);

				gNdtData->frame_count = 0;

				gNdtData->drop_frame_conut = arg;

				gNdtData->uniformity_init = 1;

				NDT_INFO("init uniformity mode!!!!drop_frame_conut %ld", arg);
				}
			else
				{
				gNdtData->uniformity_init = 0;
				
				data8 = 0x00;
				data16 = 0x00;
				ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(data8), &data8);

				ndt_stress_i2c_write(gNdtData->client, 0x61, sizeof(data8), &data8);

				ndt_stress_i2c_write(gNdtData->client, 0x62, sizeof(data16), &data16);

				ndt_stress_i2c_write(gNdtData->client, 0x63, sizeof(data16), &data16);

				ndt_stress_i2c_write(gNdtData->client, 0x64, sizeof(data16), &data16);

				ndt_stress_i2c_write(gNdtData->client, 0x65, sizeof(data16), &data16);

				ndt_stress_i2c_write(gNdtData->client, 0x66, sizeof(data16), &data16);
		
				gNdtData->frame_count = 0;

				NDT_INFO("uniformity mode end!!!!");
				}
			break;
		case NDT_IOC_READ_STATUS:
			wait_event_interruptible(gNdtData->status_wait,gNdtData->status_flag);

			gNdtData->status_flag = 0;

			gpoint_info.frame_count = gNdtData->frame_count;

			if(copy_to_user((char *)arg, (char *)&gpoint_info, sizeof(gpoint_info)) != 0)
				return -EINVAL;
			
			if(gNdtData->dmode == DEBUG_MODE_LOG)
				NDT_INFO("touch up:%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d", gpoint_info.press, gpoint_info.signal_sum, gpoint_info.base1, gpoint_info.base2, gpoint_info.base3, gpoint_info.base4, gpoint_info.param, gpoint_info.average,
								gpoint_info.x, gpoint_info.y, gpoint_info.is_mid_edge, gpoint_info.unedge_percent, gpoint_info.percent);
			
			memset(&gpoint_info, 0, sizeof(gpoint_info));
			
			gNdtData->frame_count = 0;
			break;
		default: 
			return -EINVAL;
		}
		
    return ret;
}

static const struct file_operations app_proc_fops = {
	.owner 			= THIS_MODULE,
	.open			= dev_i2c_open,
	.read			= dev_i2c_read,
	.write			= dev_i2c_write,
	.llseek 			= dev_i2c_llseek,
	.unlocked_ioctl 	= dev_i2c_ioctl,
	.release			= dev_i2c_release,
};

static int dev_i2c_init(void)
{
	int ret;

	NDT_DEBUG_FUNC();
	
	ret = register_chrdev(NDT_CHAR_MAJOR, NDT_DEVICE_NAME, &app_proc_fops);
	if(ret < 0)
		{
		NDT_ERROR("can't register major number\n");
		return ret;
		}

	gNdtData->i2c_class = class_create(THIS_MODULE, NDT_DEVICE_NAME);
	if(IS_ERR(gNdtData->i2c_class))
		{
		NDT_ERROR("failed in My_led class.\n");
		return -1;
		}
	
	device_create(gNdtData->i2c_class, NULL, MKDEV(NDT_CHAR_MAJOR,0), NULL, NDT_DEVICE_NAME);

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif


#ifdef CONFIG_NDT_USE_TEST_I2C
static int ndt_i2c_test(int delay_time)
{
	unsigned short FirmwareVer = 0;
	unsigned char i = 0;
	
	NDT_DEBUG_FUNC();
	
	do
		{
		if(!delay_time)
			mdelay(15 + i * 30);
		else
			mdelay(delay_time);
		
		if(ndt_stress_i2c_read(gNdtData->client, 0x05, sizeof(FirmwareVer), (unsigned char *)&FirmwareVer) != -1)
			{
			if(FirmwareVer != 0)
				{
				gNdtData->FirmwareVer = FirmwareVer;
				break;
				}
			NDT_INFO("Firmware Version: 0x%04x\n", FirmwareVer);
			}
		}while((++i <= 2) && !delay_time);

	if(FirmwareVer == 0)
		{
		gNdtData->appear = false;
		NDT_ERROR("NDT device test failed!!!!\n");
		return -1;
		}
	else
		{
		NDT_INFO("Firmware Version: 0x%04x\n", FirmwareVer);
		gNdtData->appear = true;
		return 0;
		}
	
	NDT_DEBUG_FUNC();
	
	return 0;
}
#endif

// ZTE_modify add proc/driver/pressure_id begin
static int pressure_proc_show(struct seq_file *m, void *v)
{
	NDT_DEBUG_FUNC();

	if(gNdtData == NULL)
		return 0;

	seq_printf(m, "NDT-KL26Z-NA-NA-NA\n");

	return 0;
}

static int pressure_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();

	return single_open(file, pressure_proc_show, NULL);
}

static const struct file_operations pressure_proc_fops = {
	.open		= pressure_proc_open,
	.read		= seq_read,
	.release		= single_release,
};

static int pressure_proc_init(void)
{
       struct proc_dir_entry *res;

	NDT_DEBUG_FUNC();

	res = proc_create("driver/pressure_id", 0444, NULL,
			  &pressure_proc_fops);
	if (!res)
	{
		NDT_ERROR("failed to create ndt_debug!!!");
		return -ENOMEM;
	}

	return 0;
}
// ZTE_modify end

// ZTE_modify add custom gain control begin
static int custom_gain_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Force screen gain=%u\n", gNdtData->custom_gain_value);

	return 0;
}

static int custom_gain_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, custom_gain_proc_show, NULL);
}

static ssize_t custom_gain_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	NDT_DEBUG_FUNC();

	if (sscanf(buffer, "%u", &(gNdtData->custom_gain_value)) != 1)
	{
		NDT_ERROR("input custom gain failed !!!");
		return -EINVAL;
	}

	NDT_INFO("custom gain %u", gNdtData->custom_gain_value);

	return count;
}


static const struct file_operations custom_gain_proc_fops = {
	.open		= custom_gain_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= custom_gain_proc_write,
};

static int custom_gain_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/custom_gain_set", S_IWUGO | S_IRUGO, NULL,
			  &custom_gain_proc_fops);
	if (!res)
	{
		NDT_ERROR("failed to create custom_gain_set!!!");
		return -ENOMEM;
	}

	return 0;
}
// ZTE_modify end

// ZTE_modify add press status begin
/*
static int press_status_open(struct inode *inode, struct file *file)
{
      pr_debug("%s\n", __func__);
	return 0;
}

static int press_status_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	
	return 0;
}

static ssize_t press_status_read(struct file *filp, char __user *usr_p,
		size_t count, loff_t *pos)
{
	int ret = count;
       pr_debug("%s count=%ld, *pos=%lld\n", __func__, count, *pos);
//	if ((count == 0) || (count > PRESS_STATUS_BUF_SIZE))
		return 0;

//	if (copy_to_user(usr_p, &press_status_buf[*pos], count))
		ret = -EFAULT;

       pr_debug("%s ret=%d\n", __func__, ret);
	return ret;
}

static loff_t press_status_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		//newpos = PRESS_STATUS_BUF_SIZE;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > PRESS_STATUS_BUF_SIZE-1) {
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:

	return newpos;
}

static const struct file_operations press_status_fops = {
	.owner 			= THIS_MODULE,
	.open			= press_status_open,
	.read			= press_status_read,
	.llseek 			= press_status_llseek,
	.release			= press_status_release,
};

#define PRESS_STATUS_MAJOR 104 
#define PRESS_STATUS_NAME "press_status"
static int press_status_init(void)
{
	int ret;
       struct class *status_class;

	ret = register_chrdev(PRESS_STATUS_MAJOR, PRESS_STATUS_NAME, &press_status_fops);
	if(ret < 0)
	{
		pr_err("can't register major number\n");
		return ret;
	}

	status_class = class_create(THIS_MODULE, PRESS_STATUS_NAME);
	if(IS_ERR(status_class))
	{
		pr_err("failed in My_led class.\n");
		return -1;
	}

	device_create(status_class, NULL, MKDEV(PRESS_STATUS_MAJOR,0), NULL, PRESS_STATUS_NAME);
	
	return 0;
}
// ZTE_modify end
*/
static int ndt_stress_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ndt_stress_board_data *bdata;
	bool ret = true;

	NDT_INFO("%s:start!",__func__);
	NDT_INFO("NDT Driver Version: %s", NDT_DRIVER_VERSION);
	//NDT_INFO("NDT Driver Built@%s, %s", __TIME__, __DATE__);
	NDT_INFO("NDT I2C Address: 0x%02x", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NDT_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	gNdtData = kzalloc(sizeof(struct ndt_stress_data), GFP_KERNEL);
	if (!gNdtData) {
		NDT_ERROR("Failed to alloc mem for rmi4_data");
		return -ENOMEM;
	}

	gNdtData->client = client;
	
	bdata = &gNdtData->bdata;

	if(parse_dt(&client->dev, bdata))
		{
		NDT_ERROR("Failed to parse dt");
		goto err_parse_dt;
		}

	if(ndt_stress_set_gpio(gNdtData))
		{
		NDT_ERROR("Failed to set up GPIO's");
		goto err_set_gpio;
		}
	
	if(ndt_stress_jump_wait() == -1)
		{
		NDT_ERROR("Failed to jump wait");
		//goto err_set_gpio;
		}

	 msleep(120);

	 gNdtData->ndt_coord_handle = create_singlethread_workqueue("ndt_coord_handle");
	 INIT_DELAYED_WORK(&gNdtData->send_coord_work, ndt_update_send_coord_work);

#if (defined CONFIG_NDT_USE_DEBUG) || (defined CONFIG_NDT_USE_UPDATE)
	gNdtData->ndt_debug_queue = create_singlethread_workqueue("ndt_debug_workqueue");
#endif

#ifdef CONFIG_NDT_USE_UPDATE
	INIT_DELAYED_WORK(&gNdtData->update_work, ndt_update_handle_work);
#endif

#ifdef CONFIG_NDT_USE_TEST_I2C
	if(ndt_i2c_test(0))
		{
		NDT_ERROR("Failed to test ndt i2c");
		ret = false;
		ndt_stress_power_switch(false, false);//  tp can't init when stress sensor firmware is bad and i2c is noisy. we must pown down
#ifdef CONFIG_NDT_USE_UPDATE
		ndt_force_firmware_update();//delay 10s force update firmware when filesystem have mounted
#endif
		}
	else
		gNdtData->update_finished = true;
#else
	gNdtData->appear = true;
#endif

	mutex_init(&(gNdtData->stress_value_mutex));

	mutex_init(&(gNdtData->i2c_mutex));

	//init_completion(&gNdtData->coord_thread_sync);
	
	init_waitqueue_head(&gNdtData->status_wait);

#ifdef CONFIG_NDT_USE_IRQ
	gNdtData->ndt_queue = create_singlethread_workqueue("ndt_irq_workqueue");
	
	INIT_DELAYED_WORK(&gNdtData->int_work, ndt_irq_handle_work);

	if(ndt_stress_request_irq(gNdtData))
		{
		NDT_ERROR("Failed to request irq");
		goto err_request_irq;
		}
#endif

#ifdef CONFIG_NDT_USE_DEBUG
	INIT_DELAYED_WORK(&gNdtData->debug_work, ndt_debug_handle_work);

	if(debug_proc_init())
		goto err_debug;
#endif

#ifdef CONFIG_NDT_USE_PV_TEST
	if(test_proc_init())
		goto err_pv_test;
#endif

      // ZTE_modify set custom gain proc command begin
      gNdtData->custom_gain_value = 100;
      custom_gain_proc_init();
      // ZTE_modify end
      // ZTE_modify add proc/driver/pressure_id begin
      pressure_proc_init();
      // ZTE_modify end
      // ZTE_modify get press status begin
      //press_status_init();
      //ZTE_modify
      compatible_proc_init();
#ifdef CONFIG_NDT_USE_CHAR_DEV
	dev_i2c_init();
#endif

#ifdef CONFIG_NDT_USE_UPDATE
	gNdtData->dmode = 0;
	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(10000));
#endif

	if(ret)
		forcetouch_vendor = 1;
	else
		forcetouch_vendor = 0;
	printk("%s:forcetouch_vendor=%u\n",__func__,forcetouch_vendor);
	
	NDT_INFO("%s:end!",__func__);

	return 0;

#ifdef CONFIG_NDT_USE_PV_TEST
err_pv_test:
	remove_proc_entry("driver/press_test", NULL);
#endif

#ifdef CONFIG_NDT_USE_DEBUG
err_debug:
	remove_proc_entry("driver/ndt_debug", NULL);
#endif

#ifdef CONFIG_NDT_USE_IRQ
err_request_irq:
#endif

#ifdef CONFIG_NDT_USE_TEST_I2C
//err_test_i2c:
#endif

err_set_gpio:
	ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		ndt_stress_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		ndt_stress_gpio_setup(bdata->power_gpio, false, 0, 0);

err_parse_dt:
	kfree(gNdtData);
	gNdtData = NULL;
	NDT_INFO("%s:end due to error!",__func__);

	return 0;
}

static int ndt_stress_remove(struct i2c_client *client)
{
	NDT_DEBUG_FUNC();

 	return 0;
}

static struct i2c_driver ndt_stress_driver = {
	.probe = ndt_stress_probe,
	.remove = ndt_stress_remove,
	.id_table = ndt_stress_id,
	.driver = {
		   .name = I2C_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = ndt_match_table,
		   },
};

static int __init ndt_stress_init(void)
{
	NDT_DEBUG_FUNC();
	
	return i2c_add_driver(&ndt_stress_driver);
}

static void __exit ndt_stress_exit(void)
{
	NDT_DEBUG_FUNC();
	
	return i2c_del_driver(&ndt_stress_driver);
}

module_init(ndt_stress_init);
module_exit(ndt_stress_exit);

MODULE_AUTHOR("cc <cui.xiaochuan@zte.com.cn>");
MODULE_DESCRIPTION("NDT Stress Driver For P839A01");
MODULE_LICENSE("GPL");
