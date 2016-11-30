/* drivers/input/touchscreen/gt1x.h
 * 
 * 2010 - 2013 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version: 1.0   
 * Revision Record: 
 *      V1.0:  first release. 2014/09/28.
 *
 */

#ifndef _GOODIX_GT1X_H_
#define _GOODIX_GT1X_H_

//#include <mach/gpio.h>
#include <linux/gpio.h>

#define GOODIX_ZTE_DEBUG_FLAG
#define GOODIX_ZTE_PROC_FLAG
#define CONFIG_CREATE_GLOVED_INTERFACE

#define IIC_MAX_TRANSFER_SIZE       250

extern int goodix_irq_gpio;
extern int goodix_reset_gpio;

#ifdef GOODIX_ZTE_PROC_FLAG
struct tsc_id_t{
	char product_id[8];
	u32 	patch_id;
	u8 	sensor_id;
};

extern struct tsc_id_t tsc_id_info;
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
struct tp_proc_entry{
	struct proc_dir_entry *proc_path_entry;
	struct proc_dir_entry *debug_entry;
	struct proc_dir_entry *firmware_ver_entry;
};

extern atomic_t inter_count;
extern atomic_t report_count;
extern int g_fwudone_status;
#endif

/* Customize your I/O ports & I/O operations */
#if 0
#define GTP_RST_PORT    102	//S5PV210_GPJ3(6)
#define GTP_INT_PORT    52	//S5PV210_GPH1(3)
#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
#else
#define GTP_RST_PORT    goodix_reset_gpio
#define GTP_INT_PORT    goodix_irq_gpio
#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
#endif
//#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            gpio_direction_input(pin);\
                                        }while(0)
#define GTP_GPIO_AS_INT(pin)            do{\
                                            GTP_GPIO_AS_INPUT(pin);\
                                        }while(0)
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)              gpio_free(pin)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

#endif /* _GOODIX_GT1X_H_ */
