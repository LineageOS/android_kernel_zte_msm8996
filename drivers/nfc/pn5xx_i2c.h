/*
 * Driver include for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltoenn <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef _PN544_H_
#define _PN544_H_

#include <linux/i2c.h>

#define PN544_MAGIC	0xE9

/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN5XX_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)
/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPI_PWR    _IOW(PN544_MAGIC, 0x02, unsigned int)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_PWR_STATUS    _IOR(PN544_MAGIC, 0x03, unsigned int)

/* DWP side this ioctl will be called
 * level 1 = Wired access is enabled/ongoing
 * level 0 = Wired access is disalbed/stopped
*/
#define P61_SET_WIRED_ACCESS _IOW(PN544_MAGIC, 0x04, unsigned int)

/*
 * NFC Init will call the ioctl to register the PID with the i2c driver
*/
#define P544_SET_NFC_SERVICE_PID _IOW(PN544_MAGIC, 0x05, unsigned int)


#endif /* _PN544_H_ */
