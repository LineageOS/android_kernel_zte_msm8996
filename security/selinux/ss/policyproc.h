/* policyproc.h -- preprocess/postprocess policy as binary image
 *
 * Copyright 2015-2016 ZTE Corp.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Written by Jia Jia <jia.jia@zte.com.cn>
 */

/*
 * Created by ZTE_BOOT_20151105
 */

#include "policydb.h"

int pp_preproc_policy(void **data, size_t *len);
int pp_postproc_policy(void **data, size_t *len);
int pp_postproc_av_perms(struct policydb *pol, u32 ssid, u32 tsid, u16 tclass, u32 *perms);