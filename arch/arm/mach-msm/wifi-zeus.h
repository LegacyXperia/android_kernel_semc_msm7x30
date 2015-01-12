/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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

#ifndef _ZEUS_WIFI_H
#define _ZEUS_WIFI_H

extern void (*wifi_status_cb)(int card_present, void *dev_id);
extern void *wifi_status_cb_devid;
extern int zeus_wifi_cd; /* WIFI virtual 'card detect' status */
extern int zeus_get_wlanmac(uint8_t *wlanmac);

#endif
