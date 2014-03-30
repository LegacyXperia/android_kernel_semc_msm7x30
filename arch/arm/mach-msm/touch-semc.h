/* arch/arm/mach-msm/touch-semc.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Adapted for SEMC 2011 devices by Michael Bestas (mikeioannina@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _TOUCH_SEMC_H
#define _TOUCH_SEMC_H

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI_SEMC
extern int cyttsp_xres(void);
extern int cyttsp_init(int on);
extern int cyttsp_wakeup(void);

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_KEY
extern int cyttsp_key_rpc_callback(u8 data[], int size);
#endif

extern struct cyttsp_platform_data cyttsp_data;
#endif

#endif
