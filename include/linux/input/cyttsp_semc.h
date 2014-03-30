/* Header file for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * include/linux/input/cyttsp_semc.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */
#ifndef _CYTTSP_SEMC_H_
#define _CYTTSP_SEMC_H_

#include <linux/input.h>

#define CY_SPI_NAME "cyttsp-spi"
#define CY_I2C_NAME "cyttsp-i2c"
/* Active Power state scanning/processing refresh interval */
#define CY_ACT_INTRVL_DFLT 0x00
/* touch timeout for the Active power */
#define CY_TCH_TMOUT_DFLT 0xFF
/* Low Power state scanning/processing refresh interval */
#define CY_LP_INTRVL_DFLT 0x0A
/*
 *defines for Gen2 (Txx2xx); Gen3 (Txx3xx)
 * use these defines to set cyttsp_platform_data.gen in board config file
 */
enum cyttsp_gen {
	CY_GEN2,
	CY_GEN3,
};
/*
 * Active distance in pixels for a gesture to be reported
 * if set to 0, then all gesture movements are reported
 * Valid range is 0 - 15
 */
enum cyttsp_act_dist {
	CY_ACT_DIST_KEEP_ASIS = -1,
	CY_ACT_DIST_CLR = 0xf0,
	CY_ACT_DIST_00 = 0,
	CY_ACT_DIST_01 = 1,
	CY_ACT_DIST_02 = 2,
	CY_ACT_DIST_03 = 3,
	CY_ACT_DIST_04 = 4,
	CY_ACT_DIST_05 = 5,
	CY_ACT_DIST_06 = 6,
	CY_ACT_DIST_07 = 7,
	CY_ACT_DIST_08 = 8,
	CY_ACT_DIST_09 = 9,
	CY_ACT_DIST_10 = 10,
	CY_ACT_DIST_11 = 11,
	CY_ACT_DIST_12 = 12,
	CY_ACT_DIST_13 = 13,
	CY_ACT_DIST_14 = 14,
	CY_ACT_DIST_15 = 15,
	CY_ACT_DIST_DFLT = CY_ACT_DIST_08,
	CY_ACT_DIST = CY_ACT_DIST_DFLT,
};

/* max num retries to read touch data */
#define CY_NUM_RETRY 100

enum cyttsp_gest {
	CY_GEST_KEEP_ASIS = -1,
	CY_GEST_GRP_CLR = 0x0f,
	CY_GEST_GRP_NONE = 0,
	CY_GEST_GRP1 =	0x10,
	CY_GEST_GRP2 = 0x20,
	CY_GEST_GRP3 = 0x40,
	CY_GEST_GRP4 = 0x80,
};

enum cyttsp_powerstate {
	CY_IDLE_STATE,
	CY_ACTIVE_STATE,
	CY_LOW_PWR_STATE,
	CY_SLEEP_STATE,
	CY_UNSURE_STATE,
};

struct cyttsp_platform_data {
	u32 maxx;
	u32 maxy;
	u32 maxz;
	u32 flags;
	enum cyttsp_gen gen;
	unsigned use_st:1;
	unsigned use_mt:1;
	unsigned use_trk_id:1;
	unsigned use_hndshk:1;
	unsigned use_timer:1;
	unsigned use_sleep:1;
	unsigned use_gestures:1;
	unsigned use_load_file:1;
	unsigned use_force_fw_update:1;
	unsigned use_virtual_keys:1;
	enum cyttsp_powerstate power_state;
	enum cyttsp_gest gest_set;
	enum cyttsp_act_dist act_dist;
	u8 act_intrvl;  /* Active refresh interval; ms */
	u8 tch_tmout;   /* Active touch timeout; ms */
	u8 lp_intrvl;   /* Low power refresh interval; ms */
	int (*wakeup)(void);
	int (*init)(int on_off);
	int (*reset)(void);
	void (*mt_sync)(struct input_dev *);
	int (*cust_spec)(u8 data[], int size);
	char *name;
	s16 irq_gpio;
	u8 idac_gain;
};

#endif /* _CYTTSP_SEMC_H_ */
