/* arch/arm/mach-msm/keypad-mango.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 * Adapted for SEMC 2011 devices by Michael Bestas (mikeioannina@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/mfd/pmic8058.h>
#include "keypad-semc.h"
/* mango GPIO slider - start */
#include <linux/gpio_event.h>
#include <linux/platform_device.h>
/* mango GPIO slider - end */

static const unsigned int pm8xxx_keymap[] = {
	KEY(0, 1, KEY_CAMERA_FOCUS),	/* FOCUS */
	KEY(0, 2, KEY_COMPOSE),		/* PICTSYMBOLS */
	KEY(0, 3, KEY_SPACE),		/* SPACE */
	KEY(0, 4, KEY_ENTER),		/* ENTER */
	KEY(0, 5, KEY_DOT),		/* PERIOD */
	KEY(0, 6, KEY_UP),		/* DPAD_UP */

	KEY(1, 0, KEY_HOME),		/* HOME */
	KEY(1, 1, KEY_CAMERA),		/* CAMERA */
	KEY(1, 3, KEY_A),		/* A */
	KEY(1, 4, KEY_D),		/* D */
	KEY(1, 5, KEY_APOSTROPHE),	/* APOSTROPHE */
	KEY(1, 6, KEY_DOWN),		/* DPAD_DOWN */

	KEY(2, 1, KEY_VOLUMEDOWN),	/* VOLUME_DOWN */
	KEY(2, 3, KEY_B),		/* B */
	KEY(2, 5, KEY_RIGHTBRACE),	/* GRAVE */
	KEY(2, 7, KEY_RIGHT),		/* DPAD_RIGHT */

	KEY(3, 0, KEY_LEFTSHIFT),	/* SHIFT_LEFT */
	KEY(3, 1, KEY_VOLUMEUP),	/* VOLUME_UP */
	KEY(3, 2, KEY_F),		/* F */
	KEY(3, 3, KEY_COMMA),		/* COMMA */
	KEY(3, 4, KEY_Z),		/* Z */
	KEY(3, 5, KEY_E),		/* E */
	KEY(3, 7, KEY_LEFT),		/* DPAD_LEFT */

	KEY(4, 1, KEY_Q),		/* Q */
	KEY(4, 3, KEY_R),		/* R */
	KEY(4, 4, KEY_C),		/* C */
	KEY(4, 5, KEY_T),		/* T */
	KEY(4, 6, KEY_LEFTALT),		/* ALT_LEFT */
	KEY(4, 7, KEY_V),		/* V */

	KEY(5, 1, KEY_G),		/* G */
	KEY(5, 2, KEY_LEFTBRACE),	/* AT */
	KEY(5, 4, KEY_Y),		/* Y */
	KEY(5, 5, KEY_H),		/* H */
	KEY(5, 6, KEY_LANGUAGE),	/* LANGUAGE_SWITCH */
	KEY(5, 7, KEY_X),		/* X */

	KEY(6, 1, KEY_U),		/* U */
	KEY(6, 2, KEY_N),		/* N */
	KEY(6, 3, KEY_J),		/* J */
	KEY(6, 5, KEY_I),		/* I */
	KEY(6, 6, KEY_K),		/* K */
	KEY(6, 7, KEY_M),		/* M */

	KEY(7, 0, KEY_W),		/* W */
	KEY(7, 1, KEY_S),		/* S */
	KEY(7, 2, KEY_O),		/* O */
	KEY(7, 4, KEY_L),		/* L */
	KEY(7, 6, KEY_BACKSPACE),	/* DEL */
	KEY(7, 7, KEY_P),		/* P */
};

static struct matrix_keymap_data pm8xxx_keymap_data = {
	.keymap_size	= ARRAY_SIZE(pm8xxx_keymap),
	.keymap		= pm8xxx_keymap,
};

struct pm8xxx_keypad_platform_data pm8xxx_keypad_data = {
	.input_name		= "pm8xxx-keypad",
	.input_phys_device	= "pm8xxx-keypad/input0",
	.num_rows		= 8,
	.num_cols		= 8,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 10,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 122000,
	.wakeup			= 1,
	.keymap_data		= &pm8xxx_keymap_data,
};

/* mango GPIO slider - start */
#define SLIDE_CLOSED_N_GPIO	180

static struct gpio_event_direct_entry gpio_switch_map[] = {
	{SLIDE_CLOSED_N_GPIO, SW_LID},
};

static struct gpio_event_input_info gpio_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = gpio_switch_map,
	.keymap_size = ARRAY_SIZE(gpio_switch_map),
	.info.no_suspend = true,
};

static struct gpio_event_info *slider_info[] = {
	&gpio_switch_info.info,
};

static struct gpio_event_platform_data gpio_slider_data = {
	.name		= "gpio-slider",
	.info		= slider_info,
	.info_count	= ARRAY_SIZE(slider_info),
};

struct platform_device gpio_slider_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_slider_data,
	},
};

static int __init gpio_slider_device_init(void)
{
	return platform_device_register(&gpio_slider_device);
}

static void __exit gpio_slider_device_exit(void)
{
	platform_device_unregister(&gpio_slider_device);
}

module_init(gpio_slider_device_init);
module_exit(gpio_slider_device_exit);
/* mango GPIO slider - end */
