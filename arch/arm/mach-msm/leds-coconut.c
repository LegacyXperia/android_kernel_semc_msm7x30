/* arch/arm/mach-msm/leds-coconut.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 * Copyright 2013 Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifdef CONFIG_LEDS_AS3676
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/leds-as3676.h>

struct as3676_platform_data as3676_platform_data = {
	.step_up_vtuning = 18,	/* 0 .. 31 uA on DCDC_FB */
	.audio_speed_down = 1,	/* 0..3 resp. 0, 200, 400, 800ms */
	.audio_speed_up = 4,	/* 0..7 resp. 0, 50, 100, 150,
					200,250,400, 800ms */
	.audio_agc_ctrl = 1,	/* 0 .. 7: 0==no AGC, 7 very aggressive*/
	.audio_gain = 7,	/* 0..7: -12, -6,  0, 6
					12, 18, 24, 30 dB */
	.audio_source = 2,	/* 0..3: 0=curr33, 1=DCDC_FB
					2=GPIO1,  3=GPIO2 */
	.step_up_lowcur = true,
	.reset_on_i2c_shutdown = true,
	.caps_mounted_on_dcdc_feedback = 1,
	.cp_control = 0x10,
	.leds[0] = {
		.name = "lcd-backlight",
		.on_charge_pump = 0,
		.max_current_uA = 20000,
		.startup_current_uA = 20000,
		.use_dls = false,
	},
	.leds[1] = {
		.name = "led_2-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
		.use_dls = true,
	},
	.leds[2] = {
		.name = "led_3-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[3] = {
		.name = "led_6-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[4] = {
		.name = "button-backlight-rgb1",
		.on_charge_pump = 0,
		.max_current_uA = 500,
		.startup_current_uA = 500,
	},
	.leds[5] = {
		.name = "button-backlight-rgb2",
		.on_charge_pump = 0,
		.max_current_uA = 500,
		.startup_current_uA = 500,
	},
	.leds[6] = {
		.name = "red",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[7] = {
		.name = "green",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[8] = {
		.name = "blue",
		.on_charge_pump = 1,
		.max_current_uA = 5000,
	},
	.leds[9] = {
		.name = "music-light-rgb1",
		.on_charge_pump = 0,
		.max_current_uA = 4350,
	},
	.leds[10] = {
		.name = "music-light-rgb2",
		.on_charge_pump = 0,
		.max_current_uA = 4350,
	},
	.leds[11] = {
		.name = "led_12-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[12] = {
		.name = "led_13-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
};
#endif

#ifdef CONFIG_LEDS_AS3676_SEMC
#include <linux/bug.h>
#include <linux/leds.h>
#include <linux/leds-as3676_semc.h>

static struct as3676_platform_led as3676_leds_mapping[] = {
	{
		.name = "lcd-backlight",
		.sinks = BIT(AS3676_SINK_01),
		.flags = AS3676_FLAG_PWM_CTRL | AS3676_FLAG_PWM_INIT
			| AS3676_FLAG_WAIT_RESUME,
		.max_current = 20000,
		.hw_max_current = 25000,
		.default_brightness = LED_FULL,
	},
	{
		.name = "button-backlight",
		.sinks = BIT(AS3676_SINK_RGB2) | BIT(AS3676_SINK_RGB3),
		.max_current = 1500,
		.hw_max_current = 25000,
	},
	{
		.name = "music-light",
		.sinks = BIT(AS3676_SINK_30) | BIT(AS3676_SINK_31),
		.flags = AS3676_FLAG_AUDIO,
		.max_current = 5000,
		.hw_max_current = 5000,
	},
	{
		.name = "red",
		.sinks = BIT(AS3676_SINK_41),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 5000,
		.hw_max_current = 25000,
	},
	{
		.name = "green",
		.sinks = BIT(AS3676_SINK_42),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 5000,
		.hw_max_current = 25000,
	},
	{
		.name = "blue",
		.sinks = BIT(AS3676_SINK_43),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 5000,
		.hw_max_current = 25000,
	},
};

struct as3676_audio_config as3676_coconut_config = {
	.current_3x = 0x0f,
	.audio_control = 0x01,
	.audio_input = 0x1F,
	.audio_output = 0x05,
};

struct as3676_platform_data as3676_platform_data = {
	.leds = as3676_leds_mapping,
	.num_leds = ARRAY_SIZE(as3676_leds_mapping),
	.als_connected = 1,
	.als_wait = 100,
	.dls_connected = false,
	.ldo_mV = 3300,
	.audio_config = &as3676_coconut_config,
};
#endif
