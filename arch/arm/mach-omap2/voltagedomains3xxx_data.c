/*
 * OMAP3 voltage domain data
 *
 * Copyright (C) 2007, 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2008, 2011 Nokia Corporation
 * Kalle Jokiniemi
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>

#include <plat/common.h>
#include <plat/cpu.h>
#include <plat/voltage.h>
#include <plat/vc.h>
#include <plat/vp.h>

#include "prm-regbits-34xx.h"
#include "omap_opp_data.h"

/*
 * VDD data
 */

static const struct omap_vfsm_instance omap3_vdd1_vfsm = {
	.voltsetup_reg = OMAP3_PRM_VOLTSETUP1_OFFSET,
	.voltsetup_mask = OMAP3430_SETUP_TIME1_MASK,
};

static struct omap_vdd_info omap3_vdd1_info;

static const struct omap_vfsm_instance omap3_vdd2_vfsm = {
	.voltsetup_reg = OMAP3_PRM_VOLTSETUP1_OFFSET,
	.voltsetup_mask = OMAP3430_SETUP_TIME2_MASK,
};

static struct voltagedomain omap3_voltdm_mpu = {
	.name = "mpu_iva",
	.scalable = true,
	.read = omap3_prm_vcvp_read,
	.write = omap3_prm_vcvp_write,
	.rmw = omap3_prm_vcvp_rmw,
	.vc = &omap3_vc_mpu,
	.vfsm = &omap3_vdd1_vfsm,
	.vp = &omap3_vp_mpu,
	.vdd = &omap3_vdd1_info,
};

static struct omap_vdd_info omap3_vdd2_info;

static struct voltagedomain omap3_voltdm_core = {
	.name = "core",
	.scalable = true,
	.read = omap3_prm_vcvp_read,
	.write = omap3_prm_vcvp_write,
	.rmw = omap3_prm_vcvp_rmw,
	.vc = &omap3_vc_core,
	.vfsm = &omap3_vdd2_vfsm,
	.vp = &omap3_vp_core,
	.vdd = &omap3_vdd2_info,
};

static struct voltagedomain omap3_voltdm_wkup = {
	.name = "wakeup",
};

static struct voltagedomain *voltagedomains_omap3[] __initdata = {
	&omap3_voltdm_mpu,
	&omap3_voltdm_core,
	&omap3_voltdm_wkup,
	NULL,
};

static const char *sys_clk_name __initdata = "sys_ck";

void __init omap3xxx_voltagedomains_init(void)
{
	struct voltagedomain *voltdm;
	int i;

	/*
	 * XXX Will depend on the process, validation, and binning
	 * for the currently-running IC
	 */
	if (cpu_is_omap3630()) {
		omap3_voltdm_mpu.volt_data = omap36xx_vddmpu_volt_data;
		omap3_voltdm_core.volt_data = omap36xx_vddcore_volt_data;
		omap3_vdd1_info.dep_vdd_info = omap36xx_vddmpu_dep_info;
	} else {
		omap3_voltdm_mpu.volt_data = omap34xx_vddmpu_volt_data;
		omap3_voltdm_core.volt_data = omap34xx_vddcore_volt_data;
		omap3_vdd1_info.dep_vdd_info = omap34xx_vddmpu_dep_info;
	}

	for (i = 0; voltdm = voltagedomains_omap3[i], voltdm; i++)
		voltdm->sys_clk.name = sys_clk_name;

	voltdm_init(voltagedomains_omap3);
};
