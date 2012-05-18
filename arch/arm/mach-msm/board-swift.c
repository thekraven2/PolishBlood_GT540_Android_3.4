/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, Polish Blood Project
 * Author: Brian Swetland <swetland@google.com>
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
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/memblock.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <mach/vreg.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include "devices.h"
#include "clock.h"
/*from ioremap from 35*/
#define MSM_L2CC_BASE         IOMEM(0xFA006000)
void __init msm_clock_init(struct clk_lookup *clock_tbl, unsigned num_clocks);
static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C0043ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(132),
		.end	= MSM_GPIO_TO_INT(132),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&smc91x_device,
};

extern struct sys_timer msm_timer;

static void __init swift_init_irq(void)
{
  	msm_init_irq();
}

static void __init swift_init(void)
{
  	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static void __init swift_map_io(void)
{
  	msm_map_common_io();
	/* Technically dependent on the SoC but using machine_is
	 * macros since socinfo is not available this early and there
	 * are plans to restructure the code which will eliminate the
	 * need for socinfo.
	 */
	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);


#ifdef CONFIG_CACHE_L2X0
		/* 7x27 has 256KB L2 cache:
			64Kb/Way and 4-Way Associativity;
			R/W latency: 3 cycles;
			evmon/parity/share disabled. */
		l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

static void __init swift_fixup(struct tag *tags, char **cmdline,
			       struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = (101*1024*1024);
}


static void __init swift_reserve(void)
{
	memblock_remove(0x0, SZ_2M);
}
static void __init swift_init_early(void)
{
	arch_ioremap_caller = __msm_ioremap_caller;
}

MACHINE_START(MSM7X27_SWIFT, "LGE GT540 SWIFT")
	.atag_offset	= 0x100,
	.map_io		= swift_map_io,
	.init_irq	= swift_init_irq,
	.init_machine	= swift_init,
	.timer		= &msm_timer,
        .fixup          = swift_fixup,
	.reserve        = swift_reserve,
        .init_early     = swift_init_early, 
MACHINE_END
