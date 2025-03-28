/*
 * Copyright (c) 2023 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

#define ADDR_TRANSLATE_RAT_BASE_ADDR		(0x44200000u)
#define PINCTRL_BASE_ADDR			(0x04080000u)
#define KICK0_UNLOCK_VAL			(0x68EF3490U)
#define KICK1_UNLOCK_VAL			(0xD172BC5AU)
#define CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET	(0x1008)
#define CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET	(0x5008)

static struct address_trans_region_config am6x_region_config[] = {
	{
		.system_addr = 0x00000000u,
		.local_addr =  0x60000000u,
		.size = address_trans_region_size_256M,
	},
	{
		.system_addr = 0x20000000u,
		.local_addr =  0xc0000000u,
		.size = address_trans_region_size_512M,
	},
	{
		.system_addr = 0x40000000u,
		.local_addr =  0x70000000u,
		.size = address_trans_region_size_256M,
	},
/*
 * Add regions here if you want to map more memory.
 */
};

static void am6x_mmr_unlock(void)
{
	uint32_t baseAddr = PINCTRL_BASE_ADDR;
	uintptr_t kickAddr;

	/* Lock 0 */
	kickAddr = baseAddr + CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET;
	sys_write32(KICK0_UNLOCK_VAL, kickAddr);   /* KICK 0 */
	kickAddr = kickAddr + sizeof(uint32_t *);
	sys_write32(KICK1_UNLOCK_VAL, kickAddr);   /* KICK 1 */

	/* Lock 1 */
	kickAddr = baseAddr + CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET;
	sys_write32(KICK0_UNLOCK_VAL, kickAddr);   /* KICK 0 */
	kickAddr = kickAddr + sizeof(uint32_t *);
	sys_write32(KICK1_UNLOCK_VAL, kickAddr);   /* KICK 1 */
}

void soc_early_init_hook(void)
{
	sys_mm_drv_ti_rat_init(am6x_region_config, ADDR_TRANSLATE_RAT_BASE_ADDR,
		ARRAY_SIZE(am6x_region_config));
	am6x_mmr_unlock();
}
