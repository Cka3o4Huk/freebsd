/*
 * bcm_socinfo.h
 *
 *  Created on: Apr 19, 2016
 *      Author: mizhka
 */

#pragma once

#include <machine/cpuregs.h>

struct bcm_socinfo {
	uint32_t id;
	uint32_t cpurate; //MHz
	uint32_t uartrate; //Hz
};

struct bcm_socinfo* bcm_lookup_by_socid(uint32_t key);
void bcm_get_socinfo(struct bcm_socinfo** socinfo);

#define BCM_SOCADDR				0x18000000
#define BCM_READ_SOCREG(reg)	BCM_READ_REG(BCM_SOCADDR, reg)
#define BCM_READ_REG(base, reg)	*((volatile uint32_t *)MIPS_PHYS_TO_KSEG1((base + reg)))

