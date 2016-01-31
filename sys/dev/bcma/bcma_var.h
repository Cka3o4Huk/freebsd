/*
 * bcma_var.h
 *
 *  Created on: Jan 24, 2016
 *      Author: mizhka
 */
#pragma once

#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/queue.h>

#include <machine/resource.h>

MALLOC_DECLARE(M_BCMA);

struct bcma_chipinfo {
	u_int16_t id;
	u_int8_t tbd :4;
	u_int8_t package :4;
	u_int8_t total_cores :4;
	u_int8_t revision :4;
};

struct bcma_eeprom_coreaddress {
	u_int8_t  type;
	u_int8_t  port;
	u_int32_t address;
	u_int32_t size;
	LIST_ENTRY(bcma_eeprom_coreaddress) next_address;
};

struct bcma_eeprom_coreinfo {
	LIST_ENTRY(bcma_eeprom_coreinfo) next_core;
	u_int16_t manuf;
	u_int16_t id;
	u_int8_t class;
	u_int8_t revision;
	u_int8_t num_mports; //master
	u_int8_t num_sports; //slave
	u_int8_t num_mwraps; //master
	u_int8_t num_swraps; //slave
	LIST_HEAD(bcma_eeprom_address_head, bcma_eeprom_coreaddress) addresses;
	struct resource_list resources;
};

struct bcma_softc {
	device_t bcma_dev;
	struct resource* mem;
	struct resource* erom_mem;
	int mem_rid;
	int erom_rid;
	struct bcma_chipinfo chip;
	LIST_HEAD(bcma_eeprom_info_head, bcma_eeprom_coreinfo) cores;
};

struct bcma_chipcommon_softc {


	int unused;
};

/**
 * Private methods of BCMA
 */
void bcma_release_resources(struct bcma_softc* sc);
void bcma_scan_eeprom(struct bcma_softc* sc);
int bcma_init_scan_eeprom(struct bcma_softc* sc, u_int32_t offset, u_int32_t size, int resource_id);
