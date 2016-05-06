/*
 * chipcbus.h
 *
 *  Created on: May 6, 2016
 *      Author: mizhka
 */

#pragma once

#include <sys/queue.h>
#define NUM_IRQS	6

struct chipcbus_spec {
	char* name;
	int type, rid;
	int port, reg;
	rman_res_t start;
	rman_res_t size;

};

struct chipcbus_softc {
	device_t dev;
	struct rman chipc_mem;
	struct rman chipc_irq;
};

struct chipcbus_reg{
	SLIST_ENTRY(chipcbus_reg) entries;
	rman_res_t start, end;
	int port, reg;
};

struct chipcbus_ivar {
	SLIST_HEAD(mem_list, chipcbus_reg) mems;
	int irq_cnt;
	SLIST_HEAD(irq_list, chipcbus_reg) irqs;
};

struct chipcbus_devinfo {
	struct resource_list resources;
};

/*
 * Prototypes
 */
void chipcbus_cleanup(device_t dev);
