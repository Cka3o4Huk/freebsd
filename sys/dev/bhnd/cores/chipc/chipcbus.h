/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef _BHND_CORES_CHIPC_CHIPCBUS_H_
#define _BHND_CORES_CHIPC_CHIPCBUS_H_

#include <sys/queue.h>
#define	NUM_IRQS	6

struct chipcbus_spec {
	char		*name;
	int 		 type, rid;
	int 		 port, reg;
	rman_res_t 	 start;
	rman_res_t	 size;
};

struct chipcbus_softc {
	device_t 	dev;
	struct rman 	chipc_mem;
	struct rman 	chipc_irq;
};

struct chipcbus_reg{
	SLIST_ENTRY(chipcbus_reg) entries;
	rman_res_t 	start, end;
	int 		port, reg;
};

struct chipcbus_ivar {
	SLIST_HEAD(mem_list, chipcbus_reg) mems;
	SLIST_HEAD(irq_list, chipcbus_reg) irqs;
};

struct chipcbus_devinfo {
	struct resource_list resources;
};

#endif /* _BHND_CORES_CHIPC_CHIPCBUS_H_ */
