/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "bcma_eromreg.h"
#include "bcma_eromvar.h"

/*
 * BCMA Enumeration ROM (EROM) Table
 * 
 * Provides auto-discovery of BCMA cores on Broadcom's HND SoC.
 * 
 * The EROM core address can be found at BCMA_CC_EROM_ADDR within the
 * ChipCommon registers. The table itself is comprised of 32-bit
 * type-tagged entries, organized into an array of variable-length
 * core descriptor records.
 * 
 * The final core descriptor is followed by a 32-bit BCMA_EROM_TABLE_EOF (0xF)
 * marker.
 */

static const char	*erom_entry_type_name (uint8_t entry);
static int		 erom_read32(struct bcma_erom *erom, uint32_t *entry);
static int		 erom_skip32(struct bcma_erom *erom);

#define	EROM_LOG(erom, fmt, ...)	\
	device_printf(erom->dev, "erom[0x%lx]: " fmt, erom->offset-4, \
	    ##__VA_ARGS__);


/** Return the type name for an EROM entry */
static const char *
erom_entry_type_name (uint8_t entry)
{
	switch (BCMA_EROM_GET_ATTR(entry, ENTRY_TYPE)) {
	case BCMA_EROM_ENTRY_TYPE_CORE:
		return "core";
	case BCMA_EROM_ENTRY_TYPE_MPORT:
		return "mport";
	case BCMA_EROM_ENTRY_TYPE_REGION:
		return "region";
	default:
		return "unknown";
	}
}

/**
 * Return the current read position.
 */
bus_size_t
bcma_erom_tell(struct bcma_erom *erom)
{
	return (erom->offset);
}

/**
 * Seek to an absolute read position.
 */
void
bcma_erom_seek(struct bcma_erom *erom, bus_size_t offset)
{
	erom->offset = offset;
}

/**
 * Read a 32-bit entry value from the EROM table without advancing the
 * read position.
 * 
 * @param erom EROM read state.
 * @param entry Will contain the read result on success.
 * @retval 0 success
 * @retval ENOENT The end of the EROM table was reached.
 * @retval non-zero The read could not be completed.
 */
int
bcma_erom_peek32(struct bcma_erom *erom, uint32_t *entry)
{
	if (erom->offset >= BCMA_EROM_TABLE_START + BCMA_EROM_TABLE_SIZE) {
		EROM_LOG(erom, "BCMA EROM table missing terminating EOF\n");
		return (EINVAL);
	}
	
	*entry = bus_read_4(erom->res, erom->offset);
	return (0);
}

/**
 * Read a 32-bit entry value from the EROM table.
 * 
 * @param erom EROM read state.
 * @param entry Will contain the read result on success.
 * @retval 0 success
 * @retval ENOENT The end of the EROM table was reached.
 * @retval non-zero The read could not be completed.
 */
static int
erom_read32(struct bcma_erom *erom, uint32_t *entry)
{
	int error;

	if ((error = bcma_erom_peek32(erom, entry)) == 0)
		erom->offset += 4;

	return (error);
}

/**
 * Read and discard 32-bit entry value from the EROM table.
 * 
 * @param erom EROM read state.
 * @retval 0 success
 * @retval ENOENT The end of the EROM table was reached.
 * @retval non-zero The read could not be completed.
 */
static int
erom_skip32(struct bcma_erom *erom)
{
	uint32_t	entry;

	return erom_read32(erom, &entry);
}

/**
 * "Open" an EROM core for reading.
 * 
 * @param resource An active resource mapping the EROM core.
 * @param offset Offset of the EROM core relative to @p resource.
 * @param[out] erom On success, will be populated with a valid EROM
 * read state.
 * 
 * @retval 0 success
 * @retval non-zero if the resource's size is insufficient for a valid EROM
 * table.
 */
int
bcma_erom_open(struct resource *resource, bus_size_t offset,
    struct bcma_erom *erom)
{
	bus_size_t count;

	/* Initialize the EROM reader */
	erom->dev = rman_get_device(resource);
	erom->res = resource;
	erom->offset = offset + BCMA_EROM_TABLE_START;
	
	/* Verify the resource size */
	count = rman_get_end(resource) - rman_get_start(resource);
	if (erom->offset > count || 
	    count - erom->offset < BCMA_EROM_TABLE_SIZE)
	{
		device_printf(erom->dev, "invalid EROM resource size\n");
		return (EINVAL);
	}

	return (0);
}

/**
 * Read the next core descriptor from the EROM table.
 * 
 * @param erom EROM read state.
 * @param[out] core On success, will be populated with the parsed core
 * descriptor data.
 * @retval 0 success
 * @retval ENOENT The end of the EROM table was reached.
 * @retval non-zero Reading or parsing the core descriptor failed.
 */
int
bcma_erom_parse_core(struct bcma_erom *erom, struct bcma_erom_core *core)
{
	uint32_t	entry;
	int		error;

	/* Parse CoreDescA */
	if ((error = erom_read32(erom, &entry)))
		return (error);
	
	/* Handle EOF */
	if (entry == BCMA_EROM_TABLE_EOF)
		return (ENOENT);
	
	if (!BCMA_EROM_ENTRY_IS(entry, CORE)) {
		EROM_LOG(erom, "Unexpected EROM entry 0x%x (type=%s)\n",
                   entry, erom_entry_type_name(entry));
		
		return (EINVAL);
	}

	core->vendor = BCMA_EROM_GET_ATTR(entry, COREA_DESIGNER);
	core->device = BCMA_EROM_GET_ATTR(entry, COREA_ID);
	
	/* Parse CoreDescB */
	if ((error = erom_read32(erom, &entry)))
		return (error);

	if (!BCMA_EROM_ENTRY_IS(entry, CORE)) {
		return (EINVAL);
	}

	core->rev = BCMA_EROM_GET_ATTR(entry, COREB_REV);
	core->num_mport = BCMA_EROM_GET_ATTR(entry, COREB_NUM_MP);
	core->num_dport = BCMA_EROM_GET_ATTR(entry, COREB_NUM_DP);
	core->num_mwrap = BCMA_EROM_GET_ATTR(entry, COREB_NUM_WMP);
	core->num_swrap = BCMA_EROM_GET_ATTR(entry, COREB_NUM_WSP);

	return (0);
}

/**
 * Read the next master port descriptor from the EROM table.
 * 
 * @param erom EROM read state.
 * @param[out] mport On success, will be populated with the parsed
 * descriptor data.
 * @retval 0 success
 * @retval non-zero Reading or parsing the descriptor failed.
 */
int
bcma_erom_parse_mport(struct bcma_erom *erom,
    struct bcma_erom_mport *mport)
{
	uint32_t	entry;
	int		error;

	/* Parse the master port descriptor */
	if ((error = erom_read32(erom, &entry)))
		return (error);
	
	if (!BCMA_EROM_ENTRY_IS(entry, MPORT))
		return (EINVAL);

	mport->port_vid = BCMA_EROM_GET_ATTR(entry, MPORT_ID);
	mport->port_num = BCMA_EROM_GET_ATTR(entry, MPORT_NUM);

	return (0);
}

/**
 * Read the next slave port region descriptor from the EROM table.
 * 
 * @param erom EROM read state.
 * @param[out] mport On success, will be populated with the parsed
 * descriptor data.
 * @retval 0 success
 * @retval ENOENT The end of the region descriptor table was reached.
 * @retval non-zero Reading or parsing the descriptor failed.
 */
int
bcma_erom_parse_sport_region(struct bcma_erom *erom,
    struct bcma_erom_sport_region *region)
{
	uint32_t	entry;
	uint8_t		size_type;
	int		error;

	/* Peek at the region descriptor */
	if (bcma_erom_peek32(erom, &entry))
		return (EINVAL);

	/* A non-region entry signals the end of the region table */
	if (!BCMA_EROM_ENTRY_IS(entry, REGION)) {
		return (ENOENT);
	} else {
		erom_skip32(erom);
	}

	region->base_addr = BCMA_EROM_GET_ATTR(entry, REGION_BASE);
	region->port_type = BCMA_EROM_GET_ATTR(entry, REGION_TYPE);
	region->port_num = BCMA_EROM_GET_ATTR(entry, REGION_PORT);
	size_type = BCMA_EROM_GET_ATTR(entry, REGION_SIZE);

	/* If region address is 64-bit, fetch the high bits. */
	if (BCMA_EROM_GET_ATTR(entry, REGION_64BIT)) {
		if ((error = erom_read32(erom, &entry)))
			return (error);
		
		region->base_addr |= ((bcma_addr_t) entry << 32);
	}

	/* Parse the region size; it's either encoded as the binary logarithm
	 * of the number of 4K pages (i.e. log2 n), or its encoded as a
	 * 32-bit/64-bit literal value directly following the current entry. */
	if (size_type == BCMA_EROM_REGION_SIZE_OTHER) {
		if ((error = erom_read32(erom, &entry)))
			return (error);

		region->size = BCMA_EROM_GET_ATTR(entry, RSIZE_VAL);

		if (BCMA_EROM_GET_ATTR(entry, RSIZE_64BIT)) {
			if ((error = erom_read32(erom, &entry)))
				return (error);
			region->size |= ((bcma_size_t) entry << 32);
		}
	} else {
		region->size = BCMA_EROM_REGION_SIZE_BASE << size_type;
	}

	/* Verify that addr+size does not overflow. */
	if (region->size != 0 &&
	    BCMA_ADDR_MAX - (region->size - 1) < region->base_addr)
	{
		EROM_LOG(erom, "%s%u: invalid address map %llx:%llx\n",
		    bcma_port_type_name(region->port_type),
		    region->port_num,
		    (unsigned long long) region->base_addr,
		    (unsigned long long) region->size);

		return (EINVAL);
	}

	return (0);
}

/**
 * Seek to the next valid core descriptor.
 * 
 * @param erom EROM read state.
 * @param[out] mport On success, will be populated with the parsed
 * descriptor data.
 * @retval 0 success
 * @retval ENOENT The end of the EROM table was reached.
 * @retval non-zero Reading or parsing the descriptor failed.
 */
int
bcma_erom_skip_core(struct bcma_erom *erom)
{
	bus_size_t	pos;
	uint32_t	entry;
	int		error;

	/* If we're currently positioned at a core descriptor, this
	 * gaurantees that we'll skip to the next core */
	if ((error = erom_read32(erom, &entry)))
		return (error);
	
	/* We may have just read the EOF marker */
	if (entry == BCMA_EROM_TABLE_EOF)
		return (ENOENT);

	while (!error) {
		/* Save our current position */
		pos = erom->offset;

		/* Fetch the next entry */
		if ((error = erom_read32(erom, &entry)))
			return (error);

		/* Detect EOF */
		if (entry == BCMA_EROM_TABLE_EOF)
			return (ENOENT);
		
		if (!BCMA_EROM_GET_ATTR(entry, ENTRY_ISVALID))
			return (EINVAL);

		switch (BCMA_EROM_GET_ATTR(entry, ENTRY_TYPE)) {
		case BCMA_EROM_ENTRY_TYPE_CORE:
			erom->offset = pos;
			return (0);

		case BCMA_EROM_ENTRY_TYPE_MPORT:
			break;
		
		case BCMA_EROM_ENTRY_TYPE_REGION:
			/* Skip region address high-bits */
			if (BCMA_EROM_GET_ATTR(entry, REGION_64BIT)) {
				if ((error = erom_skip32(erom)))
					return (error);
			}

			/* Skip literal size value */
			if (BCMA_EROM_GET_ATTR(entry, REGION_SIZE) ==
			    BCMA_EROM_REGION_SIZE_OTHER)
			{
				if ((error = erom_read32(erom, &entry)))
					return (error);
				
				/* Skip region size high bits */
				if (BCMA_EROM_GET_ATTR(entry, RSIZE_64BIT)) {
					if ((error = erom_skip32(erom)))
						return (error);
				}
			}
			
			break;

		default:
			/* Unknown entry type! */
			return (EINVAL);	
		}
	}

	return (error);
}
