/*
 * bcm_socinfo.c
 *
 *  Created on: Apr 19, 2016
 *      Author: mizhka
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include "bcm_socinfo.h"

// found on https://wireless.wiki.kernel.org/en/users/drivers/b43/soc
struct bcm_socinfo bcm_socinfos[] = {
		{0x00005300, 600, 25000000}, // BCM4706 - to check
		{0x0022B83A, 300, 20000000}, // BCM4716B0 - ASUS RT-N12
		{0x00914716, 354, 20000000}, // BCM4717A1 - to check
		{0x00A14716, 480, 20000000}, // BCM4718A1 - ASUS RT-N16
		{0x00435356, 300, 25000000}, // BCM5356A1 - to check (ASUS RT-N10, WNR1000v3)
		{0x00825357, 500, 20000000}, // BCM5358UB0 - ASUS RT-N53A1
		{0x00845357, 300, 20000000}, // BCM5357B0 - to check
		{0x00945357, 500, 20000000}, // BCM5358
		{0x00A45357, 500, 20000000}, // BCM47186B0 - Tenda N60
		{0x0085D144, 300, 20000000}, // BCM5356C0 -
		{0x00B5D144, 300, 20000000}, // BCM5357C0
		{0,0,0}
};

// Most popular BCM SoC info
struct bcm_socinfo BCM_DEFAULT_SOCINFO = {0x0, 300, 20000000};

struct bcm_socinfo* bcm_lookup_by_socid(uint32_t key){
	if(!key)
		return 0;
	struct bcm_socinfo* start = bcm_socinfos;
	for(start = bcm_socinfos; start->id > 0; start++){
		if(start->id == key)
			return start;
	}
	return 0;
}

void bcm_get_socinfo(struct bcm_socinfo** socinfo){

	/*
	 * We need Chip ID + Revision + Package
	 * Chip ID
        Mask 		Usage
		0x0000FFFF	Chip ID
		0x000F0000	Chip Revision
		0x00F00000	Package Options
		0x0F000000	Number of Cores (ChipCommon Core Revision >= 4)
		0xF0000000	Chip Type
	 */

	uint32_t socid = BCM_READ_SOCREG(0) & 0x00FFFFFF;
	*socinfo = bcm_lookup_by_socid(socid);
	if(!*socinfo)
		*socinfo = &BCM_DEFAULT_SOCINFO;
}

