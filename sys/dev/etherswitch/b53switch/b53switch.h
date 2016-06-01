/*
 * b53switch.h
 *
 *  Created on: May 25, 2016
 *      Author: mizhka
 */

#ifndef _B53SWITCH_H_
#define _B53SWITCH_H_

#define	B53_SHIFT(val, SHF)	((val << SHF ## _SHIFT) & SHF ## _MASK)
#define	B53_UNSHIFT(val, SHF)	((val & SHF ## _MASK) >> SHF ## _SHIFT)

/*
 * Macroses for READ / WRITE over parent MDIO / MII
 */

#define B53_WRITEREG(r, v)						\
	if (MDIO_WRITEREG(sc->sc_parent, B53_PSEUDOPHY_ADDR, r, v)) {	\
		device_printf(sc->sc_dev, "WRITEREG failed: %x/%x", r,  \
		    v); 						\
	}

#define B53_READREG(reg)						\
	MDIO_READREG(sc->sc_parent, B53_PSEUDOPHY_ADDR, reg)

#define B53SWITCH_LOCK(_sc)						\
	    mtx_lock(&(_sc)->sc_mtx)
#define B53SWITCH_UNLOCK(_sc)						\
	    mtx_unlock(&(_sc)->sc_mtx)
#define B53SWITCH_LOCK_ASSERT(_sc, _what)				\
	    mtx_assert(&(_sc)->sc_mtx, (_what))
#define B53SWITCH_TRYLOCK(_sc)						\
	    mtx_trylock(&(_sc)->sc_mtx)

#define B53DUMPREG(_reg)						\
	device_printf(dev, #_reg "=%08x\n", b53switch_read4(sc, _reg))
#define B53DUMP()							\
	B53DUMPREG(PORT_CTL(PORT0));					\
	B53DUMPREG(PORT_CTL(PORT1));					\
	B53DUMPREG(PORT_CTL(PORT2));					\
	B53DUMPREG(PORT_CTL(PORT3));					\
	B53DUMPREG(PORT_CTL(PORT4));					\
	B53DUMPREG(PORT_CTL(PORT5));					\
	B53DUMPREG(PORT_CTL(PORT6));					\
	B53DUMPREG(PORT_CTL(PORT7));					\
	B53DUMPREG(PORT_CTL(PORTMII));					\
	B53DUMPREG(PORTMII_STATUS_OVERRIDE);				\
	B53DUMPREG(SWITCH_DEVICEID);					\
	B53DUMPREG(SWITCH_MODE);					\
	B53DUMPREG(POWER_DOWN_MODE);					\
	B53DUMPREG(LINK_STATUS_SUMMARY);				\
	B53DUMPREG(BIST_STATUS_RC);					\
	B53DUMPREG(VLAN_GLOBAL_CTL0);					\
	B53DUMPREG(VLAN_GLOBAL_CTL1);					\
	B53DUMPREG(VLAN_GLOBAL_CTL2);					\
	B53DUMPREG(VLAN_DROP_UNTAGGED);					\
	B53DUMPREG(VLAN_GLOBAL_CTL4);					\
	B53DUMPREG(VLAN_GLOBAL_CTL5);					\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(0));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(1));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(2));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(3));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(4));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(5));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(6));				\
	B53DUMPREG(VLAN_DEFAULT_PORT_TAG(7));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(0));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(1));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(2));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(3));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(4));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(5));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(6));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(7));				\
	B53DUMPREG(PBVLAN_ALLOWED_PORTS(8));

#endif /* _B53SWITCH_H_ */
