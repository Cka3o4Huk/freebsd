/*
 * b53switch.h
 *
 *  Created on: May 25, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCH_H_
#define SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCH_H_


#define	B53_SHIFT(val, SHF)	((val << SHF ## _SHIFT) & SHF ## _MASK)
#define	B53_UNSHIFT(val, SHF)	((val & SHF ## _MASK) >> SHF ## _SHIFT)

/*
 * Macroses for READ / WRITE over parent MDIO / MII
 */

#define B53_WRITEREG(reg, val)							\
	if(MDIO_WRITEREG(sc->sc_parent, B53_PSEUDOPHY_ADDR, reg, val)) {	\
		device_printf(sc->sc_dev, "MDIO_WRITEREG failed [%x/%x]", 	\
		    reg, val); 							\
	}

#define B53_READREG(reg)							\
	MDIO_READREG(sc->sc_parent, B53_PSEUDOPHY_ADDR, reg)

#define B53SWITCH_LOCK(_sc)			\
	    mtx_lock(&(_sc)->sc_mtx)
#define B53SWITCH_UNLOCK(_sc)			\
	    mtx_unlock(&(_sc)->sc_mtx)
#define B53SWITCH_LOCK_ASSERT(_sc, _what)		\
	    mtx_assert(&(_sc)->sc_mtx, (_what))
#define B53SWITCH_TRYLOCK(_sc)			\
	    mtx_trylock(&(_sc)->sc_mtx)

#endif /* SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCH_H_ */
