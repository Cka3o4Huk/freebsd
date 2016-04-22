/*
 * chipc_spi.h
 *
 *  Created on: Mar 24, 2016
 *      Author: mizhka
 */

#pragma once

#define CHIPC_SPI_MAXRES 		2
#define CHIPC_SPI_MAXTRIES 		1000

#define CHIPC_SPI_ACTION_INPUT	1
#define CHIPC_SPI_ACTION_OUTPUT	2

#define CHIPC_SPI_FLASHCTL			0x00
#define		CHIPC_SPI_FLASHCTL_OPCODE	0x000000ff
#define		CHIPC_SPI_FLASHCTL_ACTION	0x00000700 //
/*
 * We don't use action at all. Experimentaly found, that
 *  action 0 - read current MISO byte to data register (interactive mode)
 *  action 1 = read 2nd byte to data register
 *  action 2 = read 4th byte to data register (surprise! see action 6)
 *  action 3 = read 5th byte to data register
 *  action 4 = read bytes 5-8 to data register in swapped order
 *  action 5 = read bytes 9-12 to data register in swapped order
 *  action 6 = read 3rd byte to data register
 *  action 7 = read bytes 6-9 to data register in swapped order
 * It may be wrong if CS bit is 1.
 * If CS bit is 1, you should write cmd / data to opcode byte-to-byte.
 */
#define		CHIPC_SPI_FLASHCTL_CSACTIVE	0x00001000
#define		CHIPC_SPI_FLASHCTL_START	0x80000000 //same as BUSY
#define		CHIPC_SPI_FLASHCTL_BUSY		0x80000000 //same as BUSY
#define CHIPC_SPI_FLASHADDR			0x04
#define CHIPC_SPI_FLASHDATA			0x08


struct chipc_spi_softc{
	device_t dev;
	struct bhnd_resource* bhnd_res[CHIPC_SPI_MAXRES];
	// SPI registers
	int sc_mem_rid;
	struct resource* sc_mem_res;

	// MMIO flash
	struct resource	*sc_res;
	bus_space_handle_t sc_handle;
	bus_space_tag_t	sc_tag;
	int		sc_rid;
};

/*
 * register space access macros
 */

#define	SPI_BARRIER_WRITE(sc)		bus_barrier((sc)->sc_mem_res, 0, 0, 	\
					    BUS_SPACE_BARRIER_WRITE)
#define	SPI_BARRIER_READ(sc)	bus_barrier((sc)->sc_mem_res, 0, 0, 	\
					    BUS_SPACE_BARRIER_READ)
#define	SPI_BARRIER_RW(sc)		bus_barrier((sc)->sc_mem_res, 0, 0, 	\
					    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE)

#define SPI_WRITE(sc, reg, val)	do {	\
		bus_write_4(sc->sc_mem_res, (reg), (val)); \
	} while (0)

#define SPI_READ(sc, reg)	 bus_read_4(sc->sc_mem_res, (reg))

#define SPI_SET_BITS(sc, reg, bits)	\
	SPI_WRITE(sc, reg, SPI_READ(sc, (reg)) | (bits))

#define SPI_CLEAR_BITS(sc, reg, bits)	\
	SPI_WRITE(sc, reg, SPI_READ(sc, (reg)) & ~(bits))


