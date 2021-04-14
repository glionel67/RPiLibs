///
/// \file spi_lib.h
/// \author Lionel GENEVE
/// \date 01/01/2021
/// \version 1.0
/// \brief SPI open/close/read/write functions
///

#ifndef SPI_LIB_H
#define SPI_LIB_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define DEBUG

#define SPI_READ_WRITE_BIT (0x80)

static void pabort(const char *s);

/// Initialize SPI bus
/// ss = slave select: 0=CS0, 1=CS1
int spi_init(int ss);

/// Close SPI bus
int spi_close();

/// reg = address of the register to read
/// data = data read
int spi_read(uint8_t reg, uint8_t* data);

/// reg = address of the register to read
/// data = data read
/// length = nb of bytes to read
int spi_burst_read(uint8_t reg, uint8_t* data, uint8_t length);

/// reg = address of the register to write
/// data = data to write
int spi_write(uint8_t reg, uint8_t data);

/// reg = address of the register to write
/// data = data to write
/// length = nb of bytes to write
int spi_burst_write(uint8_t reg, uint8_t* data, uint8_t length);

#endif // SPI_LIB_H
