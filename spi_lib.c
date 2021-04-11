///
/// \file spi_lib.c
/// \author Lionel GENEVE
/// \date 01/01/2021
/// \version 1.0
/// \brief SPI library with open/close/read/write functions
///

#include "spi_lib.h"

static const char *device0 = "/dev/spidev0.0"; // CS0
static const char *device1 = "/dev/spidev0.1"; // CS1

/**
 * SPI options in mode
device   (default /dev/spidev1.1)
speed    max speed (Hz)
delay    delay (usec)
bpw      bits per word
loop     loopback						mode |= SPI_LOOP;
cpha     clock phase					mode |= SPI_CPHA;
cpol     clock polarity					mode |= SPI_CPOL;
lsb      least significant bit first	mode |= SPI_LSB_FIRST;
cs-high  chip select active high		mode |= SPI_CS_HIGH;
3wire    SI/SO signals shared			mode |= SPI_3WIRE;
no-cs    no chip select					mode |= SPI_NO_CS;
ready    slave pulls low to pause		mode |= SPI_READY;
dual     dual transfer					mode |= SPI_TX_DUAL;
quad     quad transfer					mode |= SPI_TX_QUAD;
**/
static uint8_t spi_bits = 8;
static uint32_t spi_speed = 1000000; // 1 Mhz
static uint16_t spi_delay = 0;
///----- SET SPI MODE -----
///SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
///SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
///SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
///SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
static uint8_t spi_mode = SPI_MODE_0;

/// File descriptor for the SPI  device
int spi_fd = -1;

/// Message for a read or write operation
struct spi_ioc_transfer trx[2] = {0, };

static void pabort(const char *s) {
	perror(s);
	abort();
}

int spi_init(int ss) {
	int ret = 0;
	const char *device = 0;

	if(ss == 0)
		device = device0;
	else
		device = device1;

	spi_fd = open(device, O_RDWR);
	if(spi_fd < 0)
		pabort("Can't open spi\n");

	ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
	if (ret == -1)
		pabort("Can't set WR spi mode\n");

	ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
	if (ret == -1)
		pabort("Can't get RD spi mode\n");

	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
	if (ret == -1)
		pabort("Can't set WR bits per word\n");

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits);
	if (ret == -1)
		pabort("Can't get RD bits per word\n");

	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		pabort("Can't set WR max speed hz\n");

	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		pabort("Can't get RD max speed hz\n");
	/// DEBUG
#ifdef DEBUG
	fprintf(stderr, "spi_mode=0x%x\n", spi_mode);
	fprintf(stderr, "spi_bits=%d\n", spi_bits);
	fprintf(stderr, "spi_speed=%d Hz (%d KHz)\n", spi_speed, spi_speed/1000);
#endif

	/// Prepare messages
	trx[0].len = 1;
	trx[0].delay_usecs = spi_delay;
	trx[0].speed_hz = spi_speed;
	trx[0].bits_per_word = spi_bits;
	trx[0].cs_change = 0;

	trx[1].len = 1;
	trx[1].delay_usecs = spi_delay;
	trx[1].speed_hz = spi_speed;
	trx[1].bits_per_word = spi_bits;
	trx[1].cs_change = 1;

	///free(device);
	usleep(100000); // 100 ms

	return ret;
}

int spi_close() {
	int ret = 0;

	ret = close(spi_fd);
	if(ret < 0)
		pabort("Can't close SPI device\n");

	return ret;
}

int spi_read(uint8_t reg, uint8_t* data) {
	int ret = 0;

	reg |= SPI_READ_WRITE_BIT; // Read operation

	trx[0].tx_buf = (unsigned long)(&reg);
	trx[0].rx_buf = (unsigned long)NULL;

	trx[1].tx_buf = (unsigned long)NULL;
	trx[1].rx_buf = (unsigned long)data;
	trx[1].len = 1;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), trx);
	if(ret < 1) // or ret < 0 ??
		pabort("-spi_read- Can't send spi message\n");

	return ret;
}

int spi_burst_read(uint8_t reg, uint8_t* data, uint8_t length) {
	int ret = 0;

	reg |= SPI_READ_WRITE_BIT; // Read operation

	trx[0].tx_buf = (unsigned long)(&reg);
	trx[0].rx_buf = (unsigned long)NULL;

	trx[1].tx_buf = (unsigned long)NULL;
	trx[1].rx_buf = (unsigned long)data;
	trx[1].len = length;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), trx);
	if(ret < 1) // or ret < 0 ??
		pabort("-spi_burst_read- Can't send spi message\n");

	return ret;
}

int spi_write(uint8_t reg, uint8_t data) {
	int ret = 0;

	trx[0].tx_buf = (unsigned long)(&reg);
	trx[0].rx_buf = (unsigned long)NULL;

	trx[1].tx_buf = (unsigned long)(&data);
	trx[1].rx_buf = (unsigned long)NULL;
	trx[1].len = 1;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), trx);
	if(ret < 1) // or ret < 0 ??
		pabort("-spi_write- Can't send spi message\n");

	return ret;
}

int spi_burst_write(uint8_t reg, uint8_t* data, uint8_t length) {
	int ret = 0;

	trx[0].tx_buf = (unsigned long)(&reg);
	trx[0].rx_buf = (unsigned long)NULL;

	trx[1].tx_buf = (unsigned long)data;
	trx[1].rx_buf = (unsigned long)NULL;
	trx[1].len = length;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), trx);
	if (ret < 1) // or ret < 0 ??
		pabort("-spi_burst_write- Can't send spi message\n");

	return ret;
}
