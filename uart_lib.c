///
/// \file uart_lib.c
/// \author Lionel GENEVE
/// \date 01/01/2021
/// \version 1.0
/// \brief UART library with open/close/read/write functions
///

#include "uart_lib.h"

static const char* serialPortName =  "/dev/ttyAMA0"; // Name on the Raspberry Pi

int fd_uart = -1; // File descriptor

int init_uart(const char* portName, speed_t baudRate) {
	struct termios termiosSerialPort;
	/// Open serial port
	fd_uart = open(portName, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd_uart == -1) {
		perror("Error: cannot open serial port\n");
		exit(-1);
	}
	/// Configure serial port
	tcgetattr(fd_uart, &termiosSerialPort);
	termiosSerialPort.c_iflag = IGNBRK | IGNPAR;
	termiosSerialPort.c_oflag = 0;
	termiosSerialPort.c_cflag = baudRate | CS8 | CREAD | CLOCAL;
	termiosSerialPort.c_cflag &= ~PARENB;
	termiosSerialPort.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termiosSerialPort.c_cc[VMIN] = 0;
	termiosSerialPort.c_cc[VTIME] = 0;
	cfsetospeed(&termiosSerialPort, baudRate);   
	cfsetispeed(&termiosSerialPort, baudRate); 
	tcsetattr(fd_uart, TCSANOW, &termiosSerialPort);
	//ioctl(fd_uart, IOSSIOSPEED, &baudRate);
	
	return 0;
}

int close_uart(void) {
	int ret = 0;

	ret = close(fd_uart);
	if (ret < 0)
		perror("Can't close UART serial port\n");

	return ret;
}

int read_uart(uint8_t* data, uint8_t data_size) {
	return read(fd_uart, data, data_size);
}

int write_uart(uint8_t* data, uint8_t data_size) {
	return write(fd_uart, data, data_size);
}
