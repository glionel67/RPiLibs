///
/// \file uart_test.c
/// \author Lionel GENEVE
/// \date 01/01/2021
/// \version 1.0
/// \brief UART test
///

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/signal.h>
//#include <pthread.h>

#include "uart_lib.h"

// Defines
#define RX_BUFFER_SIZE (255)
#define BAUDRATE (B115200) //B9600

// Global variables
static const char* UART_PORT_NAME = "/dev/ttyAMA0";
static uint8_t stop = 0;

void signal_handler(int sig)
{
	fprintf(stderr, "[CTRL-C] signal catched!\n");
	stop = 1;
}

int main(int argc, char** argv)
{
	int ret = 0;
	uint8_t rx_buffer[RX_BUFFER_SIZE] = {0, };
	int i = 0;
	stop = 0;

	/// Init. UART
	ret = init_uart(UART_PORT_NAME, BAUDRATE);
	if (ret != 0) {
		fprintf(stderr, "init_uart error\n");
		exit(-1);
	}
	fprintf(stderr, "UART port opened\n");

	/// Catch (ctrl-c) signal to stop infinite
	signal(SIGINT, signal_handler);

    /// Main loop
	while (!stop) {
		/// Get RX data
		ret = read_uart(rx_buffer, RX_BUFFER_SIZE);
		if (ret > 0) {
			for (i = 0; i < ret; i++) {
				fprintf(stderr, "%c", rx_buffer[i]);
				ret = write_uart(rx_buffer+i, 1);
				rx_buffer[i] = 0;
			}
		}
	}

	/// Close uart
	ret = close_uart();
	if (ret != 0) {
		fprintf(stderr, "close_uart error\n");
		exit(-1);
	}

	return 0;
} // main
