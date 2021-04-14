///
/// \file uart_lib.h
/// \author Lionel GENEVE
/// \date 01/01/2021
/// \version 1.0
/// \brief UART library with open/close/read/write functions
///

#ifndef UART_LIB_H
#define UART_LIB_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

///
/// \fn init_uart
///
int init_uart(const char* portName, speed_t baudRate);

///
/// \fn close_uart
///
int close_uart(void);

///
/// \fn read_uart
///
int read_uart(uint8_t* data, uint8_t data_size);

///
/// \fn write_uart
///
int write_uart(uint8_t* data, uint8_t data_size);

#endif // UART_LIB_H
