#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "printf.h"
#include "utime.h"

#define SERIAL_ENDL "\r\n"

//#define GET_TIMESTAMP() GetMilliseconds()
#define GET_TIMESTAMP() GetMicroseconds()

// ======== ALIASES ========

#define aprintf(fmt, ...)	Serial_Printf_Async(fmt, ##__VA_ARGS__)
#define taprintf(fmt, ...)  aprintf("[%u] " fmt, GET_TIMESTAMP(), ##__VA_ARGS__)

#define aprintln(fmt, ...)  aprintf(fmt SERIAL_ENDL, ##__VA_ARGS__)
#define taprintln(fmt, ...) aprintln("[%u] " fmt, GET_TIMESTAMP(), ##__VA_ARGS__)

#define println(fmt, ...)	printf(fmt SERIAL_ENDL, ##__VA_ARGS__)
#define tprintln(fmt, ...)  println("[%u] " fmt, GET_TIMESTAMP(), ##__VA_ARGS__)

#define tprintf(fmt, ...)   printf("[%u] " fmt, GET_TIMESTAMP(), ##__VA_ARGS__)


typedef struct Serial_Config {
	UART_HandleTypeDef *huart;
} Serial_Config;

// ======== FUNCTIONS ========

void Serial_Init(Serial_Config config);

/*
 * (Blocking) writes 'size' bytes from 'data' to serial
 */
bool Serial_Write(uint8_t *data, uint16_t size);

/*
 * (Blocking) reads 'size' bytes from serial into 'data'
 */
bool Serial_Read(uint8_t *data, uint16_t size);

/*
 * (Blocking) printf to serial
 */
#define Serial_Printf(fmt, ...) printf(fmt, ##__VA_ARGS__)

/*
 * (Non blocking) printf to serial; actually just add the string to a circular buffer.
 * Serial_Flush_Tx() should be called to actually write the message to serial.
 */
bool Serial_Printf_Async(const char *fmt, ...);

/*
 * (Blocking) writes the next message submitted with Serial_Printf_Async()
 * to the serial (blocking).
 */
bool Serial_Flush_Tx();

/*
 * (Blocking) reads '\r' terminated a string of max 'size' bytes from serial.
 */
bool Serial_ReadStringCR(char *data, uint16_t size);

/*
 * (Non blocking) setup a read of a '\r' terminated string  of max 'size'
 * bytes from serial.
 * Serial_Wait_Rx() should be called in order to check that the new string has
 * been read into 'data'.
 */
bool Serial_ReadStringCR_Async(char *data, uint16_t size);

/*
 * (Blocking) wait for the next string to be read from serial
 * (the read must be setup with Serial_ReadStringCR_Async).
 */
bool Serial_Wait_Rx();

#endif /* SERIAL_H */
