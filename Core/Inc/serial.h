#ifndef SERIAL_H_
#define SERIAL_H_

#include "stdio.h"

#define println(message, ...) printf(message "\r\n", ##__VA_ARGS__)

#endif /* SERIAL_H_ */
