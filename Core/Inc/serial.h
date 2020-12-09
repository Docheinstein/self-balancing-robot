#ifndef SERIAL_H
#define SERIAL_H

#include "printf.h"

#define println(message, ...) printf(message "\r\n", ##__VA_ARGS__)

#endif /* SERIAL_H */
