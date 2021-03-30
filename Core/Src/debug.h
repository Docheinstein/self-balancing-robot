#ifndef DEBUG_H
#define DEBUG_H

/*
 * DEBUG_FMT() macro should be declared in each file using debugln() or debugf().
 * e.g.
 * #define DEBUG_FMT(fmt) "{MODULE_NAME} " fmt
 */

// Global DEBUG setting, set to 0 in production
#define DEBUG 1

#if DEBUG
#include "serial.h"

#ifndef DEBUG
#define DEBUG_FMT(fmt) fmt
#endif

#define debugf(message, ...) printf(DEBUG_FMT(message), ##__VA_ARGS__)
#define debugln(message, ...) println(DEBUG_FMT(message), ##__VA_ARGS__)

#else // DEBUG

#define debugf(message, ...)
#define debugln(message, ...)

#endif // DEBUG

#endif /* DEBUG_H */
