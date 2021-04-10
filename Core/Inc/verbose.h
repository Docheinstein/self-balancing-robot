#ifndef VERBOSE_H
#define VERBOSE_H

/*
 * VERBOSE_FMT() macro should be declared in each file using verboseln() or verbosef().
 * e.g.
 * #define VERBOSE_FMT(fmt) "{MODULE_NAME} " fmt
 */

// Global VERBOSE setting, set to 0 in production
#ifndef VERBOSE
#define VERBOSE 0
#endif

#if VERBOSE
#include "serial.h"

#ifndef VERBOSE
#define VERBOSE_FMT(fmt) fmt
#endif

#define verbosef(message, ...) printf(VERBOSE_FMT(message), ##__VA_ARGS__)
#define verboseln(message, ...) println(VERBOSE_FMT(message), ##__VA_ARGS__)

#else

#define verbosef(message, ...)
#define verboseln(message, ...)

#endif // VERBOSE

#endif /* VERBOSE_H */
