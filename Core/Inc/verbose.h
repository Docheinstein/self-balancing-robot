#ifndef VERBOSE_H
#define VERBOSE_H

/*
 * VERBOSE_FMT() macro should be declared in each
 * file using [a]verboseln() or [a]verbosef()..
 * e.g.
 * #define VERBOSE_FMT(fmt) "{MODULE_NAME} " fmt
 */

// Global VERBOSE setting, set to 0 in production
#ifndef VERBOSE
	#define VERBOSE 1
#endif

#ifndef VERBOSE_TIMESTAMP_DEFAULT
	#define VERBOSE_TIMESTAMP_DEFAULT 1
#endif

#if VERBOSE
	#include "serial.h"
	#include "stm32l4xx_hal.h"

	extern bool verbose;

	#define tverbosef(fmt, ...)  	do { if (verbose) tprintf(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)
	#define tverboseln(fmt, ...)	do { if (verbose) tprintln(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)

	#define taverbosef(fmt, ...)	do { if (verbose) taprintf(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)
	#define taverboseln(fmt, ...)	do { if (verbose) taprintln(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)

	#if VERBOSE_TIMESTAMP_DEFAULT
		#define verbosef(fmt, ...) 	 tverbosef(fmt, ##__VA_ARGS__)
		#define verboseln(fmt, ...)  tverboseln(fmt, ##__VA_ARGS__)

		#define averbosef(fmt, ...)  taverbosef(fmt, ##__VA_ARGS__)
		#define averboseln(fmt, ...) taverboseln(fmt, ##__VA_ARGS__)
	#else
		#define verbosef(fmt, ...) 	 do { if (verbose) printf(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)
		#define verboseln(fmt, ...)  do { if (verbose) println(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)

		#define averbosef(fmt, ...)  do { if (verbose) aprintf(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)
		#define averboseln(fmt, ...) do { if (verbose) aprintln(VERBOSE_FMT(fmt), ##__VA_ARGS__); } while(0)
	#endif // VERBOSE_TIMESTAMP_DEFAULT
#else
	#define tverbosef(fmt, ...) do { } while(0)
	#define tverboseln(fmt, ...) do { } while(0)

	#define taverbosef(fmt, ...) do { } while(0)
	#define taverboseln(fmt, ...) do { } while(0)

	#define verbosef(fmt, ...) do { } while(0)
	#define verboseln(fmt, ...) do { } while(0)

	#define averbosef(fmt, ...) do { } while(0)
	#define averboseln(fmt, ...) do { } while(0)
#endif // VERBOSE

#endif /* VERBOSE_H */
