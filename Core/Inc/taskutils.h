#ifndef TASKUTILS_H
#define TASKUTILS_H

#include "verbose.h"
#include "serial.h"

/*
 * The following macro are used mostly for debug for print
 * additional information about the task which are using
 * the signal wait/set mechanism.
 *
 * A task have to call TASK_PROLOGUE() at the start of the function before
 * using any of the following function.
 */

#define TASK_PROLOGUE(name, verbose) \
	static const char *TASK_NAME = #name; \
	static const bool TASK_VERBOSE = verbose; \
	UNUSED(TASK_NAME); \
	UNUSED(TASK_VERBOSE);

/* ==== PRINT FUNCTIONS ==== */

#define Task_aprintf(fmt, ...) do { \
	if (TASK_VERBOSE) aprintf("{%s} " fmt, TASK_NAME, ##__VA_ARGS__);  \
} while(0)
#define Task_aprintln(fmt, ...) do { \
	if (TASK_VERBOSE) aprintln("{%s} " fmt, TASK_NAME, ##__VA_ARGS__);  \
} while(0)implementation
#define Task_taprintf(fmt, ...) do { \
	if (TASK_VERBOSE) taprintf("{%s} " fmt, TASK_NAME, ##__VA_ARGS__); \
} while(0)
#define Task_taprintln(fmt, ...) do { \
	if (TASK_VERBOSE) taprintln("{%s} " fmt, TASK_NAME, ##__VA_ARGS__); \
} while(0)



#if VERBOSE
	#define Task_averbosef(fmt, ...) do { \
		if (TASK_VERBOSE) averbosef("{%s} " fmt, TASK_NAME, ##__VA_ARGS__); \
	} while(0)
	#define Task_averboseln(fmt, ...) do { \
		if (TASK_VERBOSE) averboseln("{%s} " fmt, TASK_NAME, ##__VA_ARGS__); \
	} while(0)
#else
	#define Task_averbosef(fmt, ...) do { } while(0)
	#define Task_averboseln(fmt, ...) do { } while(0)
#endif


/* ==== SIGNAL FUNCTIONS ==== */

#if VERBOSE
	#define Task_SignalWaitGet(ret, sigs, timeout) do {\
		if (timeout == osWaitForever) \
			Task_averboseln("Waiting forever for " #sigs); \
		else \
			Task_averboseln("Waiting %u ms for " #sigs, timeout); \
		ret = osSignalWait(sigs, timeout); \
		if (ret.status == osEventTimeout) \
			Task_averboseln("Timeout of " #sigs); \
		else { \
			if (ret.status == osEventSignal && \
				ret.value.signals & (sigs)) \
				Task_averboseln("Received " #sigs); \
			else \
				Task_averboseln("Spurious wake up by " #sigs); \
		} \
	} while(0)

	#define Task_SignalWait(sigs, timeout) do {\
		if (timeout == osWaitForever) \
			Task_averboseln("Waiting forever for " #sigs); \
		else \
			Task_averboseln("Waiting %u ms for " #sigs, timeout); \
		osSignalWait(signame, timeout); \
			Task_averboseln("Received " #sigs); \
	} while(0)

	#define Task_SignalSet(task, signame) do { \
		Task_averboseln("Sending " #signame " to " #task); \
		osSignalSet(task, signame); \
	} while(0)
#else
	#define Task_SignalWaitGet(ret, sigs, timeout) ret = osSignalWait(sigs, timeout)
	#define Task_SignalWait(sigs, timeout) osSignalWait(sigs, timeout)
	#define Task_SignalSet(task, sigs) osSignalSet(task, sigs)
#endif // VERBOSE

#endif // TASKUTILS_H
