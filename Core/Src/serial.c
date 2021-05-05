#include "serial.h"
#include "cmsis_os.h"
#include "std.h"
#include "utime.h"
#include <string.h>
#include <stdbool.h>

// A larger buffer allow to keep track of more messages,
// but we have to pay attention not to go out of memory.
#define TX_RING_SIZE 32768

#define TX_MAX_MESSAGE_SIZE 512

// Set to 1 to abort execution when buffer is full;
// otherwise messages will be lost silently.
#define ASSERT_TX_NO_BUFFER_FULL 1


static Serial_Config config;

// == TRANSMISSION ==
static struct {
	// Main buffer containing the messages
	// written with Serial_Printf_Async.
	uint8_t data[TX_RING_SIZE];
	uint16_t r;
	uint16_t w;

	// Double buffer containing the next outgoing message
	// which is going to be written by Serial_Flush_Tx.
	uint8_t outgoing[TX_MAX_MESSAGE_SIZE];

	// Keep track of pending writes still to do
	// (e.g. HAL failed to write and we have to do it again)
	uint16_t outgoing_pending_len;

	SemaphoreHandle_t mutex;
	StaticSemaphore_t mutex_block;

	SemaphoreHandle_t sem_uart_tx_ready;
	StaticSemaphore_t sem_uart_tx_ready_block;

	SemaphoreHandle_t sem_uart_tx_end;
	StaticSemaphore_t sem_uart_tx_end_block;

	SemaphoreHandle_t sem_tx_data;
	StaticSemaphore_t sem_tx_data_block;

#if ASSERT_TX_NO_BUFFER_FULL
	bool full;
#endif
} tx_ring;

// == RECEPTION ==
static struct {
	SemaphoreHandle_t mutex;
	StaticSemaphore_t mutex_block;

	SemaphoreHandle_t sem_uart_rx_end;
	StaticSemaphore_t sem_uart_rx_end_block;

	HAL_StatusTypeDef last_status;
	uint8_t *data;
	uint16_t size;
	uint16_t i;
} rx;



void Serial_Init(Serial_Config cfg)
{
	config = cfg;

	tx_ring.r = 0;
	tx_ring.w = 0;

	// We have to use the FreeRTOS calls because the CMSIS ones
	// do not support initialization of a semaphore with a value different
	// from the maximum one.
	tx_ring.mutex = xSemaphoreCreateMutexStatic(&tx_ring.mutex_block);
	tx_ring.sem_uart_tx_ready = xSemaphoreCreateBinaryStatic(&tx_ring.sem_uart_tx_ready_block);
	tx_ring.sem_uart_tx_end = xSemaphoreCreateBinaryStatic(&tx_ring.sem_uart_tx_end_block);
	tx_ring.sem_tx_data = xSemaphoreCreateCountingStatic((
			TX_RING_SIZE / (sizeof(uint16_t) + 1)) + 1, 0, &tx_ring.sem_tx_data_block);


	xSemaphoreGive(tx_ring.sem_uart_tx_ready);

	rx.mutex = xSemaphoreCreateMutexStatic(&rx.mutex_block);
	rx.sem_uart_rx_end = xSemaphoreCreateBinaryStatic(&rx.sem_uart_rx_end_block);
}


/* Function called from printf.h when printf() is used */
void _putchar(char character)
{
	HAL_UART_Transmit(config.huart, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}


bool Serial_Write(uint8_t *data, uint16_t size)
{
	return HAL_UART_Transmit(config.huart, data, size, HAL_MAX_DELAY) == HAL_OK;
}

bool Serial_Read(uint8_t *data, uint16_t size)
{
	return HAL_UART_Receive(config.huart, data, size, HAL_MAX_DELAY) == HAL_OK;
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// Transmission ended
	osSemaphoreRelease(tx_ring.sem_uart_tx_end);
}

/*
 * Custom putchar which pushes into the transmission ring
 * buffer instead of write to the serial
 */
static void _tx_putchar(char character, void *arg)
{
#if ASSERT_TX_NO_BUFFER_FULL
	if (tx_ring.full)
		return; // already full, do nothing
#endif

	if ((tx_ring.w + 1) % TX_RING_SIZE != tx_ring.r) {
		tx_ring.data[tx_ring.w] = character;
		tx_ring.w = (tx_ring.w + 1) % TX_RING_SIZE;
		*((uint16_t *) arg) += 1;
	} else {
		// Can't write, would overlap read cursor (which will lead to data loss)
#if ASSERT_TX_NO_BUFFER_FULL
		tx_ring.full = true;
#endif
	}
}

bool Serial_Printf_Async(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	bool ret = false;

	osMutexWait(tx_ring.mutex, osWaitForever);

	/*
	 * Compute the remaming buffer space.
	 * There are two possible cases:
	 * ( . = data   <space> = remaining)
	 *
	 * w >= r =>   |   r......w         |
	 * w <  r  =>  |......w      r......|
	 */
	uint16_t remaining =
			(tx_ring.w >= tx_ring.r) ?
					(TX_RING_SIZE + tx_ring.r - tx_ring.w - 1) :
					(tx_ring.r - tx_ring.w - 1);

	if (remaining >= sizeof(uint16_t) + 1) {
		/*
		 * We still have space in the buffer (at least 3 bytes).
		 * The first two bytes will be the length of the string
		 * and the trailing part the actual string.
		 * | LEN_BYTE_1 | LEN_BYTE_0 | STRING ... |
		 *		0				1		  2...
		 */

		uint16_t begin = tx_ring.w;
		uint16_t str_begin = (begin + sizeof(uint16_t)) % TX_RING_SIZE;

		tx_ring.w = str_begin;

		uint16_t written = 0;
		uint16_t len = vfctprintf(_tx_putchar, &written, fmt, args);

		tx_ring.data[ begin                    ] = (written >> 8) & 0xFF;
		tx_ring.data[(begin + 1) % TX_RING_SIZE] =  written 	  & 0xFF;

		ret = written == len;

		// Notify that new data is available
		osSemaphoreRelease(tx_ring.sem_tx_data);
	} else {
		ret = false;
	}

	osMutexRelease(tx_ring.mutex);

	va_end(args);

	return ret;
}

/* Debug routing for dump the ring buffer in case it is full */
static void _Serial_BufferFullDumpAll_Tx()
{
	println(SERIAL_ENDL "TX BUFFER FULL - dump of remaining content:");
	println(SERIAL_ENDL "--- TX BUFFER START ---");

	int n = osSemaphoreGetCount(tx_ring.sem_tx_data);
	int r = tx_ring.r;
	uint16_t wrap_len = 0;
	for (int i = 0; i < n; i++) {
		println("-- %d/%d --" , i + 1, n);

		if (wrap_len) {
			if (!Serial_Write(tx_ring.data, wrap_len)) {
				println("-- serial error --");
				break;
			}
			wrap_len = 0;
		} else {
			uint16_t begin = r;
			uint16_t str_begin = (begin + sizeof(uint16_t)) % TX_RING_SIZE;
			uint16_t len =  (tx_ring.data[ begin + 0                    ] << 8) |
							(tx_ring.data[(begin + 1) % TX_RING_SIZE]);
			uint16_t no_wrap_len = MIN(len, TX_RING_SIZE - str_begin);
			wrap_len = len - no_wrap_len;

			if (!Serial_Write(&tx_ring.data[str_begin], no_wrap_len)) {
				println("-- serial error --");
				break;
			}

			r = (str_begin + len) % TX_RING_SIZE;
		}
	}

	println(SERIAL_ENDL "--- TX BUFFER END---");
	while (true) {}; // abort
}

bool Serial_Flush_Tx()
{
	osSemaphoreWait(tx_ring.sem_tx_data, osWaitForever);
	osSemaphoreWait(tx_ring.sem_uart_tx_ready, osWaitForever);

#if ASSERT_TX_NO_BUFFER_FULL
	if (tx_ring.full) _Serial_BufferFullDumpAll_Tx();
#endif

	uint16_t str_len;

	osMutexWait(tx_ring.mutex, osWaitForever);

	if (!tx_ring.outgoing_pending_len) {
		// Copy the string into the outoing buffer for now, se that we can release
		// the mutex and updating the read index as soon as possible without any
		// additional transmission or other blocking waits.
		uint16_t begin = tx_ring.r;
		uint16_t str_begin = (begin + sizeof(uint16_t)) % TX_RING_SIZE;
		str_len =
				(tx_ring.data[ begin + 0                    ] << 8) |
				(tx_ring.data[(begin + 1) % TX_RING_SIZE]);
		uint16_t no_wrap_len = MIN(str_len, TX_RING_SIZE - str_begin);
		uint16_t wrap_len = str_len - no_wrap_len;
		memcpy(tx_ring.outgoing, &tx_ring.data[str_begin], no_wrap_len);
		if (wrap_len)
			// Copy the remaining part of the messages
			// (splitted due to end circular buffer)
			memcpy(&tx_ring.outgoing[no_wrap_len], tx_ring.data, wrap_len);
		// Advance the read buffer; note that we can do it now even if the message
		// has not been printed yet just because we have copied the message
		// in the 'outgoing' buffer
		tx_ring.r = (str_begin + str_len) % TX_RING_SIZE;
	} else {
		str_len = tx_ring.outgoing_pending_len;
	}

	osMutexRelease(tx_ring.mutex);

	if (HAL_UART_Transmit_DMA(config.huart, tx_ring.outgoing, str_len) != HAL_OK) {
		/* The transmission failed; release the channel but remember that we still
		 * have to transfer the last message; maybe we'll be more lucky the next time */
		tx_ring.outgoing_pending_len = str_len;
		osSemaphoreRelease(tx_ring.sem_tx_data);
		osSemaphoreRelease(tx_ring.sem_uart_tx_ready);
		return false;
	}

	/* Wait the end of the transmission before release the tx channel */
	tx_ring.outgoing_pending_len = 0;
	osSemaphoreWait(tx_ring.sem_uart_tx_end, osWaitForever);
	osSemaphoreRelease(tx_ring.sem_uart_tx_ready);

	return true;
}


bool Serial_ReadStringCR(char *data, uint16_t size)
{
	uint16_t c;

	for (c = 0; c < size - 1; c++) {
		if (HAL_UART_Receive(config.huart, (uint8_t *) &data[c], 1, HAL_MAX_DELAY) != HAL_OK)
			return false;
		if (data[c] == '\r')
			break;
	}

	data[c] = '\0';

	return c < size - 1;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (rx.data[rx.i] == '\r') {
		// end of string (regular case)
		rx.data[rx.i] = '\0';
		osSemaphoreRelease(rx.sem_uart_rx_end);
	} else if (rx.i == rx.size - 2) {
		// end of buffer -> truncate here
		rx.data[rx.i + 1] = '\0';
		osSemaphoreRelease(rx.sem_uart_rx_end);
	} else {
		// read next char (regular case)
		rx.i++;
		rx.last_status = HAL_UART_Receive_IT(config.huart, &rx.data[rx.i], 1);
		if (rx.last_status != HAL_OK) {
			osSemaphoreRelease(rx.sem_uart_rx_end);
		}
	}
}


bool Serial_ReadStringCR_Async(char *data, uint16_t size)
{
	if (size < 2)
		return false;

	osMutexWait(rx.mutex, osWaitForever);
	rx.data = (uint8_t *) data;
	rx.size = size;
	rx.i = 0;
	rx.last_status = HAL_UART_Receive_IT(config.huart, &rx.data[rx.i], 1);
	if (rx.last_status != HAL_OK) {
		// Release the channel if we failed without trying
		osSemaphoreRelease(rx.sem_uart_rx_end);
	}
	return rx.last_status == HAL_OK;
}

bool Serial_Wait_Rx()
{
	osSemaphoreWait(rx.sem_uart_rx_end, osWaitForever);
	bool truncated = rx.i >= rx.size - 1;
	bool ok = !truncated && rx.last_status == HAL_OK;
	osMutexRelease(rx.mutex);
	return ok;
}
