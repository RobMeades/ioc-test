/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// The number of log entries
#define MAX_NUM_LOG_ENTRIES 5000

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The possible events for the RAM log
// If you add an item here, don't forget to
// add it to gLogEventStrings also.
typedef enum {
    EVENT_NONE,
    EVENT_LOG_START,
    EVENT_LOG_STOP,
    EVENT_FILE_OPEN,
    EVENT_FILE_OPEN_FAILURE,
    EVENT_FILE_CLOSE,
    EVENT_NETWORK_START,
    EVENT_NETWORK_START_FAILURE,
    EVENT_NETWORK_STOP,
    EVENT_TCP_CONNECTED,
    EVENT_TCP_CONNECTION_PROBLEM,
    EVENT_TCP_CONFIGURED,
    EVENT_TCP_CONFIGURATION_PROBLEM,
    EVENT_I2S_START,
    EVENT_I2S_STOP,
    EVENT_BUTTON_PRESSED,
    EVENT_I2S_DMA_RX_HALF_FULL,
    EVENT_I2S_DMA_RX_FULL,
    EVENT_I2S_DMA_UNKNOWN,
    EVENT_DATAGRAM_ALLOC,
    EVENT_DATAGRAM_NUM_SAMPLES,
    EVENT_DATAGRAM_SIZE,
    EVENT_DATAGRAM_READY_TO_SEND,
    EVENT_DATAGRAM_FREE,
    EVENT_DATAGRAM_OVERFLOW_BEGINS,
    EVENT_DATAGRAM_NUM_OVERFLOWS,
    EVENT_RAW_AUDIO_DATA_0,
    EVENT_RAW_AUDIO_DATA_1,
    EVENT_STREAM_MONO_SAMPLE_DATA,
    EVENT_MONO_SAMPLE_UNUSED_BITS,
    EVENT_MONO_SAMPLE_UNUSED_BITS_MIN,
    EVENT_MONO_SAMPLE_AUDIO_SHIFT,
    EVENT_STREAM_MONO_SAMPLE_PROCESSED_DATA,
    EVENT_SEND_START,
    EVENT_SEND_STOP,
    EVENT_SEND_FAILURE,
    EVENT_SOCKET_BAD,
    EVENT_SOCKET_ERRORS_FOR_TOO_LONG,
    EVENT_TCP_SEND_TIMEOUT,
    EVENT_SEND_SEQ_SKIP,
    EVENT_FILE_WRITE_START,
    EVENT_FILE_WRITE_STOP,
    EVENT_FILE_WRITE_FAILURE,
    EVENT_SEND_DURATION_GREATER_THAN_BLOCK_DURATION,
    EVENT_SEND_DURATION,
    EVENT_NEW_PEAK_SEND_DURATION,
    EVENT_GUARD_OVERWRITE_1,
    EVENT_GUARD_OVERWRITE_2,
    EVENT_GUARD_OVERWRITE_3,
    EVENT_GUARD_OVERWRITE_4,
    EVENT_USER_1,
    EVENT_USER_2,
    EVENT_NUM_DATAGRAMS_FREE,
    EVENT_NUM_DATAGRAMS_QUEUED,
    EVENT_THROUGHPUT_BITS_S,
    EVENT_TCP_WRITE,
    EVENT_TCP_QUEUELEN,
    EVENT_TCP_SEQ,
    EVENT_TCP_SNDWND,
    EVENT_TCP_CWND,
    EVENT_TCP_WND,
    EVENT_TCP_EFFWND,
    EVENT_TCP_ACK
} LogEvent;

// An entry in the RAM log
typedef struct {
    int timestamp;
    LogEvent event;
    int parameter;
} LogEntry;

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

// Log an event plus parameter
void LOG(LogEvent event, int parameter);

#ifdef __cplusplus
}
#endif

// Initialise logging
void initLog();

// Print out the logged items
void printLog();
