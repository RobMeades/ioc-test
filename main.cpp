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

#include "mbed.h"
#include "UbloxATCellularInterface.h"
#include "UbloxPPPCellularInterface.h"
#include "EthernetInterface.h"
#include "I2S.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Define this to use Ethernet instead of Cellular
#define USE_ETHERNET

// Define this to disable guard checking
//#define DISABLE_GUARD_CHECK

// Define this to send a fixed debug audio tone instead
// of that retrieved from the I2S interface
//#define STREAM_FIXED_TONE

// The duration for which to send audio, 0 == forever
#define STREAM_DURATION_MILLISECONDS 0

// The gain shift, 0 == automatic
#define GAIN_LEFT_SHIFT 0

// Network API
#ifdef USE_ETHERNET
#  define INTERFACE_CLASS  EthernetInterface
#else
#  define INTERFACE_CLASS  UbloxPPPCellularInterface
#endif

// The credentials of the SIM in the board.  If PIN checking is enabled
// for your SIM card you must set this to the required PIN.
#define PIN "0000"

// Network credentials.  You should set this according to your
// network/SIM card.  For C030 boards, leave the parameters as NULL
// otherwise, if you do not know the APN for your network, you may
// either try the fairly common "internet" for the APN (and leave the
// username and password NULL), or you may leave all three as NULL and then
// a lookup will be attempted for a small number of known networks
// (see APN_db.h in mbed-os/features/netsocket/cellular/utils).
#define APN         NULL
#define USERNAME    NULL
#define PASSWORD    NULL

// If SERVER_NAME is defined then the URTP
// stream will be sent to the named server
// on SERVER_PORT
#define SERVER_NAME "ciot.it-sgn.u-blox.com"
#define SERVER_PORT 5065

// If this is defined then the audio part of the stream
// (i.e. minus the header) will be written to the named file
// on the SD card.
// Note: don't define both this and SERVER_NAME, there's not
// enough time to do both
//#define LOCAL_FILE "/sd/audio.bin"

// The audio sampling frequency in Hz
// Effectively this is the frequency of the WS signal on the
// I2S interface
#define SAMPLING_FREQUENCY 16000

// The duration of a block in milliseconds
#define BLOCK_DURATION_MS 20

// The number of samples in a 20 ms block.  Note that a sample
// is stereo when the audio is in raw form but is reduced to
// mono when we organise it into URTP packets, hence the size
// of a sample is different in each case (64 bits for stereo,
// 24 bits for mono)
#define SAMPLES_PER_BLOCK (SAMPLING_FREQUENCY * BLOCK_DURATION_MS / 1000)

// The number of valid bytes in each mono sample of audio received
// on the I2S interface (the number of bytes received may be larger
// but some are discarded along the way)
#define MONO_INPUT_SAMPLE_SIZE 3

// The number of milliseconds over which to average the
// audio gain control
#define AUDIO_AVERAGING_INTERVAL_MILLISECONDS 20

// The desired number of unused bits to keep in the audio processing to avoid
// clipping when we can't move fast enough due to averaging
#define AUDIO_DESIRED_UNUSED_BITS 8

// The maximum audio shift to use
#define AUDIO_MAX_SHIFT_BITS 12

// I looked at using RTP to send data up to the server
// (https://en.wikipedia.org/wiki/Real-time_Transport_Protocol
// and https://tools.ietf.org/html/rfc3551, a minimal RTP header)
// but we have no time to do audio processing and we have 16
// bit audio at 16000 samples/second to send, which doesn't
// correspond with an RTP payload type anyway.  So instead we do
// something RTP-like, in that we send fixed length datagrams at
// regular intervals. A 20 ms interval would contain a block
// of 320 samples, so 5120 bits, hence an overall data rate of
// 256 kbits/s is required.  We can do some silence detection to
// save bandwidth.
//
// The datagram format is:
//
// Byte  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
//--------------------------------------------------------
//  0    |              protocol version = 0             |
//  1    |                    unused                     |
//  2    |             Sequence number MSB               |
//  3    |             Sequence number LSB               |
//  4    |                Timestamp MSB                  |
//  5    |                Timestamp byte                 |
//  6    |                Timestamp byte                 |
//  7    |                Timestamp LSB                  |
//--------------------------------------------------------
//  8    |                 Sample 1 MSB                  |
//  9    |                 Sample 1 LSB                  |
//  11   |                 Sample 2 MSB                  |
//  12   |                 Sample 2 LSB                  |
//       |                     ...                       |
//  N    |                 Sample M MSB                  |
//  N+1  |                 Sample M LSB                  |
//
// ...where the number of samples is between 0 and 160,
// the gTimeMilliseconds is in milliseconds and the sequence number
// increments by one for each datagram.
//
// The receiving end should be able to reconstruct an audio
// stream from this gTimeMillisecondsed/ordered data.  For the sake
// of a name, call this URTP.

#define PROTOCOL_VERSION   0
#define URTP_HEADER_SIZE   8
#define URTP_SAMPLE_SIZE   2
#define URTP_BODY_SIZE     (URTP_SAMPLE_SIZE * SAMPLES_PER_BLOCK)
#define URTP_DATAGRAM_SIZE (URTP_HEADER_SIZE + URTP_BODY_SIZE)

// The maximum number of URTP datagrams that we need to store
// This is sufficient for 2 seconds of audio
#define MAX_NUM_DATAGRAMS 100

// A signal to indicate that a datagram is ready to send
#define SIG_DATAGRAM_READY 0x01

// The number of log entries
#define MAX_NUM_LOG_ENTRIES 2000

// A guard integer to check for overruns
#define GUARD_INT 0xdeadbeef

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// A linked list container
typedef struct ContainerTag {
    bool inUse;
    void * pContents;
    ContainerTag *pNext;
} Container;

// A struct to pass data to the task that sends data to the network
typedef struct {
    UDPSocket * pSock;
    SocketAddress * pServer;
} SendParams;

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
    EVENT_DATAGRAM_OVERFLOW,
    EVENT_RAW_AUDIO_DATA_0,
    EVENT_RAW_AUDIO_DATA_1,
    EVENT_STREAM_MONO_SAMPLE_DATA,
    EVENT_MONO_SAMPLE_UNUSED_BITS,
    EVENT_MONO_SAMPLE_AVERAGE_UNUSED_BITS,
    EVENT_MONO_SAMPLE_AUDIO_SHIFT,
    EVENT_STREAM_MONO_SAMPLE_PROCESSED_DATA,
    EVENT_SEND_START,
    EVENT_SEND_STOP,
    EVENT_SEND_FAILURE,
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
    EVENT_NUM_DATAGRAMS_FREE
} LogEvent;

// An entry in the RAM log
typedef struct {
    int timestamp;
    LogEvent event;
    int parameter;
} LogEntry;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Thread required by I2S driver task
static Thread gI2sTask;

// Function that forms the body of the gI2sTask
static Callback<void()> gI2STaskCallback(&I2S::i2s_bh_queue, &events::EventQueue::dispatch_forever);

// Audio buffer, enough for two blocks of stereo audio,
// where each sample takes up 64 bits (32 bits for L channel
// and 32 bits for R channel).
static uint32_t gRawAudio[(SAMPLES_PER_BLOCK * 2) * 2];

// A buffer to monitor average headroom in audio sampling
__attribute__ ((section ("CCMRAM")))
static char gAudioUnusedBits[SAMPLING_FREQUENCY / (1000 / AUDIO_AVERAGING_INTERVAL_MILLISECONDS)];
__attribute__ ((section ("CCMRAM")))
static uint32_t gGuard1;
static char *pgAudioUnusedBitsNext = gAudioUnusedBits;
static uint32_t gAudioUnusedBitsTotal = 0;
static int gAudioShift = 0;

// Task to send data off to the network server
static Thread gSendTask;

// Array of our "RTP-like" datagrams.
static char gDatagram[MAX_NUM_DATAGRAMS][URTP_DATAGRAM_SIZE];

// A linked list to manage the datagrams, must have the same
// number of elements as gDatagram
__attribute__ ((section ("CCMRAM")))
static Container gContainer[MAX_NUM_DATAGRAMS];
__attribute__ ((section ("CCMRAM")))
static uint32_t gGuard2;

// A sequence number
static int gSequenceNumber = 0;

// A millisecond timestamp
static Timer gTimeMilliseconds;

// Pointer to the next unused container
static Container *gpContainerNextEmpty = gContainer;

// Pointer to the next container to transmit
static volatile Container *gpContainerNextTx = gContainer;

// A UDP socket
static UDPSocket gSock;

// A server address
static SocketAddress gServer;

// LEDs
static DigitalOut gLedRed(LED1, 1);
static DigitalOut gLedGreen(LED2, 1);
static DigitalOut gLedBlue(LED3, 1);

// The user button
static volatile bool gButtonPressed = false;

#ifdef LOCAL_FILE
  static FILE *gpFile = NULL;
  static SDBlockDevice gSd(D11, D12, D13, D10);
  static FATFileSystem gFs("sd");
  // Writing to file is only fast enough if we
  // write a large block in one go, hence this
  // buffer (which must be a multiple of
  // URTP_BODY_SIZE in size).
  __attribute__ ((section ("CCMRAM")))
  static char gFileBuf[URTP_BODY_SIZE * (MAX_NUM_DATAGRAMS / 2)];
  __attribute__ ((section ("CCMRAM")))
  static uint32_t gGuard3;
  static char * gpFileBuf = gFileBuf;
#endif

// Record some timings so that we can see how we're doing
static int gMaxTime = 0;
static uint64_t gAverageTime = 0;
static uint64_t gNumTimes = 0;
static int gNumDatagramsFree = 0;
static int gMinNumDatagramsFree = 0;

// A logging buffer
__attribute__ ((section ("CCMRAM")))
static LogEntry gLog[MAX_NUM_LOG_ENTRIES];
__attribute__ ((section ("CCMRAM")))
static uint32_t gGuard4;
static LogEntry *gpLogNext = gLog;
static unsigned int gNumLogEntries = 0;

// A logging timestamp
static Timer gLogTime;

// Log an event plus parameter
#define LOG(x, y) gpLogNext->timestamp = gLogTime.read_us(); \
                  gpLogNext->event = x; \
                  gpLogNext->parameter = y; \
                  gpLogNext++; \
                  if (gNumLogEntries < sizeof (gLog) / sizeof (gLog[0])) { \
                      gNumLogEntries++; \
                  } \
                  if (gpLogNext >= gLog +  sizeof (gLog) / sizeof (gLog[0])) { \
                      gpLogNext = gLog; \
                  }

// The events as strings (must be kept in line with the
// LogEvent enum
static const char * gLogStrings[] = {
    "  EMPTY",
    "  LOG_START",
    "  LOG_STOP",
    "  FILE_OPEN",
    "  FILE_OPEN_FAILURE",
    "  FILE_CLOSE",
    "  NETWORK_START",
    "  NETWORK_START_FAILURE",
    "  NETWORK_STOP",
    "  I2S_START",
    "  I2S_STOP",
    "  BUTTON_PRESSED",
    "  I2S_DMA_RX_HALF_FULL",
    "  I2S_DMA_RX_FULL",
    "* I2S_DMA_UNKNOWN",
    "  DATAGRAM_ALLOC",
    "  DATAGRAM_NUM_SAMPLES",
    "  DATAGRAM_SIZE",
    "  DATAGRAM_READY_TO_SEND",
    "  DATAGRAM_FREE",
    "* DATAGRAM_OVERFLOW",
    "  RAW_AUDIO_DATA_0",
    "  RAW_AUDIO_DATA_1",
    "  STREAM_MONO_SAMPLE_DATA",
    "  MONO_SAMPLE_UNUSED_BITS",
    "  MONO_SAMPLE_AVERAGE_UNUSED_BITS",
    "  MONO_SAMPLE_AUDIO_SHIFT",
    "  STREAM_MONO_SAMPLE_PROCESSED_DATA",
    "  SEND_START",
    "  SEND_STOP",
    "* SEND_FAILURE",
    "  FILE_WRITE_START",
    "  FILE_WRITE_STOP",
    "* FILE_WRITE_FAILURE",
    "* SEND_DURATION_GREATER_THAN_BLOCK_DURATION",
    "  SEND_DURATION",
    "  NEW_PEAK_SEND_DURATION",
    "* GUARD_OVERWRITE_1",
    "* GUARD_OVERWRITE_2",
    "* GUARD_OVERWRITE_3",
    "* GUARD_OVERWRITE_4",
    "  USER_1",
    "  USER_2",
    "  NUM_DATAGRAMS_FREE"
};


#ifdef STREAM_FIXED_TONE
// A 400 Hz sine wave as a little-endian 24 bit signed PCM stream sampled at 16 kHz, generated by Audacity and
// then sign extended to 32 bits per sample
static const int gPcm400HzSigned24Bit[] = {(int) 0x00000000L, (int) 0x001004d5L, (int) 0x001fa4b2L, (int) 0x002e7d16L, (int) 0x003c3070L, (int) 0x00486861L, (int) 0x0052d7e5L, (int) 0x005b3d33L, (int) 0x00616360L, (int) 0x006523a8L,
                                           (int) 0x00666666L, (int) 0x006523a8L, (int) 0x00616360L, (int) 0x005b3d33L, (int) 0x0052d7e5L, (int) 0x00486861L, (int) 0x003c3070L, (int) 0x002e7d16L, (int) 0x001fa4b2L, (int) 0x001004d5L,
                                           (int) 0x00000000L, (int) 0xffeffb2aL, (int) 0xffe05b4eL, (int) 0xffd182e9L, (int) 0xffc3cf90L, (int) 0xffb7979eL, (int) 0xffad281bL, (int) 0xffa4c2ccL, (int) 0xff9e9ca0L, (int) 0xff9adc57L,
                                           (int) 0xff999999L, (int) 0xff9acd57L, (int) 0xff9e9ca0L, (int) 0xffa4c2ccL, (int) 0xffad281bL, (int) 0xffb7979eL, (int) 0xffc3cf90L, (int) 0xffd182e9L, (int) 0xffe05beeL, (int) 0xffeffb2aL};
static unsigned int gToneIndex = 0;
#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Indicate an event (blue)
static void event() {
    gLedBlue = 0;
    gLedGreen = 1;
    gLedRed = 1;
}

// Indicate not an event (blue off)
static void notEvent() {
    gLedBlue = 1;
}

// Indicate good (green)
static void good() {
    gLedGreen = 0;
    gLedBlue = 1;
    gLedRed = 1;
}

// Indicate bad (red)
static void bad() {
    gLedRed = 0;
    gLedGreen = 1;
    gLedBlue = 1;
}

// Toggle green
static void toggleGreen() {
    gLedGreen = !gLedGreen;
}

// All off
static void ledOff() {
    gLedBlue = 1;
    gLedRed = 1;
    gLedGreen = 1;
}

// Something to attach to the button
static void buttonCallback()
{
    gButtonPressed = true;
    LOG(EVENT_BUTTON_PRESSED, 0);
    event();
}

// Initialise the linked list
static void initContainers(Container * pContainer, int numContainers)
{
    Container *pTmp;
    Container *pStart = pContainer;

    for (int x = 0; x < numContainers; x++) {
        pContainer->inUse = false;
        gNumDatagramsFree++;
        pContainer->pContents = (void *) &(gDatagram[x]);
        pContainer->pNext = NULL;
        pTmp = pContainer;
        pContainer++;
        pTmp->pNext = pContainer;
    }
    // Handle the final pNext, make the list circular
    pTmp->pNext = pStart;
    gMinNumDatagramsFree = gNumDatagramsFree;

    LOG(EVENT_NUM_DATAGRAMS_FREE, gNumDatagramsFree);
}

// Initialise guard areas
static void initGuards()
{
    gGuard1 = GUARD_INT;
    gGuard2 = GUARD_INT;
#ifdef LOCAL_FILE
    gGuard3 = GUARD_INT;
#endif
    gGuard4 = GUARD_INT;
}

// Check guard areas
static void inline checkGuards()
{
#ifndef DISABLE_GUARD_CHECK
    if (gGuard1 != GUARD_INT) {
        LOG(EVENT_GUARD_OVERWRITE_1, (int) &gGuard1);
        LOG(EVENT_GUARD_OVERWRITE_1, gGuard1);
        bad();
    }
    if (gGuard2 != GUARD_INT) {
        LOG(EVENT_GUARD_OVERWRITE_2, (int) &gGuard2);
        LOG(EVENT_GUARD_OVERWRITE_2, gGuard2);
        bad();
    }
#  ifdef LOCAL_FILE
    if (gGuard3 != GUARD_INT) {
        LOG(EVENT_GUARD_OVERWRITE_3, (int) &gGuard3);
        LOG(EVENT_GUARD_OVERWRITE_3, gGuard3);
        bad();
    }
#  endif
    if (gGuard4 != GUARD_INT) {
        LOG(EVENT_GUARD_OVERWRITE_4, (int) &gGuard4);
        LOG(EVENT_GUARD_OVERWRITE_4, gGuard4);
        bad();
    }
#endif
}

// Take an audio sample and from it produce a signed
// output that uses the maximum number of bits
// in a 32 bit word (hopefully) without clipping.
// The algorithm is as follows:
//
// Calculate how many of bits of the input value are unused.
// Add this to a rolling average of unused bits of length
// AUDIO_AVERAGING_INTERVAL_MILLISECONDS, which starts off at 0.
// Every AUDIO_AVERAGING_INTERVAL_MILLISECONDS work out whether.
// the average number of unused is too large and, if it is,
// increase the gain, or if it is too small, decrease the gain.
static int processAudio(int monoSample)
{
    int unusedBits = 0;
    bool isNegative = ((monoSample & 0x80000000) == 0x80000000);
    int averageUnusedBits;

    // First, determine the number of unused bits
    // (avoiding testing the top bit since that is
    // never unused)
    for (int x = 30; x >= 0; x--) {
        if ((bool) (monoSample & (1 << x)) != isNegative) {
            break;
        } else {
            unusedBits++;
        }
    }
    //LOG(EVENT_MONO_SAMPLE_UNUSED_BITS, unusedBits);

    // Add the new unused bits count to the buffer and
    // update the total
    *pgAudioUnusedBitsNext = unusedBits;
    gAudioUnusedBitsTotal += unusedBits;
    pgAudioUnusedBitsNext++;
    if (pgAudioUnusedBitsNext >= gAudioUnusedBits + sizeof (gAudioUnusedBits) / sizeof(gAudioUnusedBits[0])) {
        pgAudioUnusedBitsNext = gAudioUnusedBits;
        // As we've wrapped, work out the average number of unused bits
        // and adjust the gain
        averageUnusedBits = gAudioUnusedBitsTotal / (sizeof (gAudioUnusedBits) / sizeof(gAudioUnusedBits[0]));
        LOG(EVENT_MONO_SAMPLE_AVERAGE_UNUSED_BITS, averageUnusedBits);

        if ((averageUnusedBits > AUDIO_DESIRED_UNUSED_BITS) && (gAudioShift < AUDIO_MAX_SHIFT_BITS)) {
            gAudioShift++;
            LOG(EVENT_MONO_SAMPLE_AUDIO_SHIFT, gAudioShift);
        } else if ((averageUnusedBits < AUDIO_DESIRED_UNUSED_BITS) && (gAudioShift > 0)) {
            gAudioShift--;
            LOG(EVENT_MONO_SAMPLE_AUDIO_SHIFT, gAudioShift);
        }
    }
    gAudioUnusedBitsTotal -= *pgAudioUnusedBitsNext;

#if GAIN_LEFT_SHIFT == 0
    monoSample <<= gAudioShift;
#else
    monoSample <<= GAIN_LEFT_SHIFT;
#endif

    return monoSample;
}

// Take a stereo sample in our usual form
// and return an int containing a sample
// that will fit within MONO_INPUT_SAMPLE_SIZE
// but sign extended so that it can be
// treated as an int for maths purposes
//
// An input data sample where only the LEFT
// channel is present looks like this in memory,
// dumping from pStereoSample onwards:
//
// FF FF 23 01 xx 45 FF FF
//
// ...which, assuming little-endian, would be printf()'ed as:
//
// 0x0123FFFF
// 0xFFFF45xx
//
// ...where FF FF FF FF is the unused right
// channel sample to be discarded, the byte
// ordering for the wanted left channel is
// MSB 01, middle byte 23, LSB 45 and xx is
// the byte to be discarded.
static int inline getMonoSample(uint32_t * pStereoSample)
{
    char * pByte = (char *) pStereoSample;
    unsigned int retValue;

    // LSB
    retValue =  (unsigned int) *(pByte + 5);
    // Middle byte
    retValue += ((unsigned int) *(pByte + 2)) << 8;
    // MSB
    retValue += ((unsigned int) *(pByte + 3)) << 16;
    // Sign extend
    if (retValue & 0x800000) {
        retValue |= 0xFF000000;
    }

    return (int) retValue;
}

// Fill a datagram with the audio from one block.
// pRawAudio must point to a buffer of
// SAMPLES_PER_BLOCK * 2 uint32_t's (i.e. stereo).
// Only the samples from the mono channel
// we are using are copied.
static void fillMonoDatagramFromBlock(uint32_t *pRawAudio)
{
    char * pDatagram = (char *) gpContainerNextEmpty->pContents;
    char * pBody = pDatagram + URTP_HEADER_SIZE;
    int monoSample;
    uint32_t timestamp = gTimeMilliseconds.read_ms();
    int numSamples = 0;

    // Check for overrun (nothing we can do, the oldest will just be overwritten)
    if (gpContainerNextEmpty->inUse) {
        LOG(EVENT_DATAGRAM_OVERFLOW, (int) gpContainerNextEmpty);
        event();
    } else {
        notEvent();
    }
    gpContainerNextEmpty->inUse = true;
    if (gNumDatagramsFree > 0) {
        gNumDatagramsFree--;
        if (gNumDatagramsFree < gMinNumDatagramsFree) {
            gMinNumDatagramsFree = gNumDatagramsFree;
        }
    }
    LOG(EVENT_DATAGRAM_ALLOC, (int) gpContainerNextEmpty);
    LOG(EVENT_NUM_DATAGRAMS_FREE, gNumDatagramsFree);

    // Copy in the body
    for (uint32_t *pStereoSample = pRawAudio; pStereoSample < pRawAudio + (SAMPLES_PER_BLOCK * 2); pStereoSample += 2) {
        //LOG(EVENT_RAW_AUDIO_DATA_0, *pStereoSample);
        //LOG(EVENT_RAW_AUDIO_DATA_1, *(pStereoSample + 1));
        monoSample = getMonoSample(pStereoSample);
#ifdef STREAM_FIXED_TONE
        monoSample = gPcm400HzSigned24Bit[gToneIndex];
        gToneIndex++;
        if (gToneIndex >= sizeof (gPcm400HzSigned24Bit) / sizeof (gPcm400HzSigned24Bit[0])) {
            gToneIndex = 0;
        }
#endif
        //LOG(EVENT_STREAM_MONO_SAMPLE_DATA, monoSample);
        numSamples++;
        monoSample = processAudio(monoSample);
        //LOG(EVENT_STREAM_MONO_SAMPLE_PROCESSED_DATA, monoSample);
        *pBody = (char) (monoSample >> 24);
        pBody++;
#if URTP_SAMPLE_SIZE > 1
        *pBody = (char) (monoSample >> 16);
        pBody++;
#endif
#if URTP_SAMPLE_SIZE > 2
        *pBody = (char) (monoSample >> 8);
        pBody++;
#endif
#if URTP_SAMPLE_SIZE > 3
        *pBody = (char) monoSample;
        pBody++;
#endif
    }

    // Fill in the header
    *pDatagram = PROTOCOL_VERSION;
    pDatagram++;
    *pDatagram = 0;
    pDatagram++;
    *pDatagram = (char) (gSequenceNumber >> 8);
    pDatagram++;
    *pDatagram = (char) gSequenceNumber;
    pDatagram++;
    gSequenceNumber++;
    *pDatagram = (char) (timestamp >> 24);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 16);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 8);
    pDatagram++;
    *pDatagram = (char) timestamp;

    LOG(EVENT_DATAGRAM_NUM_SAMPLES, numSamples);
    LOG(EVENT_DATAGRAM_SIZE, pBody - (char *) gpContainerNextEmpty->pContents);
    LOG(EVENT_DATAGRAM_READY_TO_SEND, (int) gpContainerNextEmpty);

    // Rotate to the next empty container
    gpContainerNextEmpty = gpContainerNextEmpty->pNext;

    // Signal that a datagram is ready to send
    gSendTask.signal_set(SIG_DATAGRAM_READY);
}

// Callback for I2S events.
// We get here when the DMA has either half-filled
// the gRawAudio buffer (so one 20 ms block) or
// completely filled it (two 20 ms blocks), or if
// an error has occurred.  We can use this as a sort
// of double buffer.
static void i2sEventCallback (int arg)
{
    if (arg & I2S_EVENT_RX_HALF_COMPLETE) {
        LOG(EVENT_I2S_DMA_RX_HALF_FULL, 0);
        fillMonoDatagramFromBlock(gRawAudio);
    } else if (arg & I2S_EVENT_RX_COMPLETE) {
        LOG(EVENT_I2S_DMA_RX_FULL, 0);
        fillMonoDatagramFromBlock(gRawAudio + (sizeof (gRawAudio) / sizeof (gRawAudio[0])) / 2);
    } else {
        LOG(EVENT_I2S_DMA_UNKNOWN, arg);
        bad();
        printf("Unexpected event mask 0x%08x.\n", arg);
    }

    checkGuards();
}

// Initialise the I2S interface and begin reading from it.
// The ICS43434 microphone outputs 24 bit words in a 64 bit frame,
// with the LR pin dictating whether the word appears in the first
// 32 bits (LR = 0, left channel, WS low) or the second 32 bits
// (LR = 1, right channel, WS high).  Each data bit is valid on the
// rising edge of SCK and the MSB of the data word is clocked out on the
// second clock edge after WS changes, as follows:
//      ___                                 ______________________   ___
// WS      \____________...________..._____/                      ...   \______
//          0   1   2       23  24      31  32  33  34     55  56     63
// SCK  ___   _   _   _       _   _      _   _   _   _       _   _      _   _
//         \_/ \_/ \_/ \...\_/ \_/ ...\_/ \_/ \_/ \_/ \...\_/ \_/ ...\_/ \_/ \_
//
// SD   ________--- ---     --- --- ___________--- ---     --- ---_____________
//              --- --- ... --- ---            --- --- ... --- ---
//              23  22       1   0             23  22       1   0
//              Left channel data              Right channel data
//
// This is known as the Philips protocol (24-bit frame with CPOL = 0 to read
// the data on the rising edge).
static bool startI2s(I2S * pI2s)
{
    bool success = false;

    if ((pI2s->protocol(PHILIPS) == 0) &&
        (pI2s->mode(MASTER_RX, true) == 0) &&
        (pI2s->format(24, 32, 0) == 0) &&
        (pI2s->audio_frequency(SAMPLING_FREQUENCY) == 0)) {
        if (gI2sTask.start(gI2STaskCallback) == osOK) {
            gTimeMilliseconds.reset();
            gTimeMilliseconds.start();
            if (pI2s->transfer((void *) NULL, 0,
                               (void *) gRawAudio, sizeof (gRawAudio),
                               event_callback_t(&i2sEventCallback),
                               I2S_EVENT_ALL) == 0) {
                success = true;
                LOG(EVENT_I2S_START, 0);
            }
        }
    }

    return success;
}

// Stop the I2S interface
static void stopI2s(I2S * pI2s)
{
    pI2s->abort_all_transfers();
    gI2sTask.terminate();
    gTimeMilliseconds.stop();
    LOG(EVENT_I2S_STOP, 0);
}

#ifdef SERVER_NAME
// Connect to the network, returning a pointer to a socket or NULL
static UDPSocket * startNetwork(INTERFACE_CLASS * pInterface)
{
    UDPSocket *pSock = NULL;

#ifndef USE_ETHERNET
    if (pInterface->init(PIN)) {
        pInterface->set_credentials(APN, USERNAME, PASSWORD);
#endif
        if (pInterface->connect() == 0) {
            if (gSock.open(pInterface) == 0) {
                gSock.set_timeout(10000);
                pSock = &gSock;
                LOG(EVENT_NETWORK_START, 0);
            }
        }
#ifndef USE_ETHERNET
    }
#endif

    return pSock;
}

// Verify that the server is there
static SocketAddress * verifyServer(INTERFACE_CLASS * pInterface)
{
    SocketAddress *pServer = NULL;

    if (pInterface != NULL) {
        if (pInterface->gethostbyname(SERVER_NAME, &gServer) == 0) {
            gServer.set_port(SERVER_PORT);
            pServer = &gServer;
        }
    }

    return pServer;
}
#endif

// Disconnect from the network
static void stopNetwork(INTERFACE_CLASS * pInterface)
{
    if (pInterface != NULL) {
        gSock.close();
        pInterface->disconnect();
#ifndef USE_ETHERNET
        pInterface->deinit();
#endif
        LOG(EVENT_NETWORK_STOP, 0);
    }
}

// The send function that forms the body of the send task
// This task runs whenever there is a datagram ready to send
static void sendData(const SendParams * pSendParams)
{
    Timer timer;
    int duration;
    int retValue;

    while (1) {
        // Wait for at least one datagram to be ready to send
        Thread::signal_wait(SIG_DATAGRAM_READY);

        while (gpContainerNextTx->inUse) {
            timer.reset();
            timer.start();
            // Send the datagram
            if ((pSendParams->pSock != NULL) && (pSendParams->pServer != NULL)) {
                LOG(EVENT_SEND_START, (int) gpContainerNextTx);
                retValue = pSendParams->pSock->sendto(*(pSendParams->pServer), gpContainerNextTx->pContents, URTP_DATAGRAM_SIZE);
                if (retValue != URTP_DATAGRAM_SIZE) {
                    LOG(EVENT_SEND_FAILURE, retValue);
                    bad();
                } else {
                    toggleGreen();
                }
                LOG(EVENT_SEND_STOP, (int) gpContainerNextTx);
            }

#ifdef LOCAL_FILE
            // Write the audio portion to file
            if (gpFile != NULL) {
                LOG(EVENT_FILE_WRITE_START, (int) gpContainerNextTx);
                MBED_ASSERT (gpFileBuf + URTP_BODY_SIZE <= gFileBuf + sizeof(gFileBuf));
                memcpy (gpFileBuf, (char *) gpContainerNextTx->pContents + URTP_HEADER_SIZE, URTP_BODY_SIZE);
                gpFileBuf += URTP_BODY_SIZE;
                if (gpFileBuf >= gFileBuf + sizeof(gFileBuf)) {
                    gpFileBuf = gFileBuf;
                    retValue = fwrite(gFileBuf, 1, sizeof(gFileBuf), gpFile);
                    if (retValue != sizeof(gFileBuf)) {
                        LOG(EVENT_FILE_WRITE_FAILURE, retValue);
                        bad();
                    }
                }
                LOG(EVENT_FILE_WRITE_STOP, (int) gpContainerNextTx);
            }
#endif

            timer.stop();
            duration = timer.read_us();
            gAverageTime += duration;
            gNumTimes++;

            if (duration > BLOCK_DURATION_MS * 1000) {
                LOG(EVENT_SEND_DURATION_GREATER_THAN_BLOCK_DURATION, duration);
            } else {
                LOG(EVENT_SEND_DURATION, duration);
            }
            if (duration > gMaxTime) {
                gMaxTime = duration;
                LOG(EVENT_NEW_PEAK_SEND_DURATION, gMaxTime);
            }

            gpContainerNextTx->inUse = false;
            gNumDatagramsFree++;
            LOG(EVENT_DATAGRAM_FREE, (int) gpContainerNextTx);
            LOG(EVENT_NUM_DATAGRAMS_FREE, gNumDatagramsFree);
            gpContainerNextTx = gpContainerNextTx->pNext;
        }
    }
}

// Print out the log
void printLog()
{
    LogEntry * pItem = gpLogNext;

    // Rotate to the start of the log
    for (unsigned int x = 0; x < (sizeof (gLog) / sizeof (gLog[0])) - gNumLogEntries; x++) {
        pItem++;
        if (pItem >= gLog + sizeof (gLog) / sizeof (gLog[0])) {
            pItem = gLog;
        }
    }

    printf ("------------- Log starts -------------\n");
    for (unsigned int x = 0; x < gNumLogEntries; x++) {
        printf ("%6.3f: %s %d (%#x)\n", (float) pItem->timestamp / 1000,
                gLogStrings[pItem->event], pItem->parameter, pItem->parameter);
        pItem++;
        if (pItem >= gLog + sizeof (gLog) / sizeof (gLog[0])) {
            pItem = gLog;
        }
    }
    printf ("-------------- Log ends --------------\n");
}

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

// Entry point
int main(void)
{
    INTERFACE_CLASS *pInterface = NULL;
    I2S *pMic = new I2S(PB_15, PB_10, PB_9);
    SendParams sendParams;
    InterruptIn userButton(SW0);

    memset (gLog, 0, sizeof (gLog));
    gLogTime.reset();
    gLogTime.start();
    LOG(EVENT_LOG_START, 0);

    initGuards();

    memset (gAudioUnusedBits, 0, sizeof(gAudioUnusedBits));

    sendParams.pServer = NULL;
    sendParams.pSock = NULL;

    // Attach a function to the user button
    userButton.rise(&buttonCallback);
    
    // Initialise the linked list
    initContainers(gContainer, sizeof (gContainer) / sizeof (gContainer[0]));

    good();

#ifdef LOCAL_FILE
    printf("Opening file %s...\n", LOCAL_FILE);
    gSd.init();
    gFs.mount(&gSd);
    remove (LOCAL_FILE);
    // Sometimes we fail to open the file unless there's a pause here
    // after any existing file is removed
    wait_ms(1000);
    gpFile = fopen(LOCAL_FILE, "wb+");
    if (gpFile != NULL) {
        LOG(EVENT_FILE_OPEN, 0);
#endif

#ifdef SERVER_NAME
# ifdef USE_ETHERNET
    printf("Connecting via Ethernet interface...\n");
    pInterface = new INTERFACE_CLASS();
# else
    printf("Starting up, please wait up to 180 seconds to connect to the packet network...\n");
    pInterface = new INTERFACE_CLASS(MDMTXD, MDMRXD, 460800);
# endif
        sendParams.pSock = startNetwork(pInterface);
        if (sendParams.pSock != NULL) {
            sendParams.pSock->set_timeout(10000);

            printf("Verifying that the server is there...\n");
            sendParams.pServer = verifyServer(pInterface);
            if (sendParams.pServer != NULL) {
#endif
                printf ("Starting task to send data...\n");
                if (gSendTask.start(callback(sendData, &sendParams)) == osOK) {

                    printf("Reading from I2S and sending data until the user button is pressed...\n");
                    if (startI2s(pMic)) {

#if STREAM_DURATION_MILLISECONDS > 0
                        printf("Streaming audio for %d milliseconds...\n", STREAM_DURATION_MILLISECONDS);
                        wait_ms(STREAM_DURATION_MILLISECONDS);
#else
                        printf("Streaming audio until the user button is pressed...\n");
                        while (!gButtonPressed);
#endif
                        printf("Stopping...\n");
                        stopI2s(pMic);
                        // Wait for any on-going transmissions to complete
                        wait_ms(2000);
                        // This commented out as it doesn't return reliably for me
                        //gSendTask.terminate();
                        //gSendTask.join();
                        stopNetwork(pInterface);
                        ledOff();
                        printf("Stopped.\n");
                    } else {
                        bad();
                        printf("Unable to start reading from I2S.\n");
                    }
                } else {
                    bad();
                    printf("Unable to start start sending task.\n");
                }
#ifdef SERVER_NAME
            } else {
                bad();
                printf("Unable to locate server.\n");
            }
        } else {
            bad();
            LOG(EVENT_NETWORK_START_FAILURE, 0);
            printf("Unable to connect to the network and open a socket.\n");
        }
#endif

#ifdef LOCAL_FILE
        printf("Closing file %s on SD card...\n", LOCAL_FILE);
        fclose(gpFile);
        gpFile = NULL;
        LOG(EVENT_FILE_CLOSE, 0);
        gFs.unmount();
        gSd.deinit();
        printf("File closed.\n");
    } else {
        bad();
        LOG(EVENT_FILE_OPEN_FAILURE, 0);
        printf("Unable to open file.\n");
    }
#endif

    LOG(EVENT_LOG_STOP, 0);
    printLog();

    if (gNumTimes > 0) {
        printf("Stats:\n");
        printf("Worst case time to perform a send: %d ms.\n", gMaxTime);
        printf("Average time to perform a send: %d ms.\n", (int) (gAverageTime / gNumTimes));
        printf("Minimum number of datagram(s) free %d.\n", gMinNumDatagramsFree);
    }
}
