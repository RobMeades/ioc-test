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
#include "I2S.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Cellular API
#define INTERFACE_CLASS  UbloxPPPCellularInterface

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

// The margin, in bits, to keep in the audio processing to avoid
// clipping when we can't move fast enough due to averaging
#define AUDIO_MARGIN_BITS 4

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
#define MAX_NUM_LOG_ENTRIES 1000

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
    EVENT_RAW_AUDIO_NEW_OFFSET,
    EVENT_STREAM_MONO_SAMPLE_DATA,
    EVENT_MONO_SAMPLE_UNUSED_BITS,
    EVENT_MONO_SAMPLE_AVERAGE_UNUSED_BITS,
    EVENT_MONO_SAMPLE_DESIRED_LEFT_SHIFT,
    EVENT_STREAM_MONO_SAMPLE_PROCESSED_DATA,
    EVENT_AUDIO_FORMAT_NOT_RECOGNSED,
    EVENT_INVALID_AUDIO_DATA,
    EVENT_SEND_START,
    EVENT_SEND_STOP,
    EVENT_SEND_FAILURE,
    EVENT_FILE_WRITE_START,
    EVENT_FILE_WRITE_STOP,
    EVENT_FILE_WRITE_FAILURE,
    EVENT_SEND_DURATION_GREATER_THAN_BLOCK_DURATION,
    EVENT_NEW_PEAK_SEND_DURATION,
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

// The last position that audio appeared in the rotation buffer
static int gLastRotationPosition = 0;

// A buffer to monitor average headroom in audio sampling
__attribute__ ((section ("CCMRAM")))
static char gAudioUnusedBits[SAMPLING_FREQUENCY / (1000 / AUDIO_AVERAGING_INTERVAL_MILLISECONDS)];
static char *pgAudioUnusedBitsNext = gAudioUnusedBits;
static uint32_t gAudioUnusedBitsTotal = 0;

// Task to send data off to the network server
static Thread gSendTask;

// Array of our "RTP-like" datagrams.
static char gDatagram[MAX_NUM_DATAGRAMS][URTP_DATAGRAM_SIZE];

// A linked list to manage the datagrams, must have the same
// number of elements as gDatagram
__attribute__ ((section ("CCMRAM")))
static Container gContainer[MAX_NUM_DATAGRAMS];

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
    "  RAW_AUDIO_NEW_OFFSET",
    "  STREAM_MONO_SAMPLE_DATA",
    "  MONO_SAMPLE_UNUSED_BITS",
    "  MONO_SAMPLE_AVERAGE_UNUSED_BITS",
    "  MONO_SAMPLE_DESIRED_LEFT_SHIFT",
    "  STREAM_MONO_SAMPLE_PROCESSED_DATA",
    "* AUDIO_FORMAT_NOT_RECOGNSED",
    "* INVALID_AUDIO_DATA",
    "  SEND_START",
    "  SEND_STOP",
    "* SEND_FAILURE",
    "  FILE_WRITE_START",
    "  FILE_WRITE_STOP",
    "* FILE_WRITE_FAILURE",
    "* SEND_DURATION_GREATER_THAN_BLOCK_DURATION",
    "  NEW_PEAK_SEND_DURATION",
    "  NUM_DATAGRAMS_FREE"
};

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

// Indicate not bad (not red)
static void notBad() {
    gLedRed = 1;
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

// Take an audio sample and from it produce a signed
// output that uses the maximum number of bits
// in a 32 bit word (hopefully) without clipping.
// The algorithm is as follows:
//
// Calculate how many of bits of the input value are unused.
// Add this to a rolling average of unused bits of length
// AUDIO_AVERAGING_INTERVAL_SAMPLES, which starts off at 0.
// Calculate the new average number of unused bits.
// The output sample is then shifted up by the average number
// of unused bits minus a margin of AUDIO_MARGIN_BITS.
static int processAudio(int monoSample)
{
    int unusedBits = 0;
    bool isNegative = ((monoSample & 0x80000000) == 0x80000000);
    int desiredLeftShift;

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
    if (pgAudioUnusedBitsNext > gAudioUnusedBits + sizeof (gAudioUnusedBits) / sizeof(gAudioUnusedBits[0])) {
        pgAudioUnusedBitsNext = gAudioUnusedBits;
    }
    gAudioUnusedBitsTotal -= *pgAudioUnusedBitsNext;

    // Work out the average number of unused bits
    desiredLeftShift = gAudioUnusedBitsTotal / (sizeof (gAudioUnusedBits) / sizeof(gAudioUnusedBits[0]));
    //LOG(EVENT_MONO_SAMPLE_AVERAGE_UNUSED_BITS, desiredLeftShift);

    // Now shift the input up by the average number of unused bits
    // minus a headroom of AUDIO_MARGIN_BITS
    if (desiredLeftShift >= AUDIO_MARGIN_BITS) {
        desiredLeftShift -= AUDIO_MARGIN_BITS;
    }
    //LOG(EVENT_MONO_SAMPLE_DESIRED_LEFT_SHIFT, desiredLeftShift);

    return monoSample << desiredLeftShift;
}

// Rotate a stereo sample into our usual form
// and return an int containing a sample
// that will fit within MONO_INPUT_SAMPLE_SIZE,
// bytes or 0xFFFFFFFF if the format makes no sense.
//
// By experiment, the position of the data
// in a stereo sample can shift, possibly
// due to glitches on the I2S interface.
// For instance, a read of a stereo sample where
// only the LEFT channel is present might
// end up in memory looking like this:
//
// FF FF FF FF 23 45 xx 01
//
// or like this:
//
// FF FF 23 45 xx 01 FF FF
//
// ...where FF FF FF FF is the unused right
// channel sample to be discarded, the byte,
// ordering for the wanted left channel is
// MSB 01, middle byte 23, LSB 45 and xx is
// the byte to be discarded.
// Order within a 16 bit read is always
// maintained, it would seem.
static int inline rotateRawAudio(uint32_t * pStereoSample)
{
    char * pByte = (char *) pStereoSample;
    int found = 0;
    int x = 0;
    int retValue = 0xFFFFFFFF;

    // Find the position of FF FF FF FF,
    // taking account of the fact that it
    // might go "around the corner" (hence the +2)
    for (x = 0; (x < (8 + 2)) && (found < 4); x++) {
        if (*pByte == 0xFF) {
            found++;
        } else {
            found = 0;
        }
        pByte++;
        if (pByte >= ((char *) pStereoSample) + 8) {
            pByte = (char *) pStereoSample;
        }
    }

    // Having found it, dump the bytes in the
    // output in the correct order
    if (found == 4) {
        // This for diagnostics
        if (x != gLastRotationPosition) {
            LOG(EVENT_RAW_AUDIO_NEW_OFFSET, x);
            gLastRotationPosition = x;
        }
        // pByte is now pointing at the LSB,
        // modulo 8 positions in memory
        retValue = 0;
        for (x = 0; x < 4; x++) {
            switch (x) {
                case 0: // Middle byte
                    retValue += ((int) *pByte) << 8;
                    break;
                case 1: // MSB
                    retValue += ((int) *pByte) << 16;
                    break;
                case 2: // xx byte
                    // If the top bit of the MSB is set,
                    // fill this with 0xFF for sign
                    // extension
                    if (retValue & 0x800000) {
                        retValue |= 0xFF000000;
                    }
                    break;
                case 3: // LSB
                    retValue += *pByte;
                    break;
                default:
                    break;
            }
            pByte++;
            if (pByte >= ((char *) pStereoSample) + 8) {
                pByte = (char *) pStereoSample;
            }
        }
    }

    return retValue;
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
    int possiblyInvalidDataCount = 0;
    int numSamples = 0;
#if 0
    uint16_t debug = 0;
#endif

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
        monoSample = rotateRawAudio(pStereoSample);
        //LOG(EVENT_STREAM_MONO_SAMPLE_DATA, monoSample);
        if (monoSample != (int) 0xFFFFFFFF) {
            // TODO: should we throw the whole block away if
            // any one stereo sample is not retrievable?
            // Process the audio for clipping
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
        } else {
            possiblyInvalidDataCount++;
            LOG(EVENT_AUDIO_FORMAT_NOT_RECOGNSED, 0);
#if 0
            *pBody = (char) (debug >> 8);
            pBody++;
            *pBody = (char) debug;
            pBody++;
            debug++;
#endif
        }
    }

    if (possiblyInvalidDataCount < SAMPLES_PER_BLOCK) {
        notBad();
        toggleGreen();
    } else {
        bad();
        LOG(EVENT_INVALID_AUDIO_DATA, 0);
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

    if (pInterface->init(PIN)) {
        pInterface->set_credentials(APN, USERNAME, PASSWORD);
        if (pInterface->connect() == 0) {
            if (gSock.open(pInterface) == 0) {
                gSock.set_timeout(10000);
                pSock = &gSock;
                LOG(EVENT_NETWORK_START, 0);
            }
        }
    }

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
        pInterface->deinit();
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
            duration = timer.read_ms();
            gAverageTime += duration;
            gNumTimes++;

            if (duration > BLOCK_DURATION_MS) {
                LOG(EVENT_SEND_DURATION_GREATER_THAN_BLOCK_DURATION, duration);
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
        printf("Starting up, please wait up to 180 seconds to connect to the packet network...\n");
        pInterface = new INTERFACE_CLASS(MDMTXD, MDMRXD, 460800);
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

                        //while (!gButtonPressed);
                        printf("Actually, just sending for a fixed duration (20 seconds).\n");
                        wait_ms(20000);
                        printf("User button pressed, stopping...\n");
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
