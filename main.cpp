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
#include "log.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Define this to set a fixed duration for which to send audio
#define STREAM_DURATION_MILLISECONDS 30000

// Define this to use Ethernet instead of Cellular
//#define USE_ETHERNET

// Define this to use TCP instead of UDP
#define USE_TCP

// Define this to use UNICAM compression
#define USE_UNICAM

// Define this to send a fixed debug audio tone instead
// of that retrieved from the I2S interface
//#define STREAM_FIXED_TONE

// If SERVER_NAME is defined then the URTP
// stream will be sent to the named server
// on SERVER_PORT
#define SERVER_NAME "ciot.it-sgn.u-blox.com"
#define SERVER_PORT 5065

// Define this to fix the gain shift value
//#define GAIN_LEFT_SHIFT 12

#ifdef USE_TCP
#  define SOCKET TCPSocket
#else
#  define SOCKET UDPSocket
#endif

// The maximum amount of time allowed to send a datagram over TCP
#define TCP_SEND_TIMEOUT_MS 1500

// The overhead to add to requested TCP/UDP buffer sizes to allow
// for IP headers
#define IP_HEADER_OVERHEAD 40

// How long to wait between retries when establishing the link
#define RETRY_WAIT_SECONDS 5

// If we've had consecutive socket errors for this long, it's gone bad
#define MAX_DURATION_SOCKET_ERRORS_MS 1000

// The send data task will run anyway this interval,
// necessary in order to terminate it in an orderly fashion
#define SEND_DATA_RUN_ANYWAY_TIME_MS 1000

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

// The desired number of unused bits to keep in the audio processing to avoid
// clipping when we can't move fast enough due to averaging
#define AUDIO_DESIRED_UNUSED_BITS 4

// The maximum audio shift to use (established by experiment)
#define AUDIO_MAX_SHIFT_BITS 12

// I looked at using RTP to send data up to the server
// (https://en.wikipedia.org/wiki/Real-time_Transport_Protocol
// and https://tools.ietf.org/html/rfc3551, a minimal RTP header)
// but that needs a specialised client.  Since we will be talking
// to our own server we have the flexibility to do something
// much simpler, though still RTP-like. Call it URTP.  We send
// simple datagrams at 20 ms intervals. The header looks like this:
//
// Byte  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
//--------------------------------------------------------
//  0    |               Sync byte = 0x5A                |
//  1    |              Audio coding scheme              |
//  2    |              Sequence number MSB              |
//  3    |              Sequence number LSB              |
//  4    |                Timestamp MSB                  |
//  5    |                Timestamp byte                 |
//  6    |                Timestamp byte                 |
//  7    |                Timestamp byte                 |
//  8    |                Timestamp byte                 |
//  9    |                Timestamp byte                 |
//  10   |                Timestamp byte                 |
//  11   |                Timestamp LSB                  |
//  12   |       Number of samples in datagram MSB       |
//  13   |       Number of samples in datagram LSB       |
//
// ...where:
//
// - Sync byte is always 0x5A, used to sync a frame over a
//   streamed connection (e.g. TCP).
// - Audio coding scheme is one of:
//   - PCM_SIGNED_16_BIT_16000_HZ
//   - UNICAM_COMPRESSED_8_BIT_16000_HZ
//   - UNICAM_COMPRESSED_10_BIT_16000_HZ
// - Sequence number is a 16 bit sequence number, incremented
//   on sending of each datagram.
// - Timestamp is a uSecond timestamp representing the moment
//   of the start of the audio in this datagram.
// - Number of bytes to follow is the size of the audio payload
//   the follows in this datagram.
//
// When the audio coding scheme is PCM_SIGNED_16_BIT_16000_HZ,
// the payload is as follows:
//
// Byte  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
//--------------------------------------------------------
//  14   |                 Sample 0 MSB                  |
//  15   |                 Sample 0 LSB                  |
//  16   |                 Sample 1 MSB                  |
//  17   |                 Sample 1 LSB                  |
//       |                     ...                       |
//  N    |                 Sample M MSB                  |
//  N+1  |                 Sample M LSB                  |
//
// ...where the number of [big-endian] signed 16-bit samples is between
// 0 and 320, so 5120 bits, plus 112 bits of header, gives
// an overall data rate of 261.6 kbits/s.
//
// When the audio coding scheme is UNICAM_COMPRESSED_8_BIT_16000_HZ,
// the payload is as follows:
//
// Byte  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
//--------------------------------------------------------
//  14   |              Block 0, Sample 0                |
//  15   |              Block 0, Sample 1                |
//       |                   ...                         |
//  28   |              Block 0, Sample 14               |
//  29   |              Block 0, Sample 15               |
//  30   |     Block 0 shift     |     Block 1 shift     |
//  31   |              Block 1, Sample 0                |
//  32   |              Block 1, Sample 1                |
//       |                     ...                       |
//  45   |              Block 1, Sample 14               |
//  46   |              Block 1, Sample 15               |
//  47   |     Block 1 shift     |     Block 2 shift     |
//       |                     ...                       |
//  N    |     Block M-1 shift   |     Block M shift     |
//  N+1  |              Block M, Sample 0                |
//  N+2  |              Block M, Sample 1                |
//       |                     ...                       |
//  N+15 |              Block M, Sample 14               |
//  N+16 |              Block M, Sample 15               |
//
// ...where the number of blocks is between 0 and 20, so 330
// bytes in total, plus a 14 byte header gives an overall data
// rate of 132 kbits/s.
//
// If the audio coding scheme is UNICAM_COMPRESSED_10_BIT_16000_HZ,
// the payload follows the pattern above but with 10 bit
// samples packed as follows:
//
// Byte  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
//--------------------------------------------------------
//  14   |  Block 0, Sample 0                            |
//  15   |           |  Block 0, Sample 1                |
//  16   |                       |  Block 0, Sample 2    |
//  17   |                                   |  Block 0, |
//  18   | Sample 3                                      |
//  19   |  Block 0, Sample 4                            |
//  20   |           |  Block 0, Sample 5                |
//       |                     ...                       |
//  29   |  Block 0, Sample 12                           |
//  30   |           |  Block 0, Sample 13               |
//  31   |                       |  Block 0, Sample 14   |
//  32   |                                   |  Block 0, |
//  33   | Sample 15                                     |
//  34   |     Block 0 shift     |     Block 1 shift     |
//  35   |  Block 1, Sample 0                            |
//  36   |           |  Block 1, Sample 1                |
//  37   |                       |  Block 1, Sample 2    |
//  38   |                                   |  Block 1, |
//  39   | Sample 3                                      |
//       |                     ...                       |
//
// The number of blocks is still between 0 and 20 but now each
// block is 20 bytes plus half a byte of shift value, so 410
// bytes in total, plus a 14 byte header gives an overall data
// rate of 169.6 kbits/s.
//
// The receiving end should be able to reconstruct an audio
// stream from this.  For the sake of a name, call this URTP.

// UNICAM parameters
#define SAMPLES_PER_UNICAM_BLOCK      (SAMPLING_FREQUENCY / 1000)
#define UNICAM_CODED_SAMPLE_SIZE_BITS 8 // Must be 8 or 10
#define UNICAM_BLOCKS_PER_BLOCK       (SAMPLES_PER_BLOCK / SAMPLES_PER_UNICAM_BLOCK)
#define TWO_UNICAM_BLOCKS_SIZE        (((SAMPLES_PER_UNICAM_BLOCK * UNICAM_CODED_SAMPLE_SIZE_BITS) / 8) * 2 + 1)

// URTP parameters
#define SYNC_BYTE               0x5a
#define URTP_SEQ_NUM_OFFSET     2
#define URTP_HEADER_SIZE        14
#define URTP_SAMPLE_SIZE        2
#ifdef USE_UNICAM
# define URTP_BODY_SIZE         ((UNICAM_BLOCKS_PER_BLOCK / 2) * TWO_UNICAM_BLOCKS_SIZE)
#else
# define URTP_BODY_SIZE         (URTP_SAMPLE_SIZE * SAMPLES_PER_BLOCK)
#endif
#define URTP_DATAGRAM_SIZE      (URTP_HEADER_SIZE + URTP_BODY_SIZE)

// The maximum number of URTP datagrams that we can store
#define MAX_NUM_DATAGRAMS 200

// A signal to indicate that a datagram is ready to send
#define SIG_DATAGRAM_READY 0x01

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The audio coding scheme
typedef enum {
    PCM_SIGNED_16_BIT_16000_HZ = 0,
    UNICAM_COMPRESSED_8_BIT_16000_HZ = 1,
    UNICAM_COMPRESSED_10_BIT_16000_HZ = 2
} AudioCoding;

// A linked list container
typedef struct ContainerTag {
    bool inUse;
    void * pContents;
    ContainerTag *pNext;
} Container;

// A struct to pass data to the task that sends data to the network
typedef struct {
    SOCKET * pSock;
    SocketAddress * pServer;
} SendParams;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Thread required by I2S driver task
static Thread *gpI2sTask = NULL;

// Function that forms the body of the gI2sTask
static const Callback<void()> gI2STaskCallback(&I2S::i2s_bh_queue, &events::EventQueue::dispatch_forever);

// Audio buffer, enough for two blocks of stereo audio,
// where each sample takes up 64 bits (32 bits for L channel
// and 32 bits for R channel).
// Note: can't be in CCMRAM as DMA won't reach there
static uint32_t gRawAudio[(SAMPLES_PER_BLOCK * 2) * 2];

// Variables to monitor the headroom in audio sampling
static int gAudioShiftSampleCount = 0;
static int gAudioUnusedBitsMin = 0x7FFFFFFF;
static int gAudioShift = 0;

#ifdef USE_UNICAM
// Buffer for UNICAM coding
__attribute__ ((section ("CCMRAM")))
static int gUnicamBuffer[SAMPLES_PER_UNICAM_BLOCK];
#endif

// Task to send data off to the network server
static Thread *gpSendTask = NULL;

// Array of our "RTP-like" datagrams.
static char gDatagram[MAX_NUM_DATAGRAMS][URTP_DATAGRAM_SIZE];

// A linked list to manage the datagrams, must have the same
// number of elements as gDatagram
__attribute__ ((section ("CCMRAM")))
static Container gContainer[MAX_NUM_DATAGRAMS];

// A sequence number
static int gSequenceNumber = 0;

// A timer that generates the timestamp for datagrams
__attribute__ ((section ("CCMRAM")))
static Timer gTimeDatagram;

// Pointer to the next unused container
static Container *gpContainerNextEmpty = gContainer;

// Pointer to the next container to transmit
static volatile Container *gpContainerNextTx = gContainer;

// A count of the number of consecutive datagram
// overflows that have occurred (for diagnostics)
static int gNumDatagramOverflows = 0;

// A UDP or TCP socket
__attribute__ ((section ("CCMRAM")))
static SOCKET gSock;

// Flag to indicate that the network connection is good
static volatile bool gNetworkConnected = false;

// A server address
__attribute__ ((section ("CCMRAM")))
static SocketAddress gServer;

// LEDs
__attribute__ ((section ("CCMRAM")))
static DigitalOut gLedRed(LED1, 1);
__attribute__ ((section ("CCMRAM")))
static DigitalOut gLedGreen(LED2, 1);
__attribute__ ((section ("CCMRAM")))
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
  static char * gpFileBuf = gFileBuf;
#endif

// Record some timings so that we can see how we're doing
static int gMaxTime = 0;
static uint64_t gAverageTime = 0;
static uint64_t gNumTimes = 0;
static unsigned int gNumDatagramsFree = 0;
static unsigned int gMinNumDatagramsFree = 0;
static unsigned int gNumSendFailures = 0;
static unsigned int gNumSendTookTooLong = 0;
static unsigned int gNumSendSeqSkips = 0;
static unsigned int gBytesSent = 0;
static unsigned int gPossibleBadAudio = 0;
__attribute__ ((section ("CCMRAM")))
static Ticker gThroughputTicker;

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

    gNumDatagramsFree = 0;
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
// AUDIO_AVERAGING_INTERVAL_MILLISECONDS, which starts off at 0.
// Every AUDIO_AVERAGING_INTERVAL_MILLISECONDS work out whether.
// the average number of unused is too large and, if it is,
// increase the gain, or if it is too small, decrease the gain.
static int processAudio(int monoSample)
{
    int unusedBits = 0;
    int absSample = monoSample;

    //LOG(EVENT_STREAM_MONO_SAMPLE_DATA, monoSample);
    // First, determine the number of unused bits
    // (avoiding testing the top bit since that is
    // never unused)
    if (absSample < 0) {
        absSample = -absSample;
    }
    for (int x = 30; x >= 0; x--) {
        if (absSample & (1 << x)) {
            break;
        } else {
            unusedBits++;
        }
    }
    //LOG(EVENT_MONO_SAMPLE_UNUSED_BITS, unusedBits);

#ifndef GAIN_LEFT_SHIFT
    monoSample <<= gAudioShift;
#else
    monoSample <<= GAIN_LEFT_SHIFT;
#endif

    // Add the new unused bits count to the buffer and
    // update the minimum
    if (unusedBits < gAudioUnusedBitsMin) {
        gAudioUnusedBitsMin = unusedBits;
    }
    gAudioShiftSampleCount++;
    // If we've had a block's worth of data, work out how muhc gain we may be
    // able to apply for the next period
    if (gAudioShiftSampleCount >= SAMPLING_FREQUENCY / (1000 / BLOCK_DURATION_MS)) {
        gAudioShiftSampleCount = 0;
        //LOG(EVENT_MONO_SAMPLE_UNUSED_BITS_MIN, gAudioUnusedBitsMin);
        if (gAudioShift > gAudioUnusedBitsMin) {
            gAudioShift = gAudioUnusedBitsMin;
        }
        if ((gAudioUnusedBitsMin - gAudioShift > AUDIO_DESIRED_UNUSED_BITS) && (gAudioShift < AUDIO_MAX_SHIFT_BITS)) {
            gAudioShift++;
            LOG(EVENT_MONO_SAMPLE_AUDIO_SHIFT, gAudioShift);
        } else if ((gAudioUnusedBitsMin - gAudioShift < AUDIO_DESIRED_UNUSED_BITS) && (gAudioShift > 0)) {
            gAudioShift--;
            LOG(EVENT_MONO_SAMPLE_AUDIO_SHIFT, gAudioShift);
        }

        // Increment the minimum number of unused bits in the period
        // to let the number "relax"
        gAudioUnusedBitsMin++;
    }

    //LOG(EVENT_STREAM_MONO_SAMPLE_PROCESSED_DATA, monoSample);

    return monoSample;
}

// Take a stereo sample in our usual form
// and return an int containing a sample
// that will fit within MONO_INPUT_SAMPLE_SIZE
// but sign extended so that it can be
// treated as an int for maths purposes.
//
// - The numbers on the I2S interface are 32
//   bits wide, left channel then right channel.
// - We only need the left channel, pStereoSample
//   should point at the sample representing the
//   left channel.
// - Only 24 of the 32 bits are valid (see the Philips
//   format description lower down).
// - The I2S interface reads the data in 16 bit chunks.
// - This processor is little endian.
//
// What does that mean for getting the mono sample?  Well,
// dumping from pStereoSample in memory gives this:
//
// 23 01 xx 45 FF FF FF FF
//
// ...which, assuming little-endian, would be printf()'ed as:
//
// 0x45xx0123
// 0xFFFFFFFF
//
// ...where FF FF FF FF is the unused right
// channel sample to be discarded, the byte
// ordering for the wanted left channel is
// MSB 01, middle byte 23, LSB 45 and xx is
// the byte to be discarded.
//
// That said, I have a feeling the I2S interface can be
// inconsistent in how it reads, see below...
//
// ----------- OLD COMMENT STARTS -----------
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
// ----------- OLD COMMENT ENDS -----------
static int inline getMonoSample(const uint32_t * pStereoSample)
{
    const char * pByte = (const char *) pStereoSample;
    unsigned int retValue;

    // LSB
    //retValue =  (unsigned int) *(pByte + 5);
    retValue =  (unsigned int) *(pByte + 3);
    // Middle byte
    //retValue += ((unsigned int) *(pByte + 2)) << 8;
    retValue += ((unsigned int) *(pByte + 0)) << 8;
    // MSB
    //retValue += ((unsigned int) *(pByte + 3)) << 16;
    retValue += ((unsigned int) *(pByte + 1)) << 16;
    // Sign extend
    if (retValue & 0x800000) {
        retValue |= 0xFF000000;
    }

    // The xx bytes are actually always 0xFF
    // If they're not, maybe it's because
    // we've somehow slipped to the other pattern
    if (*(pByte + 2) != 0xFF) {
        LOG(EVENT_POSSIBLE_BAD_AUDIO, *(pByte + 2));
        gPossibleBadAudio++;
    }

#ifdef STREAM_FIXED_TONE
    retValue = gPcm400HzSigned24Bit[gToneIndex];
    gToneIndex++;
    if (gToneIndex >= sizeof (gPcm400HzSigned24Bit) / sizeof (gPcm400HzSigned24Bit[0])) {
        gToneIndex = 0;
    }
#endif

    return (int) retValue;
}

#ifdef USE_UNICAM
// Take a buffer of pRawAudio, point to
// SAMPLES_PER_BLOCK * 2 uint32_t's (i.e. stereo)
// and code the samples from the left channel
// (i.e. the even uint32_t's) into pDest.
//
// Here we use the principles of NICAM coding, see
// http://www.doc.ic.ac.uk/~nd/surprise_97/journal/vol2/aps2/
// We take 1 ms of audio data, so 16 samples (SAMPLES_PER_UNICAM_BLOCK),
// and work out the peak.  Then we shift all the samples in the
// block down so that they fit in just UNICAM_CODED_SAMPLE_SIZE_BITS.
// Then we put the shift value in the lower four bits of the next
// byte. In order to pack things neatly, the shift value for the
// following block is encoded into the upper four bits, followed by
// the shifted samples for that block, etc. This represents
// UNICAM_COMPRESSED_x_BIT_16000_HZ.
static int codeUnicam(const uint32_t *pRawAudio, char * pDest)
{
    int monoSample;
    int absSample;
    int maxSample = 0;
    int numBytes = 0;
    int numBlocks = 0;
    unsigned int i = 0;
    int usedBits;
    int shiftValue32Bit;
    int shiftValueCoded;
    bool isEvenBlock;
#if UNICAM_CODED_SAMPLE_SIZE_BITS != 8
    int compressedSampleBitShift = 0;
    int compressedSample;
#endif
    char *pDestOriginal = pDest;

    for (const uint32_t *pStereoSample = pRawAudio; pStereoSample < pRawAudio + (SAMPLES_PER_BLOCK * 2); pStereoSample += 2) {
        //LOG(EVENT_RAW_AUDIO_DATA_0, *pStereoSample);
        //LOG(EVENT_RAW_AUDIO_DATA_1, *(pStereoSample + 1));
        monoSample = getMonoSample(pStereoSample);
        monoSample = processAudio(monoSample);
        // Put the samples into the unicam buffer and track the max abs value
        absSample = monoSample;
        if (absSample < 0) {
            absSample = -absSample;
        }
        if (absSample > maxSample) {
            maxSample = absSample;
        }
        gUnicamBuffer[i] = monoSample;
        i++;
        if (i >= sizeof (gUnicamBuffer) / sizeof (gUnicamBuffer[0])) {
            i = 0;
            shiftValue32Bit = 0;
            usedBits = 32;

            //LOG(EVENT_UNICAM_MAX_ABS_VALUE, maxSample);
            // Once we have a buffer full, work out the shift value
            // to just fit the maximum value into 8 bits.  First
            // find the number of bits used (avoid testing the top
            // bit since that is always used)
            for (int x = 30; x >= 0; x--) {
                if ((maxSample & (1 << x)) != 0) {
                    break;
                } else {
                    usedBits--;
                }
            }
            //LOG(EVENT_UNICAM_MAX_VALUE_USED_BITS, usedBits);
            maxSample = 0;
            if (usedBits >= UNICAM_CODED_SAMPLE_SIZE_BITS) {
                shiftValue32Bit = usedBits - UNICAM_CODED_SAMPLE_SIZE_BITS;
            }
            //LOG(EVENT_UNICAM_SHIFT_VALUE, shiftValue32Bit);

            // If we're on an odd block number, write the shift value into the upper
            // nibble of the preceding byte of the output.
            //
            // shiftValue32Bit shifts a 32 bit number (which we know will be a
            // 16 bit number shifted towards the left) into an 8 bit number.
            // The shift value that we code into the output stream should be
            // the number required to return the 8 bit number into a 16 bit
            // number.
            if (shiftValue32Bit >= 16) {
                shiftValueCoded = shiftValue32Bit - 16;
            }

            //LOG(EVENT_UNICAM_CODED_SHIFT_VALUE, shiftValueCoded);
            isEvenBlock = false;
            if ((numBlocks & 1) == 0) {
                isEvenBlock = true;
            }
            if (!isEvenBlock) {
                *pDest = (*pDest & 0xF0) | shiftValueCoded;
                //LOG(EVENT_UNICAM_CODED_SHIFTS_BYTE, *pDest);
                pDest++;
            }

            // Write into the output all the values in the buffer shifted down by this amount
            for (unsigned int x = 0; x < sizeof (gUnicamBuffer) / sizeof (gUnicamBuffer[0]); x++) {
                //LOG(EVENT_UNICAM_SAMPLE, gUnicamBuffer[x]);
#if UNICAM_CODED_SAMPLE_SIZE_BITS != 8
                compressedSample = gUnicamBuffer[x];
                // With 10-bit unicam, first shift the sample into position
                compressedSample >>= shiftValue32Bit;
                //LOG(EVENT_UNICAM_COMPRESSED_SAMPLE, compressedSample);
                // compressedSample now contains the 10 bit sample as follows:
                // xxxxxxxx xxxxxxxx xxxxxx98 76543210

                // Write the first portion
                // Note: unsigned to avoid sign extension in this case
                *pDest |= (*pDest & ~((unsigned char) 0xFF >> compressedSampleBitShift)) |
                          ((unsigned int) compressedSample >> (compressedSampleBitShift + UNICAM_CODED_SAMPLE_SIZE_BITS - 8));
                //LOG(EVENT_UNICAM_10_BIT_CODED_SAMPLE, *pDest);
                pDest++;
                // Shift the sample down again for the remaining bits
                // Then write the remaining bits into the next byte
                *pDest |= (*pDest & ((unsigned char) 0xFF >> (compressedSampleBitShift + UNICAM_CODED_SAMPLE_SIZE_BITS - 8))) |
                           (compressedSample << (8 - (compressedSampleBitShift + UNICAM_CODED_SAMPLE_SIZE_BITS - 8)));
                // Move the shift value on
                compressedSampleBitShift += UNICAM_CODED_SAMPLE_SIZE_BITS - 8;
                if (compressedSampleBitShift >= 8) {
                    compressedSampleBitShift = 0;
                }
                // Move the destination pointer on if the shift has wrapped
                if (compressedSampleBitShift == 0) {
                    //LOG(EVENT_UNICAM_10_BIT_CODED_SAMPLE, *pDest);
                    pDest++;
                }
#else
                // With 8-bit unicam, it's nice and simple
                *pDest = gUnicamBuffer[x] >> shiftValue32Bit;
                //LOG(EVENT_UNICAM_COMPRESSED_SAMPLE, *pDest);
                pDest++;
#endif
            }

            // If we're on an even block number...
            if (isEvenBlock) {
                *pDest = (*pDest & 0x0F) | (shiftValueCoded << 4);
            }

            numBlocks++;
        }
    }

    numBytes = pDest - pDestOriginal;
    if (isEvenBlock) {
        numBytes++;
    }

    //LOG(EVENT_UNICAM_BLOCKS_CODED, numBlocks);
    //LOG(EVENT_UNICAM_BYTES_CODED, numBytes);

    return numBytes;
}

#else

// Take a buffer of pRawAudio, point to
// SAMPLES_PER_BLOCK * 2 uint32_t's (i.e. stereo)
// and copy the samples from the left channel
// (i.e. the even uint32_t's) into pDest.
// Each byte is passed through audio processing
// before it is coded.
// This represents PCM_SIGNED_16_BIT_16000_HZ.
static int codePcm(const uint32_t *pRawAudio, char * pDest)
{
    int monoSample;
    int numSamples = 0;

    for (const uint32_t *pStereoSample = pRawAudio; pStereoSample < pRawAudio + (SAMPLES_PER_BLOCK * 2); pStereoSample += 2) {
        //LOG(EVENT_RAW_AUDIO_DATA_0, *pStereoSample);
        //LOG(EVENT_RAW_AUDIO_DATA_1, *(pStereoSample + 1));
        monoSample = getMonoSample(pStereoSample);
        numSamples++;
        monoSample = processAudio(monoSample);

        *pDest = (char) (monoSample >> 24);
        pDest++;
#if URTP_SAMPLE_SIZE > 1
        *pDest = (char) (monoSample >> 16);
        pDest++;
#endif
#if URTP_SAMPLE_SIZE > 2
        *pDest = (char) (monoSample >> 8);
        pDest++;
#endif
#if URTP_SAMPLE_SIZE > 3
        *pDest = (char) monoSample;
        pDest++;
#endif
    }

    //LOG(EVENT_DATAGRAM_NUM_SAMPLES, numSamples);

    return numSamples * URTP_SAMPLE_SIZE;
}
#endif

// Fill a datagram with the audio from one block.
// pRawAudio must point to a buffer of
// SAMPLES_PER_BLOCK * 2 uint32_t's (i.e. stereo).
// Only the samples from the left channel
// (i.e. the even uint32_t's) are used.
static void fillMonoDatagramFromBlock(const uint32_t *pRawAudio)
{
    char * pDatagram = (char *) gpContainerNextEmpty->pContents;
    uint64_t timestamp = gTimeDatagram.read_high_resolution_us();
    int numBytesAudio = 0;

    // Check for overrun (nothing we can do, the oldest will just be overwritten)
    if (gpContainerNextEmpty->inUse) {
        if (gNumDatagramOverflows == 0) {
            LOG(EVENT_DATAGRAM_OVERFLOW_BEGINS, (int) gpContainerNextEmpty);
        }
        gNumDatagramOverflows++;
        event();
    } else {
        if (gNumDatagramOverflows > 0) {
            LOG(EVENT_DATAGRAM_NUM_OVERFLOWS, gNumDatagramOverflows);
            gNumDatagramOverflows = 0;
        }
        notEvent();
    }
    gpContainerNextEmpty->inUse = true;
    if (gNumDatagramsFree > 0) {
        gNumDatagramsFree--;
        if (gNumDatagramsFree < gMinNumDatagramsFree) {
            gMinNumDatagramsFree = gNumDatagramsFree;
        }
    }
    //LOG(EVENT_DATAGRAM_ALLOC, (int) gpContainerNextEmpty);
    //LOG(EVENT_NUM_DATAGRAMS_FREE, gNumDatagramsFree);

    // Copy in the body ASAP in case DMA catches up with us
#ifdef USE_UNICAM
    numBytesAudio = codeUnicam (pRawAudio,  pDatagram + URTP_HEADER_SIZE);
#else
    numBytesAudio = codePcm (pRawAudio,  pDatagram + URTP_HEADER_SIZE);
#endif
    // Fill in the header
    *pDatagram = SYNC_BYTE;
    pDatagram++;
#ifdef USE_UNICAM
# if UNICAM_CODED_SAMPLE_SIZE_BITS == 8
    *pDatagram = UNICAM_COMPRESSED_8_BIT_16000_HZ;
# else
    *pDatagram = UNICAM_COMPRESSED_10_BIT_16000_HZ;
# endif
#else
    *pDatagram = PCM_SIGNED_16_BIT_16000_HZ;
#endif
    pDatagram++;
    *pDatagram = (char) (gSequenceNumber >> 8);
    pDatagram++;
    *pDatagram = (char) gSequenceNumber;
    pDatagram++;
    gSequenceNumber++;
    *pDatagram = (char) (timestamp >> 56);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 48);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 40);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 32);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 24);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 16);
    pDatagram++;
    *pDatagram = (char) (timestamp >> 8);
    pDatagram++;
    *pDatagram = (char) timestamp;
    pDatagram++;
    *pDatagram = (char) (numBytesAudio >> 8);
    pDatagram++;
    *pDatagram = (char) numBytesAudio;
    pDatagram++;

    //LOG(EVENT_DATAGRAM_SIZE, pDatagram - (char *) gpContainerNextEmpty->pContents + numBytesAudio);
    //LOG(EVENT_DATAGRAM_READY_TO_SEND, (int) gpContainerNextEmpty);

    // Rotate to the next empty container
    gpContainerNextEmpty = gpContainerNextEmpty->pNext;

    // Signal that a datagram is ready to send
    if (gpSendTask != NULL) {
        gpSendTask->signal_set(SIG_DATAGRAM_READY);
    }
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
        //LOG(EVENT_I2S_DMA_RX_HALF_FULL, 0);
        fillMonoDatagramFromBlock(gRawAudio);
    } else if (arg & I2S_EVENT_RX_COMPLETE) {
        //LOG(EVENT_I2S_DMA_RX_FULL, 0);
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
        if (gpI2sTask == NULL) {
            gpI2sTask = new Thread();
        }
        if (gpI2sTask->start(gI2STaskCallback) == osOK) {
            gTimeDatagram.reset();
            gTimeDatagram.start();
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
    if (gpI2sTask != NULL) {
       gpI2sTask->terminate();
       gpI2sTask->join();
       delete gpI2sTask;
       gpI2sTask = NULL;
    }
    gTimeDatagram.stop();
    LOG(EVENT_I2S_STOP, 0);
}

#ifdef SERVER_NAME
// Connect to the network, returning a pointer to a socket or NULL
static SOCKET * startNetwork(INTERFACE_CLASS * pInterface)
{
    SOCKET *pSock = NULL;

#ifndef USE_ETHERNET
    if (pInterface->init(PIN)) {
        pInterface->set_credentials(APN, USERNAME, PASSWORD);
#endif
        if (pInterface->connect() == 0) {
            if (gSock.open(pInterface) == 0) {
                gSock.set_timeout(1000);
                pSock = &gSock;
                gNetworkConnected = true;
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
        gNetworkConnected = false;
        LOG(EVENT_NETWORK_STOP, 0);
    }
}

#ifdef USE_TCP
// Make the TCP connection and configure it
static bool connectTcp(const SendParams * pSendParams)
{
    bool success = false;
    const int setOption = 1;
    int retValue;

    retValue = pSendParams->pSock->connect(*(pSendParams->pServer));
    if (retValue == 0) {
        // Set TCP_NODELAY (1) in level IPPROTO_TCP (6) to 1
        LOG(EVENT_TCP_CONNECTED, 0);
        retValue = pSendParams->pSock->setsockopt(6, 1, &setOption, sizeof(setOption));
        if (retValue == 0) {
            success = true;
            LOG(EVENT_TCP_CONFIGURED, 0);
        } else {
            LOG(EVENT_TCP_CONFIGURATION_PROBLEM, retValue);
        }
    } else {
        LOG(EVENT_TCP_CONNECTION_PROBLEM, retValue);
    }

    return success;
}

// Send a buffer of data over a TCP socket
static int tcpSend(SOCKET * pSock, const char * pData, int size)
{
    int x = 0;
    int count = 0;
    Timer timer;

    timer.start();
    while ((count < size) && (timer.read_ms() < TCP_SEND_TIMEOUT_MS)) {
        x = pSock->send(pData + count, size - count);
        if (x > 0) {
            count += x;
        }
    }
    timer.stop();

    if (count < size) {
        LOG(EVENT_TCP_SEND_TIMEOUT, size - count);
    }

    if (x < 0) {
        count = x;
    }

    return count;
}
#endif

// The send function that forms the body of the send task
// This task runs whenever there is a datagram ready to send
static void sendData(const SendParams * pSendParams)
{
    Timer sendDurationTimer;
    Timer badSendDurationTimer;
    int duration;
    int retValue;
    int expectedSeq = 0;
    int thisSeq;
    bool okToDelete = false;

    while (gNetworkConnected) {
        // Wait for at least one datagram to be ready to send
        Thread::signal_wait(SIG_DATAGRAM_READY, SEND_DATA_RUN_ANYWAY_TIME_MS);

        while (gpContainerNextTx->inUse) {
            okToDelete = false;
            sendDurationTimer.reset();
            sendDurationTimer.start();
            // Send the datagram
            if ((pSendParams->pSock != NULL) && (pSendParams->pServer != NULL)) {
                //LOG(EVENT_SEND_START, (int) gpContainerNextTx);
                thisSeq = (((int) *((char *) gpContainerNextTx->pContents + URTP_SEQ_NUM_OFFSET)) << 8) +
                          *((char *) gpContainerNextTx->pContents + URTP_SEQ_NUM_OFFSET + 1);
                //LOG(EVENT_SEND_SEQ, thisSeq);
                if (expectedSeq != thisSeq) {
                    LOG(EVENT_SEND_SEQ_SKIP, expectedSeq);
                    event();
                    gNumSendSeqSkips++;
                } else {
                    notEvent();
                }
                expectedSeq = thisSeq + 1;
#ifdef USE_TCP
                retValue = tcpSend(pSendParams->pSock, (const char *) gpContainerNextTx->pContents, URTP_DATAGRAM_SIZE);
#else
                retValue = pSendParams->pSock->sendto(*(pSendParams->pServer), gpContainerNextTx->pContents, URTP_DATAGRAM_SIZE);
#endif
                if (retValue != URTP_DATAGRAM_SIZE) {
                    badSendDurationTimer.start();
                    LOG(EVENT_SEND_FAILURE, retValue);
                    bad();
                    gNumSendFailures++;
                } else {
                    gBytesSent += retValue;
                    okToDelete = true;
                    badSendDurationTimer.stop();
                    badSendDurationTimer.reset();
                    toggleGreen();
                }
                //LOG(EVENT_SEND_STOP, (int) gpContainerNextTx);

                if (retValue < 0) {
                    // If the connection has gone, set a flag that will be picked up outside this function and
                    // cause us to start again
                    if (badSendDurationTimer.read_ms() > MAX_DURATION_SOCKET_ERRORS_MS) {
                        LOG(EVENT_SOCKET_ERRORS_FOR_TOO_LONG, badSendDurationTimer.read_ms());
                        badSendDurationTimer.stop();
                        badSendDurationTimer.reset();
                        bad();
                        gNetworkConnected = false;
                    }
                    if ((retValue == NSAPI_ERROR_NO_CONNECTION) ||
                        (retValue == NSAPI_ERROR_CONNECTION_LOST) ||
                        (retValue == NSAPI_ERROR_NO_SOCKET)) {
                        LOG(EVENT_SOCKET_BAD, retValue);
                        bad();
                        gNetworkConnected = false;
                    }
                }
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
                    } else {
                        // If we aren't sending stuff over the socket then
                        // just writing it to disk means that it can be
                        // deleted
                        if ((pSendParams->pSock == NULL) {
                            okToDelete = true;
                        }
                    }
                }
                LOG(EVENT_FILE_WRITE_STOP, (int) gpContainerNextTx);
            }
#endif

            sendDurationTimer.stop();
            duration = sendDurationTimer.read_us();
            gAverageTime += duration;
            gNumTimes++;

            if (duration > BLOCK_DURATION_MS * 1000) {
#ifndef USE_TCP
                LOG(EVENT_SEND_DURATION_GREATER_THAN_BLOCK_DURATION, duration);
#endif
                LOG(EVENT_NUM_DATAGRAMS_QUEUED, (sizeof (gContainer) / sizeof (gContainer[0])) - gNumDatagramsFree);
                gNumSendTookTooLong++;
            } else {
                //LOG(EVENT_SEND_DURATION, duration);
            }
            if (duration > gMaxTime) {
                gMaxTime = duration;
                LOG(EVENT_NEW_PEAK_SEND_DURATION, gMaxTime);
            }

            if (okToDelete) {
                gpContainerNextTx->inUse = false;
                gNumDatagramsFree++;
                //LOG(EVENT_DATAGRAM_FREE, (int) gpContainerNextTx);
                //LOG(EVENT_NUM_DATAGRAMS_FREE, gNumDatagramsFree);
                gpContainerNextTx = gpContainerNextTx->pNext;
            }
        }
    }
}

// Monitor throughput numbers, for debug only
static void throughputMonitor()
{
    if (gBytesSent > 0) {
        LOG(EVENT_THROUGHPUT_BITS_S, gBytesSent << 3);
        gBytesSent = 0;
    }
}

#ifdef USE_UNICAM
// For the UNICAM compression scheme, we need
// the right shift operation to be arithmetic
// (so preserving the sign bit) rather than
// logical.  This function tests that this is
// the case
static bool testArithmeticRightShift()
{
    int negative = -1;
    return (negative >> 1) < 0;
}
#endif

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
    int retValue;

    printf("\n");

#ifdef USE_UNICAM
    if (testArithmeticRightShift()) {
#endif
        gThroughputTicker.attach_us(callback(&throughputMonitor), 1000000);
        initLog();
        LOG(EVENT_LOG_START, 0);

        sendParams.pServer = NULL;
        sendParams.pSock = NULL;

        // Attach a function to the user button
        userButton.rise(&buttonCallback);

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
#ifndef STREAM_DURATION_MILLISECONDS
            while (!gButtonPressed) {
#else
            {
#endif
                sendParams.pSock = startNetwork(pInterface);
                if (sendParams.pSock != NULL) {
                    good();

                    printf("Verifying that the server exists...\n");
#ifndef STREAM_DURATION_MILLISECONDS
                    while (gNetworkConnected && !gButtonPressed) {
#else
                    if (gNetworkConnected) {
#endif
                        sendParams.pServer = verifyServer(pInterface);
                        if (sendParams.pServer != NULL) {
                            good();
# ifdef USE_TCP
                            printf("Connecting TCP...\n");
#  ifndef STREAM_DURATION_MILLISECONDS
                            while (gNetworkConnected && !gButtonPressed) {
#  else
                            if (gNetworkConnected) {
#  endif
                                if (connectTcp(&sendParams)) {
                                    good();
                                    printf("Connected.\n");
# endif
#endif
                                    printf ("Setting up audio buffers...\n");
                                    initContainers(gContainer, sizeof (gContainer) / sizeof (gContainer[0]));
                                    printf ("Starting task to send data...\n");
                                    if (gpSendTask == NULL) {
                                        gpSendTask = new Thread();
                                    }
                                    retValue = gpSendTask->start(callback(sendData, &sendParams));
                                    if (retValue == osOK) {
                                        printf("Send data task started.\n");
                                        printf("Starting I2S...\n");
                                        if (startI2s(pMic)) {
                                            printf("I2S started.\n");
#ifndef STREAM_DURATION_MILLISECONDS
                                            printf("Streaming audio until the user button is pressed.\n");
                                            while (gNetworkConnected && !gButtonPressed) {};
#else
                                            printf("Streaming audio for %d milliseconds.\n", STREAM_DURATION_MILLISECONDS);
                                            wait_ms(STREAM_DURATION_MILLISECONDS);
#endif
                                            if (gButtonPressed) {
                                                printf("Stopping...\n");
                                                stopI2s(pMic);
                                                // Wait for any on-going transmissions to complete
                                                wait_ms(2000);
                                            } else {
                                                printf("Network connection lost, stopping...\n");
                                                stopI2s(pMic);
                                                // Tidy up: should really do this in
                                                // the "button pressed" case as well but,
                                                // for reasons I don't understand, the
                                                // send task won't return from termination
                                                // in that case.
                                                gpSendTask->terminate();
                                                gpSendTask->join();
                                                delete gpSendTask;
                                                gpSendTask = NULL;
                                            }

                                            // Tidy up
                                            stopNetwork(pInterface);

                                            if (gButtonPressed) {
                                                printf("Stopped.\n");
                                                ledOff();
                                            } else {
#ifndef STREAM_DURATION_MILLISECONDS
                                                printf("Trying again in %d second(s)...\n", RETRY_WAIT_SECONDS);
                                                wait_ms(RETRY_WAIT_SECONDS * 1000);
#endif
                                            }
                                        } else {
                                            bad();
                                            printf("Unable to start reading from I2S.\n");
                                        }
                                    } else {
                                        bad();
                                        printf("Unable to start sending task (error %d).\n", retValue);
                                    }
#ifdef SERVER_NAME
# ifdef USE_TCP
                                } else {
                                    bad();
                                    stopNetwork(pInterface);
                                    printf("Unable to make TCP connection to %s:%d, trying again in %d second(s)...\n",
                                           sendParams.pServer->get_ip_address(),
                                           sendParams.pServer->get_port(),
                                           RETRY_WAIT_SECONDS);
                                    wait_ms(RETRY_WAIT_SECONDS * 1000);
                                }
                            } // While (gNetworkConnected && !gButtonPressed)
# endif
                        } else {
                            bad();
                            printf("Unable to locate server, trying again in %d second(s)...\n", RETRY_WAIT_SECONDS);
                            wait_ms(RETRY_WAIT_SECONDS * 1000);
                        }
                    } // While (gNetworkConnected && !gButtonPressed)
                } else {
                    bad();
                    LOG(EVENT_NETWORK_START_FAILURE, 0);
                    printf("Unable to connect to the network and open a socket, trying again in %d second(s)...\n", RETRY_WAIT_SECONDS);
                    stopNetwork(pInterface);
                    wait_ms(RETRY_WAIT_SECONDS * 1000);
                }
            } // While (!gButtonPressed)
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
            printf("Worst case time to perform a send: %d us.\n", gMaxTime);
            printf("Average time to perform a send: %d us.\n", (int) (gAverageTime / gNumTimes));
            printf("Minimum number of datagram(s) free %d.\n", gMinNumDatagramsFree);
            printf("Number of send failure(s) %d,\n", gNumSendFailures);
            printf("%d send(s) took longer than %d ms (%d%% of the total),\n", gNumSendTookTooLong,
                   BLOCK_DURATION_MS, (int) (gNumSendTookTooLong * 100 / gNumTimes));
            printf("Number of send(s) where a sequence number was skipped %d,\n", gNumSendSeqSkips);
            printf("Possible bad audio has been seen %d time(s).\n", gPossibleBadAudio);
        }
#ifdef USE_UNICAM
    } else {
        bad();
        printf("This platform does not perform arithmetic right shifts, which are required for UNICAM, cannot run.\n");
    }
#endif
}
