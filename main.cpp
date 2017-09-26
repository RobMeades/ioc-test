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
#include "urtp.h"
#include "log.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Define this to set a fixed duration for which to send audio
//#define STREAM_DURATION_MILLISECONDS 5000

// Define this to use Ethernet instead of Cellular
//#define USE_ETHERNET

// Define this to use TCP instead of UDP
#define USE_TCP

// If SERVER_NAME is defined then the URTP
// stream will be sent to the named server
// on SERVER_PORT
#define SERVER_NAME "ciot.it-sgn.u-blox.com"
#define SERVER_PORT 5065

#ifdef USE_TCP
#  define SOCKET TCPSocket
#else
#  define SOCKET UDPSocket
#endif

// The maximum amount of time allowed to send a datagram over TCP
#define TCP_SEND_TIMEOUT_MS 1500

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

// A signal to indicate that a datagram is ready to send
#define SIG_DATAGRAM_READY 0x01

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

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

// Datagram storage for URTP
static char datagramStorage[URTP_DATAGRAM_STORE_SIZE];

// Task to send data off to the network server
static Thread *gpSendTask = NULL;

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
static unsigned int gNumSendFailures = 0;
static unsigned int gNumSendTookTooLong = 0;
static unsigned int gBytesSent = 0;
__attribute__ ((section ("CCMRAM")))
static Ticker gSecondTicker;

/* ----------------------------------------------------------------
 * STATIC DEBUG FUNCTIONS
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

/* ----------------------------------------------------------------
 * URTP CODEC AND ITS CALLBACK FUNCTIONS
 * -------------------------------------------------------------- */

// Callback for when a datagram is ready for sending
static void datagramReadyCb(const char * datagram)
{
    if (gpSendTask != NULL) {
        // Send the signal to the sending task
        gpSendTask->signal_set(SIG_DATAGRAM_READY);
    }
}

// Callback for when the datagram list starts to overflow
static void datagramOverflowStartCb()
{
    event();
}

// Callback for when the datagram list stops overflowing
static void datagramOverflowStopCb(int numOverflows)
{
    notEvent();
}

// The URTP codec
__attribute__ ((section ("CCMRAM")))
static Urtp urtp(&datagramReadyCb, &datagramOverflowStartCb, &datagramOverflowStopCb);

/* ----------------------------------------------------------------
 * ALL OTHER STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Something to attach to the button
static void buttonCallback()
{
    gButtonPressed = true;
    LOG(EVENT_BUTTON_PRESSED, 0);
    event();
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
        urtp.codeAudioBlock(gRawAudio);
    } else if (arg & I2S_EVENT_RX_COMPLETE) {
        //LOG(EVENT_I2S_DMA_RX_FULL, 0);
        urtp.codeAudioBlock(gRawAudio + (sizeof (gRawAudio) / sizeof (gRawAudio[0])) / 2);
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
        // TODO This commented out as sometimes it never returns, needs more investigation
        //gSock.close();
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
    const char * urtpDatagram = NULL;
    Timer sendDurationTimer;
    Timer badSendDurationTimer;
    int duration;
    int retValue;
    bool okToDelete = false;

    while (gNetworkConnected) {
        // Wait for at least one datagram to be ready to send
        Thread::signal_wait(SIG_DATAGRAM_READY, SEND_DATA_RUN_ANYWAY_TIME_MS);

        while ((urtpDatagram = urtp.getUrtpDatagram()) != NULL) {
            okToDelete = false;
            sendDurationTimer.reset();
            sendDurationTimer.start();
            // Send the datagram
            if ((pSendParams->pSock != NULL) && (pSendParams->pServer != NULL)) {
                //LOG(EVENT_SEND_START, (int) urtpDatagram);
#ifdef USE_TCP
                retValue = tcpSend(pSendParams->pSock, urtpDatagram, URTP_DATAGRAM_SIZE);
#else
                retValue = pSendParams->pSock->sendto(*(pSendParams->pServer), urtpDatagram, URTP_DATAGRAM_SIZE);
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
                //LOG(EVENT_SEND_STOP, (int) urtpDatagram);

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
                LOG(EVENT_FILE_WRITE_START, (int) urtpDatagram);
                MBED_ASSERT (gpFileBuf + URTP_BODY_SIZE <= gFileBuf + sizeof(gFileBuf));
                memcpy (gpFileBuf, (char *) urtpDatagram + URTP_HEADER_SIZE, URTP_BODY_SIZE);
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
                LOG(EVENT_FILE_WRITE_STOP, (int) urtpDatagram);
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
                gNumSendTookTooLong++;
            } else {
                //LOG(EVENT_SEND_DURATION, duration);
            }
            if (duration > gMaxTime) {
                gMaxTime = duration;
                LOG(EVENT_NEW_PEAK_SEND_DURATION, gMaxTime);
            }

            if (okToDelete) {
                urtp.setUrtpDatagramAsRead(urtpDatagram);
            }
        }
    }
}

// Monitoring operation on a 1 second tick
static void monitor()
{
    // Monitor throughput
    if (gBytesSent > 0) {
        LOG(EVENT_THROUGHPUT_BITS_S, gBytesSent << 3);
        gBytesSent = 0;
        LOG(EVENT_NUM_DATAGRAMS_QUEUED, urtp.getUrtpDatagramsAvailable());
    }
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
    int retValue;

    printf("\n");

    gSecondTicker.attach_us(callback(&monitor), 1000000);
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
    pInterface = new INTERFACE_CLASS(MDMTXD, MDMRXD, 230400);
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
                                printf ("Setting up audio codec...\n");
                                if (urtp.init((void *) &datagramStorage)) {
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
                                            }

                                            // Tidy up
                                            gpSendTask->terminate();
                                            gpSendTask->join();
                                            delete gpSendTask;
                                            gpSendTask = NULL;
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
                                } else {
                                    bad();
                                    printf("Unable to initialise audio codec.\n");
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
        printf("Minimum number of datagram(s) free %d.\n", urtp.getUrtpDatagramsFreeMin());
        printf("Number of send failure(s) %d,\n", gNumSendFailures);
        printf("%d send(s) took longer than %d ms (%d%% of the total).\n", gNumSendTookTooLong,
               BLOCK_DURATION_MS, (int) (gNumSendTookTooLong * 100 / gNumTimes));
    }
}
