#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>


#define MSP_PORT_INBUF_SIZE 64
#define MSP_PORT_OUTBUF_SIZE 256
#define MAX_MSP_PORT_COUNT 2
#define SELECT_TIMEOUT 0
#define SELECT_TIMEOUT_US 75000
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define FC_VERSION_MAJOR 1
#define FC_VERSION_MINOR 14
#define FC_VERSION_PATCH_LEVEL 0
#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4 
/*#define MSP_PROTOCOL_VERSION 0
#define API_VERSION_MAJOR 1 // increment when major changes are made
#define API_VERSION_MINOR 22 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR*/


#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])
#define UNUSED(x) (void)(x)



static const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.


typedef enum portMode_t {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_t;

typedef enum portOptions_t {
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3
} portOptions_t;


typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app


typedef struct serialPort_s {

    const struct serialPortVTable *vTable;      //Struct to hold all the serialport function pointers

    uint8_t identifier;
    portMode_t mode;
    portOptions_t options;

    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    // FIXME rename member to rxCallback
    serialReceiveCallbackPtr callback;              //function typedef for serialcallback as defined in line 37
} serialPort_t;

typedef struct {
    serialPort_t port;
    int fd;
    int deviceState;
    bool buffering;
} uartPort_t;


struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);

    uint8_t (*serialTotalRxWaiting)(serialPort_t *instance);
    uint8_t (*serialTotalTxFree)(serialPort_t *instance);

    uint8_t (*serialRead)(serialPort_t *instance);

    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);

    bool (*isSerialTransmitBufferEmpty)(serialPort_t *instance);

    void (*setMode)(serialPort_t *instance, portMode_t mode);

    void (*writeBuf)(serialPort_t *instance, void *data, int count);
    // Optional functions used to buffer large writes.
    void (*beginWrite)(serialPort_t *instance);
    void (*endWrite)(serialPort_t *instance);
};

typedef enum {
    MSP_MODE_SERVER,
    MSP_MODE_CLIENT
} mspPortMode_e;


typedef bool (*mspCommandSenderFuncPtr)(); // msp command sender function prototype

typedef enum {
    IDLE,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
    HEADER_DATA,
    MESSAGE_RECEIVED
} mspState_e;



typedef struct mspPort_s {
    serialPort_t *port;                      // NULL when unused.
    mspPortMode_e mode;

    mspCommandSenderFuncPtr commandSenderFn;   // NULL when unused.

    mspState_e c_state;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t cmdMSP;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    //uint8_t tempBuf[MSP_PORT_INBUF_SIZE];
} mspPort_t;


typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuff_t* is equivalent to uint8_t **)
    uint8_t *end;
} sbuf_t;

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    int16_t result;
} mspPacket_t;


static void usbVcpWrite(serialPort_t *instance, uint8_t c);
uint8_t usbTxBytesFree(serialPort_t *instance);
static uint8_t usbVcpRead(serialPort_t *instance);
static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate);
static void usbVcpSetMode(serialPort_t *instance, portMode_t mode);
serialPort_t* usbVcpOpen(void);
uint8_t usbTxBytesFree(serialPort_t *instance);
uint8_t serial_waiting(serialPort_t *instance);
bool usb_txbuffer_empty(serialPort_t *instance);


void serialBeginWrite(serialPort_t *instance);
void serialWriteBuf(serialPort_t *instance, uint8_t *data, int count);
void serialWrite(serialPort_t *instance, uint8_t ch);
void serialEndWrite(serialPort_t *instance);
uint8_t serialRxBytesWaiting(serialPort_t *instance);
uint8_t serialRead(serialPort_t *instance);


typedef struct {
    uint32_t bitrate;
    uint8_t format;
    uint8_t paritytype;
    uint8_t datatype;
} LINE_CODING;


typedef enum _DEVICE_STATE {
    UNCONNECTED = 0,
    ATTACHED,
    POWERED,
    SUSPENDED,
    ADDRESSED,
    CONFIGURED
} DEVICE_STATE;

serialPort_t* usartInitAllIOSignals(void);
void usbInit(void);

void mspSerialProcess(void);
void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort);


mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteData(sbuf_t *dst, const void *data, int len);