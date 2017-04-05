#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include "lib.h"

#define edison_port "/dev/ttyMFD2";



bool data_read = true;
bool data_available = false;
int read_pos;
int temp_data_len;
char temp_buff[100];


char portname[20] = edison_port;


uartPort_t USB;

LINE_CODING linecoding = 
{ 
    115200, /* baud rate*/
    0x00, /* stop bits-1*/
    0x00, /* parity - none*/
    0x08 /* no. of bits 8*/
};


void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}


uint8_t serialRxBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalRxWaiting(instance);
}


uint8_t serialTxBytesFree(serialPort_t *instance)
{
    return instance->vTable->serialTotalTxFree(instance);
}


uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}


void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}


bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}


void serialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->vTable->setMode(instance, mode);
}


void serialBeginWrite(serialPort_t *instance)
{
    if (instance->vTable->beginWrite)
        instance->vTable->beginWrite(instance);
}


void serialEndWrite(serialPort_t *instance)
{
    if (instance->vTable->endWrite)
        instance->vTable->endWrite(instance);
}


void serialWriteBuf(serialPort_t *instance, uint8_t *data, int count)
{
    uint8_t *p;
    if (instance->vTable->writeBuf) {
        instance->vTable->writeBuf(instance, data, count);
    } else {
        for (p = data; count > 0; count--, p++) {

            while (!serialTxBytesFree(instance)) {
            };
            serialWrite(instance, *p);
        }
    }
}


static const struct serialPortVTable usbTable[] = {
    {
        .serialWrite = usbVcpWrite,                                     //used
        .serialTotalRxWaiting = serial_waiting,                         //used
        .serialTotalTxFree = usbTxBytesFree,                            //not used
        .serialRead = usbVcpRead,                                       //used
        .serialSetBaudRate = usbVcpSetBaudRate,                         //used only in gps
        .isSerialTransmitBufferEmpty = usb_txbuffer_empty,              //used
        .setMode = usbVcpSetMode,                                       //used, TBD
        .beginWrite = NULL,                                             //not needed
        .endWrite = NULL,                                               //not needed
        .writeBuf = NULL                                                //not needed
    }
};



uint8_t usbIsConnected(void)
{
    if(USB.deviceState != UNCONNECTED)
        return true;
    else
        return false;
}




uint32_t usbWrite(uint8_t* str, int len)
{
    //Don't write if USB is not connected
    if(usbIsConnected() == false)
    {
        printf("USB not connected\t%d\n",USB.deviceState);
        return -1;
    }
    int wlen;
    wlen = write(USB.fd, str, len);
    return wlen;
}



uint8_t serial_waiting(serialPort_t *instance)
{
    
    if(!data_read)
    {
        return 1;
    }    

    UNUSED(instance);
    fd_set readset;                             //for the select function
    FD_ZERO(&readset);
    FD_SET(USB.fd, &readset);
    uint32_t result;
        
    struct timeval tv = {SELECT_TIMEOUT, SELECT_TIMEOUT_US};   // sleep for ten minutes!

    result = select(USB.fd + 1, &readset, NULL, NULL, &tv);

    if(result > 0)
    {
        temp_data_len = read(USB.fd, temp_buff, sizeof(temp_buff));
        data_available = true;
        data_read = false;
        read_pos = 0;
    }

    //printf("result:%d\n",result);
    
    return result;
}

int32_t usbRead(uint8_t* buf, int len)
{
    if(!data_available)
    {
        //TBD
    }
    else
    {
        read_pos++;
        if(read_pos <= temp_data_len)
        {
            *buf = temp_buff[read_pos-1];
            return 1;
        }

        else
        {
            data_read = true;
            data_available = false;
            return -1;
        }
    }
}


uint8_t usbTxBytesFree(serialPort_t *instance)
{
    // Because we block upon transmit and don't buffer bytes, our "buffer" capacity is effectively unlimited.
    UNUSED(instance);
    return 255;
}


bool usb_txbuffer_empty(serialPort_t *instance)
{
    return true;
}

serialPort_t* usartInitAllIOSignals(void)        //usartIrqHandler() not setup in the original version of cleanflight in this function
{
    USB.deviceState = UNCONNECTED;
    USB.port.vTable = usbTable;
    usbInit();
    return &USB.port;
}


void SetUsbAttributes(int fd)       //UART characteristics hardcoded here!!!
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        //printf("Error from tcgetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, (speed_t)(linecoding.bitrate));
    cfsetispeed(&tty, (speed_t)(linecoding.bitrate));


    //These values are hardcoded for now
    //Todo is to change this based on the value defined in the struct LINE_CODING
    tty.c_cflag |= (CLOCAL | CREAD);    //ignore modem controls
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         //8-bit characters
    tty.c_cflag &= ~PARENB;     //no parity bit
    tty.c_cflag &= ~CSTOPB;     //only need 1 stop bit
    tty.c_cflag &= ~CRTSCTS;    //no hardware flowcontrol

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        exit(EXIT_FAILURE);
    }
    return;
 
}


int usbOpen(void)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0) {
        exit(EXIT_FAILURE);
    }
    USB.fd = fd;
    USB.deviceState = CONFIGURED;
    return fd;
}


void usbInit(void)
{
    int fd = usbOpen();
    tcflush(fd,TCIOFLUSH);
    SetUsbAttributes(fd);    
}


static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    usbWrite(&c,1);
}


static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t buf;

    int len = usbRead(&buf, 1);

    return buf;
}


static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
    return;
    //does nothing for now
}


static void usbVcpSetMode(serialPort_t *instance, portMode_t mode)
{
    UNUSED(instance);
    UNUSED(mode);
    // TODO implement
}

void sbufWriteU8(sbuf_t *dst, uint8_t val)
{
    *dst->ptr++ = val;
}

void sbufWriteU16(sbuf_t *dst, uint16_t val)
{
    sbufWriteU8(dst, val >> 0);
    sbufWriteU8(dst, val >> 8);
}

void sbufWriteU32(sbuf_t *dst, uint32_t val)
{
    sbufWriteU8(dst, val >> 0);
    sbufWriteU8(dst, val >> 8);
    sbufWriteU8(dst, val >> 16);
    sbufWriteU8(dst, val >> 24);
}

void sbufWriteData(sbuf_t *dst, const void *data, int len)
{
    memcpy(dst->ptr, data, len);
    dst->ptr += len;
}

