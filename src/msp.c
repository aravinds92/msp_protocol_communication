#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "msp_protocol.h"
#include "lib.h"

#define BUILD_DATE_LENGTH 11
#define BUILD_TIME_LENGTH 8
#define GIT_SHORT_REVISION_LENGTH   7
#define MAX_VOLTAGE_METERS 2
#define TARGET_BOARD_IDENTIFIER "EDISON"
#define BOARD_IDENTIFIER_LENGTH 6


const char * const buildDate = __DATE__;
const char * const buildTime = __TIME__;
const char * const shortGitRevision = "1234567";
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

static uint8_t mspSerialChecksum(uint8_t checksum, uint8_t byte)
{
    return checksum ^ byte;
}

static uint8_t mspSerialChecksumBuf(uint8_t checksum, uint8_t *data, int len)
{
    while(len-- > 0) {
        checksum = mspSerialChecksum(checksum, *data++);
    }

    return checksum;
}


void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
{
    buf->end = buf->ptr;
    buf->ptr = base;
}


uint8_t* sbufPtr(sbuf_t *buf)
{
    return buf->ptr;
}


int sbufBytesRemaining(sbuf_t *buf)
{
    return buf->end - buf->ptr;
}

void sbufWriteString(sbuf_t *dst, const char *string)
{
    sbufWriteData(dst, string, strlen(string));
}



int mspServerCommandHandler(mspPacket_t *cmd, mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;

    float attitude_roll, attitude_pitch, attitude_yaw;

    static int n = 0;
    int i;

    int len = sbufBytesRemaining(src);
    //printf("%d\n",MSP_BOXNAMES);        
    printf("command code: %d\n",cmd->cmd);
    switch (cmd->cmd) 
    {
        case MSP_API_VERSION:
            //printf("code 1\n");
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU8(dst, API_VERSION_MAJOR);
            sbufWriteU8(dst, API_VERSION_MINOR);
            break;

        case MSP_FC_VARIANT:
            sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
            sbufWriteU8(dst, FC_VERSION_MAJOR);
            sbufWriteU8(dst, FC_VERSION_MINOR);
            sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
            break;

        case MSP_BOARD_INFO:
            sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);


#ifdef USE_HARDWARE_REVISION_DETECTION
            sbufWriteU16(dst, hardwareRevision);
#else
            sbufWriteU16(dst, 0); // No hardware revision available.
#endif
            sbufWriteU8(dst, 0);  // 0 == FC, 1 == OSD, 2 == FC with OSD
            break;

        case MSP_BUILD_INFO:
            printf("date\n");
            sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
            sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
            sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
            break;

            // DEPRECATED - Use MSP_API_VERSION
        case MSP_IDENT:
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 255);
            sbufWriteU32(dst, 65535); // "capability"
            break;

        case MSP_STATUS:
            //printf("MSP_STATUS\n");
            sbufWriteU16(dst, 0);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, 3);           //Sensors in the system
            sbufWriteU32(dst, 127);
            sbufWriteU8(dst, 127);
            sbufWriteU16(dst, 30000);
            break;

        case MSP_UID:
            sbufWriteU32(dst, 0);
            sbufWriteU32(dst, 0);
            sbufWriteU32(dst, 0);
            break;

        case MSP_BATTERY_CONFIG:
            //Showing battery to be full for now
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 255);
            sbufWriteU16(dst, 65535);
            sbufWriteU8(dst, 255);
            break;

        case MSP_ACC_TRIM:
            //printf("MSP_ACC_TRIM\n");
            sbufWriteU16(dst, 1000);            //pitch
            sbufWriteU16(dst, 1000);            //roll
            break;
        
        case MSP_BOXNAMES:
            sbufWriteString(dst, "None");
            //serializeBoxNamesReply(reply);
            break;

        case MSP_VOLTAGE_METERS:
            // write out voltage, once for each meter.
            /*for (int i = 0; i < MAX_VOLTAGE_METERS; i++) {
                uint16_t voltage = getVoltageMeter(i)->vbat;
                sbufWriteU8(dst, (uint8_t)constrain(voltage, 0, 255));
            }*/
            for (i = 0; i < MAX_VOLTAGE_METERS; i++) {
                sbufWriteU8(dst, 255);
            }
            break;


        case MSP_MISC:
            sbufWriteU16(dst, 65535);

            sbufWriteU16(dst, 65535);
            sbufWriteU16(dst, 65535);
            sbufWriteU16(dst, 65535);

            sbufWriteU16(dst, 65535);

            sbufWriteU8(dst, 0); // gps_type
            sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            sbufWriteU8(dst, 0); // gps_ubx_sbas

            
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 255);
            sbufWriteU8(dst, 0);

            sbufWriteU16(dst, 65535);
            
            break;

        
        case MSP_ATTITUDE:
            
            sbufWriteU16(dst, 120);     //roll
            sbufWriteU16(dst, 120);     //pitch
            sbufWriteU16(dst, 340);     //yaw
            break;


        case MSP_ANALOG: {
            sbufWriteU8(dst, 0);
            sbufWriteU16(dst, 65535); // milliamp hours drawn from battery
            sbufWriteU16(dst, 65535);

            sbufWriteU16(dst, 65535); // send amperage in 0.001 A steps. Negative range is truncated to zero
            
            break;
        }

    }

    return 1;     // message was handled successfully
}


int mspProcessCommand(mspPacket_t *command, mspPacket_t *reply)
{
    // initialize reply by default
    reply->cmd = command->cmd;

    int status = mspServerCommandHandler(command, reply);
    reply->result = status;

    return status;
}


void mspSerialEncode(mspPort_t *msp, mspPacket_t *packet)
{
    //printf("here\n");
    serialBeginWrite(msp->port);
    int len = sbufBytesRemaining(&packet->buf);
    uint8_t hdr[] = {'$', 'M', packet->result < 0 ? '!' : (msp->mode == MSP_MODE_SERVER ? '>' : '<'), len, packet->cmd};
    uint8_t csum = 0;                                       // initial checksum value
    serialWriteBuf(msp->port, hdr, sizeof(hdr));
    csum = mspSerialChecksumBuf(csum, hdr + 3, 2);          // checksum starts from len field
    if(len > 0) {
        serialWriteBuf(msp->port, sbufPtr(&packet->buf), len);
        csum = mspSerialChecksumBuf(csum, sbufPtr(&packet->buf), len);
        //printf("checksum:%d\n",csum);
    }
    serialWrite(msp->port, csum);
    serialEndWrite(msp->port);
}


void mspSerialProcessReceivedCommand(mspPort_t *msp)
{
   uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t message = {
        .buf = {
            .ptr = outBuf,
            .end = ARRAYEND(outBuf),
        },
        .cmd = -1,
        .result = 0,
    };

    mspPacket_t command = {
        .buf = {
            .ptr = msp->inBuf,
            .end = msp->inBuf + msp->dataSize,
        },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    mspPacket_t *reply = &message;

    uint8_t *outBufHead = reply->buf.ptr;
    int status = mspProcessCommand(&command, reply);

    if (status) {
        //printf("Command code: %d\nWriting to PC\n",command.cmd);
        //printf("Command code: %d\n",command.cmd);
        // reply should be sent back
        sbufSwitchToReader(&reply->buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, reply);
    }

    msp->c_state = IDLE;
}



static bool mspSerialProcessReceivedByte(mspPort_t *msp, uint8_t c)
{
    //printf("char:%c\tstate:%d\n",c,msp->c_state);
    switch(msp->c_state) {
        default:                 // be conservative with unexpected state
        case IDLE:
            if (c != '$')        // wait for '$' to start MSP message
            {
                if(c == 'M')
                {
                    msp->c_state = HEADER_M;
                    mspSerialProcessReceivedByte(msp, 'M');
                    return true;                   
                }
                return false;
            }
            msp->c_state = HEADER_M;
            break;
        case HEADER_M:
            msp->c_state = (c == 'M') ? HEADER_ARROW : IDLE;
            break;
        case HEADER_ARROW:
            msp->c_state = HEADER_ARROW;
            switch(c) {
                case '<': // COMMAND
                    if (msp->mode == MSP_MODE_SERVER) {
                        msp->c_state = HEADER_SIZE;
                    }
                    break;
                case '>': // REPLY
                    if (msp->mode == MSP_MODE_CLIENT) {
                        msp->c_state = HEADER_SIZE;
                    }
                    break;
                default:
                    break;
            }
            break;
        case HEADER_SIZE:
            if (c > MSP_PORT_INBUF_SIZE) {
                msp->c_state = IDLE;
            } else {
                msp->dataSize = c;
                msp->offset = 0;
                msp->c_state = HEADER_CMD;
            }
            break;
        case HEADER_CMD:
            msp->cmdMSP = c;
            msp->c_state = HEADER_DATA;
            break;
        case HEADER_DATA:
            if(msp->offset < msp->dataSize) {
                msp->inBuf[msp->offset++] = c;
            } else {
                uint8_t checksum = 0;
                checksum = mspSerialChecksum(checksum, msp->dataSize);
                checksum = mspSerialChecksum(checksum, msp->cmdMSP);
                checksum = mspSerialChecksumBuf(checksum, msp->inBuf, msp->dataSize);
                //printf("c:%d\tchecksum:%d\n",c,checksum);
                if(c == checksum)
                {
                    msp->c_state = MESSAGE_RECEIVED;
                    //printf("processing received command\n");
                }
                else
                    msp->c_state = IDLE;
            }
            break;
    }
    return true;
}



void mspSerialProcess(void)
{
    int* fd;
    int flag = 0;
    int i;
    //printf("Processing\n");
    for (i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        mspPort_t *msp = &mspPorts[i];
        if (!msp->port) {
            continue;
        }
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialRxBytesWaiting(msp->port))) {
            flag = 1;
            uint8_t c = serialRead(msp->port);
            bool consumed = mspSerialProcessReceivedByte(msp, c);

            /*if (!consumed) {
                evaluateOtherData(msp->port, c);
            }*/

            if (msp->c_state == MESSAGE_RECEIVED) {
            	if (msp->mode == MSP_MODE_SERVER) {
            		mspSerialProcessReceivedCommand(msp);
            	}

                break; // process one command at a time so as not to block and handle modal command immediately
            }
        }

        if(flag == 1)
        {
            //printf("Inside\n");
            flag = 0;
        }

        // TODO consider extracting this outside the loop and create a new loop in mspClientProcess and rename mspProcess to mspServerProcess
        //for msp client
        if (msp->c_state == IDLE && msp->commandSenderFn && !bytesWaiting) {
            uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
            mspPacket_t message = {
                .buf = {
                    .ptr = outBuf,
                    .end = ARRAYEND(outBuf),
                },
                .cmd = -1,
                .result = 0,
            };

            mspPacket_t *command = &message;

            uint8_t *outBufHead = command->buf.ptr;

            bool shouldSend = msp->commandSenderFn(command); // FIXME rename to request builder

            if (shouldSend) {
                sbufSwitchToReader(&command->buf, outBufHead); // change streambuf direction

                mspSerialEncode(msp, command);
            }

            msp->commandSenderFn = NULL;
        }

        fd = (int*)((void*)mspPorts[0].port + sizeof(serialPort_t));
        tcflush(*fd,TCIOFLUSH);
    }
}


void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}