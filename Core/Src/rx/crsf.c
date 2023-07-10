#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "rx/crsf.h"
#include "common/crc.h"
#include "driver/system.h"
#include "common/time.h"
#include "common/maths.h"
#include "rx/crsf_protocol.h"
#include "rx/rx.h"
#include "stm32f4xx_hal_uart.h"
#include "pidControl.h"
#include "Dshot.h"
#include "micros.h"



#define CRSF_TIME_NEEDED_PER_FRAME_US   1750 // a maximally sized 64byte payload will take ~1550us, round up to 1750.
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz


#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

extern UART_HandleTypeDef UART1_Handler;
extern uint16_t my_motor_value[4];
extern UART_HandleTypeDef UART1_Handler;
 bool crsfFrameDone = false;
 crsfFrame_t crsfFrame;
 crsfFrame_t crsfChannelDataFrame;
static timeUs_t crsfFrameStartAtUs = 0;
uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
static float channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;

static uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;
uint8_t currentValue = 0;
uint8_t crsfFrameStatus(void);
static uint8_t crsfFrameCRC(void);
static void crsfDataReceive(uint16_t c);
static void parseRcToAngleRate(void);
static void parseRawRCvalue(void);
uint16_t ThroValue;
RcAngleRateData AngleEXPRateData;
/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit=
 *
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */
//*********************************************************
/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

/*
 * 0x1C Link statistics RX
 * Payload:
 *
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink RSSI ( % )
 * uint8_t Downlink Package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * uint8_t Uplink RF Power ( db )
 */

/*
 * 0x1D Link statistics TX
 * Payload:
 *
 * uint8_t Uplink RSSI ( dBm * -1 )
 * uint8_t Uplink RSSI ( % )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Downlink RF Power ( db )
 * uint8_t Uplink FPS ( FPS / 10 )
 */


//static timeUs_t lastLinkStatisticsFrameUs;

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

static uint8_t crsfFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

static void crsfDataReceive(uint16_t c)
{
    //rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;
     static uint8_t crsfFramePosition = 0;

// #if defined(USE_CRSF_V3)
//     static uint8_t crsfFrameErrorCnt = 0;
// #endif

     // const timeUs_t currentTimeUs = microsISR();
       const timeUs_t currentTimeUs = micros();
       // const timeUs_t currentTimeUs = micros_HAL();

     if (cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
// #if defined(USE_CRSF_V3)
        // if (crsfFramePosition > 0) {
//             // count an error if full valid frame not received within the allowed time.
//             crsfFrameErrorCnt++;
        // }
// #endif
        crsfFramePosition = 0;
     }
     if (crsfFramePosition == 0) {
        crsfFrameStartAtUs = currentTimeUs;
    }

    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : MIN(crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

    if(crsfFramePosition < fullFrameLength){
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        if (crsfFramePosition >= fullFrameLength){
            crsfFramePosition = 0;
            const uint8_t crc = crsfFrameCRC(); //receive complete
            if(crc == crsfFrame.bytes[fullFrameLength -1]){ //crsfFrame帧中的crc位（tip：数组从0开始）
// #if defined(USE_CRSF_V3)
//                 crsfFrameErrorCnt = 0;
// #endif
                switch (crsfFrame.frame.type)
                {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                    if(crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER){
                        //rxRuntimeState->lastRcFrameTimeUs = currentTimeUs;
                        crsfFrameDone = true;
                        memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
                    }
                    break;
#if defined(USE_TELEMETRY_CRSF) && defined(USE_MSP_OVER_TELEMETRY)
                case CRSF_FRAMETYPE_MSP_REQ:
                case CRSF_FRAMETYPE_MSP_WRITE: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    if (bufferCrsfMspFrame(frameStart, crsfFrame.frame.frameLength - 4)) {
                        //crsfScheduleMspResponse(crsfFrame.frame.payload[1]);
                    }
                    break;
                }
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
                case CRSF_FRAMETYPE_DEVICE_PING:
                    //crsfScheduleDeviceInfoResponse();
                    break;
                case CRSF_FRAMETYPE_DEVICE_INFO:
                    //crsfHandleDeviceInfoResponse(crsfFrame.frame.payload);
                    break;
                case CRSF_FRAMETYPE_DISPLAYPORT_CMD: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    //crsfProcessDisplayPortCmd(frameStart);
                    break;
                }
#endif
#if defined(USE_CRSF_LINK_STATISTICS)

                case CRSF_FRAMETYPE_LINK_STATISTICS: {
                    // if to FC and 10 bytes + CRSF_FRAME_ORIGIN_DEST_SIZE
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)) {
                        //const crsfLinkStatistics_t* statsFrame = (const crsfLinkStatistics_t*)&crsfFrame.frame.payload;
                        //handleCrsfLinkStatisticsFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_LINK_STATISTICS_RX: {
                    break;
                }
                case CRSF_FRAMETYPE_LINK_STATISTICS_TX: {
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE)) {
                        // const crsfLinkStatisticsTx_t* statsFrame = (const crsfLinkStatisticsTx_t*)&crsfFrame.frame.payload;
                        // handleCrsfLinkStatisticsTxFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#endif
#endif
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_COMMAND:
                    if ((crsfFrame.bytes[fullFrameLength - 2] == crsfFrameCmdCRC()) &&
                        (crsfFrame.bytes[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
                        //crsfProcessCommand(crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE);
                    }
                    break;
#endif
                default:
                    break;
                }
            }
            else {
// #if defined(USE_CRSF_V3)
//                 if (crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD)
//                     crsfFrameErrorCnt++;
// #endif
            }
        }
#if defined(USE_CRSF_V3)
        if (crsfBaudNegotiationInProgress() || isEepromWriteInProgress()) {
            // don't count errors when negotiation or eeprom write is in progress
            crsfFrameErrorCnt = 0;
        } else if (crsfFrameErrorCnt >= CRSF_FRAME_ERROR_COUNT_THRESHOLD) {
            // fall back to default speed if speed mismatch detected
            setCrsfDefaultSpeed();
            crsfFrameErrorCnt = 0;
        }
#endif
    }

}

uint8_t crsfFrameStatus(void)
{

    if(crsfFrameDone){
        crsfFrameDone = false;

        // unpack the RC Chanels
        if (crsfChannelDataFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
            const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfChannelDataFrame.frame.payload;
            channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
            crsfChannelData[0] = rcChannels ->chan0;
            crsfChannelData[1] = rcChannels ->chan1;
            crsfChannelData[2] = rcChannels ->chan2;
            crsfChannelData[3] = rcChannels ->chan3;
            crsfChannelData[4] = rcChannels ->chan4;
            crsfChannelData[5] = rcChannels ->chan5;
            crsfChannelData[6] = rcChannels ->chan6;
            crsfChannelData[7] = rcChannels ->chan7;
            crsfChannelData[8] = rcChannels ->chan8;
            crsfChannelData[9] = rcChannels ->chan9;
            crsfChannelData[10] = rcChannels->chan10;
            crsfChannelData[11] = rcChannels->chan11;
            crsfChannelData[12] = rcChannels->chan12;
            crsfChannelData[13] = rcChannels->chan13;
            crsfChannelData[14] = rcChannels->chan14;
            crsfChannelData[15] = rcChannels->chan15;
        } else{
            // use subset RC frame structure (0x17)
            uint8_t readByteIndex = 0;
            const uint8_t *payload = crsfChannelDataFrame.frame.payload;

            // get the configuration byte
            uint8_t configByte = payload[readByteIndex++];

            // get the channel number of start channel
            uint8_t startChannel = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            uint8_t channelBits;
            uint16_t channelMask;
            uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;
            switch (channelRes) {
            case CRSF_SUBSET_RC_RES_CONF_10B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
                break;
            default:
            case CRSF_SUBSET_RC_RES_CONF_11B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_12B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_13B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
                break;
            }

            // do nothing for the reserved configuration bit
            configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            uint8_t numOfChannels = ((crsfChannelDataFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1) * 8) / channelBits;

            // unpack the channel data
            uint8_t bitsMerged = 0;
            uint32_t readValue = 0;
            for (uint8_t n = 0; n < numOfChannels; n++) {
                while (bitsMerged < channelBits) {
                    uint8_t readByte = payload[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                crsfChannelData[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
         parseRawRCvalue();
         parseRcToAngleRate();
        return RX_FRAME_COMPLETE;


    }
    return RX_FRAME_PENDING;
}


//遥控器通道2解析函数-->油门值
static void parseRawRCvalue()
{
    uint16_t _min = CRSF_MIN;
    uint16_t _max = CRSF_MAX;
    ThroValue =  MAP_U16(crsfChannelData[2],
                    _min,_max,ANALOG_MIN,ANALOG_MAX);

}

//遥控器Roll-Pitch-Yaw通道转换函数
//172-1811  ->  -45 -- 45 angle 或 -80 -- 80 angle speed
static void parseRcToAngleRate(void)
{
    uint16_t _min = CRSF_MIN;
    uint16_t _max = CRSF_MAX;
  if (crsfChannelData[5] <= 992) {
   AngleEXPRateData.YawEXPAngleRate = (int)MAP_F(crsfChannelData[3], _min, _max, ANGLE_SPEED_MIN, ANGLE_SPEED_MAX);
   AngleEXPRateData.RollEXPAngleRate = (int)MAP_F(crsfChannelData[0], _min, _max, ANGLE_MIN, ANGLE_MAX);
   AngleEXPRateData.PitchEXPAngleRate = (int)MAP_F(crsfChannelData[1], _min, _max, ANGLE_MIN, ANGLE_MAX);
   AngleEXPRateData.PitchEXPAngleRate = -AngleEXPRateData.PitchEXPAngleRate;
   AngleEXPRateData.YawEXPAngleRate = -AngleEXPRateData.YawEXPAngleRate;
  }
  else if (crsfChannelData[5] > 992) {
   AngleEXPRateData.YawEXPAngleRate = (int)MAP_F(crsfChannelData[3], _min, _max, ANGLE_SPEED_MIN, ANGLE_SPEED_MAX);
   AngleEXPRateData.RollEXPAngleRate = (int)MAP_F(crsfChannelData[0], _min, _max, ANGLE_SPEED_MIN, ANGLE_SPEED_MAX);
   AngleEXPRateData.PitchEXPAngleRate = (int)MAP_F(crsfChannelData[1], _min, _max, ANGLE_SPEED_MIN, ANGLE_SPEED_MAX);
   AngleEXPRateData.PitchEXPAngleRate = -AngleEXPRateData.PitchEXPAngleRate;
   AngleEXPRateData.YawEXPAngleRate = -AngleEXPRateData.YawEXPAngleRate;
  }
  if (AngleEXPRateData.PitchEXPAngleRate == -0)
   AngleEXPRateData.PitchEXPAngleRate = 0;
  if (AngleEXPRateData.YawEXPAngleRate == -0)
   AngleEXPRateData.YawEXPAngleRate = 0;
}

//接收机回传Telemetry遥测，待完成
void crsfRxWriteTelemetryData(const void *data,int len)
{
    len = MIN(len,(int)sizeof(telemetryBuf));
    memcpy(telemetryBuf,data,len);
    telemetryBufLen = len;
}

bool crsfRxIsTelemetryBufEmpty(void)
{
    return telemetryBufLen == 0;
}

void crsfRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if(telemetryBufLen > 0)
    {
        HAL_UART_Transmit_DMA(&UART1_Handler,telemetryBuf,telemetryBufLen);
        telemetryBufLen = 0;
    }
}




inline void ReceiveCRSFdataFromUart(void)
{
    HAL_UART_Receive_DMA(&UART1_Handler,&currentValue,1);

}




//接收机串口接受完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    crsfDataReceive(currentValue);

}



