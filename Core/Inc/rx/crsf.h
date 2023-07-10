#pragma once

#include "crsf_protocol.h"
#include "stm32f4xx_hal.h"
#define CRSF_MIN        172
#define CRSF_MID        992
#define CRSF_MAX        1811
#define ANALOG_MIN  48U
#define ANALOG_MID  1024U
#define ANALOG_MAX  2047U
#define ANGLE_MIN -45.0
#define ANGLE_MAX 45.0
#define ANGLE_SPEED_MIN -85
#define ANGLE_SPEED_MAX 85

#define CRSF_MAX_CHANNEL        16
#define CRSFV3_MAX_CHANNEL      24

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_STARTING_CHANNEL_MASK        0x1F
#define CRSF_SUBSET_RC_RES_CONFIGURATION_BITS       2
#define CRSF_SUBSET_RC_RES_CONFIGURATION_MASK       0x03
#define CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS  1

#define CRSF_RC_CHANNEL_SCALE_LEGACY                1.21964612568639f
#define CRSF_SUBSET_RC_RES_CONF_10B                 0
#define CRSF_SUBSET_RC_RES_BITS_10B                 10
#define CRSF_SUBSET_RC_RES_MASK_10B                 0x03FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_10B            1.0f
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_11B            0.5f
#define CRSF_SUBSET_RC_RES_CONF_12B                 2
#define CRSF_SUBSET_RC_RES_BITS_12B                 12
#define CRSF_SUBSET_RC_RES_MASK_12B                 0x0FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_12B            0.25f
#define CRSF_SUBSET_RC_RES_CONF_13B                 3
#define CRSF_SUBSET_RC_RES_BITS_13B                 13
#define CRSF_SUBSET_RC_RES_MASK_13B                 0x1FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_13B            0.125f

#define CRSF_RSSI_MIN (-130)
#define CRSF_RSSI_MAX 0
#define CRSF_SNR_MIN (-30)
#define CRSF_SNR_MAX 20

/* For documentation purposes
typedef enum {
    CRSF_RF_MODE_4_FPS = 0,
    CRSF_RF_MODE_50_FPS,
    CRSF_RF_MODE_150_FPS,
} crsfRfMode_e;
*/

typedef struct crsfFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union crsfFrame_u {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;

typedef struct crsfPayloadLinkstatistics_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

typedef struct
{
  float PitchEXPAngleRate;
  float YawEXPAngleRate;
  float RollEXPAngleRate;
}RcAngleRateData;


void crsfRxWriteTelemetryData(const void *data, int len);
void crsfRxSendTelemetryData(void);
bool crsfRxIsTelemetryBufEmpty(void); // check this function before using crsfRxWriteTelemetryData()
void ReceiveCRSFdataFromUart(void);
uint8_t crsfFrameStatus(void);
