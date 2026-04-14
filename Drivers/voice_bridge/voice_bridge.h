#ifndef VOICE_BRIDGE_H
#define VOICE_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_config.h"

#ifndef VOICE_BRIDGE_ENABLE
#define VOICE_BRIDGE_ENABLE     1
#endif

#define VB_SOF_0            0xA5
#define VB_SOF_1            0x5A
#define VB_FRAME_LEN        16
#define VB_PAYLOAD_LEN      12

#define VB_CMD_ID_MIN       0
#define VB_CMD_ID_MAX       15

#define VB_SPEED_STEP       1.0f
#define VB_POSITION_STEP    0.5f

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t source;
    int16_t command_id;
    uint16_t seq;
    uint16_t prob_q15;
    uint32_t ts_ms;
} vb_payload_t;

void vb_init(void);
void vb_poll(void);
void vb_uart_rx_callback(uint8_t byte);

#ifdef __cplusplus
} 
#endif

#endif
