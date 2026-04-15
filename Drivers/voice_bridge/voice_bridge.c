#include "voice_bridge.h"
#include "global_def.h"
#include "main.h"
#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "cmsis_os2.h"
#include "motor_foc.h"
#include "motor_control_service.h"
#include "motor_runtime_param.h"
#include "app_event.h"

#define VB_QUEUE_SIZE   8
#define VB_CMD_HANDLER_NUM  16
#define TURN_STEP_RAD  (PI / 4)
#define UTURN_RAD   (PI)
#define VB_BRAKE_STEP_RAD         (VB_SPEED_STEP * 0.25f)
#define VB_BRAKE_INTERVAL_MS      (30u)

typedef enum {
    VB_WAIT_SOF0 = 0,
    VB_WAIT_SOF1,
    VB_WAIT_LEN,
    VB_RECV_PAYLOAD,
    VB_WAIT_XOR
} vb_rx_state_t;

typedef enum {
    VB_DRIVE_MODE_POSITION = 0,
    VB_DRIVE_MODE_SPEED
} vb_drive_mode_t;

typedef void (*vb_cmd_handler_t)(const vb_payload_t *payload);

static vb_drive_mode_t s_drive_mode = VB_DRIVE_MODE_POSITION;
static vb_cmd_handler_t vb_cmd_handlers[VB_CMD_HANDLER_NUM] = {NULL};

static vb_rx_state_t s_rx_state = VB_WAIT_SOF0;
static uint8_t s_rx_len = 0u;
static uint8_t s_rx_payload[VB_PAYLOAD_LEN];
static uint8_t s_rx_idx = 0u;
static vb_payload_t s_queue[VB_QUEUE_SIZE];

static volatile uint8_t s_q_head = 0u;
static volatile uint8_t s_q_tail = 0u; 
static volatile uint32_t s_q_drop_count = 0u;
static uint8_t s_brake_active = 0u;


static void vb_on_frame_ok(const uint8_t *payload12);

static void vb_brake_cancel(void) {
    s_brake_active = 0u;
}

static void vb_brake_step_once(void) {
    motor_control_context_t ctx;
    float base = 0.0f;
    float target = 0.0f;

    if (!s_brake_active) {
        return;
    }

    motor_control_get_context(&ctx);
    base = (ctx.type == control_type_speed) ? ctx.speed : motor_speed;

    if (base > VB_BRAKE_STEP_RAD) {
        target = base - VB_BRAKE_STEP_RAD;
    } else if (base < -VB_BRAKE_STEP_RAD) {
        target = base + VB_BRAKE_STEP_RAD;
    } else {
        target = 0.0f;
        s_brake_active = 0u;
    }

    motor_control_service_apply_mode(2, target, 0, 0);

    if (s_brake_active) {
        osDelay(VB_BRAKE_INTERVAL_MS);
        app_event_request_cmd_recv();
    }
}

static uint8_t vb_calc_xor(uint8_t len, const uint8_t *payload) {
    uint8_t x = len;
    uint8_t i;
    for (i = 0; i < VB_PAYLOAD_LEN; i++) {
        x ^= payload[i];
    }
    return x;
}

void vb_uart_rx_callback(uint8_t byte) {
    switch (s_rx_state) {
        case VB_WAIT_SOF0:
            if (byte == VB_SOF_0) { 
                s_rx_state = VB_WAIT_SOF1; 
            }
            break;

        case VB_WAIT_SOF1:
            if (byte == VB_SOF_1) { 
                s_rx_state = VB_WAIT_LEN;
            } else if (byte == VB_SOF_0) {
                s_rx_state = VB_WAIT_SOF1;
            } else {
                s_rx_state = VB_WAIT_SOF0;
            }
            break;

        case VB_WAIT_LEN:
                s_rx_len = byte;
                if (s_rx_len == VB_PAYLOAD_LEN) {
                    s_rx_idx = 0u;
                    s_rx_state = VB_RECV_PAYLOAD;
                } else {
                    s_rx_state = VB_WAIT_SOF0;
                }
            break;
        
        case VB_RECV_PAYLOAD:
            s_rx_payload[s_rx_idx++] = byte;
            if (s_rx_idx >= VB_PAYLOAD_LEN) {
                s_rx_state = VB_WAIT_XOR;
            }
            break;

        case VB_WAIT_XOR:
            if (byte == vb_calc_xor(s_rx_len, s_rx_payload)) {
                vb_on_frame_ok(s_rx_payload);
            }
            s_rx_state = VB_WAIT_SOF0;
            break;

        default:
            s_rx_state = VB_WAIT_SOF0;
            break;
        }
}

uint16_t vb_read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | (uint16_t)(p[1] << 8);
}

int16_t vb_read_i16_le(const uint8_t *p) {
    return (int16_t)vb_read_u16_le(p);
}

uint32_t vb_read_u32_le(const uint8_t *p) {
    return (uint32_t)(p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
} 

int vb_queue_push(const vb_payload_t *frame) {
    uint8_t next = 0;
    
    if (frame == NULL) return 0;

    __disable_irq();
    next = (uint8_t)((s_q_head + 1u) % VB_QUEUE_SIZE);

    if (next == s_q_tail) {
        s_q_drop_count++;
        __enable_irq();
        return 0;
    }

    s_queue[s_q_head] = *frame;
    s_q_head = next;
    __enable_irq();

    return 1;
}

int vb_queue_pop(vb_payload_t *out) {
    if(out == NULL) return 0;

    __disable_irq();
    if (s_q_head == s_q_tail) {
        __enable_irq();
        return 0;
    }

    *out = s_queue[s_q_tail];
    s_q_tail = (uint8_t)((s_q_tail + 1u) % VB_QUEUE_SIZE);
    __enable_irq();

    return 1;
}

static void vb_cmd_accel_handler(const vb_payload_t *payload) {
    (void)payload;

    s_drive_mode = VB_DRIVE_MODE_SPEED;

    motor_control_context_t ctx;
    float target;
    float base = 0.0f;

    motor_control_get_context(&ctx);

    base = (ctx.type == control_type_speed) ? ctx.speed : motor_speed;
    target = base + VB_SPEED_STEP;

    motor_control_service_apply_mode(2, target, 0, 0);
}

static void vb_cmd_decel_handler(const vb_payload_t *payload) {
    (void)payload;

    s_drive_mode = VB_DRIVE_MODE_SPEED;

    motor_control_context_t ctx;
    float target;
    float base = 0.0f;

    motor_control_get_context(&ctx);

    base = (ctx.type == control_type_speed) ? ctx.speed : motor_speed;
    target = base - VB_SPEED_STEP;

    motor_control_service_apply_mode(2, target, 0, 0);
}

static void vb_cmd_brake_handler(const vb_payload_t *payload) {
    (void)payload;

    s_drive_mode = VB_DRIVE_MODE_SPEED;
    s_brake_active = 1u;
}

static void vb_cmd_stop_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_service_apply_mode(0, 0, 0, 0);
}

static int vb_get_dir(float v, float eps) {
    if (v > eps) return 1;
    if (v < -eps) return -1;
    return 0;
}

static void vb_speed_apply_by_dir_request(int desired_dir, const char *tag) {
    motor_control_context_t ctx;
    float speed_base, abs_speed, target;
    int curr_dir;

    motor_control_get_context(&ctx);
    speed_base = (ctx.type == control_type_speed) ? ctx.speed : motor_speed;
    curr_dir = vb_get_dir(speed_base, 0.01f);

    if (curr_dir == desired_dir) {
        printf("VB %s: direction already matched\r\n", tag);
        return;
    }

    abs_speed = fabsf(speed_base);
    if (abs_speed < 0.01f) abs_speed = VB_SPEED_STEP;

    target = (curr_dir == 0) ? (desired_dir * abs_speed) : (-speed_base);
    (void)motor_control_service_apply_mode(2, target, 0, 0);
}

static void vb_cmd_turnLeft_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_context_t ctx;
    float base = 0.0f;
    motor_control_get_context(&ctx);
    if (s_drive_mode == VB_DRIVE_MODE_POSITION) {
        base = (ctx.type == control_type_position) ? ctx.position : motor_mech_angle;

        motor_control_service_apply_mode(1, base - TURN_STEP_RAD, 0, 0);
    } else {
        vb_speed_apply_by_dir_request(-1, "left");
    }
} 

static void vb_cmd_turnRight_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_context_t ctx;
    float base = 0.0f;
    motor_control_get_context(&ctx);

    if (s_drive_mode == VB_DRIVE_MODE_POSITION) {
        base = (ctx.type == control_type_position) ? ctx.position : motor_mech_angle;

        motor_control_service_apply_mode(1, base + TURN_STEP_RAD, 0, 0);
    } else {
        vb_speed_apply_by_dir_request(1, "right");
    }   
}

static void vb_cmd_UturnLeft_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_context_t ctx;
    float base = 0.0f;
    motor_control_get_context(&ctx);
    
    base = (ctx.type == control_type_position) ? ctx.position : motor_mech_angle;

    motor_control_service_apply_mode(1, base - UTURN_RAD, 0, 0);
}

static void vb_cmd_UturnRight_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_context_t ctx;
    float base = 0.0f;
    motor_control_get_context(&ctx);
    
    base = (ctx.type == control_type_position) ? ctx.position : motor_mech_angle; 

    motor_control_service_apply_mode(1, base + UTURN_RAD, 0, 0);
}
 
static void vb_cmd_reverse_handler(const vb_payload_t *payload) {
    motor_control_context_t ctx;
    float speed_base;
    float abs_speed;
    float target;

    (void)payload;

    motor_control_get_context(&ctx);
    
    speed_base = (ctx.type == control_type_speed) ? ctx.speed: motor_speed;

    abs_speed = fabsf(speed_base);
    if (abs_speed < 0.01f) {
        abs_speed = VB_SPEED_STEP;
    }
 
    target = -abs_speed;

    (void)motor_control_service_apply_mode(2, target, 0, 0);
}

static void vb_cmd_mode_pos_handler(const vb_payload_t *payload) {
    (void)payload;

    s_drive_mode = VB_DRIVE_MODE_POSITION;

    motor_control_service_apply_mode(1, motor_mech_angle, 0 , 0);
}

static void vb_cmd_mode_speed_handler(const vb_payload_t *payload) {
    (void)payload;

    s_drive_mode = VB_DRIVE_MODE_SPEED;

    motor_control_service_apply_mode(2, 0, 0, 0);
}

static void vb_cmd_status_handler(const vb_payload_t *payload) {
    (void)payload;

    app_event_request_status_print();
}

static void vb_cmd_reset_handler(const vb_payload_t *payload) {
    (void)payload;
    
    NVIC_SystemReset();
}

static void vb_cmd_calibration_handler(const vb_payload_t *payload) {
    (void)payload;

    motor_control_service_start_calibration();
}

static void vb_cmd_handlers_init(void) {
    vb_cmd_handlers[0]  = NULL;
    vb_cmd_handlers[1]  = NULL;
    vb_cmd_handlers[2]  = vb_cmd_accel_handler;
    vb_cmd_handlers[3]  = vb_cmd_decel_handler;
    vb_cmd_handlers[4]  = vb_cmd_brake_handler;
    vb_cmd_handlers[5]  = vb_cmd_stop_handler;
    vb_cmd_handlers[6]  = vb_cmd_turnLeft_handler;
    vb_cmd_handlers[7]  = vb_cmd_turnRight_handler;
    vb_cmd_handlers[8]  = vb_cmd_UturnLeft_handler;
    vb_cmd_handlers[9]  = vb_cmd_UturnRight_handler;
    vb_cmd_handlers[10] = vb_cmd_reverse_handler;
    vb_cmd_handlers[11] = vb_cmd_mode_pos_handler;
    vb_cmd_handlers[12] = vb_cmd_mode_speed_handler;
    vb_cmd_handlers[13] = vb_cmd_status_handler;
    vb_cmd_handlers[14] = vb_cmd_reset_handler;
    vb_cmd_handlers[15] = vb_cmd_calibration_handler;
}

void vb_poll(void) {
    vb_payload_t frame;
    static uint32_t s_last_drop_count = 0u;

    if (s_q_drop_count != s_last_drop_count) {
        uint32_t delta = s_q_drop_count - s_last_drop_count;
        s_last_drop_count = s_q_drop_count;
        printf("VB queue full drop=%lu total=%lu\r\n", (unsigned long)delta, (unsigned long)s_q_drop_count);
    }

    while (vb_queue_pop(&frame)) {
        if (frame.command_id < VB_CMD_ID_MIN || frame.command_id > VB_CMD_ID_MAX) {
            continue;
        }
        if (frame.command_id != 4) {
            vb_brake_cancel();
        }
        vb_cmd_handler_t handler = vb_cmd_handlers[frame.command_id];
        if (handler != NULL) handler(&frame);
        
        printf("VB cmd=%d seq=%u prob=%u\r\n", frame.command_id, (unsigned int)frame.seq, (unsigned int)frame.prob_q15);
    }

    if (s_brake_active) {
        vb_brake_step_once();
    }
}

void vb_decode_payload(vb_payload_t *out, const uint8_t *payload) {
    out->ver = payload[0];
    out->source = payload[1];
    out->command_id = vb_read_i16_le(&payload[2]);
    out->seq = vb_read_u16_le(&payload[4]);
    out->prob_q15 = vb_read_u16_le(&payload[6]);
    out->ts_ms = vb_read_u32_le(&payload[8]);
}

static void vb_on_frame_ok(const uint8_t *payload12) {
    vb_payload_t frame;
    vb_decode_payload(&frame, payload12);

    (void)vb_queue_push(&frame);   
}

void vb_init(void) {
    // 清状态机
    s_rx_state = VB_WAIT_SOF0;
    s_rx_len = 0u;

    // 清接受索引和缓存
    s_rx_idx = 0u;
    
    // 清队列头尾
    s_q_head = 0u;
    s_q_tail = 0u;
    s_brake_active = 0u;

    s_drive_mode = VB_DRIVE_MODE_POSITION;

    memset(s_rx_payload, 0, sizeof(s_rx_payload));
    memset(s_queue, 0, sizeof(s_queue));

    vb_cmd_handlers_init();
}
