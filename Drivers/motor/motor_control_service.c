#include "motor_control_service.h"

#include <stdio.h>

#include "stm32f1xx_hal.h"
#include "motor_foc.h"
#include "motor_runtime_param.h"

int motor_control_service_apply_mode(int mode, float a, float b, float c)
{
    motor_control_context_t ctx;
    motor_control_get_context(&ctx);

    if (mode != 0 && zero_cali_state != CALI_DONE) {
        return -2;
    }

    switch (mode) {
        case 0:
            ctx.type = control_type_null;
            break;
        case 1:
            ctx.type = control_type_position;
            ctx.position = a;
            break;
        case 2:
            ctx.type = control_type_speed;
            ctx.speed = a;
            break;
        case 3:
            ctx.type = control_type_torque;
            ctx.torque_norm_d = a;
            ctx.torque_norm_q = b;
            break;
        case 4:
            ctx.type = control_type_speed_torque;
            ctx.speed = a;
            ctx.max_torque_norm = b;
            break;
        case 5:
            ctx.type = control_type_position_speed_torque;
            ctx.position = a;
            ctx.max_speed = b;
            ctx.max_torque_norm = c;
            break;
        default:
            return -1;
    }

    motor_control_set_context(&ctx);
    test_mode = 0;
    return 0;
}

void motor_control_service_apply_pid(
    float p_pos, float i_pos, float d_pos,
    float p_spd, float i_spd, float d_spd,
    float p_td, float i_td, float d_td,
    float p_tq, float i_tq, float d_tq
)
{
    set_motor_pid(
        p_pos, i_pos, d_pos,
        p_spd, i_spd, d_spd,
        p_td, i_td, d_td,
        p_tq, i_tq, d_tq
    );
}

void motor_control_service_print_status(void)
{
    motor_control_context_t ctx;
    motor_control_get_context(&ctx);
    printf("MODE=%d test=%d cali=%d pos=%.3f spd=%.3f td=%.3f tq=%.3f mspd=%.3f mtq=%.3f\r\n",
           (int)ctx.type,
           test_mode,
           (int)zero_cali_state,
           ctx.position,
           ctx.speed,
           ctx.torque_norm_d,
           ctx.torque_norm_q,
           ctx.max_speed,
           ctx.max_torque_norm);
}

void motor_control_service_set_test_mode(int mode)
{
    test_mode = mode;
}

void motor_control_service_estop(void)
{
    motor_control_estop();
    test_mode = 0;
}

void motor_control_service_start_calibration(void)
{
    motor_control_estop();
    __disable_irq();
    zero_cali_state = CALI_LOCKING;
    test_mode = 99;
    __enable_irq();
}
