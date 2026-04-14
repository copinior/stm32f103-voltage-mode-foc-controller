#ifndef __MOTOR_CONTROL_SERVICE_H
#define __MOTOR_CONTROL_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

int motor_control_service_apply_mode(int mode, float a, float b, float c);
void motor_control_service_apply_pid(
    float p_pos, float i_pos, float d_pos,
    float p_spd, float i_spd, float d_spd,
    float p_td, float i_td, float d_td,
    float p_tq, float i_tq, float d_tq
);
void motor_control_service_print_status(void);
void motor_control_service_set_test_mode(int mode);
void motor_control_service_estop(void);
void motor_control_service_start_calibration(void);

#ifdef __cplusplus
}
#endif

#endif
