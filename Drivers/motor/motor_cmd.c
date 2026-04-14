#include "motor_cmd.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "motor_control_service.h"
#include "app_event.h"

static char uart_cmd_buf[128];
static uint16_t uart_cmd_len = 0;

static void process_uart_cmd(char *line)
{
    if (line == NULL || line[0] == '\0') return;

    while (*line == ' ') line++;
    if (*line == '\0') return;

    char cmd = (char)toupper((unsigned char)line[0]);

    if (cmd == 'M') {
        int mode = -1;
        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
        int ret = -1;
        int need = 0;
        int n = sscanf(line + 1, "%d %f %f %f", &mode, &a, &b, &c);

        if (n < 1) {
            printf("ERR M\r\n");
            return;
        }

        switch (mode) {
            case 0: need = 1; break;
            case 1: need = 2; break;
            case 2: need = 2; break;
            case 3: need = 3; break;
            case 4: need = 3; break;
            case 5: need = 4; break;
            default:
                printf("ERR M MODE\r\n");
                return;
        }

        if (n < need) {
            printf("ERR M ARGS\r\n");
            return;
        }

        ret = motor_control_service_apply_mode(mode, a, b, c);
        if (ret == 0) {
            printf("OK M %d\r\n", mode);
            app_event_request_status_print();
        } else if (ret == -2) {
            printf("ERR CALI\r\n");
        } else {
            printf("ERR M\r\n");
        }
        return;
    }

    if (cmd == 'T') {
        int t = 0;
        if (sscanf(line + 1, "%d", &t) == 1) {
            motor_control_service_set_test_mode(t);
            printf("OK T %d\r\n", t);
            app_event_request_status_print();
        } else {
            printf("ERR T\r\n");
        }
        return;
    }

    if (cmd == 'E') {
        motor_control_service_estop();
        printf("OK E\r\n");
        app_event_request_status_print();
        return;
    }

    if (cmd == 'C') {
        motor_control_service_start_calibration();
        printf("OK C\r\n");
        app_event_request_status_print();
        return;
    }

    if (cmd == 'S') {
        app_event_request_status_print();
        return;
    }

    if (cmd == 'P') {
        float p_pos = 0.0f, i_pos = 0.0f, d_pos = 0.0f;
        float p_spd = 0.0f, i_spd = 0.0f, d_spd = 0.0f;
        float p_td = 0.0f, i_td = 0.0f, d_td = 0.0f;
        float p_tq = 0.0f, i_tq = 0.0f, d_tq = 0.0f;
        int n = sscanf(line + 1,
                       "%f %f %f %f %f %f %f %f %f %f %f %f",
                       &p_pos, &i_pos, &d_pos,
                       &p_spd, &i_spd, &d_spd,
                       &p_td, &i_td, &d_td,
                       &p_tq, &i_tq, &d_tq);
        if (n == 12) {
            motor_control_service_apply_pid(
                p_pos, i_pos, d_pos,
                p_spd, i_spd, d_spd,
                p_td, i_td, d_td,
                p_tq, i_tq, d_tq
            );
            printf("OK P\r\n");
            app_event_request_status_print();
        } else {
            printf("ERR P\r\n");
        }
        return;
    }

    if (cmd == 'H') {
        printf("C\r\n");
        printf("M mode a b c\r\n");
        printf("T val\r\n");
        printf("P 12floats\r\n");
        printf("S\r\n");
        printf("E\r\n");
        return;
    }

    printf("ERR cmd\r\n");
}

void motor_cmd_init(void)
{
    uart_cmd_len = 0;
    memset(uart_cmd_buf, 0, sizeof(uart_cmd_buf));
}

void motor_cmd_poll(void)
{
  uint8_t ch = 0;
    while (usart1_read_byte(&ch)) {
        if (ch == '\r' || ch == '\n') {
            if (uart_cmd_len > 0) {
                uart_cmd_buf[uart_cmd_len] = '\0';
                process_uart_cmd(uart_cmd_buf);
                uart_cmd_len = 0;
            }
            continue;
        }

        if (uart_cmd_len < (sizeof(uart_cmd_buf) - 1)) {
            uart_cmd_buf[uart_cmd_len++] = (char)ch;
        } else {
            uart_cmd_len = 0;
        }
    }
}
