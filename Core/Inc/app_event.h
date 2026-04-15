/**
 * @file app_event.h
 * @brief Application-level event interface.
 *
 * Drivers should request actions through this interface instead of
 * directly calling RTOS primitives.
 */
#ifndef __APP_EVENT_H
#define __APP_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_EVENT_STATUS_PRINT    (1UL << 0)
#define APP_EVENT_CMD_RECV        (1UL << 1)

void app_event_bind_print_flag(void *flag_handle);
void app_event_request_status_print(void);
void app_event_request_cmd_recv(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_EVENT_H */
