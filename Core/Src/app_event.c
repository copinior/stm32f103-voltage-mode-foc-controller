#include "app_event.h"
#include "cmsis_os2.h"

static osEventFlagsId_t s_print_flag = NULL;

void app_event_bind_print_flag(void *flag_handle)
{
    s_print_flag = (osEventFlagsId_t)flag_handle;
}

void app_event_request_status_print(void)
{
    if (s_print_flag == NULL) {
        return;
    }
    (void)osEventFlagsSet(s_print_flag, APP_EVENT_STATUS_PRINT);
}
