/**
 * @file app_log.c
 * @brief 异步日志系统实现
 */
#include "app_log.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define LOG_QUEUE_SIZE 16
#define LOG_MSG_MAX_LEN 128

typedef struct {
    log_level_t level;
    char message[LOG_MSG_MAX_LEN];
} log_message_t;

static osMessageQueueId_t log_queue = NULL;
static osThreadId_t log_task_handle = NULL;

/* 日志级别字符串 */
static const char* level_strings[] = {
    "[DEBUG]",
    "[INFO] ",
    "[WARN] ",
    "[ERROR]"
};

/* 日志输出任务 */
static void log_task(void *argument)
{
    log_message_t msg;
    
    for (;;) {
        if (osMessageQueueGet(log_queue, &msg, NULL, osWaitForever) == osOK) {
            printf("%s %s\r\n", level_strings[msg.level], msg.message);
        }
    }
}

/* 初始化日志系统 */
void app_log_init(void)
{
    /* 创建消息队列 */
    log_queue = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(log_message_t), NULL);
    
    /* 创建日志任务（低优先级，避免影响实时任务） */
    const osThreadAttr_t log_task_attr = {
        .name = "LogTask",
        .stack_size = 512 * 4,
        .priority = osPriorityBelowNormal,
    };
    log_task_handle = osThreadNew(log_task, NULL, &log_task_attr);
}

/* 通用日志函数 */
static void log_message(log_level_t level, const char *fmt, va_list args)
{
    if (log_queue == NULL) {
        return; /* 日志系统未初始化 */
    }
    
    log_message_t msg;
    msg.level = level;
    vsnprintf(msg.message, LOG_MSG_MAX_LEN, fmt, args);
    
    /* 非阻塞发送，如果队列满则丢弃 */
    osMessageQueuePut(log_queue, &msg, 0, 0);
}

void log_debug(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_message(LOG_LEVEL_DEBUG, fmt, args);
    va_end(args);
}

void log_info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_message(LOG_LEVEL_INFO, fmt, args);
    va_end(args);
}

void log_warn(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_message(LOG_LEVEL_WARN, fmt, args);
    va_end(args);
}

void log_error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_message(LOG_LEVEL_ERROR, fmt, args);
    va_end(args);
}

/* 同步日志 - 立即输出 */
void log_sync(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\r\n");
}
