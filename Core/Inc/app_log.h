/**
 * @file app_log.h
 * @brief 异步日志系统 - 使用消息队列避免阻塞
 */
#ifndef __APP_LOG_H
#define __APP_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 日志级别 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

/* 初始化日志系统（创建队列和任务） */
void app_log_init(void);

/* 异步日志接口 - 不阻塞调用者 */
void log_debug(const char *fmt, ...);
void log_info(const char *fmt, ...);
void log_warn(const char *fmt, ...);
void log_error(const char *fmt, ...);

/* 同步日志接口 - 立即输出（仅用于紧急情况） */
void log_sync(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LOG_H */
