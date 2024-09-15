//
// Created by ZJZ0 on 24-7-14.
//

#pragma once
#ifndef BRIDGE_BSP_LOG_H
#define BRIDGE_BSP_LOG_H

#include "main.h"
#include "Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.h"
#include "Middlewares/Third_Party/SEGGER/Config/SEGGER_RTT_Conf.h"

#define BUFFER_INDEX 0

#define LOG_LEVEL_DBG 0
#define LOG_LEVEL_INF 1
#define LOG_LEVEL_WRN 2
#define LOG_LEVEL_ERR 3
#define LOG_LEVEL_OFF 4

// define LOG_ACTIVE_LEVEL to turn off low log level in compile time
#ifndef LOG_ACTIVE_LEVEL
#define LOG_ACTIVE_LEVEL LOG_LEVEL_DBG
#endif

#define LOG(type, color, format, ...)                       \
    SEGGER_RTT_printf(BUFFER_INDEX,"  %s%s" format"\r\n%s", \
                      color,                                \
                      type,                                 \
                      ##__VA_ARGS__,                        \
                      RTT_CTRL_RESET)


#if LOG_ACTIVE_LEVEL <= LOG_LEVEL_DBG
#define LOG_DEBUG(format, ...) LOG("[DEBUG]", RTT_CTRL_TEXT_BRIGHT_WHITE, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(format, ...) (void)0;
#endif

#if LOG_ACTIVE_LEVEL <= LOG_LEVEL_INF
#define LOG_INFO(format, ...) LOG("[INFO]", RTT_CTRL_TEXT_BRIGHT_WHITE, format, ##__VA_ARGS__)
#else
#define LOG_INFO(format, ...) (void)0;
#endif
#if LOG_ACTIVE_LEVEL <= LOG_LEVEL_WRN
#define LOG_WARN(format, ...) LOG("[WARN]", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#else
#define LOG_WARN(format, ...) (void)0;
#endif
#if LOG_ACTIVE_LEVEL <= LOG_LEVEL_ERR
#define LOG_ERROR(format, ...) LOG("[ERROR]", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#else
#define LOG_ERROR(format, ...) (void)0;
#endif

#ifdef __cplusplus
extern "C" {
#endif

void BSP_LogInit(void);

#ifdef __cplusplus
}
#endif

#endif // BRIDGE_BSP_LOG_H
