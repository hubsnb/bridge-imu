//
// Created by ZJZ0 on 24-7-14.
//

#pragma once
#ifndef BRIDGE_BSP_DWT_H
#define BRIDGE_BSP_DWT_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t sec;
    int32_t nsec;
} DWT_Time;

/**
 * @brief 初始化DWT, 传入参数为CPU频率, 单位MHz
 *
 * @param CPU_Freq_mHz c板为168MHz, A板为180MHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

uint32_t DWT_CYCCNT(void);

/**
 * @brief 通过两次CYCCNT计算时间间隔
 * @warning 注意CYCCNT大小为uint32_t
 * @param end_CYCCNT
 * @param start_CYCCNT
 * @return
 */
float getDuration(uint32_t end_CYCCNT, uint32_t start_CYCCNT);
double getDuration64(uint32_t end_CYCCNT, uint32_t start_CYCCNT);

void DWT_Delay(float Delay);

#ifdef __cplusplus
}
#endif

#endif // BRIDGE_BSP_DWT_H
