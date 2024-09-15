//
// Created by ZJZ0 on 24-7-14.
//

#pragma once
#ifndef BRIDGE_APPLICATION_SYSTEM_CLOCK_H
#define BRIDGE_APPLICATION_SYSTEM_CLOCK_H

#include "main.h"
#include "Bsp/DWT/DWT.h"
#include "Bsp/Log/Log.h"

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Init(void);
void SystemClock_Task(void);

#ifdef __cplusplus
}
#endif



#endif // BRIDGE_APPLICATION_SYSTEM_CLOCK_H
