//
// Created by Administrator on 2024/7/20.
//

#ifndef INS_H
#define INS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {

#endif
#include "Middlewares/Device/BMI088/BMI088driver.h"

void Ins_Init(uint8_t calibrate);
void Ins_Task();

#ifdef __cplusplus
}

#endif

#endif