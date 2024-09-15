//
// Created by ZJZ0 on 24-7-14.
//

#include "DWT.h"

static float CPU_FREQ_Hz = 0;

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = (float)CPU_Freq_mHz * 1000000.0f;
}

uint32_t DWT_CYCCNT(void)
{
    volatile uint32_t CYCCNT = DWT->CYCCNT;
    return CYCCNT;
}

float getDuration(uint32_t end_CYCCNT, uint32_t start_CYCCNT)
{
    return (float)(end_CYCCNT - start_CYCCNT) / CPU_FREQ_Hz;
}

double getDuration64(uint32_t end_CYCCNT, uint32_t start_CYCCNT)
{
    return (double)(end_CYCCNT - start_CYCCNT) / CPU_FREQ_Hz;
}
void DWT_Delay(float Delay)
{
    const volatile uint32_t tickstart = DWT->CYCCNT;
    while((DWT->CYCCNT - tickstart) < (double)(Delay * CPU_FREQ_Hz))
    {}
}