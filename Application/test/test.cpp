//
// Created by ZJZ0 on 24-7-14.
//

#include "test.hpp"
#include "Bsp/Log/Log.h"

void Test_Init(void)
{
        // Test test;
    LOG_INFO("%s", "Test INFO output");
}
void Test_Task(void)
{
    LOG_DEBUG("%s", "Test output");
}
