//
// Created by Administrator on 2024/9/15.
//

#ifndef QESKF_HPP
#define QESKF_HPP

#ifdef __cplusplus
extern "C" {
#include "qeskf.h"
#endif




        void QESKF_Updata(float quat[4],User_IMU_t *imu,float dt);


#ifdef __cplusplus
}

#endif




#endif //QESKF_HPP
