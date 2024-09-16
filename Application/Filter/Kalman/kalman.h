//
// Created by Administrator on 2024/9/15.
//

#ifndef KALMAN_H
#define KALMAN_H
#include "kalman.hpp"
#ifdef __cplusplus
extern "C" {

#endif

        void Kalman_Init(Kalman_t *kf, const uint8_t LenXFPQ, const uint8_t LenZHR,float * Q,float *R);
        void Kalman_Updata(Kalman_t *kf);



#ifdef __cplusplus
}

#endif


#endif //KALMAN_H

