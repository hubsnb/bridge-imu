//
// Created by Administrator on 2024/9/15.
//

#ifndef KALMAN_HPP
#define KALMAN_HPP

#ifdef __cplusplus
extern "C" {

#include  "Middlewares/Third_Party/CMSIS-DSP/Include/arm_math.h"
#endif /* __cplusplus */

        struct Kalman_t {
                float * Q;//预测噪声
                float * R;//量测噪声
                float * H;//量测矩阵
                float * H_t;
                float * P;//均方误差
                float * Fk;//预测矩阵
                float * Fk_t;
                float * Kk;//滤波增益
                float * I;//单位矩阵
                float * X_hat;
                float * Xk;
                float * P_xz;
                float * P_zz;
                float * P_zz_inv;
                float * Z;//量测测量值
                float * tempKH;
                float * tempHP;
                float * tempHX;
                float * hz;//H*X的实际版本
                arm_matrix_instance_f32 sQ,sR,sH,sH_t,sP,sFk,sFk_t,sKk,sI,sX_hat,sXk,sP_xz,sP_zz,sP_zz_inv,sZ,stempKH,stempHP,stempHX,shz;
        };

#ifdef __cplusplus
}


#endif /* __cplusplus */
#endif //KALMAN_HPP
