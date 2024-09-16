//
// Created by Administrator on 2024/9/15.
//

#include "kalman.h"
#include <stdlib.h>
void Kalman_Init(Kalman_t *kf, const uint8_t LenXFPQ, const uint8_t LenZHR,float * Q,float *R)
{
        //分配内存
        const size_t sizeof_float = sizeof(float);
        const size_t sizex = LenXFPQ * sizeof_float;
        const size_t sizexx = LenXFPQ * LenXFPQ * sizeof_float;
        const size_t sizexz = LenXFPQ * LenZHR * sizeof_float;
        const size_t sizezz = LenZHR * LenZHR * sizeof_float;
        const size_t sizez = LenZHR * sizeof_float;
        kf->Q = (float *)malloc(sizexx);
        memset(kf->Q,0,sizexx);

        kf->R = (float *)malloc(sizezz);
        memset(kf->R,0,sizezz);

        kf->H = (float *)malloc(sizexz);
        memset(kf->H,0,sizexz);

        kf->H_t = (float *)malloc(sizexz);
        memset(kf->H_t,0,sizexz);

        kf->P = (float *)malloc(sizexx);
        memset(kf->P,0,sizexx);

        kf->Fk = (float *)malloc(sizexx);
        memset(kf->Fk,0,sizexx);

        kf->Fk_t = (float *)malloc(sizexx);
        memset(kf->Fk_t,0,sizexx);

        kf->Kk = (float *)malloc(sizexz);
        memset(kf->Kk,0,sizexz);

        kf->I = (float *)malloc(sizexx);
        memset(kf->I,0,sizexx);

        kf->X_hat = (float *)malloc(sizex);
        memset(kf->X_hat,0,sizex);

        kf->Xk = (float *)malloc(sizex);
        memset(kf->Xk,0,sizex);

        kf->P_xz = (float *)malloc(sizexz);
        memset(kf->P_xz,0,sizexz);

        kf->P_zz = (float *)malloc(sizezz);
        memset(kf->P_zz,0,sizezz);

        kf->P_zz_inv = (float *)malloc(sizezz);
        memset(kf->P_zz_inv,0,sizezz);

        kf->Z = (float *)malloc(sizez);
        memset(kf->Z,0,sizez);

        kf->tempKH = (float *)malloc(sizexx);
        memset(kf->tempKH,0,sizexx);

        kf->tempHP = (float *)malloc(sizexz);
        memset(kf->tempHP,0,sizexz);

        kf->tempHX = (float *)malloc(sizexz);
        memset(kf->tempHX,0,sizexz);

        kf->hz = (float *)malloc(sizez);
        memset(kf->hz,0,sizez);



        //预测
        arm_mat_init_f32(&kf->sXk, LenXFPQ, 1, kf->Xk);
        arm_mat_init_f32(&kf->sX_hat, LenXFPQ, 1, kf->X_hat);
        arm_mat_init_f32(&kf->sFk, LenXFPQ, LenXFPQ, kf->Fk);
        //获得Pk/k-1
        arm_mat_init_f32(&kf->sFk_t, LenXFPQ, LenXFPQ, kf->Fk_t);
        arm_mat_init_f32(&kf->sQ, LenXFPQ, LenXFPQ, kf->Q);
        arm_mat_init_f32(&kf->sP, LenXFPQ, LenXFPQ, kf->P);
        arm_mat_init_f32(&kf->sH, LenZHR, LenXFPQ, kf->H);
        arm_mat_init_f32(&kf->stempHP, LenZHR, LenXFPQ, kf->tempHP);
        //Kk
        arm_mat_init_f32(&kf->sH_t, LenXFPQ, LenZHR, kf->H_t);
        arm_mat_init_f32(&kf->sR, LenZHR, LenZHR, kf->R);
        arm_mat_init_f32(&kf->sKk, LenXFPQ, LenZHR, kf->Kk);
        arm_mat_init_f32(&kf->sP_xz, LenXFPQ, LenZHR, kf->P_xz);
        arm_mat_init_f32(&kf->sP_zz, LenZHR, LenZHR, kf->P_zz);
        arm_mat_init_f32(&kf->sP_zz_inv, LenZHR, LenZHR, kf->P_zz_inv);
        arm_mat_init_f32(&kf->stempHX, LenZHR, 1, kf->tempHX);
        arm_mat_init_f32(&kf->shz, LenZHR, 1, kf->hz);
        arm_mat_init_f32(&kf->stempKH, LenXFPQ, LenXFPQ, kf->tempKH);
        arm_mat_init_f32(&kf->sI, LenXFPQ, LenXFPQ, kf->I);
        arm_mat_init_f32(&kf->sZ, LenZHR, 1, kf->Z);
        memcpy(kf->Q,Q,sizexx);
        memcpy(kf->R,R,sizezz);

}


/**
 * @note 更新kalman，取余参数矩阵需要提前赋值
 * @param kf kalman需要的全部状态
 */
void Kalman_Updata(Kalman_t *kf)
{
        static arm_status kf_state;
        //预测
        kf_state = arm_mat_mult_f32(&kf->sFk,&kf->sXk, &kf->sX_hat);
        //获得Pk/k-1
        kf_state = arm_mat_trans_f32(&kf->sFk,&kf->sFk_t);
        kf_state = arm_mat_mult_f32(&kf->sFk, &kf->sP, &kf->sP);
        kf_state = arm_mat_mult_f32(&kf->sP, &kf->sFk_t, &kf->sP);
        kf_state = arm_mat_add_f32(&kf->sP, &kf->sQ, &kf->sP);

        //获得Kk
        //1获得Pxz
        kf_state = arm_mat_trans_f32(&kf->sH, &kf->sH_t);
        kf_state = arm_mat_mult_f32(&kf->sP, &kf->sH_t, &kf->sP_xz);

        //2获得Pzz-1
        kf_state = arm_mat_mult_f32(&kf->sH,&kf->sP, &kf->stempHP);
        kf_state = arm_mat_mult_f32(&kf->stempHP, &kf->sH_t, &kf->sP_zz);
        kf_state = arm_mat_add_f32(&kf->sP_zz, &kf->sR, &kf->sP_zz);
        kf_state = arm_mat_inverse_f32(&kf->sP_zz, &kf->sP_zz_inv);

        kf_state = arm_mat_mult_f32(&kf->sP_xz, &kf->sP_zz_inv, &kf->sKk);

        //获得Xkw
        // kf_state = arm_mat_mult_f32(&kf->sH, &kf->sX_hat, &kf->stempHX);
        kf_state = arm_mat_sub_f32(&kf->sZ, &kf->shz, &kf->stempHX);
        kf_state = arm_mat_mult_f32(&kf->sKk, &kf->stempHX, &kf->sXk);
        kf_state = arm_mat_add_f32(&kf->sX_hat, &kf->sXk, &kf->sXk);

        //跟新Pk
        kf_state = arm_mat_mult_f32(&kf->sKk, &kf->sH,&kf->stempKH);
        kf_state = arm_mat_sub_f32(&kf->sI, &kf->stempKH, &kf->stempKH);
        kf_state = arm_mat_mult_f32(&kf->stempKH, &kf->sP, &kf->sP);


}