//
// Created by Administrator on 2024/9/15.
//

#include "kalman.hpp"

void Kalman_Init(Kalman_t *kf) {

}

void Kalman_Updata(Kalman_t *kf, arm_matrix_instance_f32 *sx_minus, arm_matrix_instance_f32 * sP_minus)
{
        //预测
        arm_mat_mult_f32(&kf->sFk,sx_minus,&kf->sXk);
        //获得Pk/k-1
        arm_mat_trans_f32(&kf->sFk,&kf->sFk_t);
        arm_mat_mult_f32(&kf->sFk, sP_minus, &kf->sP);
        arm_mat_mult_f32(&kf->sP, &kf->sFk_t, &kf->sP);
        arm_mat_add_f32(&kf->sP, &kf->sQ, &kf->sP);

        //获得Kk
        //1获得Pxz
        arm_mat_trans_f32(&kf->sH, &kf->sH_t);
        arm_mat_mult_f32(&kf->sP, &kf->sH_t, &kf->sP_xz);

        //2获得Pzz-1
        arm_mat_mult_f32(&kf->sH,&kf->sP, &kf->sP_zz);
        arm_mat_mult_f32(&kf->sP_zz, &kf->sH_t, &kf->sP_zz);
        arm_mat_add_f32(&kf->sP_zz, &kf->sR, &kf->sP_zz);
        arm_mat_inverse_f32(&kf->sP_zz, &kf->sP_zz_inv);

        arm_mat_m
}