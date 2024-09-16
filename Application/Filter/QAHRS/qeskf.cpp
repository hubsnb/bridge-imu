//
// Created by Administrator on 2024/9/15.
//

#include "qeskf.hpp"
#include "Application/Filter/Kalman/kalman.h"
#include "Middlewares/Device/BMI088/BMI088driver.h"
#include  "Application/Sensor/INS/ins.h"
#include "Application/Sensor/INS/ins.hpp"
Kalman_t glv_qeskf;
float Q[36] = {
        0.001, 0,   0,   0,   0,   0,
        0,   0.001, 0,   0,   0,   0,
        0,   0,   0.001, 0,   0,   0,
        0,   0,   0,   0.0001, 0,   0,
        0,   0,   0,   0,   0.0001, 0,
        0,   0,   0,   0,   0,   0.0001
    };
float R[9] = {
        100, 0, 0,
        0, 100, 0,
        0, 0, 100
};

float I[36] = {
        0.1, 0,   0,   0,   0,   0,
        0,   0.1, 0,   0,   0,   0,
        0,   0,   0.1, 0,   0,   0,
        0,   0,   0,   0.1, 0,   0,
        0,   0,   0,   0,   0.1, 0,
        0,   0,   0,   0,   0,   0.1
    };
void QESKF_Init()
{
        //初始化Q R
        Kalman_Init(&glv_qeskf,6,3,Q,R);
        memcpy(glv_qeskf.I,I,sizeof(I));
        //初始化零偏


}

void QESKF_Updata(float quat[4],User_IMU_t *imu,float dt)
{
        static float gN = (gNORM);

        static float Cbton[9];
        static float acc[3];
        acc[0] = imu->Accel[0];
        acc[1] = imu->Accel[1];
        acc[2] = imu->Accel[2];
        static arm_matrix_instance_f32 sCbton;
        static arm_matrix_instance_f32 sacc;
        arm_mat_init_f32(&sCbton,3,3,Cbton);
        arm_mat_init_f32(&sacc,3,1,acc);
        //赋值预测，量测

        glv_qeskf.Fk[0] = 1;
        glv_qeskf.Fk[1] = 0;
        glv_qeskf.Fk[2] = 0;
        glv_qeskf.Fk[3] = -dt;
        glv_qeskf.Fk[4] = 0;
        glv_qeskf.Fk[5] = 0;

        glv_qeskf.Fk[6] = 0;
        glv_qeskf.Fk[7] = 1;
        glv_qeskf.Fk[8] = 0;
        glv_qeskf.Fk[9] = 0;
        glv_qeskf.Fk[10] = -dt;
        glv_qeskf.Fk[11] = 0;

        glv_qeskf.Fk[12] = 0;
        glv_qeskf.Fk[13] = 0;
        glv_qeskf.Fk[14] = 1;
        glv_qeskf.Fk[15] = 0;
        glv_qeskf.Fk[16] = 0;
        glv_qeskf.Fk[17] = -dt;

        glv_qeskf.Fk[18] = 0;
        glv_qeskf.Fk[19] = 0;
        glv_qeskf.Fk[20] = 0;
        glv_qeskf.Fk[21] = 1;
        glv_qeskf.Fk[22] = 0;
        glv_qeskf.Fk[23] = 0;

        glv_qeskf.Fk[24] = 0;
        glv_qeskf.Fk[25] = 0;
        glv_qeskf.Fk[26] = 0;
        glv_qeskf.Fk[27] = 0;
        glv_qeskf.Fk[28] = 1;
        glv_qeskf.Fk[29] = 0;

        glv_qeskf.Fk[30] = 0;
        glv_qeskf.Fk[31] = 0;
        glv_qeskf.Fk[32] = 0;
        glv_qeskf.Fk[33] = 0;
        glv_qeskf.Fk[34] = 0;
        glv_qeskf.Fk[35] = 1;

        float qww = quat[0] * quat[0];
        float qwx = quat[0] * quat[1];
        float qwy = quat[0] * quat[2];
        float qwz = quat[0] * quat[3];
        float qxx = quat[1] * quat[1];
        float qxy = quat[1] * quat[2];
        float qxz = quat[1] * quat[3];
        float qyy = quat[2] * quat[2];
        float qyz = quat[2] * quat[3];
        float qzz = quat[3] * quat[3];
        glv_qeskf.H[0] = gN * (qxy - qxx);
        glv_qeskf.H[1] = gN * (qww - qxx - qxy + qzz);
        glv_qeskf.H[2] = gN * (-2.0f * qwx - qxz - qyz);
        glv_qeskf.H[3] = 0;
        glv_qeskf.H[4] = 0;
        glv_qeskf.H[5] = 0;

        glv_qeskf.H[6] = gN * (-1.0f * qww + qxx + qyy - qzz);
        glv_qeskf.H[7] = 0;
        glv_qeskf.H[8] = gN * (2.0f * qxz - 2.0f * qwy);
        glv_qeskf.H[9] = 0;
        glv_qeskf.H[10] = 0;
        glv_qeskf.H[11] = 0;

        glv_qeskf.H[12] = gN * (2.0f * qwx + 2.0f * qyz);
        glv_qeskf.H[13] = gN * (2.0f * qwy - qxz);
        glv_qeskf.H[14] = 0;
        glv_qeskf.H[15] = 0;
        glv_qeskf.H[16] = 0;
        glv_qeskf.H[17] = 0;

        //假定每次滤波后误差为0
        for(int i = 0;i < 6;i++) {
                glv_qeskf.Xk[i] = 0;
        }
        quatoc(quat,Cbton);
        arm_mat_mult_f32(&sCbton,&sacc,&sacc);
        glv_qeskf.hz[0] = acc[0];
        glv_qeskf.hz[1] = acc[1];
        glv_qeskf.hz[2] = -gN;

        glv_qeskf.Z[0] = 0;
        glv_qeskf.Z[1] = 0;
        glv_qeskf.Z[2] = -gN;
        Kalman_Updata(&glv_qeskf);
        // DEBUG
        static float P[36];
        static float Kk[18];
        static float Pxz[18];
        static float Pzz_inv[9];
        static float tempKH[36];

        memcpy(P,glv_qeskf.P,sizeof(P));
        memcpy(Kk,glv_qeskf.Kk,sizeof(Kk));
        memcpy(Pxz,glv_qeskf.P_xz,sizeof(Pxz));
        memcpy(Pzz_inv,glv_qeskf.P_zz_inv,sizeof(Pzz_inv));
        memcpy(tempKH,glv_qeskf.tempKH,sizeof(tempKH));
        //
        static float error[6];
        memcpy(error,glv_qeskf.Xk,24);
        float Q_error[4];
        float theta_error[3];
        float zero_error[3];
        theta_error[0] = error[0];
        theta_error[1] = error[1];
        theta_error[2] = 0;
        imu->zero_offset[0] = error[0];
        imu->zero_offset[1] = error[1];
        imu->zero_offset[2] = error[2];

        rtoqua(theta_error,Q_error);
        qmul(quat,Q_error,quat);
        Q_normal(quat);
        static float angle333[3];
        quatroeular(quat,angle333);
        //补偿
        //误差清零

}
