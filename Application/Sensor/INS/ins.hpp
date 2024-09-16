//
// Created by Administrator on 2024/7/20.
//

#ifndef INS_HPP
#define INS_HPP




#include "Application/Sensor/INS/ins.h"
#include <string>
#include "Bsp/Log/Log.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include  "Middlewares/Third_Party/CMSIS-DSP/Include/arm_math.h"
enum Sample {
        ConicalErrorTwo = 3,
        SimpleandLast,
};
enum Cal_Method {
        Quaternion = 100,
        DirectionCosineMatrix,
};

        struct CosineMatrix {
                float C_btob_minus[9];
                float C_btoi[9];
                float C_n_minuston[9];
                arm_matrix_instance_f32 sC_btob_minus;
                arm_matrix_instance_f32 sC_btoi;
                arm_matrix_instance_f32 sC_n_minuston;
        };
struct EqualRotationAxis {
        float dt;
        float theta_sample[3];
        float theta_sample_second[3];
        float theta_sample_last[3];
        float theta_det[3];
        CosineMatrix C;

        float Q_init[4];
        float Q_btob_minus[4];
        float Q_btoi[4];

        float phi[3];//等效旋转矢量
        float phi_anti[9];//反对称阵
        arm_matrix_instance_f32 sphi_anti;


        Sample sample_state;


};

struct Velocity {
        float v_det[3];
        float v_n_sf[3];
        float g_n[3];
        float v_n_m[3];

        float v_temp[3];
        arm_matrix_instance_f32 sv_temp;
        arm_matrix_instance_f32 sv_n_m;
        arm_matrix_instance_f32 sg_n;
        arm_matrix_instance_f32 sv_n_sf;

};


struct INS {
        struct Velocity v;
        struct EqualRotationAxis ERaxis;
        float yaw_pitch_roll[3];
};


void quatroeular(const float *Q_btoi,float eular_angle[3]);
void quatoc(float Q[4], float C[9]);
void qmul(const float p[4], const float q[4], float result[4]);
void rtoqua(const float phi3[3],float * Q_btoi);
void Multi_SubSampling(struct EqualRotationAxis * eraxis ,const User_IMU_t * Imu,const float dt ,Cal_Method CM, enum Sample sample_state);
void User_Read_Imu();
void Velocity_Update(struct Velocity * v, struct EqualRotationAxis * eraxis, const User_IMU_t * Imu,const float dt);
void crossProduct(const float u[3], const float v[3], float result[3]);
void arrayToAntiSymmetricMatrix(const float *array, float *matrix);
void rtoc(const float phi3[3], arm_matrix_instance_f32 *C_btoi);
void atttoeular(const float* C_btoi,float angle3[3]);
void rtoeular(float phi[3], float Q_btoi[4], float angle[3], struct CosineMatrix *C, enum Cal_Method Cm);
void Q_normal(float Q_btoi[4]);






#ifdef __cplusplus
}


#endif /* __cplusplus */


#endif //INS_HPP
