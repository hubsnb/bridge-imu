//
// Created by Administrator on 2024/7/20.
//
#include "ins.hpp"
#include "Core/Inc/spi.h"
#include "Bsp/DWT/DWT.h"
#include <string>
#include <sys/stat.h>
#include "Middlewares/Device/BMI088/BMI088driver.h"
#include "config.h"
#include "Application/Filter/Kalman/kalman.h"
#include "Application/Filter/QAHRS/qeskf.hpp"
const float Pif =  3.1415927;
uint32_t DWT_IMU_DT;
static float glv_sample_cnt = 0;

struct INS glv_ins;
struct User_IMU_t Imu_info;
float eye[9] =
        {1,0,0,\
         0,1,0,\
         0,0,1};
arm_matrix_instance_f32 seye3 = {3,3,eye};

/**
 * @note 这里广泛使用全局变量，有些抽象的变量修改请注意
 * @param calibrate 1 表示标0飘
 */
void Ins_Init(uint8_t calibrate)
{
        BMI088_Init(&hspi1, calibrate);
        glv_ins.ERaxis.Q_btoi[0] = {1};
        arm_mat_init_f32(&glv_ins.ERaxis.C.sC_btob_minus,3,3,glv_ins.ERaxis.C.C_btob_minus);
        arm_mat_init_f32(&glv_ins.ERaxis.C.sC_btoi,3,3,glv_ins.ERaxis.C.C_btoi);
        arm_mat_init_f32(&glv_ins.ERaxis.C.sC_n_minuston,3,3,glv_ins.ERaxis.C.C_n_minuston);
        arm_mat_init_f32(&glv_ins.ERaxis.sphi_anti,3,3,glv_ins.ERaxis.phi_anti);

        arm_mat_init_f32(&glv_ins.v.sv_temp,3,1,glv_ins.v.v_temp);
        arm_mat_init_f32(&glv_ins.v.sv_n_m,3,1,glv_ins.v.v_n_m);
        arm_mat_init_f32(&glv_ins.v.sv_n_sf,3,1,glv_ins.v.v_n_sf);
        arm_mat_init_f32(&glv_ins.v.sg_n,3,1,glv_ins.v.g_n);

        glv_ins.ERaxis.C.sC_btoi.pData[0] = 1;
        glv_ins.ERaxis.C.sC_btoi.pData[3] = 1;
        glv_ins.ERaxis.C.sC_btoi.pData[6] = 1;

        glv_ins.v.g_n[2] = (-BMI088.gNorm);
        Imu_info.zero_offset[0] = GxOFFSET;
        Imu_info.zero_offset[1] = GyOFFSET;
        Imu_info.zero_offset[2] = GzOFFSET;

}

/**
 * @details 注意有没有init
 */
void Ins_Task()
{
        glv_ins.ERaxis.dt = getDuration(DWT_CYCCNT(),DWT_IMU_DT);
        DWT_IMU_DT = DWT_CYCCNT();
        Imu_info.Gyro[0] -= Imu_info.zero_offset[0];
        Imu_info.Gyro[1] -= Imu_info.zero_offset[1];
        Imu_info.Gyro[2] -= Imu_info.zero_offset[2];
        Multi_SubSampling(&glv_ins.ERaxis ,&Imu_info, glv_ins.ERaxis.dt,Quaternion,SimpleandLast);
        Velocity_Update(&glv_ins.v,&glv_ins.ERaxis,&Imu_info, glv_ins.ERaxis.dt);
        QESKF_Updata(glv_ins.ERaxis.Q_btoi,&Imu_info,glv_ins.ERaxis.dt);


}

void Multi_SubSampling(struct EqualRotationAxis * eraxis ,const User_IMU_t * Imu,const float dt ,Cal_Method CM, enum Sample sample_state)
{
        dstatic(float,cross_result[3]);

        switch (sample_state) {

                case SimpleandLast:
                        User_Read_Imu();
                        eraxis->theta_sample[0] = dt * Imu->Gyro[0];
                        eraxis->theta_sample[1] = dt * Imu->Gyro[1];
                        eraxis->theta_sample[2] = dt * Imu->Gyro[2];
                        for(int i=0; i<3; i++){eraxis->theta_det[i] = eraxis->theta_sample[i];}

                        crossProduct(eraxis->theta_sample_last,eraxis->theta_sample ,cross_result);
                        eraxis->phi[0] = eraxis->theta_sample[0] + 1.0f/12.0f * cross_result[0];
                        eraxis->phi[1] = eraxis->theta_sample[1] + 1.0f/12.0f * cross_result[1];
                        eraxis->phi[2] = eraxis->theta_sample[2] + 1.0f/12.0f * cross_result[2];
                         for(int i=0;i < 3;i++) {eraxis->theta_sample_last[i] = eraxis->theta_sample[i];}
                        rtoqua(eraxis->phi, eraxis->Q_btob_minus);
                        qmul(eraxis->Q_btoi, eraxis->Q_btob_minus,eraxis->Q_btoi);
                        // rtoeular(eraxis->phi, eraxis->Q_btoi, glv_ins.yaw_pitch_roll,&eraxis->C, CM);
                         break;
                case ConicalErrorTwo:

                        User_Read_Imu();
                        if(glv_sample_cnt == 0) {
                                eraxis->theta_sample[0] = dt * Imu->Gyro[0];
                                eraxis->theta_sample[1] = dt * Imu->Gyro[1];
                                eraxis->theta_sample[2] = dt * Imu->Gyro[2];
                                memcpy(eraxis->theta_det,eraxis->theta_sample,3);
                                glv_sample_cnt++;
                        }
                        else if(glv_sample_cnt == 1) {
                                eraxis->theta_sample_second[0] = dt * Imu->Gyro[0];
                                eraxis->theta_sample_second[1] = dt * Imu->Gyro[1];
                                eraxis->theta_sample_second[2] = dt * Imu->Gyro[2];
                                memcpy(eraxis->theta_det,eraxis->theta_sample_second,3);
                                glv_sample_cnt--;
                        }

                        crossProduct(eraxis->theta_sample,eraxis->theta_sample_second ,cross_result);
                        eraxis->phi[0] = eraxis->theta_sample[0] + eraxis->theta_sample_second[0] + 2.0f/3.0f * cross_result[0];
                        eraxis->phi[1] = eraxis->theta_sample[1] + eraxis->theta_sample_second[1] + 2.0f/3.0f * cross_result[1];
                        eraxis->phi[2] = eraxis->theta_sample[2] + eraxis->theta_sample_second[2] + 2.0f/3.0f * cross_result[2];
                        rtoeular(eraxis->phi, eraxis->Q_btoi, glv_ins.yaw_pitch_roll,&eraxis->C, CM);



                        break;



                default:
                        break;
        }




}

/**
 *
 * @param v 速度相关结构体
 * @param eraxis 等效旋转矢量，用里面的四元数和方向余弦矩阵
 * @param Imu imu测量数据
 * @param dt 运行时间
 * @note 屁用没有，写着玩的
 */
void Velocity_Update(struct Velocity * v, struct EqualRotationAxis * eraxis, const User_IMU_t * Imu,const float dt)
{
        for(int i=0; i<3; i++){v->v_det[i] = Imu->Accel[i] * dt;}
        crossProduct(eraxis->theta_det,v->v_det,v->v_temp);
        for(int i=0; i<3; i++){v->v_temp[i] = 1.0f/2.0f * v->v_temp[i] + v->v_det[i];}
        quatoc(eraxis->Q_btoi, eraxis->C.C_btoi);
        arm_mat_mult_f32(&eraxis->C.sC_btoi, &v->sv_temp, &v->sv_n_sf);
        arm_mat_scale_f32(&v->sg_n, dt, &v->sv_temp);
        arm_mat_add_f32(&v->sv_n_sf, &v->sv_temp, &v->sv_temp);
        arm_mat_add_f32(&v->sv_temp, &v->sv_n_m, &v->sv_n_m);
}

/**
 *
 * @param Q 输入四元数
 * @param C 输出方向余弦矩阵
 */
void quatoc(float Q[4], float C[9]) {
        float q00 = Q[0]*Q[0];
        float q11 = Q[1]*Q[1];
        float q22 = Q[2]*Q[2];
        float q33 = Q[3]*Q[3];
        float q01 = Q[0]*Q[1];
        float q02 = Q[0]*Q[2];
        float q03 = Q[0]*Q[3];
        float q12 = Q[1]*Q[2];
        float q13 = Q[1]*Q[3];
        float q23 = Q[2]*Q[3];

        C[0] = q00 - q11 - q22 - q33;
        C[1] = 2.0f * (q12 - q03);
        C[2] = 2.0f * (q13 + q02);
        C[3] = 2.0f * (q12 + q03);
        C[4] = q00 - q11 + q22 - q33;
        C[5] = 2.0f * (q23 - q01);
        C[6] = 2.0f * (q13 - q02);
        C[7] = 2.0f * (q23 + q01);
        C[8] = q00 - q11 - q22 + q33;

}


/**
 * 将等效旋转矢量转换为欧拉角
 * @param phi 采样等效选择矢量
 * @param Q_btoi 变换四元数
 * @param angle imu角度
 * @param C 方向余弦矩阵大礼包
 * @param Cm 计算方法
 */
void rtoeular(float phi[3], float Q_btoi[4], float angle[3], struct CosineMatrix *C, enum Cal_Method Cm)
{
        switch (Cm) {
                case Quaternion:
                        rtoqua(phi,Q_btoi);
                        quatroeular(Q_btoi, angle);
                break;

                case DirectionCosineMatrix:
                        rtoc(phi, &C->sC_btob_minus);

                        //arm_mat_mult_f32(&seye3,&C->sC_btoi,&C->sC_btoi);
                        arm_mat_mult_f32(&C->sC_btoi,&C->sC_btob_minus,&C->sC_btoi);
                        atttoeular(C->C_btoi,angle);
                break;
        }
}


/**
 *
 * @param phi3[3] 传入所需phi
 * @param C_btoi 传出方向余弦阵
 */
void rtoc(const float phi3[3], arm_matrix_instance_f32 * C_btoi)
{
        //
        float Q[4];
        float C[9];
        rtoqua(phi3,Q);
        quatoc(Q,C);
        memcpy(C_btoi->pData,C,36);

        // static float temp1[9],temp2[9];
        // static arm_matrix_instance_f32 stemp1 = {3,3,temp1};
        // static arm_matrix_instance_f32 stemp2 = {3,3,temp2};
        // static float phi3anti[9];
        // static arm_matrix_instance_f32 sphi3anti = {3,3,phi3anti};
        // static float abs_phi = sqrtf(\
        //           phi3[0] * phi3[0]\
        //         + phi3[1] * phi3[1]\
        //         + phi3[2] * phi3[2]);
        // arrayToAntiSymmetricMatrix(phi3,phi3anti);
        // arm_mat_scale_f32(&sphi3anti, sinf(abs_phi)/abs_phi, &stemp1);
        // arm_mat_mult_f32(&sphi3anti, &sphi3anti,&stemp2);
        // arm_mat_scale_f32(&stemp2, (1 - cosf(abs_phi)) / (abs_phi * abs_phi), &stemp2);
        // arm_mat_add_f32(&seye3,&stemp1,&stemp1);
        // arm_mat_add_f32(&stemp2,&stemp1,C_btoi);

}

/**
 *
 * @param array 传入的三维矢量
 * @param matrix 传出的反对称阵
 */
void arrayToAntiSymmetricMatrix(const float *array, float *matrix) {
        matrix[0] = 0;
        matrix[1] = -array[2];
        matrix[2] = array[1];
        matrix[3] = array[2];
        matrix[4] = 0;
        matrix[5] = -array[0];
        matrix[6] = -array[1];
        matrix[7] = array[0];
        matrix[8] = 0;
}





/**
 *
 * @param Q_btoi 传入四元数，imu里一般是有旋转矩阵内涵
 * @return angle 按名称顺序返回三个角度
 * @note 代码三个角度顺序有问题还没有修真
 */
void quatroeular(const float *Q_btoi,float eular_angle[3])
{

        dstatic(float,qqqq);
        qqqq = Q_btoi[2] * Q_btoi[3] + Q_btoi[0] * Q_btoi[1];
        if(std::fabs(2.0f*qqqq) <= 0.99) {
                eular_angle[0] = asinf(2.0f*(qqqq)) * 180.0f / Pif;
                eular_angle[1] = -1.0f*atan2(2.0f*(Q_btoi[1] * Q_btoi[3] - Q_btoi[0] * Q_btoi[2]), \
                        Q_btoi[0] * Q_btoi[0]\
                        - Q_btoi[1] * Q_btoi[1]\
                        - Q_btoi[2] * Q_btoi[2]\
                        + Q_btoi[3] * Q_btoi[3]) * 180.0f / Pif;
                eular_angle[2] = -1.0f*atan2(2.0f*(Q_btoi[1] * Q_btoi[2] - Q_btoi[0] * Q_btoi[3]), \
                        Q_btoi[0] * Q_btoi[0]\
                        - Q_btoi[1] * Q_btoi[1]\
                        + Q_btoi[2] * Q_btoi[2]\
                        - Q_btoi[3] * Q_btoi[3]) * 180.0f / Pif;
        }
        else if(std::fabs(2.0f*qqqq) > 0.99) {
                eular_angle[0] = asinf(2.0f*(qqqq)) * 180.0f / Pif;
                eular_angle[1] = 1.0f*atan2(2.0f*(Q_btoi[1] * Q_btoi[3] + Q_btoi[0] * Q_btoi[2]), \
                          Q_btoi[0] * Q_btoi[0]\
                        + Q_btoi[1] * Q_btoi[1]\
                        - Q_btoi[2] * Q_btoi[2]\
                        - Q_btoi[3] * Q_btoi[3]) * 180.0f / Pif;
                //eular_angle[1] = asinf(Q_btoi[1] * Q_btoi[3] + Q_btoi[0] * Q_btoi[2]);
                eular_angle[2] = 0;

        }

}

/**
 *
 * @param C_btoi 传入姿态阵
 * @param angle3 传出欧拉角
 */
void atttoeular(const float* C_btoi,float angle3[3])
{
        angle3[0] = asinf(C_btoi[7]) * 180.f/Pif;
        angle3[1] = -atan2f(C_btoi[6], C_btoi[8])* 180.f/Pif;
        angle3[2] = -atan2f(C_btoi[1], C_btoi[4])* 180.f/Pif;

}


/**
 *
 * @param phi3 等效旋转矢量，长度代表旋转角度，方向代表旋转轴
 * @param Q_btoi 转换出的四元数，传入是因为要用到上一次的
 * @note
 */
void rtoqua(const float phi3[3],float * Q_btoi)
{
        //等效选择轴转换为四元数
        dstatic(float, abs_phi);
        abs_phi = sqrtf(phi3[0] * phi3[0] + phi3[1] * phi3[1] + phi3[2] * phi3[2]);

        static float Q_btob_minus[4];
        Q_btoi[0] = cosf(abs_phi/2.0f);
        Q_btoi[1] = phi3[0] / abs_phi * sinf(abs_phi/2.0f);
        Q_btoi[2] = phi3[1] / abs_phi * sinf(abs_phi/2.0f);
        Q_btoi[3] = phi3[2] / abs_phi * sinf(abs_phi/2.0f);

        Q_normal(Q_btoi);
}

void Q_normal(float Q_btoi[4])
{
        dstatic(float, abs);
        abs = sqrtf(Q_btoi[0] * Q_btoi[0] + Q_btoi[1] * Q_btoi[1] + Q_btoi[2] * Q_btoi[2] + Q_btoi[3] * Q_btoi[3]);
        for(int i=0;i<4;i++){Q_btoi[i] = Q_btoi[i]/abs;}

}

/**
 *
 * @param p 左四元数
 * @param q 右四元数
 * @param result 乘后的四元数
 */
void qmul(const float p[4], const float q[4], float result[4]) {
        result[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
        result[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
        result[2] = p[0] * q[2] + p[2] * q[0] + p[3] * q[1] - p[1] * q[3];
        result[3] = p[0] * q[3] + p[3] * q[0] + p[1] * q[2] - p[2] * q[1];
}

/**
 * @note 接口函数，方便移植打包的，虽然写到一半才想起这个事情
 */
void User_Read_Imu()
{

        BMI088_Read(&BMI088);
        //这个地方赋值就不共享地址了，感觉有坑
        for(int i=0;i<3;i++) {
                Imu_info.Accel[i] = BMI088.Accel[i];
                Imu_info.Gyro[i] = BMI088.Gyro[i];
        }
}


void crossProduct(const float u[3], const float v[3], float result[3]) {
        result[0] = u[1] * v[2] - u[2] * v[1];
        result[1] = u[2] * v[0] - u[0] * v[2];
        result[2] = u[0] * v[1] - u[1] * v[0];
}
