//
// Created by SuperChen on 2024/11/24.
//

#ifndef ADIS16467_TRY1_QUATERNIONEKF_H
#define ADIS16467_TRY1_QUATERNIONEKF_H
#include "stdio.h"
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // ��Ԫ������ֵ
    float GyroBias[3]; // ��������ƫ����ֵ

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // ��Ԫ�����¹�������
    float Q2; // ��������ƫ��������
    float R;  // ���ٶȼ���������

    float dt; // ��̬��������
    mat ChiSquare;
    float ChiSquare_Data[1];      // ���������⺯��
    float ChiSquareTestThreshold; // ����������ֵ
    float lambda;                 // ��������

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif //ADIS16467_TRY1_QUATERNIONEKF_H
