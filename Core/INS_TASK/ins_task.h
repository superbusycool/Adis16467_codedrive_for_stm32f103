//
// Created by SuperChen on 2024/11/21.
//

#ifndef ADIS16467_TRY1_INS_TASK_H
#define ADIS16467_TRY1_INS_TASK_H
#include "stdio.h"

extern struct imu_ops imu_ops;

typedef struct
{
    float gyro[3];  // ���ٶ�
    float accel[3]; // ���ٶ�
    // ����Ҫ���ӽ��ٶ�����
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
} attitude_t; // ���ս���õ��ĽǶ�,�Լ�yawת�����ܽǶ�(�����Ȧ����)

typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float motion_accel_b[3]; // ����������ٶ�
    float motion_accel_n[3]; // ����ϵ���ٶ�

    float accel_lpf; // ���ٶȵ�ͨ�˲�ϵ��

    // bodyframe�ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    // ���ٶ��ڻ���ϵ��XY����ļн�
    // float atanxz;
    // float atanyz;

    // IMU����ֵ
    float gyro[3];  // ���ٶ�
    float accel[3]; // ���ٶ�
    // λ��
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;

    uint8_t init;
} ins_t;

/* ����������װ���Ĳ��� */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float yaw;
    float pitch;
    float roll;
} imu_param_t;

#endif //ADIS16467_TRY1_INS_TASK_H
