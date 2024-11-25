//
// Created by SuperChen on 2024/11/21.
//
/*
* Change Logs:
* Date            Author          Notes
* 2023-09-15      ChuShicheng     first version
*/

#include "ins_task.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

static ins_t ins;
static imu_param_t imu_param;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static void ins_init(void);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3]);
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
static void InitQuaternion(float *init_q4);




/**
 * @brief 初始化 ins 解算系统
 *
 */
static void ins_init(void)
{
    imu_param.scale[X] = 1;
    imu_param.scale[Y] = 1;
    imu_param.scale[Z] = 1;
    imu_param.yaw = 0;
    imu_param.pitch = 0;
    imu_param.roll = 0;
    imu_param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    // noise of accel is relatively big and of high freq,thus lpf is used
    ins.accel_lpf = 0.0085;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        imu_ops.accel_read(acc_init);
        dwt_delay_s(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

