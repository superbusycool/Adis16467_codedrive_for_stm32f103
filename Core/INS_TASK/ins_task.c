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
 * @brief ��ʼ�� ins ����ϵͳ
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

// ʹ�ü��ٶȼƵ����ݳ�ʼ��Roll��Pitch,��Yaw��0,�������Ա����ڳ�ʼʱ�����̬�������
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // ����ϵ�������ٶ�ʸ��,��һ����Ϊ(0,0,1)
    float axis_rot[3] = {0};           // ��ת��
    // ��ȡ100�μ��ٶȼ�����,ȡƽ��ֵ��Ϊ��ʼֵ
    for (uint8_t i = 0; i < 100; ++i)
    {
        imu_ops.accel_read(acc_init);
        dwt_delay_s(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // ����ԭʼ���ٶ�ʸ���͵���ϵ�������ٶ�ʸ���ļн�
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // ��ǹ�ʽ,������Ϊ0(û��z�����)
}

