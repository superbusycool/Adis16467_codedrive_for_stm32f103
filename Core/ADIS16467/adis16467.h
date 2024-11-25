//
// Created by SuperChen on 2024/11/21.
//

#ifndef ADIS16467_TRY1_ADIS16467_H
#define ADIS16467_TRY1_ADIS16467_H
#include "gpio.h"

typedef signed long    err_t;
#define SPI_NSS_GPIO_Port    GPIOA
#define SPI_NSS_Pin          GPIO_PIN_4


typedef struct
{
    int16_t DIAG_STAT;
    int16_t X_GYRO;
    int16_t Y_GYRO;
    int16_t Z_GYRO;
    int16_t X_ACCL;
    int16_t Y_ACCL;
    int16_t Z_ACCL;
    int16_t TEMP;
    int16_t DATA_CNTR;
    int16_t Checknum;
}ADIS_t;

struct imu_ops{
    err_t (*imu_init)(void);
    err_t (*gyro_read)(float data[3]);
    err_t (*accel_read)(float data[3]);
    err_t (*burst_read)(float accel[3],float gyro[3],float temp);
    float (*temp_read)(void);
};
#endif //ADIS16467_TRY1_ADIS16467_H
