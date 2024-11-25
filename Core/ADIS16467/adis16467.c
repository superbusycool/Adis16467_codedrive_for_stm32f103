//
// Created by SuperChen on 2024/11/21.
//

#include "adis16467.h"
#include "spi.h"
#include "stdio.h"

#define Reserved    0x00      //0x00,0x01
#define DIAG_STAT   0x02      //0x02,0x03

#define X_GYRO_LOW   0x04      //0x04, 0x05  Output, x-axis gyroscope, low word 下面同理
#define X_GYRO_OUT   0x06     //0x06, 0x07   Output, x-axis gyroscope, high word
#define Y_GYRO_LOW   0x08     //0x08,0x09
#define Y_GYRO_OUT   0x0A     //0x0A,0x0B
#define Y_GYRO_OUT   0x0C     //0x0C,0x0D
#define Z_GYRO_OUT   0x0E     //0x0E,0x0F

#define X_ACCL_LOW   0x10    //0x10,0x11
#define X_ACCL_OUT   0x12    //0x12,0x13
#define Y_ACCL_LOW   0x14    //0x14,0x15
#define Y_ACCL_OUT   0x16    //0x16,0x17
#define Z_ACCL_LOW   0x18    //0x18,0x19
#define Z_ACCL_OUT   0x1A    //0x1A,0x1B

#define TEMP_OUT     0x1C    //0x1C,0x1D
#define TIME_STAMP   0x1E    //0x1E,0x1F Output, time stamp
#define Reserved     0x20    //0x20,0x21,Reserved
#define DATA_CNTR    0x22    //0x22,0x23 New data counter

#define X_DELTANG_LOW 0x24   //0x24,0x25 Output, x-axis delta angle, low word
#define X_DELTANG_OUT 0x26   //0x26,0x27 Output, x-axis delta angle, high word
#define Y_DELTANG_LOW 0x28   //0x28,0x29
#define Y_DELTANG_OUT 0x2A   //0x2A,0x2B
#define Z_DELTANG_LOW 0x2C   //0x2C,0x2D
#define Z_DELTANG_OUT 0x2E   //0x2E,0X2F

#define X_DELTVEL_LOW 0x30   //0x30,0x31 Output, x-axis delta velocity, low word
#define X_DELTVEL_OUT 0x32   //0x32,0x33 Output, x-axis delta velocity, high word
#define Y_DELTVEL_LOW 0x34   //0x34,0x35
#define Y_DELTVEL_OUT 0x36   //0x37,0x38
#define Z_DELTVEL_LOW 0x38   //0x38,0x39
#define Z_DELTVEL_OUT 0x3A   //0x3A,0x3B

#define Reserved  0X3C    //0X3C to 0X3F

#define XG_BIAS_LOW     0x40   //0x40,0x41     XG_BIAS_LOW->ZA_BIAS_HIGH,Defualt Value 0x0000 Calibration, offset, gyroscope, x-axis, low word
#define XG_BIAS_HIGH    0x42   //0x42,0x43     Calibration, offset, gyroscope, x-axis, high word
#define YG_BIAS_LOW     0x44   //0x44,0x45
#define YG_BIAS_HIGH    0x46   //0x46,0x44
#define ZG_BIAS_LOW     0x48   //0x48,0x49
#define ZG_BIAS_HIGH    0x4A   //0x4A,0x4B

#define XA_BIAS_LOW     0x4C   //0x4C,0x4D
#define XA_BIAS_HIGH    0x4E   //0x4E,0x4F
#define YA_BIAS_LOW     0x50   //0x450,0x51
#define YA_BIAS_HIGH    0x52   //0x52,0x53
#define ZA_BIAS_LOW     0x54   //0x54,0x55
#define ZA_BIAS_HIGH    0x56   //0x56,0x57

#define Reserved    0x58   //0x58 to 0x5B

#define FILT_CTRL   0x5C   //0X5C,0X5D  Control, Bartlett window FIR filter  defualt 0x0000
#define RANG_MDL    0x5E   //0x5E,0x5F  Measurement range (model specific) identifier
#define MSC_CTRL    0x60   //0x60,0x61  Control, input/output and other miscellaneous option
#define UP_SCALE    0x62   //0x62,0x63  Control, scale factor for input clock, pulse per second (PPS) mode
#define DEC_RATE    0x64   //0x64,0x65  Control, decimation filter (output data rate)
#define NULL_CNFG   0x66   //0x66,0x67  Control, bias estimation period
#define GLOB_CMD    0x68   //0x68,0x69  Control, global commands
#define Reserved    0x6A   //0x6A to 0x6B

#define FIRM_REV      0x6C   //0x6C,0x6D  Identification, firmware revision
#define FIRM_DM       0x6E   //0x6E,0x6F  Identification, date code, day and month
#define FIRM_Y        0x70   //0x70,0x71  Identification, date code, year
#define PROD_ID       0x72   //0x72,0x73  Identification, device number 0x4053
#define SERIAL_NUM    0x74   //0x74,0x75  Identification, serial number
#define USER_SCR_1    0x76   //0x76,0x77  User Scratch Register 1
#define USER_SCR_2    0x78   //0x78,0x79  User Scratch Register 2
#define USER_SCR_3    0x7A   //0x7A,0x7B  User Scratch Register 3
#define FLSHCNT_LOW   0x7C   //0x7C,0x7D  Output, flash memory write cycle counter, lower word
#define FLSHCNT_HIGH  0x7E   //0x7E,0x7F  Output, flash memory write cycle counter, upper word

#define SPI_DIR_READ  0x80
#define SPI_DIR_WRITE 0x00


#define ADIS_NSS_HIGH    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)   //置1
#define ADIS_NSS_LOW  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET)   //置0
#define ADX_SPI hspi1 //使用哪个spi
static uint8_t        tx_buff[14] = { 0xff };			//MPU6500数据变量（加速度，温度，角度）

#define x 0
#define y 1
#define z 2

#define gyro_scale_factor  40*65536   //对应 1度/s
#define accel_scale_factor  1.25 / (65536 * 1000)    //1->1.25/2^16mg
#define temp_scale_factor   10     //0.1摄氏度->1

ADIS_t  imu;    //burst read 数据存放处
static int16_t Burst_read_Cmd=0x6800;//Burst Read指令


//adis16467单次写命令
uint8_t Adis_TandR(uint16_t trans)
{
    ADIS_NSS_LOW;					//开始通讯
    uint8_t result;
    static HAL_StatusTypeDef state;
    state = HAL_SPI_TransmitReceive(&ADX_SPI, &trans, &result, 1, 0xFF);		//写入命令地址
    if(state!=HAL_OK)
    {
        while(1);
    }
    printf("transmit success!");
    ADIS_NSS_HIGH;					//结束通讯
    HAL_Delay(50);
    return result;
}

/**
* @brief  对ADIS16467寄存器内部写操作函数
* @param[in]   addr 写的地址
* @param[in]   value 写的值
* @retval  0  成功
* @par 日志
*
*/
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value)
{
    addr|=SPI_DIR_WRITE;//写数据的掩码
    uint16_t Tx_tmp=(addr<<8) | value;
    Adis_TandR(Tx_tmp);
    return 0;
}


//读取寄存器的函数,因为SPI的方式 在连续读取上有效率优势,所以建议连续读取
/*
@parameter:
	addr_Reg 待读取的寄存器数组,Rx_point
	Rx_point 读取后的数据放的位置
	length	 欲读取的寄存器数量
*/
//adis16467读取多字节数据
int8_t ADX_Read_Reg(uint8_t addr_Reg,uint16_t* Rx_point,uint8_t length){

    uint8_t addr;
    uint8_t buffer[2];

    for(uint8_t i =0;i<length;i++){

        addr = SPI_DIR_READ | addr_Reg;
        buffer[0] = Adis_TandR(addr);  //低地址低八位
        addr = SPI_DIR_READ | (addr_Reg+0x01);
        buffer[1] = Adis_TandR(addr);  //高地址高八位

        Rx_point[i] = buffer[1] <<8 | buffer[0]; //十六位读出数据,只是一位寄存器的读取值例如X_GYRO_LOW

        addr_Reg += 2*i;
    }
}

static err_t ADIS16467_init(void){

    uint16_t ID;
    ADX_Read_Reg(PROD_ID,(uint16_t *)ID,1);
    if(ID != 0x4053){
        printf("ADIS16467 ID error");
        while(1);                       //loop
    }

    ADX_Write_Reg(GLOB_CMD,0x80);
    ADX_Write_Reg((GLOB_CMD+0x01),0x00);  //软件复位
    HAL_Delay(400);

    ADX_Write_Reg(FILT_CTRL,0x05);
    ADX_Write_Reg((FILT_CTRL+0x01),0x00);  //use filter

    ADX_Write_Reg(DEC_RATE,0x01);
    ADX_Write_Reg((DEC_RATE+0x01),0x00);  //均值滤波器 set output rate  2000/(DEC_RATE+1) 1000Hz(now)

    ADX_Write_Reg(NULL_CNFG,0x0A);
    ADX_Write_Reg((NULL_CNFG+0x01),0x3F);  //启动个轴accel和gyro的bias计算
    HAL_Delay(400);

    ADX_Write_Reg(USER_SCR_1,0x05);
    ADX_Write_Reg((USER_SCR_1+0x01),0x00);  //for user to store information

    ADX_Write_Reg(RANG_MDL,0x07);
    ADX_Write_Reg((RANG_MDL+0x01),0x00);  //identify which model,此处为01 = ±500°/sec (ADIS16467-2BMLZ)

    ADX_Write_Reg(GLOB_CMD,0x7F);
    ADX_Write_Reg((GLOB_CMD+0x01),0x00);  //启动各功能更新自检 更新bias_correction计算结果
    HAL_Delay(400);

#ifdef ADS_Calicration
    ADS_Calibrate();
#endif



}

static err_t ADIS16467_gyro_read(float data[3]){  //单独读取时数据位32-bits,accel同理

    uint16_t buffer[6];
    int32_t gyro_raw[3];
    ADX_Read_Reg(X_GYRO_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    gyro_raw[x] = buffer[1] << 16 | buffer[0];
    gyro_raw[y] = buffer[3] << 16 | buffer[2];
    gyro_raw[x] = buffer[5] << 16 | buffer[4];

    data[0] = gyro_raw[x] / gyro_scale_factor;
    data[1] = gyro_raw[y] / gyro_scale_factor;
    data[2] = gyro_raw[z] / gyro_scale_factor;


}

static err_t ADIS16467_accel_read(float data[3]){

    uint16_t buffer[6];
    int32_t accel_raw[3];
    ADX_Read_Reg(X_ACCL_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    accel_raw[x] = buffer[1] << 16 | buffer[0];
    accel_raw[y] = buffer[3] << 16 | buffer[2];
    accel_raw[x] = buffer[5] << 16 | buffer[4];

    data[0] = accel_raw[x] * accel_scale_factor;
    data[1] = accel_raw[y] * accel_scale_factor;
    data[2] = accel_raw[z] * accel_scale_factor;

}

static err_t ADIS16467_gyro_bias_read(int32_t accel_gyro_bias_raw[3]){

    uint16_t buffer[6];
//    int32_t accel_gyro_bias_raw[3];
    ADX_Read_Reg(XG_BIAS_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    accel_gyro_bias_raw[x] = buffer[1] << 16 | buffer[0];
    accel_gyro_bias_raw[y] = buffer[3] << 16 | buffer[2];
    accel_gyro_bias_raw[x] = buffer[5] << 16 | buffer[4];

//    data[0] = accel_gyro_bias_raw[x];
//    data[1] = accel_gyro_bias_raw[y];
//    data[2] = accel_gyro_bias_raw[z];


}

static err_t ADIS16467_accel_bias_read(int32_t accel_bias_raw[3]){

    uint16_t buffer[6];
//    int32_t accel_bias_raw[3];
    ADX_Read_Reg(XA_BIAS_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    accel_bias_raw[x] = buffer[1] << 16 | buffer[0];
    accel_bias_raw[y] = buffer[3] << 16 | buffer[2];
    accel_bias_raw[x] = buffer[5] << 16 | buffer[4];

//    data[0] = accel_bias_raw[x];
//    data[1] = accel_bias_raw[y];
//    data[2] = accel_bias_raw[z];


}

static err_t ADIS16467_delta_angle_read(float data[3]){

    uint16_t buffer[6];
    int32_t DELANG_raw[3];
    ADX_Read_Reg(X_DELTANG_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    DELANG_raw[x] = buffer[1] << 16 | buffer[0];
    DELANG_raw[y] = buffer[3] << 16 | buffer[2];
    DELANG_raw[x] = buffer[5] << 16 | buffer[4];

    data[0] = DELANG_raw[x];
    data[1] = DELANG_raw[y];
    data[2] = DELANG_raw[z];

}

static err_t ADIS16467_delta_vel_read(float data[3]){

    uint16_t buffer[6];
    int32_t DELTVEL_raw[3];
    ADX_Read_Reg(X_DELTVEL_LOW,(uint16_t *)buffer,6);  //一个寄存器读取出的数据算一位
    DELTVEL_raw[x] = buffer[1] << 16 | buffer[0];
    DELTVEL_raw[y] = buffer[3] << 16 | buffer[2];
    DELTVEL_raw[x] = buffer[5] << 16 | buffer[4];

    data[0] = DELTVEL_raw[x];
    data[1] = DELTVEL_raw[y];
    data[2] = DELTVEL_raw[z];

}

static float ADIS16467_temp_read(void){

    uint16_t buffer[1];
    int32_t temp_raw[1];
    static float temp;
    ADX_Read_Reg(TEMP_OUT,(uint16_t *)buffer,1);  //一个寄存器读取出的数据算一位

    temp_raw[0] = buffer[1] << 16 | buffer[0];

    temp = temp_raw[0] /temp_scale_factor;

    return temp;
}

static err_t ADIS16467_burst_read(float accel[3],float gyro[3],float temp){    //使用burst_read一次性读取全部数据,数据需校验,详情请看datasheet,burst读取的数据位16-bits
    ADIS_NSS_LOW;

    uint8_t checksum;
    uint8_t burst_raw[19] = {0};

    HAL_SPI_TransmitReceive(&ADX_SPI,&Burst_read_Cmd,(uint8_t *)burst_raw,19,0xFF);

    for(uint8_t i=0;i<18;i++){
        checksum += burst_raw[i];
    }if(checksum ==  burst_raw[18]){//通过和校验

        gyro[x] = (float)(burst_raw[2] << 8 | burst_raw[3]);
        gyro[y] = (float)(burst_raw[4] << 8 | burst_raw[5]);
        gyro[z] = (float)(burst_raw[6] << 8 | burst_raw[7]);

        accel[x] = (float)(burst_raw[8] << 8 | burst_raw[9]);
        accel[y] = (float)(burst_raw[10] << 8 | burst_raw[11]);
        accel[x] = (float)(burst_raw[12] << 8 | burst_raw[13]);

        temp = (float)(burst_raw[14] << 8 | burst_raw[15]);

    }


    ADIS_NSS_HIGH;

}

void Uint32toUint8(uint32_t * data,uint8_t * data1){

    data1[0] =  data[0] & 0xFF;
    data1[1] = (data[1] >> 8) & 0xFF;
    data1[2] = (data[2] >> 16) & 0xFF;
    data1[3] = (data[3] >> 24) & 0xFF;

}
static void ADS_Calibrate(void){

    int32_t accel_bias[3];
    int32_t gyro_bias[3];
    int32_t accel_bias_temp[3];
    int32_t gyro_bias_temp[3];
    uint8_t count = 500;  //count次取平均
    uint8_t Gyro_Bias[4];
    uint8_t Accel_Bias[4];

    for(uint8_t i = 0;i<count;i++){
        ADIS16467_gyro_bias_read((int32_t *)gyro_bias[3]);
        gyro_bias_temp[0] += gyro_bias[0];
        gyro_bias_temp[1] += gyro_bias[1];
        gyro_bias_temp[2] += gyro_bias[2];

        ADIS16467_accel_bias_read((int32_t *)accel_bias[3]);
        accel_bias_temp[0] += accel_bias[0];
        accel_bias_temp[1] += accel_bias[1];
        accel_bias_temp[2] += accel_bias[2];

    }
    gyro_bias_temp[0] = ~(gyro_bias_temp[0] / count) + 0x01; //取反
    gyro_bias_temp[1] = ~(gyro_bias_temp[1] / count) + 0x01;
    gyro_bias_temp[2] = ~(gyro_bias_temp[2] / count) + 0x01;

    accel_bias_temp[0] = ~(accel_bias_temp[0] /count) + 0x01;
    accel_bias_temp[1] = ~(accel_bias_temp[1] /count) + 0x01;
    accel_bias_temp[2] = ~(accel_bias_temp[2] /count) + 0x01;

    for(uint8_t i=0;i<3;i++){
        Uint32toUint8((uint32_t *)gyro_bias_temp,(uint8_t *)Gyro_Bias);
        ADX_Write_Reg(XG_BIAS_LOW,Gyro_Bias[0]);
        ADX_Write_Reg((XG_BIAS_LOW+0x01),Gyro_Bias[1]);
        ADX_Write_Reg((XG_BIAS_LOW+0x02),Gyro_Bias[2]);
        ADX_Write_Reg((XG_BIAS_LOW+0x03),Gyro_Bias[3]);

        Uint32toUint8((uint32_t *)accel_bias_temp,(uint8_t *)Accel_Bias);
        ADX_Write_Reg(XA_BIAS_LOW,Accel_Bias[0]);
        ADX_Write_Reg((XA_BIAS_LOW+0x01),Accel_Bias[1]);
        ADX_Write_Reg((XA_BIAS_LOW+0x02),Accel_Bias[2]);
        ADX_Write_Reg((XA_BIAS_LOW+0x03),Accel_Bias[3]);

    }


}

struct imu_ops imu_ops = {
        .imu_init  =   ADIS16467_init,
        .gyro_read =  ADIS16467_gyro_read,
        .accel_read = ADIS16467_accel_read,
        .burst_read = ADIS16467_burst_read,
        .temp_read =  ADIS16467_temp_read,
};