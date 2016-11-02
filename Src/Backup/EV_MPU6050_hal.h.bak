/*
 * EV_MPU6050_hal.h
 *
 *  Created on: 01 но€б. 2016 г.
 *      Author: ostapchuk_v
 */


#ifndef APPLICATION_MY_LIBS_EV_MPU6050_HAL_H_
#define APPLICATION_MY_LIBS_EV_MPU6050_HAL_H_



#include "stm32f4xx_hal.h"
#include "EV_MPU6050_hal.h"
//#include "stm32f0xx_hal_i2c.h"


#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

////////////////////////////////////////////////////////////////////////////////
#define MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */
//////////////////////////////////////////////////////////////////////////////////


typedef enum  {
	MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	MPU6050_Result_Error,              /*!< Unknown error */
	MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} MPU6050_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_Gyroscope;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct  {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} MPU6050;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} MPU6050_Interrupt;


//////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Device DeviceNumber, MPU6050_Accelerometer AccelerometerSensitivity, MPU6050_Gyroscope GyroscopeSensitivity);

MPU6050_Result MPU6050_SetGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Gyroscope GyroscopeSensitivity);

MPU6050_Result MPU6050_SetAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Accelerometer AccelerometerSensitivity);

MPU6050_Result MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, uint8_t rate);

MPU6050_Result MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

MPU6050_Result MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

MPU6050_Result MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Interrupt* InterruptsStruct);

MPU6050_Result MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

MPU6050_Result MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

MPU6050_Result MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

MPU6050_Result MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

////////////////////////////////////////////////////////////////////////////////////////////////////


#endif /* APPLICATION_MY_LIBS_EV_MPU6050_HAL_H_ */
