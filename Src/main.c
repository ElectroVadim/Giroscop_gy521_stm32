/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define MPU6050_I2C_ADDR			0xD0
/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)
////////////////////////////////////////////////////////////////////////////////
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


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//////////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;

MPU6050 mpu1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	MPU6050_Result result ;
//	uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
//	uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	result = MPU6050_Init(&hi2c1,&mpu1,MPU6050_Device_0,MPU6050_Accelerometer_2G,MPU6050_Gyroscope_250s );
	HAL_Delay(500);

	HAL_Delay(500);
	MPU6050_ReadTemperature(&hi2c1,&mpu1);
	float temper = mpu1.Temperature;
	MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	int16_t g_x = mpu1.Gyroscope_X;
	int16_t g_y = mpu1.Gyroscope_Y;
	int16_t g_z = mpu1.Gyroscope_Z;

	MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	int16_t a_x = mpu1.Accelerometer_X;
	int16_t a_y = mpu1.Accelerometer_Y;
	int16_t a_z = mpu1.Accelerometer_Z;


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Device DeviceNumber, MPU6050_Accelerometer AccelerometerSensitivity, MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t WHO_AM_I = (uint8_t)MPU6050_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];


	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t address = DataStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,address,2,5)!=HAL_OK)
	{
				return MPU6050_Result_Error;
	}
	/* Check who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, address, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
			return MPU6050_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, address, &temp, 1, 1000) != HAL_OK)
		{
			return MPU6050_Result_Error;
		}

		/* Checking */
		while(temp != MPU6050_I_AM)
		{
				/* Return error */
				return MPU6050_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU6050 */
	//------------------
		/* Format array to send */
		d[0] = MPU6050_PWR_MGMT_1;
		d[1] = 0x00;

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU6050_Result_Error;
		}
	//------------------

	/* Set sample rate to 1kHz */
	MPU6050_SetDataRate(I2Cx,DataStruct, MPU6050_DataRate_1KHz);

	/* Config accelerometer */
	MPU6050_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	MPU6050_SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	/* Format array to send */
	d[0] = MPU6050_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU6050_ACCEL_CONFIG;

	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU6050_GYRO_CONFIG;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return MPU6050_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case MPU6050_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
				break;
			case MPU6050_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
				break;
			case MPU6050_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
				break;
			case MPU6050_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU6050_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU6050_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t reg = MPU6050_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read temperature */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU6050_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 14, 1000) != HAL_OK);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU6050_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU6050_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct)
{
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU6050_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return MPU6050_Result_Ok;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
