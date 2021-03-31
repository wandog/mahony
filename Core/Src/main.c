/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kalman.h"
#include "MahonyAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI 3.14159265
//MPU6500
#define MPU9250_ADDR 0xD0 //0x68<<1
#define WHO_AM_I_REG 0x75 //return value 0x71
#define PWR_MGMT_1_REG 0x6B
#define PWR_MGMT_2_REG 0x6C
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_CONFIG2_REG 0x1D
#define WOM_THR_REG 0x1F
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define INT_ENABL_REG 0x38
#define MOT_DETECT_CTRL_REG 0x69
#define INT_PIN_CFG_REG 0x37
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define FIFO_COUNTH      0x72
#define FIFO_R_W         0x74

// AK8963 registers
#define AK8963_ADDR 0x0C
#define AK8963_CNTL1_REG 0x0A
#define AK8963_WHOAMI_REG 0x00 //device id = 0x48
#define AK8963_HXL_REG 0x03 //Measurement Data
#define AK8963_ASAX      0x10	// Fuse ROM x-axis sensitivity adjustment value

uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Mag_X_RAW = 0;
int16_t Mag_Y_RAW = 0;
int16_t Mag_Z_RAW = 0;

int16_t Temp_RAW = 0;


uint8_t data_rec[6];
uint8_t data;
int16_t x,y,z;
float xg,yg,zg;


float magCalibration[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0} ,magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, Temp;
kfilter* gxf,* gyf,* gzf;

double angle[3];
double prior[3];
double roll,pitch;
double rad2deg = 180/3.1415;
float mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
float dt=0.01;
double base[3]={0,0,0};
double acce[3];


void MPU9250_Init (void)
{
    uint8_t check;
    uint8_t Data;

	// check WHO_AM_I

    HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

    if (check == 0x71)
	{
		//Reset
		Data = 0X80;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		//Wake up
        Data = 0X00;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE = 1kHZ
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer +-2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscope +-250
		Data = 0x11;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

		// Set DLPF Gyro Bandwidth = 188hz
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

		// Set DLPF ACCEL Bandwidth = 5hz
		Data = 0x06;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG2_REG, 1, &Data, 1, 1000);

		//The logic level for INT pin is active low
		//BYPASS Enable
		Data = 0x82;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);

		// Auto selects the best available clock source ?? PLL if ready, else use the Internal oscillator
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Enable ACCEL & GYRO
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);
	}

	HAL_I2C_Mem_Read (&hi2c1, (AK8963_ADDR<<1),AK8963_WHOAMI_REG,1, &check, 1, 1000);

	if (check == 0x48){
		// Single measurement mode
		// 16-bit output
		Data = 0x11;
		HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000);
	}
}
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    uint8_t Data;
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000);	 // Power down magnetometer
    HAL_Delay(10);
    Data = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000);	// Enter Fuse ROM access mode
    HAL_Delay(10);
    HAL_I2C_Mem_Read (&hi2c1, (AK8963_ADDR<<1),AK8963_ASAX,1, &rawData, 3, 1000); // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000);	 // Power down magnetometer
    HAL_Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    Data = 1 << 4 | Mmode;
    HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000); // Set magnetometer data resolution and sample ODR
    HAL_Delay(10);
}


void MPU9250_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0 ;
	Ay = Accel_Y_RAW/16384.0 ;
    Az = Accel_Z_RAW/16384.0 ;
    Ax-=accelBias[0];
	Ay-=accelBias[1];
    Az-=accelBias[2];
}


void MPU9250_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps
	 check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0 - gyroBias[0];
	Gy = Gyro_Y_RAW/131.0 - gyroBias[1];
	Gz = Gyro_Z_RAW/131.0 - gyroBias[2];
}

void MPU9250_Read_Temp (void)
{
	uint8_t Rec_Data[2];

	// Read 2 BYTES of data starting from TEMP_OUT_H_REG register

	HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);

	Temp_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);

	/*** convert the RAW values into the temperature in degrees C
	 	 	 ****/

	Temp=21+((double)Temp_RAW)/333.87;
}

void MPU9250_Read_Mag (void)
{
	uint8_t Rec_Data[6];
	uint8_t Data = 0x11;
	// Read 6 BYTES of data starting from AK8963_HXL_REG register

	HAL_I2C_Mem_Read (&hi2c1, (AK8963_ADDR<<1), AK8963_HXL_REG, 1, Rec_Data, 6, 1000);

	Mag_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data [0]);
	Mag_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data [2]);
	Mag_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data [4]);

	/*** convert the RAW values into µT
		 check HXL Register              ****/
	Mx = (float)Mag_X_RAW*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
	My = (float)Mag_Y_RAW*mRes*magCalibration[1] - magBias[1];
	Mz = (float)Mag_Z_RAW*mRes*magCalibration[2] - magBias[2];
	Mx *= magScale[0];
	My *= magScale[1];
	Mz *= magScale[2];

	HAL_I2C_Mem_Write(&hi2c1, (AK8963_ADDR<<1), AK8963_CNTL1_REG, 1, &Data, 1, 1000);
}
void magcalMPU9250(float * dest1, float * dest2)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
    char *str = "turn sensor \n\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
    HAL_Delay(1000);

    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < sample_count; ii++) {
        MPU9250_Read_Mag ();  // Read the mag data
        mag_temp[0]=Mag_X_RAW;
        mag_temp[1]= Mag_Y_RAW;
        mag_temp[2]= Mag_Z_RAW;
    for (int jj = 0; jj < 3; jj++) {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }
    char *str2 = "mag done\n\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

void calibrateMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	uint8_t Data;



// Configure device for bias calculation
	Data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);   // Turn on internal clock source
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, I2C_MST_CTRL, 1, &Data, 1, 1000);		 // Disable I2C master
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, USER_CTRL, 1, &Data, 1, 1000);	  // Disable FIFO and I2C master modes
	Data = 0x0C;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, USER_CTRL, 1, &Data, 1, 1000);	  // Reset FIFO and DMP


// Configure MPU6050 gyro and accelerometer for bias calculation

	Data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, CONFIG_REG, 1, &Data, 1, 1000);     // Set low-pass filter to 188 Hz
	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);  // Set sample rate to 1 kHz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);	 // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);	 // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
	Data = 0x40;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, USER_CTRL, 1, &Data, 1, 1000);  // Enable FIFO
	Data = 0x78;
	for(int j=0;j<100;j++){
		MPU9250_Read_Accel();
		MPU9250_Read_Gyro();
		accel_bias[0]+=Accel_X_RAW;
		accel_bias[1]+=Accel_Y_RAW;
		accel_bias[2]+=Accel_Z_RAW;
		gyro_bias[0]+=Gyro_X_RAW;
		gyro_bias[1]+=Gyro_Y_RAW;
		gyro_bias[2]+=Gyro_Z_RAW;
		HAL_Delay(1);
	}
	packet_count = 100;// How many sets of full gyro and accelerometer data for averaging


    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}



// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;




// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void setfilter_angle(){				//use acce to get initial angle

	roll = atan2(Ay,Az) * rad2deg;
	pitch = atan(-Ax / sqrt(Ay*Ay+Az*Az)) * rad2deg;
	float yaw = atan(My/Mx) * rad2deg;
	setAngle(gxf,roll);
	setAngle(gyf,pitch);
	setAngle(gzf,yaw);

	gxf->bias=1.9362;
	gyf->bias=1.3398;
	gzf->bias=0.84063;

	angle[0]=roll;
	angle[1]=pitch;
	angle[2]=0;
	prior[0]=roll;
	prior[1]=pitch;
	prior[2]=0;
}

void Clock_Start(){
	TIM2->CNT=0x00;
	HAL_TIM_Base_Start(&htim2);
}

int Clock_End(){
	int result;
	result = TIM2->CNT;
	HAL_TIM_Base_Stop(&htim2);
	return result;
}


//static void MX_SPI2_Init(void)
//{
//
//	/* USER CODE BEGIN SPI2_Init 0 */
//
//	/* USER CODE END SPI2_Init 0 */
//
//	/* USER CODE BEGIN SPI2_Init 1 */
//
//	/* USER CODE END SPI2_Init 1 */
//	/* SPI2 parameter configuration*/
//	hspi2.Instance = SPI2;
//	hspi2.Init.Mode = SPI_MODE_MASTER;
//	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//	hspi2.Init.NSS = SPI_NSS_SOFT;
//	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//	hspi2.Init.CRCPolynomial = 10;
//	if (HAL_SPI_Init(&hspi2) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	/* USER CODE BEGIN SPI2_Init 2 */
//
//	/* USER CODE END SPI2_Init 2 */
//}


void adxl_write(uint8_t address,uint8_t value){
	uint8_t data[2];
	data[0]=address|0x40;
	data[1]=value;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,data,2,100);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
}

void adxl_read_data(uint8_t address){
	address |= 0x80;
	address |= 0x40;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,&address,1,100);
	HAL_SPI_Receive(&hspi2,data_rec,6,100);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
}

void adxl_read(uint8_t address){
	address |= 0x80;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,&address,1,100);
	HAL_SPI_Receive(&hspi2,&data,1,100);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
}

void adxl_init(void){
	adxl_write(0x31,0x00); //DATA_FORMAT=> +-2g
	adxl_write(0x2d,0x00); //POWER_CTL => Wakeup Bits
	adxl_write(0x2c,0x0A); //100hz
	adxl_write(0x2d,0x08); //POWER_CTL => Measure Bit
	adxl_write(0x2e,0x00); //INT_ENABLE => close
	adxl_write(0x24,0x4b); //THRESH_ACT =>75
	adxl_write(0x27,0x40); //ACT_INACT_CTL => dc-coupled operation,enable ACT_X
	adxl_write(0x2f,0x10); //INT_MAP => Activity map INT2
	adxl_write(0x2e,0x93); //INT_ENABLE => open
}

void getData_from_ADXL(int id){

	unsigned char buffer[100]="\0";
//	adxl_read(0x30);
//	memset(buffer, '\0', 100);
//	sprintf(buffer,"adxl : %x \n\r",data);
//	while(HAL_UART_GetState(&huart2)!=HAL_UART_STATE_READY);
//
//	if(HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),100)!=HAL_OK){
//		Error_Handler();
//	}
	adxl_read_data(0x32);
	x=(data_rec[1]<<8)|data_rec[0];
	y=(data_rec[3]<<8)|data_rec[2];
	z=(data_rec[5]<<8)|data_rec[4];

	Ax=x;
	Ay=y;
	Az=z;

//	xg = x*0.0078;
//	yg = y*0.0078;
//	zg = z*0.0078;
//	memset(buffer, '\0', 100);
//	sprintf(buffer,"%d,%d,%d,%d\n\r",id,x,y,z);
//	while(HAL_UART_GetState(&huart2)!=HAL_UART_STATE_READY);
//	if(HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),100)!=HAL_OK){
//		Error_Handler();
//	}

//	memset(buffer, '\0', 100);
//	sprintf(buffer,"%d,%7.2f,%7.2f,%7.2f\n\r",id,xg,yg,zg);
//	while(HAL_UART_GetState(&huart2)!=HAL_UART_STATE_READY);
//	if(HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),100)!=HAL_OK){
//		Error_Handler();
//	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
	SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	MPU9250_Init();
	HAL_Delay(2000);

	calibrateMPU9250(gyroBias,accelBias);

	initAK8963(magCalibration);
	magcalMPU9250(magBias, magScale);
	//gxf=new_Kalman();
	//gyf=new_Kalman();
	//gzf=new_Kalman();
	adxl_init();



	//setfilter_angle();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float ahrs[]={0,0,0};
	int m=0;
//	unsigned char buffer[30]="\0";
//	while(1){
//
//		HAL_Delay(1000);
//	}

	while (1)
	{

		Clock_Start();
//		MPU9250_Read_Accel();
		MPU9250_Read_Gyro();
		MPU9250_Read_Mag();
		getData_from_ADXL(100);

		MahonyAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, My, Mx, Mz);

		prior[2]=angle[2];
		prior[0]=angle[0];
		prior[1]=angle[1];
	 	ahrs[0]=atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * rad2deg;
	 	ahrs[1]=asinf(-2.0f * (q1*q3 - q0*q2)) * rad2deg;
	 	ahrs[2]=atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* rad2deg;


//	 	float halfvx = q1 * q3 - q0 * q2;
//	 	float halfvy = q0 * q1 + q2 * q3;
//	 	float halfvz = q0 * q0 - 0.5f + q3 * q3;
	 	 /*//get baseline
	 	 	base[0]=-1*sin(angle[1]/rad2deg)*sqrt(Ax*Ax+Ay*Ay+Az*Az);
	 	 	base[1]=cos(angle[1]/rad2deg)*sin(angle[0]/rad2deg)*sqrt(Ax*Ax+Ay*Ay+Az*Az);
	 	 	base[2]=cos(angle[1]/rad2deg)*cos(angle[0]/rad2deg)*sqrt(Ax*Ax+Ay*Ay+Az*Az);
	 	 // get acce
	 	 	acce[0]=Ax-base[0];	acce[1]=Ay-base[1];	acce[2]=Az-base[2];*/
		char mes[86];
		//below is for linear acc caculation
		sprintf(mes,"%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n\r" ,Ax, Ay, Az,q0 ,q1 ,q2 ,q3 );

		//marked for temp
//		sprintf(mes,"%7.2f,%7.2f,%7.2f\n",ahrs[0],ahrs[1],ahrs[2]);
		HAL_UART_Transmit(&huart2, (uint8_t *)mes, strlen(mes), HAL_MAX_DELAY);

		//below is for 345 ic



//		adxl_read(0x32);
//		x=(data_rec[1]<<8)|data_rec[0];
//		y=(data_rec[3]<<8)|data_rec[2];
//		z=(data_rec[5]<<8)|data_rec[4];
//
//		xg = x*0.0078;
//		yg = y*0.0078;
//		zg = z*0.0078;
//		unsigned char buffer[30]="\0";
//		int id=0;
//		memset(buffer, '\0', 30);
//		sprintf(buffer,"%d,%+.3f,%+.3f,%+.3f\n",id,xg,yg,zg);
//		id++;
//		HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),HAL_MAX_DELAY);
		//above is for 345 ic

		//sprintf(mes,"%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n",Ax,Ay,Az,accelBias[0],accelBias[1],accelBias[2]);

	 	 // calculate delay time
		int cost = Clock_End()/2;

		if(dt*1000-cost>0)
			HAL_Delay (dt*1000-cost);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 36000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
