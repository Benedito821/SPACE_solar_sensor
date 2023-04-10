/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  <math.h>
#include "as7265x.h"
#include  <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/////////////////////////QMC7983//////////////////////////////////////
#define DATA_X_LSB_REG 0x00
#define DATA_X_MSB_REG 0x01
#define DATA_Y_LSB_REG 0x02
#define DATA_Y_MSB_REG 0x03
#define DATA_Z_LSB_REG 0x04
#define DATA_Z_MSB_REG 0x05

#define TOUT_LSB_REG  0x07
#define TOUT_MSB_REG  0x08

#define STATUS_REG_1 0x06
#define STATUS_REG_2 0x0C

#define CONTROL_REG_1 0x09
#define CONTROL_REG_2 0x0A

#define SET_RESET_PERIOD_REG 0x0B

#define QMC7983_ADDR (0x2C)<<1 // 0101100 - tied to  VSS
#define CHIPID_REG 0x0D

#define PI 3.141592
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float mag_fi_meas[3] = {0.0}; //G
float mag_fi_x,mag_fi_y,mag_fi_z = 0.0;
float total_magn = 0.0;
float temp_meas = 0.0; //degrees
float heading = 0.0; //degrees
float ADC_Rad = 0.0;
float ADC_Ilum = 0.0;
float ADC_ISence1 = 0.0;
float ADC_ISence2 = 0.0;

char uart_buffer [1000]  ;

uint16_t sensor_raw_vals[4] = {0};
extern as7265x_raw_channels_t raw_channels;
uint8_t raw_temp_data[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void QMC7983_config(void);
void QMC7983_read_magn_field(void);
void QMC7983_read_temp (void);
void QMC7983_read_heading (void);
void AS72651_config(void);
void AS72651_read(void);
void Current_Left(uint8_t PWM);
void Current_Right(uint8_t PWM);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 1000);
}
//int __io_putchar(int ch){
//
//	HAL_UART_Transmit(&hlpuart1,(uint8_t*)ch,1,1000);
//	return ch;
//}

void QMC7983_config(void){
	uint8_t config_data[4]={0x00,
							0x35,
							0xFF,
							0x40};
	HAL_I2C_Mem_Read(&hi2c1, QMC7983_ADDR,CHIPID_REG,1, config_data, 1,1000); // check device ID
	if(config_data[0]!=0){
		HAL_I2C_Mem_Write(&hi2c1,QMC7983_ADDR ,CONTROL_REG_1,1, config_data+1, 1,1000); //set Mode,ODR,RNG,OSR (full scale range of 16G)
		HAL_I2C_Mem_Write(&hi2c1,QMC7983_ADDR ,SET_RESET_PERIOD_REG,1, config_data+2, 1,1000); //set SET/RESET Period
		HAL_I2C_Mem_Write(&hi2c1,QMC7983_ADDR ,CONTROL_REG_2,1, config_data+3, 1,1000); //enable the roll-over function
	}
}

void QMC7983_read_magn_field(void){
	uint8_t status_flag = 0x00;
	uint8_t raw_data[6];
	HAL_I2C_Mem_Read(&hi2c1, QMC7983_ADDR,STATUS_REG_1,1, &status_flag, 1,1000);
	if((status_flag & 1U<<0)==1){
		HAL_I2C_Mem_Read(&hi2c1, QMC7983_ADDR,DATA_X_LSB_REG,1, raw_data, 6,1000);
		//because of full scale range is +/-16 , then the coefficient must be 1/1.000 according to datasheet
		//1G = 10^-4 T
		mag_fi_meas[0] = (int16_t)(raw_data[1]<<8 | raw_data[0])/1000.0;
		mag_fi_meas[1] = (int16_t)(raw_data[3]<<8 | raw_data[2])/1000.0;
		mag_fi_meas[2] = (int16_t)(raw_data[5]<<8 | raw_data[4])/1000.0;
		mag_fi_x = mag_fi_meas[0];
		mag_fi_y = mag_fi_meas[1];
		mag_fi_z = mag_fi_meas[2];
		total_magn = sqrtf(mag_fi_meas[0]*mag_fi_meas[0]+mag_fi_meas[1]*mag_fi_meas[1]+mag_fi_meas[2]*mag_fi_meas[2]);
	}
}

void QMC7983_read_temp (void){
	uint8_t status_flag = 0x00;
	uint8_t raw_temp_data[2];

	HAL_I2C_Mem_Read (&hi2c1, QMC7983_ADDR,STATUS_REG_1,1, &status_flag, 1,1000);
	if((status_flag & 1U<<0)==1)
	{
		HAL_I2C_Mem_Read (&hi2c1, QMC7983_ADDR,TOUT_LSB_REG,1, raw_temp_data, 2,1000);
		temp_meas = (int16_t)(raw_temp_data[1]<<8 |raw_temp_data[0])/100.0;
	}
}


void QMC7983_read_heading (void){

	QMC7983_read_magn_field();
	if(mag_fi_meas[0]==0)
	{
		if(mag_fi_meas[1]<0)
			heading = 90;
		else
			heading = 0;
	}
	else
	{
		heading =  atan2f(mag_fi_meas[1],mag_fi_meas[0])*180/PI - 11.71; //11.71 magnetic declination in spb
		if(heading>=360)
			heading -= 360;
		if(heading<0)
			heading += 360;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		ADC_Rad = sensor_raw_vals[0]*(3.3/4096); // 2^12
		ADC_ISence1 = sensor_raw_vals[1]*(3.3/4096/0.8);  //  Amper
		ADC_ISence2 = (sensor_raw_vals[2]-2.45/3.3*4096)*(3.3/4096/0.8);  //  Amper
		ADC_Ilum = sensor_raw_vals[3]*(3.3/4096);
}


void Current_Left(uint8_t PWM)
{
	HAL_GPIO_WritePin(Coil_Up_GPIO_Port, Coil_Up_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Coil_Dwn_GPIO_Port, Coil_Dwn_Pin, GPIO_PIN_SET);

	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_3);//Up
	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);//Dwn

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);

}

void Current_Rigth(uint8_t PWM)
{
	HAL_GPIO_WritePin(Coil_Dwn_GPIO_Port, Coil_Dwn_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Coil_Up_GPIO_Port, Coil_Up_Pin, GPIO_PIN_SET);

	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);//Dwn
	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_3);//Up


	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //as7265x_config();
  QMC7983_config();
  HAL_Delay(100);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)sensor_raw_vals,4);
  HAL_TIM_Base_Start_IT(&htim2);
  //Current_Rigth(50);
  //Current_Left(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  QMC7983_read_heading();
	  HAL_Delay(100);
	  QMC7983_read_temp ();
	  HAL_Delay(100);
//	  as7265x_read();
//	  HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
//  as7265x_soft_reset();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
//	uint16_t captured_val = 0 ;
//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
//		captured_val = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
//	}
//
//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
//		captured_val = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
//	}
//	return;
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
