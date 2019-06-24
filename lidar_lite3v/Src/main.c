/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *  LIDAR LITE VER 3 WITH STM32F407 DISCOVERY
 *  It should work on STM32F407 and all other STM32Fxxx devices
 *
 *  @author     jagadish  jadhav
 *  @email      jagadish.etc@gmail.com
 *  @ide        System Workbence (eclipse)
 *

`* PIN DESCIPTION
 * i uesd the i2c1 interface of stm32 to communicate with lidar & uart 2 to send distace to monitor/screen
 * we can we any other i2c and uart. we just need to change respective handle type def.
 * CONNECTION:
 * 			STM32F407						LIDAR LITR V3
 * 				VCC								VCC
 * 				GND								GND
 * 			I2C1-PB6(SCL)						SCL
 * 			I2C1-PBA(SDA)						SDA
 * 				PA2-TX
 * 				PA3-RX
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LIDAR_ADD 0x62<<1										// i2c slave address of lidar lite

uint32_t m_distance,object_distance;							// define the variable to get distance value
uint8_t cmd[1];
uint8_t data[2]={10};
char str[100];
/* USER CODE END PTD */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;									// handle type def for i2c1

UART_HandleTypeDef huart2;									// handle type def for uart2

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	configuration_set(4);    										// select any mode of operation of lidar i used mode 4
	/* USER CODE END 2 */

	/* Infinite loop */

	while (1)
	{
		/* USER CODE BEGIN 3 */
		object_distance = Get_distance();							// function give distance of detected object in cm

		sprintf(str,"%d\t cm\n\r", object_distance);				// send distance through uart
		HAL_UART_Transmit(&huart2,(uint8_t *)str,sizeof(str),10);
		HAL_Delay(50);												// 50ms delay
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

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
	hi2c1.Init.ClockSpeed = 100000;
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void configuration_set(int configur)      // configuration setting for lidar. it provide different mode of detection
{

	cmd[0] = 0x04;
	HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x00,1,cmd,1,0x100);
	switch(configur)
	{
	case 0://default mode , balance mode
		cmd[0]=0x80;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x04;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0x00;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;

	case 1://short range, high speed
		cmd[0]=0x1d;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x08;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0x00;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;

	case 2://default range, higher speed short range
		cmd[0]=0x80;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x08;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0x00;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;


	case 3://maximum Range
		cmd[0]=0xff;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x08;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0x00;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;

	case 4://high sensitivity detection, high  measurement
		cmd[0]=0x80;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x08;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0x80;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;

	case 5://low sensitivity detection , low  measurement
		cmd[0]=0x80;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x02,1,cmd,1,0x1000);
		cmd[0]=0x08;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x04,1,cmd,1,0x1000);
		cmd[0]=0xb0;
		HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x1c,1,cmd,1,0x1000);
		break;
	}
}

/* USER CODE END 4 */
int Get_distance()     // function to get distance
{
	cmd[0]=0x04;
	HAL_I2C_Mem_Write(&hi2c1,LIDAR_ADD ,0x00,1,cmd,1,100);
	cmd[0]=0x8f;
	HAL_I2C_Master_Transmit(&hi2c1,LIDAR_ADD,cmd,1,100);
	HAL_I2C_Master_Receive(&hi2c1,LIDAR_ADD,data,2,100);
	m_distance = (data[0]<<8)|(data[1]);
	return m_distance ;
}

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
