/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	TEMP 0
#define HUMI 1
#define SHT2x_ADDR							(0x40	<< 1)  
#define SHT2x_HOLD_MASTER_T			0xE3
#define SHT2x_HOLD_MASTER_RH		0xE5
#define SHT2x_NOHOLD_MASTER_T		0xF3
#define SHT2x_NOHOLD_MASTER_RH 	0xF5
#define SHT2x_WRITE_USER_REG 		0xE6
#define SHT2x_READ_USER_REG			0xE7
#define SHT2x_SOFR_RESET				0xFE
#define STK_CTRL 	*(unsigned int*)	0xE000E010
#define STK_LOAD 	*(unsigned int*)	0xE000E014
#define STK_VAL  	*(unsigned int*)	0xE000E018
#define STK_CALIB *(unsigned int*)	0xE000E01C
	
#define ENABLE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxData;
FILE __stdout;
uint16_t adcData[2];
uint32_t pwmF;
uint16_t scale[] = {523, 587, 659, 698, 783, 880, 987, 1046};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE* stream)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch, 1, 0xffff);
	return ch;
}
void menu();
void shiftLED();
void MoodLight();
void Piano();
void streetLight();
void Tem();
void SHT20_Init();
float SHT20(int);
void SSensor();
void usDelay(uint16_t us);
void SysTic_Init();
void HAL_Delay_Porting();
void Ssound_on();
void Ssound_off();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &rxData, sizeof(rxData));
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData, 1);
	
	HAL_TIM_Base_Start(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	menu();
  while (1)
  {
		HAL_Delay(500);
		if(rxData == '1'){
			shiftLED();
			}
		else if(rxData == '2'){
			MoodLight();
			}
		else if(rxData == '3'){
			Piano();
			}
		else if(rxData == '4'){
			streetLight();
			}
		else if(rxData == '5'){
			Tem();
			}
		else if(rxData == '6'){
			SSensor();
		}
		else if(rxData == '0'){
			printf("program Exit");
			return 0;
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 	if(huart->Instance == USART2) {
		HAL_UART_Receive_IT(&huart2, &rxData, sizeof(rxData));
	}
}
void menu()																									// 메뉴 ?¨?
{		
		printf("\r\n");
		printf("** M E N U **\r\n");
		printf("1. LED Shift\r\n");
		printf("2. Mood Light\r\n");
		printf("3. Piano \r\n");
		printf("4. Street Light\r\n");
		printf("5. TEMP / HUMI \r\n");
		printf("6. Sensor\r\n");
		printf("0. Program Exit \r\n");
		//printf("select >>    ");
		//printf("%c", rxData);
}
void shiftLED()																							// LED?¬??¸
{
	printf("\r\n");
	printf("LED Right : r , LED Left : l , LED stop : s\r\n");
	printf("Return Menu : m");
	static int n = 0;
	while(1){
	//printf("1.");
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin << n, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin << n, 0);
	if(rxData == 'm') {menu(); break;}	
	else{
		if(rxData == 'r') {n++;}
		else if(rxData == 'l') {n--;}
		else if(rxData == 's') {n = n;}
		if (n>=8){ n=0;}
		else if ( n<0 ) {n = 7;}	
	}
	//HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin << n, 1);
	//HAL_Delay(500);
	//HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin << n, 0);

	//if (n>=8){ n=0;}
	//else if ( n<0 ) {n = 7;}
	//else if(rxData == 'm') {menu(); break;}
	}	
}
void Piano1()
{
	TIM1->ARR = pwmF - 1;
	TIM1->CCR3 = pwmF / 2;
	HAL_Delay(100);
	TIM1->ARR = 0;
	TIM1->CCR3 = 0; 
	rxData = 0;
}
void Piano()
{
	printf("\r\n");
	printf("Piano Mode Start\r\n");
	printf("Key : a,s,d,f,g,h,j,k\r\n");
	printf("Return Menu : m");
	while(1){
		HAL_Delay(100);
		if(rxData == 'm') {menu(); break;}
		else{
			if(rxData == 'a')
			{
				pwmF = 10000000 / scale[0]; 
				Piano1();
			}
			else if(rxData == 's')
			{
				pwmF = 10000000 / scale[1]; 
				Piano1();
			}
			else if(rxData == 'd')
			{
				pwmF = 10000000 / scale[2]; 
				Piano1();
			}
			else if(rxData == 'f')
			{
				pwmF = 10000000 / scale[3]; 
				Piano1();
			}
			else if(rxData == 'g')
			{
				pwmF = 10000000 / scale[4]; 
				Piano1();
			}	
			else if(rxData == 'h')
			{
				pwmF = 10000000 / scale[5]; 
				Piano1();
			}
			else if(rxData == 'j')
			{
				pwmF = 10000000 / scale[6]; 
				Piano1();
			}
			else if(rxData == 'k')
			{
				pwmF = 10000000 / scale[7]; 
				Piano1();
			}
		}	
	}
}
void MoodLight()																							// 무드?±
{
	uint32_t CCRVal = 0;
	printf("\r\n");
	printf("MoodLight Mode Start\r\n");
	printf("Return Menu : m \r\n");
	int change = 5;
	
	while(1)
	{
		TIM3->CCR1 = CCRVal;
		HAL_Delay(100);
		CCRVal += change;
		
		if(rxData == 'm') {
			TIM3->CCR1 = 0;
			menu(); 
			break;
		}
		else if(CCRVal >= 50 || CCRVal <= 0)
		{
			change = -change;
		}
	}
}
void streetLight()
{
	static int i = 0;
	printf("\r\n");
	printf("Street Light Mode Start\r\n");
	printf("Return Menu: m \r\n");
	while(1){
		printf("adc : %d\r\n", adcData[0]);
		if(rxData == 'm') {menu(); break;}
		else
		{
			HAL_Delay(1000);
			if(adcData[0] > 4000){
				for(i=0; i<8;i++){
					HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin>>i, 0);
					HAL_Delay(100);}
			}
			else if(adcData[0] > 3500 && adcData[0] < 3800){
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);}
			else if (adcData[0] > 3000 && adcData[0] < 3500){
				for(i=0; i<3;i++){
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin<<i, 1);
					HAL_Delay(100);
				}
				for(i=0; i<5; i++){
					HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin>>i, 0);
					HAL_Delay(100);
				}
			}				
			else if (adcData[0] > 2500 && adcData[0] < 3000){ 
				for(i=0; i<5;i++){
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin<<i, 1);
					HAL_Delay(100);}
				for(i=0; i<3; i++){
					HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin>>i, 0);
					HAL_Delay(100);
				}
			}
			else if (adcData[0] > 2000 && adcData[0] < 2500){
				for(i=0; i<7;i++){
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin<<i, 1);
					HAL_Delay(100);
				}
				for(i=0; i<1; i++){
					HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin>>i, 0);
					HAL_Delay(100);
				}
			}
			else if (adcData[0] > 1500 && adcData[0] < 2000){
				for(i=0; i<8;i++){
						HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin<<i, 1);
						HAL_Delay(100);}
			}
						
				//if(adcData[0] <= 3500){
				//HAL_Delay(200);
				//HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);}
			//else{HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
		}
			
				
	}
}	
void SHT20_Init()
{
	if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SHT2x_ADDR, (uint8_t*)SHT2x_SOFR_RESET, 1, 0xffff) == HAL_OK) {
		printf("SHT20 RESET FAIL!!");
	} else {
			printf("SHT20 REST SUCCESS!!");
	}	
}
float SHT20(int select)
{
	uint8_t I2CData[3];
	uint16_t SLAVER_ADDR = SHT2x_ADDR;
	uint16_t sensor;
	float ConverData = 0.00;
	if(select == TEMP) {
		I2CData[0] = SHT2x_NOHOLD_MASTER_T;
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVER_ADDR, (uint8_t*)I2CData,1, 0xffff);
		HAL_Delay(100);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)SLAVER_ADDR, (uint8_t*)I2CData, 2, 0xffff);
		//I2CData[0], I2CData[1],
		sensor = I2CData[0] << 8 | I2CData[1];
		ConverData= -46.85 + 175.72 / 65536 * (float)sensor; }
	if(select == HUMI){
		I2CData[0] = SHT2x_NOHOLD_MASTER_RH;
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVER_ADDR, (uint8_t*)I2CData, 1, 0xffff);
		HAL_Delay(100);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)SLAVER_ADDR, (uint8_t*)I2CData, 2, 0xffff);
		sensor = I2CData[0] << 8 | I2CData[1];
		ConverData = -6.0 + 125.0 / 65536 * (float)sensor;
		}
	return ConverData;
}
void Tem()
{
	printf("\r\n");
	printf("Temperature, Humidity Mode Start\r\n");
	printf("Return Menu: m \r\n");
	static float temperature, humidity;
	while(1){
		if(rxData == 'm') {menu(); break;}
		else{
		temperature = SHT20(TEMP);
		humidity = SHT20(HUMI);
		printf("temp : %.2f HUMI : %.2f\r\n ", temperature, humidity);
		HAL_Delay(500);}
	}
}
void SSensor()
{
		uint16_t cnt = 0;
		uint32_t EchoTime = 0;
		printf("Back Sensor Mode Start\r\n");
		while(1){
			SysTic_Init();
			HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, 1);
			usDelay(15);
			HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, 0);
			while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) ==0);
			STK_CTRL |=(1<<ENABLE); //systick timer start
			while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1);
			EchoTime = HAL_GetTick();
			STK_CTRL &= ~(1<<ENABLE);
		
			printf("Distance = %.1lf cm\r\n", EchoTime /2 * 0.034);

			HAL_Delay_Porting();
			HAL_Delay(200);
			
			uint32_t	t= (EchoTime /2 * 0.034) ;
			if(rxData == 'm') {menu(); break;}
			else{
				if(t > 35.0 ){
					Ssound_off();
				}					
				else if(t >= 25.0 && t <35){
					Ssound_on();
					HAL_Delay(500);
					Ssound_off();
				}
				else if(t >= 15.0 && t <25.0){
					Ssound_on();
					HAL_Delay(250);
					Ssound_off();
				}
				else if(t>10 && t <15){
					Ssound_on();
					HAL_Delay(100);
					Ssound_off();
				}
				else if(t <=10 && t >5)
				{
					Ssound_on();
					HAL_Delay(50);
					Ssound_off();
				}
				else if(t <= 5)
				{
					Ssound_on();
				}
			}
		}	
}
void Ssound_on()
{
	pwmF = 10000000 / 2093 ; 
	TIM1->ARR = pwmF - 1;
	TIM1->CCR3 = pwmF / 2;
}
void Ssound_off()
{
	TIM1->ARR = 0;
	TIM1->CCR3 = 0;
}
void usDelay(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim10,0);
	while(__HAL_TIM_GET_COUNTER(&htim10) < us);
}
void SysTic_Init()
{
	STK_LOAD =  100-1;
	STK_VAL = 0;
	STK_CTRL = 6;
	uwTick = 0;
}
void HAL_Delay_Porting()
{
	STK_LOAD = 100000 -1 ;
	STK_CTRL |= 7;	
}
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
