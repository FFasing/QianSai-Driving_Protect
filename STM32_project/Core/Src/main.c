/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx1[50];
int rx1_ok = 0;
int rx1_count = 0;

uint8_t rx2[256];
int rx2_ok = 0;
int rx2_count = 0;

uint8_t rx3[2][150];
uint8_t tx3_0_5s[]={"$PCAS02,1000*2E\r\n"};
uint8_t tx3_GNNS[]={"$PCAS04,7*1E\r\n"};
uint8_t tx3_vtg_gga[]={"$PCAS03,1,0,0,0,0,1,0,0,0,0,,,0,0,,,,0*32\r\n"};
uint8_t tx[20]={'\0'};
int rx3_ok = 0;
int rx3_count = 0;
int32_t speed = 0;//速度
uint8_t position[30];//经纬度存储数组
int N_num = 0,K_num = 0;
int gps_ok = 1;
int pps = 0;
int jingdu = 10240,weidu = 2212,jingdu_poi = 55136,weidu_poi = 39661;
double weidu_real = 0,jingdu_real = 0;
double weidu_real2 = 0,jingdu_real2 = 0;
uint8_t rx6[50];
int rx6_ok = 0;
int rx6_count = 0;
int32_t juli = 0;

int mode = 0;//0摄像头，1雷达״�
int a = 0;
int F_beep = 10;
int key_long = 0;
int time = 0;
int tim13_ok = 0;
bool beep_mode=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uint unsigned int
#define uchar unsigned char

struct keys
{
	unsigned char judge_sta;
	bool key_sta;
	bool single_flag;
	bool long_flag;
	unsigned int key_time;
};
struct keys key[4];

struct keys key[4]={0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM13)
	{
		tim13_ok = 1;
	}
	if(htim->Instance==TIM11)
	{
		 key[0].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
//		 key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
//		 key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
//		 key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(uchar i=0;i<4;i++)	   //????4??????
		{
			//break????switch??????????4??for????????????????10ms???ж???????????????????????????????ó????????????????10m
			switch(key[i].judge_sta)
			{
				case 0:
				{
					if(key[i].key_sta==0) //???key[i].key_sta==0?????????￡???????????????????
					{
						key[i].judge_sta=1;
						key[i].key_time=0;
					}
					break;				
				}
				case 1:
				{
					if(key[i].key_sta==0) 	 //???10ms???ж????0????????????
					{
						key[i].judge_sta=2;  //??????????ж?????
					}
					else
					{
						key[i].judge_sta=0;
					}
					break;					
				}
				case 2:
				{
					if(key[i].key_sta==1) 	 //?????1????????????????
					{
						key[i].judge_sta=0;  //????????
						if(key[i].key_time<=50)
						{
							key[i].single_flag=1;//???λ??1
						}
					}
					else
					{
						key[i].key_time++;
						if(key[i].key_time>50) //???????3???700ms???ж???????????10ms?????Σ??????????70
						{
							key[i].long_flag=1;
						}
					}					
				}
				break;
			}
		}
	}
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int32_t dist1=23;
uint8_t k210_rx[20],k210_rx_length=0,k210_rx_ok=0;
uint8_t k210_warning=0;
//put into main that init of k210
void k210_init(UART_HandleTypeDef *huart)
{
	__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}
//put into USARTX_IRQHandler that interrupt of uart
void k210_uart_interrupt(UART_HandleTypeDef *huart, uint8_t rx_buffer[],uint8_t *rx_length, uint8_t *rx_ok)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE)!=0)
	{
		if(*rx_length==0)	memset((char *)rx_buffer,'\0',20);
		HAL_UART_Receive(huart,&rx_buffer[*rx_length],1,HAL_MAX_DELAY);
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_RXNE);
		uint8_t i;
		i=*rx_length;
		i++;
		*rx_length=i;
	}
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)!=0)
	{
		if(*rx_ok==0) *rx_ok=1;
		*rx_length=0;
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_IDLE);
	}
}
//this is used to stm32 receive information of k210 by useing uart
//temp is 1 that sentinel mode warning
//temp is 2 that nomoral mode mild warning
//temp is 3 that nomoral mode severe warning
void k210_uart_receive(uint8_t rx_buffer[], uint8_t *rx_ok, uint8_t *temp)
{
	uint8_t flag=*rx_ok;
	if(flag)
	{
		flag=0;
		if(strcmp((char *)rx_buffer,"Car")==0)
		{
			*temp=1;
		}
		if(strcmp((char *)rx_buffer,"car")==0)
		{
			*temp=2;
		}
		if(strcmp((char *)rx_buffer,"CAR")==0)
		{
			*temp=3;
		}
	}
}
//this is used to convert the K210 mode
void k210_transform_mode(UART_HandleTypeDef *huart)
{
	uint8_t text[20];
	sprintf((char *)text,"mod");
	HAL_UART_Transmit(huart,text,strlen((char *)text),HAL_MAX_DELAY);
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==TIM14)
//	{
//		key_long = 1;
//		if(mode == 0)--
//			mode = 1;
//		else if(mode == 1)
//			mode = 0;
//		HAL_TIM_Base_Stop_IT(&htim14);
//	}
//	if(htim->Instance==TIM13)
//	{
//		tim13_ok = 1;
//	}
//}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) 
{
  HAL_UART_Transmit(&huart1,(unsigned char*)&ch,1,50);
  return ch;
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
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim14);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
	printf("666");
//	HAL_UART_Receive_IT(&huart6,&rx6,1);  //串口接收初始化
	HAL_Delay(500);
	HAL_UART_Transmit(&huart3,tx3_vtg_gga,sizeof(tx3_vtg_gga)-1,100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart3,tx3_GNNS,sizeof(tx3_GNNS)-1,100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart3,tx3_0_5s,sizeof(tx3_0_5s)-1,100);
	ESP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  k210_init(&huart1);
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_3);
  while (1)
  {

		k210_warning = 0;
		memset((char *)k210_rx,'\0',20);
		k210_uart_receive(k210_rx,&k210_rx_ok,&k210_warning);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		////////////////////////////////////////////////////////////////////////////////////////////////
		//wifi
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);

	  if(tim13_ok == 1)
		{
			tim13_ok = 0;
			Esp_PUB(juli,44,mode,weidu_real2,jingdu_real2,speed,time++);
//			Esp_PUB(888,44,mode,33.666666,333.333333,128,time++);
			if(time > 6)
			time = 0;
			HAL_Delay(100);
		}
		////////////////////////////////////////////////////////////////////////////////////////////////
		//蜂鸣器警报
		if(mode==0)
		{
			
			if(k210_warning == 1)
			{
				F_beep = 2;
				__HAL_TIM_SetAutoreload(&htim12,10000/F_beep);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(10000/F_beep)*0.5);
				HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			}
			else if(k210_warning == 2)
			{
				F_beep = 5;
				__HAL_TIM_SetAutoreload(&htim12,10000/F_beep);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(10000/F_beep)*0.5);
				HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			}
			else if(k210_warning == 3)
			{
				F_beep = 10;
				
				__HAL_TIM_SetAutoreload(&htim12,10000/F_beep);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(10000/F_beep)*0.5);
				HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			}
			else if(k210_warning == 0)
			{
				HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
			}
			//k210_warning = 0;
			//memset((char *)k210_rx,'\0',20);
		}
		else if(mode == 1)
		{
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			if(juli>=250)
			{
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
			}
			else if(juli>=200&&juli<250)
			{
				F_beep = 1;
			}
			else if(juli>=150&&juli<200)
			{
				F_beep = 3;
			}
			else if(juli>=100&&juli<150)
			{
				F_beep = 5;
			}
			else if(juli>=50&&juli<100)
			{
				F_beep = 7;
			}
			else if(juli>=0&&juli<50)
			{
				F_beep = 9;
			}
		}
		__HAL_TIM_SetAutoreload(&htim12,10000/F_beep);
		__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(10000/F_beep)*0.5);
		HAL_Delay(500);
		
		
		if(beep_mode==1)
		{
			F_beep = 5;
			__HAL_TIM_SetAutoreload(&htim12,10000/F_beep);
			__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(10000/F_beep)*0.5);
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
		}
		else if(beep_mode==0)
		{
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
		}
////		else if(mode == 1)
////		{
////			__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
////			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);
////		}
////		else if(juli>=0&&juli<200)
////		{
////			F_beep = -(19.0/169.0)*juli+(3950.0/169.0);
////		}
		////////////////////////////////////////////////////////////////////////////////////////////////
		//模式灯
		if(mode == 0)
		{
			HAL_GPIO_WritePin(SHAOBING_MODE_GPIO_Port, SHAOBING_MODE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DRIVE_MODE_GPIO_Port, DRIVE_MODE_Pin, GPIO_PIN_RESET);
		}
		else if(mode == 1)
		{
			HAL_GPIO_WritePin(SHAOBING_MODE_GPIO_Port, SHAOBING_MODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DRIVE_MODE_GPIO_Port, DRIVE_MODE_Pin, GPIO_PIN_SET);
		}
		////////////////////////////////////////////////////////////////////////////////////////////////
		//模式切换按键
//		if(HAL_GPIO_ReadPin(MODE_GPIO_Port,MODE_Pin)==0&&a==0)
//		{
//			__HAL_TIM_SET_COUNTER(&htim14,0);
//			HAL_TIM_Base_Start_IT(&htim14);
//			a = 1;
//		}
//		else if(HAL_GPIO_ReadPin(MODE_GPIO_Port,MODE_Pin)==1&&a==1)
//		{
//			HAL_TIM_Base_Stop_IT(&htim14);
//			if(key_long==0)
//				k210_transform_mode(&huart1);
//			else if(key_long==1)
//				key_long = 0;
//			a = 0;
//		}
		if(key[0].single_flag==1)
		{
			k210_transform_mode(&huart1);
//			beep_mode=!beep_mode;
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_3);
			key[0].single_flag=0;
		}
		if(key[0].long_flag==1)
		{
			mode=!mode;
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_3);
			key[0].long_flag=0;
		}
		
		////////////////////////////////////////////////////////////////////////////////////////////////
		//K210，串口1
		if(k210_rx_ok == 1)
		{
			
		}
		////////////////////////////////////////////////////////////////////////////////////////////////
		//GPS，串口3
		if(rx3_ok==1)
		{
			rx3_ok = 0;
			if(rx3[0][N_num+7]=='K')
			{
				speed = rx3[0][N_num+2]-48;
				sprintf((char*)tx,"%dkm/h",speed);
				HAL_UART_Transmit(&huart1,tx,5,100);
			}
			else if(rx3[0][N_num+8]=='K')
			{
				speed = (rx3[0][N_num+2]-48)*10+(rx3[0][N_num+3]-48);
				sprintf((char*)tx,"%dkm/h",speed);
				HAL_UART_Transmit(&huart1,tx,6,100);
			}
			else if(rx3[0][N_num+9]=='K')
			{
				speed = (rx3[0][N_num+2]-48)*100+(rx3[0][N_num+3]-48)*10+(rx3[0][N_num+4]-48);
				sprintf((char*)tx,"%dkm/h",speed);
				HAL_UART_Transmit(&huart1,tx,7,100);
			}
			if(rx3[1][1]=='G'&&rx3[1][2]=='N'&&rx3[1][3]=='G'&&rx3[1][4]=='G'&&rx3[1][5]=='A')
			{
				for(int i = 18;i<44;i++)
				{
					position[i-18] = rx3[1][i];
				}
				sscanf((char*)position,"%d.%d,N,%d.%d,E",&weidu,&weidu_poi,&jingdu,&jingdu_poi);
				weidu_real = (weidu/10%10*10+weidu%10)+(weidu_poi/100000.0);
				jingdu_real = (jingdu/10%10*10+jingdu%10)+(weidu_poi/100000.0);
				weidu_real2 = (weidu/100+weidu_real/60.0);
				jingdu_real2 = (jingdu/100+jingdu_real/60.0);
				//HAL_UART_Transmit(&huart2,position,30,100);
			}
		}
		////////////////////////////////////////////////////////////////////////////////////////////////
		//雷达，串口6
		if(rx6_ok==1)
		{
			rx6_ok = 0;
			juli = rx6[4]*256 + rx6[3];
			juli = 100;
			printf("%d\n",juli);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
