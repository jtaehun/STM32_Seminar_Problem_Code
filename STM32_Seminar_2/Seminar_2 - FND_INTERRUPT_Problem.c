#include "main.h"
#include "string.h"
#include "stdbool.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////// 초기 설정 ////////////////////////////////////////////////////////

// LED
int LED[8] = {LED_1_Pin, LED_2_Pin, LED_3_Pin, LED_4_Pin, LED_5_Pin, LED_6_Pin, LED_7_Pin, LED_8_Pin}; // LED 핀 배열

GPIO_TypeDef* LED_PORT[8] = {LED_1_GPIO_Port, LED_2_GPIO_Port, LED_3_GPIO_Port, LED_4_GPIO_Port, LED_5_GPIO_Port, LED_6_GPIO_Port, LED_7_GPIO_Port, LED_8_GPIO_Port}; // LED 포트 배열


// FND
int FND[8] = {FND_A_Pin, FND_B_Pin, FND_C_Pin, FND_D_Pin, FND_E_Pin, FND_F_Pin, FND_G_Pin, FND_DP_Pin}; // FND 핀 배열

GPIO_TypeDef* FND_PORT[8] = {FND_A_GPIO_Port, FND_B_GPIO_Port, FND_C_GPIO_Port, FND_D_GPIO_Port, FND_E_GPIO_Port, FND_F_GPIO_Port, FND_G_GPIO_Port, FND_DP_GPIO_Port}; // FND 포트 배열


// TRANSISTOR
int TRANSISTOR[2] = {TRANSISTOR_1_Pin, TRANSISTOR_2_Pin}; // TRANSISTOR 핀 배열

GPIO_TypeDef* TRANSISTOR_PORT[2] = {TRANSISTOR_1_GPIO_Port, TRANSISTOR_2_GPIO_Port}; // TRANSISTOR 포트 배열


// 0 ~ 9
uint8_t FND_PATTERN[10] = {0xBF, 0x86, 0xDB, 0xCF, 0xE6, 0xED, 0xFD, 0x87, 0xFF, 0xEF}; // FND 핀 배열에 따라 계산 Ex) 0 - 1011 1111 - 0xBF


//////////////////////////////////////////////////// LED, FND 초기화 ////////////////////////////////////////////////////

void LED_FND_INIT(void)
{
	for (int i = 0; i < 8; i++)
	{
		LED_PORT[i] -> BSRR = LED[i];

	}

	for (int i = 0; i < 8; i++)
	{
		FND_PORT[i] -> BSRR = FND[i];
	}

}


//////////////////////////////////////////////////// FND 제어 ////////////////////////////////////////////////////

int number = 0; // FND에 표현할 숫자 넘버

void FND_Display(void)
{

	int One_Number = number % 10; // 1의 자리 숫자 계산
	int Ten_Number = number / 10; // 10의 자리 숫자 계산

	for (int i = 0; i < 8; i++)
	{
		if ((FND_PATTERN[Ten_Number] >> i) & 0x01)
		{
			FND_PORT[i] -> BSRR = FND[i] << 0x10;
		}
		else
		{
			FND_PORT[i] -> BSRR = FND[i];
		}


	}

	// TRANSISTOR[1] - 왼쪽 FND 제어 즉, 10의 자리 제어
	TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[0] << 0x10;
	TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[1];

	HAL_Delay(10); // 매우 짧은 시간 동안 십의 자리와 일의 자리 전환 (동적 구동)

	for (int i = 0; i < 8; i++)
	{

		if ((FND_PATTERN[One_Number] >> i) & 0x01)
		{
			FND_PORT[i] -> BSRR = FND[i] << 0x10;
		}
		else
		{
			FND_PORT[i] -> BSRR = FND[i];
		}


	}

	TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[1] << 0x10;
	TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[0];

	HAL_Delay(10); // 매우 짧은 시간 동안 십의 자리와 일의 자리 전환 (동적 구동)

}


//////////////////////////////////////////////////// 인터럽트 ////////////////////////////////////////////////////

int Flag_1 = 0;
int Flag_2 = 0;
int Flag_3 = 0;
int Flag_4 = 0;

int Last_Time_1 = 0;
int Last_Time_2 = 0;
int Last_Time_3 = 0;
int Last_Time_4 = 0;



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

int Current_Time = HAL_GetTick();

  if(GPIO_Pin == BUTTON_1_Pin)
  {
	  if((Current_Time - Last_Time_1) > 20)
	  {
		  Flag_2 = 0;
		  Flag_3 = 0;
		  Flag_4 = 0;
		  Flag_1 = 1;

		  Last_Time_1 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_2_Pin)
  {
	  if((Current_Time - Last_Time_2) > 20)
	  {
		  Flag_1 = 0;
		  Flag_3 = 0;
		  Flag_4 = 0;
		  Flag_2 = 1;

		  Last_Time_2 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_3_Pin)
  {
	  if((Current_Time - Last_Time_3) > 20)
	  {
		  Flag_1 = 0;
		  Flag_2 = 0;
		  Flag_4 = 0;
		  Flag_3 = 1;

		  Last_Time_3 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_4_Pin)
  {
	  if((Current_Time - Last_Time_4) > 20)
	  {
		  Flag_1 = 0;
		  Flag_2 = 0;
		  Flag_3 = 0;
		  Flag_4 = 1;

		  Last_Time_4 = Current_Time;
	  }
  }
}

//////////////////////////////////////////////////// Problem_1 ////////////////////////////////////////////////////


int display_count = 0;

void Problem_1(void) // FND 0 ~ 99 증가 (99 -> 0)
{


	    display_count++;
	    if (display_count >= 10) { // 20ms * 5 = 100ms마다 숫자 증가
	        number++;
	        display_count = 0; // 카운트 초기화
	    }

		if (number > 99) // 99를 초과하면 0으로 다시 초기화
		{
			number = 0;
		}

}


//////////////////////////////////////////////////// Problem_2 ////////////////////////////////////////////////////


void Problem_2(void) // FND 99 ~ 0 감소 (0 -> 99)
{

		while(!(GPIOC->IDR & BUTTON_2_Pin))
		{
			FND_Display();

		    display_count++;
		    if (display_count >= 10) { // 20ms * 5 = 100ms마다 숫자 증가
		        number--;
		        display_count = 0; // 카운트 초기화
		    }

			if (number == 0) // 0이면 99로 이동
			{
				number = 99;
			}
		}

}


//////////////////////////////////////////////////// Problem_3 ////////////////////////////////////////////////////

int j = -1; // j를 -1로 초기화

void Problem_3(void) // 누를 때마다 LED, FND 증가
{

		LED_PORT[j] -> BSRR = LED[j]; // LED OFF

		j++; // 인덱스 0부터 시작 즉, 누르면, LED[0] 부터 ON

		LED_PORT[j] -> BSRR = LED[j] << 0x10; // LED ON

		if (j > 7) // 7을 초과하면 다시 j를 0으로 초기화
		{
			j = 0;
			LED_PORT[j] -> BSRR = LED[j] << 0x10;
		}

		number++; // 버튼을 누를 때마다 FND 숫자값이 1씩 증가

		if (number > 99) // 99를 초과하면 0으로 다시 초기화
		{
			number = 0;
		}

}



//////////////////////////////////////////////////// Problem_4 ////////////////////////////////////////////////////


void Problem_4(void) // FND를 0으로 RESET
{

	number = 0; // FND 숫자값을 0으로 초기화

}




//////////////////////////////////////////////////// main 문 ////////////////////////////////////////////////////


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();



  EXTI->RTSR= (1<<9)|(1<<12)|(1<<11);
  EXTI->FTSR= (1<<6);

  LED_FND_INIT(); // LED, FND 초기화


  while (1)
  {
	  FND_Display(); // FND ON

	  if(Flag_1 == 1)
	  {
		  Problem_1();
	  }
	  else if(Flag_2 == 1)
	  {
		  Problem_2();

		  Flag_1 = 1;
	  }
	  else if(Flag_3 == 1)
	  {
		  Problem_3();

		  HAL_Delay(100);

		  Flag_3 = 0;
	  }
	  else if(Flag_4 == 1)
	  {
		  Problem_4();
	  }



  }

}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, FND_E_Pin|FND_D_Pin|FND_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_6_Pin|LED_7_Pin|LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FND_B_Pin|FND_A_Pin|FND_C_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FND_DP_Pin|FND_F_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TRANSISTOR_1_Pin|TRANSISTOR_2_Pin|LED_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FND_E_Pin FND_D_Pin FND_G_Pin */
  GPIO_InitStruct.Pin = FND_E_Pin|FND_D_Pin|FND_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_6_Pin LED_7_Pin LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_6_Pin|LED_7_Pin|LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin FND_B_Pin FND_A_Pin LD3_Pin
                           FND_C_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|FND_B_Pin|FND_A_Pin|LD3_Pin
                          |FND_C_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FND_DP_Pin FND_F_Pin */
  GPIO_InitStruct.Pin = FND_DP_Pin|FND_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_4_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_4_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRANSISTOR_1_Pin TRANSISTOR_2_Pin */
  GPIO_InitStruct.Pin = TRANSISTOR_1_Pin|TRANSISTOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_1_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_5_Pin */
  GPIO_InitStruct.Pin = LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_8_Pin */
  GPIO_InitStruct.Pin = LED_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_8_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
