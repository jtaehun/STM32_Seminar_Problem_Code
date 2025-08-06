#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


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

ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


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

// 1 ~ F
uint8_t FND_PATTERN_2[15] = {0x86, 0xDB, 0xCF, 0xE6, 0xED, 0xFD, 0x87, 0xFF, 0xEF, 0xF7, 0xFC, 0xB9, 0xDE, 0xF9, 0xF1};

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
	  if((Current_Time - Last_Time_1) > 20) // 디바운싱 20
	  {
		  Flag_2 = 0;
		  Flag_3 = 0;
		  Flag_4 = 0;
		  Flag_1 = 1; // Flag_1 ON

		  Last_Time_1 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_2_Pin)
  {
	  if((Current_Time - Last_Time_2) > 20) // 디바운싱 20
	  {
		  Flag_1 = 0;
		  Flag_3 = 0;
		  Flag_4 = 0;
		  Flag_2 = 1; // Flag_2 ON

		  Last_Time_2 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_3_Pin)
  {
	  if((Current_Time - Last_Time_3) > 20) // 디바운싱 20
	  {
		  Flag_1 = 0;
		  Flag_2 = 0;
		  Flag_4 = 0;
		  Flag_3 = 1; // Flag_3 ON

		  Last_Time_3 = Current_Time;
	  }
  }

  else if(GPIO_Pin == BUTTON_4_Pin)
  {
	  if((Current_Time - Last_Time_4) > 20) // 디바운싱 20
	  {
		  Flag_1 = 0;
		  Flag_2 = 0;
		  Flag_3 = 0;
		  Flag_4 = 1; // Flag_4 ON

		  Last_Time_4 = Current_Time;
	  }
  }
}


//////////////////////////////////////////////////// UART ////////////////////////////////////////////////////


void UART3_INIT (void) // UART 초기화
{
	RCC -> APB1ENR |= (1<<18); // UART3 클럭 활성화
	RCC -> AHB1ENR |= (1<<3); // GPIOD 클럭 활성화

	GPIOD -> MODER |= (2<<16); // PD8(RX)- Alternate Function 모드 설정 (UART 통신)
	GPIOD -> MODER |= (2<<18); // PD9(TX) - Alternate Function 모드 설정 (UART 통신)

	GPIOD -> OSPEEDR |= (3<<16) | (3<<18); // PD8, PD9 - HIGH SPEED 설정

	USART3 -> CR1 = 0; // CR1 레지스터 초기화
	USART3 -> CR1 |= (1<<0); // USART 활성화
	USART3 -> CR1 |= (1<<2); // 수신(RX) 활성화
	USART3 -> CR1 |= (1<<3); // 송신(TX) 활성화

	USART3 -> BRR = (26<<4); // Baud Rate 설정 - baud : 115200, fCK : 48MHz
}

void UART3_Send(char *str) // 문자열 송신
{

	while (*str) // 문자열이 모두 전송될때 까지 while
	{
		USART3->TDR = *str++; // TDR 레지스터에 문자 저장 후 다음 문자 위치로 포인터 이동

		while (!(USART3->ISR & (1<<6))); // 전송 완료(TC) 플래그가 설정될 때까지 대기
    }
}


//////////////////////////////////////////////////// TIM ////////////////////////////////////////////////////


void TIM1_INIT(void)
{
	RCC -> APB2ENR |= (1<<0); // TIM1 활성화

	TIM1 -> CR1 = 0; // TIM1 설정 초기화
	TIM1 -> CR1 |= (0<<1); // UEV(Update Event) 활성화
	TIM1 -> CR1 |= (1<<4); // DOWN Counter mode 설정
	TIM1 -> CR1 |= (0<<5); // Edge-aligned mode 설정
	TIM1 -> CR1 |= (0<<7); // ARR(Auto Reload) 비활성화
	TIM1 -> CR1 |= (0<<8); // Clock DIVISION 비활성화

	TIM1 -> PSC = 96000 - 1; // 프리스케일러 값

	// APB2에 공급되는 클럭 48MHz
	// 96000000 / (96000 - 1 + 1) -> 1000Hz

	//TIM1 -> ARR = 0xFFFF;
	TIM1 -> ARR = 250 - 1; // ARR 값

	// 주기 = (250 - 1 + 1) / 1000Hz -> 250ms
	// 250ms 마다 Interrupt 발생

	TIM1 -> CR1 |= (1<<0); // 타이머 카운터 활성화

	while(!(TIM1 -> SR & (1<<0))); // 인터럽트 플레그가 발생할 때까지 대기
}



//////////////////////////////////////////////////// Problem_1 ////////////////////////////////////////////////////

int LED_State = 0;

void Problem_1(void)
{
    if (TIM1->SR & (1 << 0)) // 인터럽트 플래그 SET
    {
        TIM1->SR &= ~(1 << 0); // 인터럽트 플래그 초기화

        // 현재 LED를 끄기 (이전 상태의 LED를 끔)
        if (LED_State > 0) {
            LED_PORT[LED_State - 1] -> BSRR = LED[LED_State - 1];
        } else {
            // 마지막 LED를 끄는 경우
            LED_PORT[7] -> BSRR = LED[7];
        }

        // 현재 LED를 켜기 (GPIO 핀을 LOW로 만듦)
        LED_PORT[LED_State] -> BSRR = LED[LED_State] << 0x10;

        // 다음 LED 상태로 업데이트
        LED_State = (LED_State + 1) % 8;
    }
}



//////////////////////////////////////////////////// 마이크로초 Delay ////////////////////////////////////////////////////

void usDelay(uint32_t micro_sec)
{
	RCC -> APB2ENR |= (1<<1); // TIM8 활성화

	if(micro_sec < 2) micro_sec = 2; // 최소 지연 시간 : 2마이크로초

	TIM8 -> ARR = micro_sec - 1; // ARR 지연 값
	TIM8 -> EGR = 1; // 업데이트 이벤트 활성화
	TIM8 -> SR &= ~1; // 인터럽트 플래그 초기화
	TIM8 -> CR1 |= 1; // Counter 활성화

	while(!(TIM8 -> SR & (1<<0))); // 인터럽트 플레그가 발생할 때까지 대기
}



//////////////////////////////////////////////////// 초음파 센서 ////////////////////////////////////////////////////

int distance = 0;
uint32_t echo_duration = 0;

void ultrasonic_sensor(void)
{
	GPIOF->BSRR = GPIO_PIN_14 << 0x10; // Trig_Pin Reset - 초음파 측정 준비
	usDelay(3); // 초음파 생성 대기 (3us)


	GPIOF->BSRR = GPIO_PIN_14; // Trig_Pin Set - 초음파 측정 시작
	usDelay(10); // 10us 동안 초음파 발사
	GPIOF->BSRR = GPIO_PIN_14 << 0x10; // Trig_Pin Reset

	while(!(GPIOE->IDR & GPIO_PIN_9)); // Echo_Pin이 Set이 될때까지 대기

	echo_duration = 0; // Echo 시간 측정 시간

	while(GPIOE->IDR & GPIO_PIN_9) // Echo_Pin이 Set 상태인 동안
	{
		echo_duration++; // 시간 측정
		usDelay(2);
	}

	distance = echo_duration / 58; // 거리 계산

}



//////////////////////////////////////////////////// FND ////////////////////////////////////////////////////

void FND_Dispaly(int Num)
{
	int One_Number = Num % 10; // 1의 자리 숫자 계산
	int Ten_Number = Num / 10; // 10의 자리 숫자 계산

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



//////////////////////////////////////////////////// Problem_2 ////////////////////////////////////////////////////

void Problem_2(void)
{
	ultrasonic_sensor(); // 초음파 센서 ON
	FND_Dispaly(distance); // 측정 거리값 FND 출력
}



//////////////////////////////////////////////////// Problem_3 ////////////////////////////////////////////////////

void Problem_3(void)
{
	char Buffer[100];

	ultrasonic_sensor(); // 초음파 센서 ON
	FND_Dispaly(distance); // 측정 거리값 FND 출력

	sprintf(Buffer, "%d cm\r\n", distance); // 측정 거리값 버퍼 저장, 매핑(문자열)
	UART3_Send(Buffer); // UART3을 통해 결과 전송
}



//////////////////////////////////////////////////// Problem_4 ////////////////////////////////////////////////////

void Problem_4(void)
{
	LED_FND_INIT(); // LED, FND 초기화

	// 초음파 센서 OFF
	GPIOF -> BSRR = GPIO_PIN_14 << 0x10; // Trig Pin Reset
	GPIOE -> BSRR = GPIO_PIN_9 << 0x10; // Echo Pin Reset

	distance = 0; // 측정 거리 초기화
}



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ETH_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);

//////////////////////////////////////////////////// main 문 ////////////////////////////////////////////////////

int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ETH_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();


/////////////////// 인터럽트 설정 ///////////////////


  RCC -> AHB1ENR |= (1<<2); // GPIOC 클럭 활성화
  RCC -> AHB1ENR |= (1<<1); // GPIOB 클럭 활성화
  RCC -> APB2ENR |= (1<<14); // SYSCFG 활성화

  SYSCFG->EXTICR[2] |= (2 << 4); // PC9 설정 (EXTI9 -> PC 포트)
  SYSCFG->EXTICR[1] |= (2 << 8); // PC6 설정 (EXTI6 -> PC 포트)
  SYSCFG->EXTICR[3] |= (1 << 0); // PB12 설정 (EXTI12 -> PB 포트)
  SYSCFG->EXTICR[2] |= (1 << 12); // PB11 설정 (EXTI11 -> PB 포트)

  EXTI->IMR = (1<<9)|(1<<6)|(1<<12)|(1<<11); // 인터럽트 마스크 해제, 인터럽트 활성화

  EXTI->RTSR = (1<<9)|(1<<6)|(1<<12)|(1<<11); // EXTI 상승 에지 트리거 활성화


/////////////////// 초기 설정 ///////////////////

  LED_FND_INIT(); // LED, FND 초기화

  UART3_INIT(); // UART 초기화

  TIM1_INIT(); // 타이머 설정

/////////////////////////////////////////////


  while (1)
  {
	  FND_Dispaly(distance);

	  if(Flag_1 == 1)
	  {
		  Problem_1();
	  }
	  else if(Flag_2 == 1)
	  {
		  Problem_2();
	  }
	  else if(Flag_3 == 1)
	  {
		  Problem_3();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 96-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  HAL_GPIO_WritePin(GPIOF, LED_3_Pin|LED_4_Pin|Trig_Pin_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : FND_E_Pin FND_D_Pin FND_G_Pin Trig_Pin_Pin */
  GPIO_InitStruct.Pin = FND_E_Pin|FND_D_Pin|FND_G_Pin|Trig_Pin_Pin;
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

  /*Configure GPIO pin : Echo_Pin_Pin */
  GPIO_InitStruct.Pin = Echo_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_Pin_GPIO_Port, &GPIO_InitStruct);

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
