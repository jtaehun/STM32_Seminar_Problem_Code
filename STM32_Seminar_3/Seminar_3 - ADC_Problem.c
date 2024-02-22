#include "main.h"
#include "string.h"
#include "stdlib.h"


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

ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


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
uint8_t FND_PATTERN_1[10] = {0xBF, 0x86, 0xDB, 0xCF, 0xE6, 0xED, 0xFD, 0x87, 0xFF, 0xEF}; // FND 핀 배열에 따라 계산 Ex) 0 - 1011 1111 - 0xBF

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


void UART3_Recv(char *str) // 문자열 수신
{
    while (1)
    {
        while(!(USART3->ISR & (1<<5))); // 데이터가 읽을 준비가 될 때까지 대기
        *str = USART3->RDR; // 수신된 데이터 읽기

        USART3->TDR = *str; // 입력한 문자 시리얼 창에 출력

        while (!(USART3->ISR & (1<<6))); // 전송 완료(TC) 플래그가 설정될 때까지 대기

        if (*str == '\r' || *str == '\n') // 엔터 시 break;
        {
            break;
        }
        str++; // 포인터를 증가시켜 문자를 다음 위치로 이동
    }
}

//////////////////////////////////////////////////// ADC ////////////////////////////////////////////////////


void ADC_INIT(void) // ADC 초기화
{

	RCC -> APB2ENR |= (1<<8); // ADC1 클럭 활성화
	RCC -> AHB1ENR |= (1<<0); // GPIOA 클럭 활성화

	ADC1 -> CR1 = (1<<8); // SCAN 모드 활성화 (2개의 채널 제어)
	ADC1 -> CR1 &= ~(1<<24); // ADC 분해능 12Bit 설정

	ADC1 -> CR2 = (1<<1); // 연속 ADC 변환 모드 활성화
	ADC1 -> CR2 |= (1<<10); // ADC 변환 완료 신호 활성화 (오버런 감지)

	ADC1 -> SMPR2 &= ~((1<<0) | (1<<9)); // PA0, PA3핀에 15 사이클의 샘플링 시간 설정

	ADC1 -> SQR1 |= (1<<20); // 2개의 채널 변환 수행 (PA0, PA3)

	GPIOA -> MODER |= (3<<0); // PA0 - Analog mode 설정
	GPIOA -> MODER |= (3<<6); // PA3 - Analog mode 설정

}


void ADC_Enable (void) // ADC ON
{
	ADC1 -> CR2 |= (1<<0); // A/D Converter ON / OFF
}


void ADC_Start (int Pin_Num) // ADC 변환
{


	ADC1 -> SQR3 = 0;

	ADC1 -> SQR3 |= (Pin_Num << 0); // ADC 변환 수행할 핀 설정

	ADC1 -> SR = 0; // 상태 레지스터 초기화

	ADC1 -> CR2 |= (1 << 30); // 정규 채널 변환 활성화

}



void ADC_Wait (void) // ADC 변환 대기
{
	while (!(ADC1 -> SR & (1<<1))); // EOC 비트가 1 (Conversion complete) 일 때까지 while
}


uint16_t ADC_GetVal (void) // ADC 변환 결과 반환
{
	return ADC1 -> DR; // 변환된 값을 읽고 반환
}


//////////////////////////////////////////////////// Problem_1 ////////////////////////////////////////////////////


void Problem_1(void)
{

	UART3_Send("-----버튼을 누르세요-----\r\n");
	UART3_Send("\n1. 초기화\r\n");
	UART3_Send("2. 문제2\r\n");
	UART3_Send("3. 문제3\r\n");
	UART3_Send("4. 문제4\r\n");
	UART3_Send("\n--------------------------\r\n");

	LED_FND_INIT(); // LED, FND 초기화

	GPIOA -> BSRR = (1<<0); // PA0 비활성화 (ADC_PIN1)
	GPIOA -> BSRR = (1<<3); // PA3 비활성화 (ADC_PIN2)

}


//////////////////////////////////////////////////// Problem_2 ////////////////////////////////////////////////////


void Binary(int value) // 2진수 변환
{

	for (int i = 0; i < 4; ++i) {

		if (value & (1 << i)) // value 값의 i번째 비트 검사
		{
			LED_PORT[i] -> BSRR = LED[i] << 0x10; // 1이면, 해당 LED ON
		}
		else
		{
			LED_PORT[i] -> BSRR = LED[i]; // 0이면, 해당 LED OFF
		}

	}
}


void Problem_2(void)
{
	  char buffer_2[100];

	  ADC_Start(0); // Pin_Num = 0, PA0 센서 ADC
	  ADC_Wait(); // ADC 변환 대기

	  uint16_t ADC_Value = ADC_GetVal(); // ADC 변환 값 읽고 반환

	  uint16_t ADC_MAP_Value = (ADC_Value * 15) / 4095; // ADC 값을 1 ~ 15 범위로 매핑

	  sprintf(buffer_2, "%d\r\n", ADC_MAP_Value); // 매핑된 값을 문자열 버퍼에 저장

	  UART3_Send(buffer_2); // 문자열 전송

// ------------------------------ LED, FND 제어 ------------------------------

	  Binary(ADC_MAP_Value); // ADC 값 LED 출력 (LED 4개로 2진수 표현)


	  int One_Number = ADC_MAP_Value % 10; // FND 1의 자리 숫자 계산
	  int Ten_Number = ADC_MAP_Value / 10; // FND 10의 자리 숫자 계산

	  for (int i = 0; i < 8; i++)
	  {
		  if ((FND_PATTERN_1[Ten_Number] >> i) & 0x01) // SET_Bit(1) 확인
		  {
			  FND_PORT[i] -> BSRR = FND[i] << 0x10; // 해당 FND 값 ON
		  }
		  else
		  {
			  FND_PORT[i] -> BSRR = FND[i]; // FND OFF
		  }
	  }

	  // TRANSISTOR[1] - 왼쪽 FND 제어, 10의 자리 제어
	  TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[0] << 0x10; // TRANSISTOR ON
	  TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[1]; // TRANSISTOR OFF

	  HAL_Delay(10); // 매우 짧은 시간 동안 십의 자리와 일의 자리 전환 (동적 구동)

	  for (int i = 0; i < 8; i++)
	  {
		  if ((FND_PATTERN_1[One_Number] >> i) & 0x01) // SET_Bit(1) 확인
		  {
			  FND_PORT[i] -> BSRR = FND[i] << 0x10; // 해당 FND 값 ON
		  }
		  else
		  {
			  FND_PORT[i] -> BSRR = FND[i]; // FND OFF
		  }
	  }

	  // TRANSISTOR[0] - 오른쪽 FND 제어, 1의 자리 제어
	  TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[1] << 0x10; // TRANSISTOR ON
	  TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[0]; // TRANSISTOR OFF

	  HAL_Delay(10); // 매우 짧은 시간 동안 십의 자리와 일의 자리 전환 (동적 구동)

}


//////////////////////////////////////////////////// Problem_3 ////////////////////////////////////////////////////

uint16_t ADC_VAL[2] = {0,0}; // 조도센서 2개 사용 (ADC 2개 변환)

void Problem_3(void)
{

	char buffer_3_1[100];
	char buffer_3_2[100];

//----------------------------------- PA0 센서 ADC -----------------------------------


	  ADC_Start(0); // Pin_Num = 0, PA0 센서 ADC
	  ADC_Wait(); // ADC 변환 대기

	  uint16_t ADC_Value_1 = ADC_GetVal(); // PA0의 ADC 변환 값 읽고 반환

	  uint16_t ADC_MAP_Value_1 = (ADC_Value_1 * 15) / 4095; // ADC 값을 1 ~ 15 범위로 매핑

	  sprintf(buffer_3_1, "%d\r\n", ADC_MAP_Value_1); // 매핑된 값을 문자열로 변환

	  UART3_Send("ADC_Value_1 : ");
	  UART3_Send(buffer_3_1);

//----------------------------------- PA3 센서 ADC -----------------------------------

	  ADC_Start(3); // Pin_Num = 3, PA3 센서 ADC
	  ADC_Wait(); // ADC 변환 대기

	  uint16_t ADC_Value_2 = ADC_GetVal(); // PA3의 ADC 변환 값 읽고 반환

	  uint16_t ADC_MAP_Value_2 = (ADC_Value_2 * 15) / 4095; // ADC 값을 1 ~ 15 범위로 매핑

	  sprintf(buffer_3_2, "%d\r\n", ADC_MAP_Value_2); // 매핑된 값을 문자열로 변환

	  UART3_Send("ADC_Value_2 : ");
	  UART3_Send(buffer_3_2);
	  UART3_Send("\r\n");

//----------------------------------- FND 제어 -----------------------------------

	  //---------------------------PA0 ADC---------------------------

	  int FND_Number_1 = ADC_MAP_Value_1 - 1; // 인덱스 0부터 시작

	  for (int i = 0; i < 8; i++)
	  {
		  if ((FND_PATTERN_2[FND_Number_1] >> i) & 0x01)
		  {
			  FND_PORT[i] -> BSRR = FND[i] << 0x10;
		  }
		  else
		  {
			  FND_PORT[i] -> BSRR = FND[i];
		  }
	  }

	  // TRANSISTOR[1] - 왼쪽 FND에 PA0 ADC 값 표현
	  TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[0] << 0x10;
	  TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[1];

	  HAL_Delay(10); // 매우 짧은 시간 동안 전환 (동적 구동)


	  //---------------------------PA3 ADC---------------------------

	  int FND_Number_2 = ADC_MAP_Value_2 - 1; // 인덱스 0부터 시작

	  for (int i = 0; i < 8; i++)
	  {
		  if ((FND_PATTERN_2[FND_Number_2] >> i) & 0x01)
		  {
			  FND_PORT[i] -> BSRR = FND[i] << 0x10;
		  }
		  else
		  {
			  FND_PORT[i] -> BSRR = FND[i];
		  }
	  }

	  // TRANSISTOR[0] - 오른쪽 FND에 PA3 ADC 값 표현
	  TRANSISTOR_PORT[1] -> BSRR = TRANSISTOR[1] << 0x10;
	  TRANSISTOR_PORT[0] -> BSRR = TRANSISTOR[0];

	  HAL_Delay(10); // 매우 짧은 시간 동안 전환 (동적 구동)

}


//////////////////////////////////////////////////// Problem_4 ////////////////////////////////////////////////////


void Problem_4(void)
{
	char buffer_4[100];
	int value;

	UART3_Send("DAC 값을 입력하세요 (0 ~ 4095) : ");

	UART3_Recv(buffer_4); // 입력한 데이터를 버퍼에 저장
	UART3_Send("\r\n");

	value = atoi(buffer_4); // 버퍼에 저장된 문자열을 정수로 변환

	if(value >= 0 && value <= 4095) // 정수 값 0 ~ 4095 (12bit)
	{

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // 설정한 DAC 채널 1을시작

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value); // DAC 채널 1 값을 12비트 오른쪽 정렬, 입력 받은 value 값에 따라 LED 밝기 제어
	}
	else
	{
		UART3_Send("DAC 범위 내의 숫자가 아닙니다");
		UART3_Send("\r\n");
	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ETH_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);


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
  LED_FND_INIT();

  EXTI->RTSR= (1<<9)|(1<<6)|(1<<12)|(1<<11); // EXTI 상승 에지 트리거 활성화

  UART3_INIT(); // UART

  ADC_INIT(); // ADC
  ADC_Enable();

  while (1)
  {

	  if(Flag_1 == 1)
	  {
		  Problem_1();

		  Flag_1 = 0;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
