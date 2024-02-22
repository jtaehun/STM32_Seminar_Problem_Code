#include "main.h"
#include <stdlib.h>
#include <time.h>

// LED1
#define LED1 GPIO_PIN_0
#define LED_PORT1 GPIOC

// LED2
#define LED2 GPIO_PIN_3
#define LED_PORT2 GPIOD

// LED3
#define LED3 GPIO_PIN_2
#define LED_PORT3 GPIOG

// LED4
#define LED4 GPIO_PIN_3
#define LED_PORT4 GPIOG

// LED5
#define LED5 GPIO_PIN_2
#define LED_PORT5 GPIOE

// LED6
#define LED6 GPIO_PIN_4
#define LED_PORT6 GPIOE

// LED7
#define LED7 GPIO_PIN_5
#define LED_PORT7 GPIOE

// LED8
#define LED8 GPIO_PIN_2
#define LED_PORT8 GPIOF


// LED 배열
const int LED[8] =  {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8};
// LED_PORT 배열
const int LED_PORT[8] = {LED_PORT1 ,LED_PORT2 ,LED_PORT3 ,LED_PORT4 ,LED_PORT5 ,LED_PORT6 ,LED_PORT7 ,LED_PORT8};


// BUTTON1
#define BUTTON1 GPIO_PIN_0
#define BUTTON_PORT1 GPIOA

// BUTTON2
#define BUTTON2 GPIO_PIN_8
#define BUTTON_PORT2 GPIOF

// BUTTON3
#define BUTTON3 GPIO_PIN_4
#define BUTTON_PORT3 GPIOA

// BUTTON4
#define BUTTON4 GPIO_PIN_0
#define BUTTON_PORT4 GPIOB


// uint8_t : 8bit (0 ~ 255)
static uint8_t LED_State = 0x01;

// 문제1 : 오른쪽으로 LED SHIFT
void Problem1(void) {

	for (int i = 0; i < 8; ++i) {
		HAL_GPIO_WritePin(LED_PORT[i], LED[i], (LED_State & (1 << i)) ? 1 : 0); // LED_State 비트 왼쪽 쉬프트
	}

	LED_State = (LED_State << 1 | (LED_State >> 7)); // LED 1개 제외 모두 비트 0
}



// 문제2 : LED 중첩 증가
void Problem2(void) {

	if (LED_State == 0xFF) {

		LED_State = 0x01;  // 초기화
		}

	else {

		for (int i = 0; i < 8; ++i) {
			HAL_GPIO_WritePin(LED_PORT[i], LED[i], (LED_State & (1 << i)) ? 1 : 0); // LED_State 비트 왼쪽 쉬프트
		}
		LED_State = (LED_State << 1) | LED_State; // LED 중첩하면서 왼쪽 쉬프트
	}
}



// 문제3 : 버튼으로 반복 이동
int Button3_Pressed = 0; // 버튼 상태
int Button3_Pre_State = 0; // 이전 버튼 상태

void BUTTON3_Toggle(void) { // BUTTON3 - Toggle

    int Button3_State = HAL_GPIO_ReadPin(BUTTON_PORT3, BUTTON3); // BUTTON3 상태

    if (Button3_State == 1 && Button3_Pre_State == 0) { // 현재 버튼 상태가 1이고 이전 버튼 상태가 0이면

        Button3_Pressed = !Button3_Pressed; // 버튼 상태 값 토글
    }

    Button3_Pre_State = Button3_State; // 현재 버튼 상태를 이전 버튼 상태로 저장
}

void Problem3(void) {

	BUTTON3_Toggle(); // Toggle 함수 호출

	if (Button3_Pressed){

		for (int i = 0; i < 8; ++i) {
			HAL_GPIO_WritePin(LED_PORT[i], LED[i], (LED_State & (1 << i)) ? 1 : 0); // LED_State 비트 왼쪽 쉬프트
		}

		LED_State = (LED_State << 1) | (LED_State >> 7); // LED 중첩하면서 왼쪽 쉬프트
	}
	else {
		// 정지
	}
}


// 문제4 : 반복 패턴 구현
uint8_t LED_BIT_PATTERN[] = {0x81, 0x42, 0x24, 0x18, 0x24, 0x42, 0x81}; // 8비트의 LED 패턴 배열

int j = 0;

void Problem4(void) {
    // 패턴을 표시
    for (int i = 0; i < 8; i++) {

        int LED_BIT = (LED_BIT_PATTERN[j] >> i) & 1; // 현재 패턴을 8개의 LED에 순차적으로 표시
        HAL_GPIO_WritePin(LED_PORT[i], LED[i], LED_BIT ? 1 : 0); // 각 비트 중 해당 비트가 1이면 LED ON, 아니면 LED OFF
    }

    // 패턴 이동
    j = (j + 1) % 7;  // j의 값 : 0 ~ 6
    HAL_Delay(100);
}


/////////////////////////////////////////////////////////


void SystemClock_Config(void); // 시스템 클럭 설정
static void MX_GPIO_Init(void); // GPIO 포트 초기화


/////////////////////////////////////////////////////////

int main(void) {

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  while (1) {

	  // BUTTON1
	  if(HAL_GPIO_ReadPin(BUTTON_PORT1, BUTTON1) == 1) {
		  Problem1();
		  HAL_Delay(200);
	  }

	  // BUTTON2
	  if(HAL_GPIO_ReadPin(BUTTON_PORT2, BUTTON2) == 1) {
		  Problem2();
		  HAL_Delay(500);
	  }

	  //BUTTON3
	  BUTTON3_Toggle();

	  if (Button3_Pressed) {
		  Problem3();
		  HAL_Delay(200);
	  }

	  // BUTTON4
	  if(HAL_GPIO_ReadPin(BUTTON_PORT4, BUTTON4) == 1) {
	 	  Problem4();
	 	  HAL_Delay(200);
	  }

  }
}






// 시스템 클럭 설정
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}



// GPIO 포트 초기화
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);



///////////////////////// BUTTON /////////////////////////

  /*Configure GPIO pins : PA0 - button1 /  PA4 - button3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF8 - button2 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 - button4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

///////////////////////////////////////////////////////////


////////////////////////// LED ///////////////////////////

  /*Configure GPIO pin : PC0 - led1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 - led2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 - led3 / PG3 - led4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE2 - led5 / PE4 - led6 / PE5 - led7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 - led8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

///////////////////////////////////////////////////////////







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
