/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>
#include "stdio.h"
#include "message_buffer.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
/* DWT (Data Watchpoint and Trace) registers, only exists on ARM Cortex with a DWT unit */
#define KIN1_DWT_CONTROL             (*((volatile uint32_t*)0xE0001000))
/*!< DWT Control register */
#define KIN1_DWT_CYCCNTENA_BIT       (1UL<<0)
/*!< CYCCNTENA bit in DWT_CONTROL register */
#define KIN1_DWT_CYCCNT              (*((volatile uint32_t*)0xE0001004))
/*!< DWT Cycle Counter register */
#define KIN1_DEMCR                   (*((volatile uint32_t*)0xE000EDFC))
/*!< DEMCR: Debug Exception and Monitor Control Register */
#define KIN1_TRCENA_BIT              (1UL<<24)
/*!< Trace enable bit in DEMCR register */

#define KIN1_InitCycleCounter() \
  KIN1_DEMCR |= KIN1_TRCENA_BIT
  /*!< TRCENA: Enable trace and debug block DEMCR (Debug Exception and Monitor Control Register */

#define KIN1_ResetCycleCounter() \
  KIN1_DWT_CYCCNT = 0
  /*!< Reset cycle counter */

#define KIN1_EnableCycleCounter() \
  KIN1_DWT_CONTROL |= KIN1_DWT_CYCCNTENA_BIT
  /*!< Enable cycle counter */

#define KIN1_DisableCycleCounter() \
  KIN1_DWT_CONTROL &= ~KIN1_DWT_CYCCNTENA_BIT
  /*!< Disable cycle counter */

#define KIN1_GetCycleCounter() \
  KIN1_DWT_CYCCNT
  /*!< Read cycle counter register */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart2;

/* Definitions for blink01 */
osThreadId_t blink01Handle;
const osThreadAttr_t blink01_attributes = {
  .name = "blink01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask02 */
// osPriorityAboveNormal
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t xSemaphore = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartBlink01(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t cycles[100]; /* number of cycles */
MessageBufferHandle_t xStreamBuffer;
const int bytesToSend = 16;
uint8_t ucArrayToSend[16];

EventGroupHandle_t eventGroup; // used for testing event notification
QueueHandle_t xQueue1; // used for testing notification implemented by queue

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
  eventGroup = xEventGroupCreate(); // create an event group
  xQueue1 = xQueueCreate(1, sizeof(uint32_t)); // create a queue
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xSemaphore = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xStreamBuffer = xMessageBufferCreate( bytesToSend+16 );
  if( xStreamBuffer == NULL )
  {
      printf("Not enough memory!");
  }

  for (int i=0;i<bytesToSend;++i) {
  	ucArrayToSend[i] = 0;
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink01 */
  blink01Handle = osThreadNew(StartBlink01, NULL, &blink01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

//void sendCompleteCallback(void * xUpdatedMessageBuffer) {
//	printf("send complete\r\n");
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlink01 */
/**
  * @brief  Function implementing the blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void *argument)
{
  /* USER CODE BEGIN 5 */
	/*
	 // example for counting cycles and print out to console
	  KIN1_InitCycleCounter(); // enable DWT hardware

  // Infinite loop
	  unsigned times[100];
	  char buffer[40];
  for(int i=0;i<100;++i)
  {
		KIN1_EnableCycleCounter(); // start counting
		KIN1_ResetCycleCounter();
		size_t xBytesSent = xMessageBufferSend( xStreamBuffer,
										( void * ) ucArrayToSend,
										sizeof( ucArrayToSend ), 0);
		cycles = KIN1_GetCycleCounter(); // get cycle counter
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "%u\r\n", cycles), 500);
		times[i] = cycles;
		KIN1_DisableCycleCounter(); // disable counting if not used any more

		size_t xReceivedBytes = xMessageBufferReceive( xStreamBuffer,
											   ( void * ) ucArrayToSend,
											   sizeof( ucArrayToSend ),
											   0 );


  }
  */


	/*
	 // example for testing acquire and release a semaphore
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	for (int i = 0; i < 50; ++i) {
		vTaskDelay(1);
		KIN1_EnableCycleCounter();
		KIN1_ResetCycleCounter();
		xSemaphoreGive(xSemaphore);
		vTaskDelay(1);
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cycles[i*2 + 1] = KIN1_GetCycleCounter();
	}
	*/
	uint32_t valueToSend1 = 20;


	/*
	 // example for testing normal direct to task notification
	// deprecated
	uint32_t receivedValue = 0;
	for (int i = 0; i < 50; ++i) {
		vTaskDelay(1);
		KIN1_EnableCycleCounter();

		if (xTaskNotify(myTask02Handle, valueToSend, eSetValueWithoutOverwrite) == pdTRUE) {
			++valueToSend;
		}


		KIN1_ResetCycleCounter();
		vTaskDelay(1);

//		if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1000) == pdPASS) {
//			cycles[i * 2 + 1] = KIN1_GetCycleCounter();
//		}
		while (1) {
			if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1) == pdPASS) {
				cycles[i * 2 + 1] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}
	}
	*/



	 // example for testing normal direct to task notification
	for (int i = 0; i < 100; ++i) {
		KIN1_EnableCycleCounter();
		while (1) {
			KIN1_ResetCycleCounter();
			if (xTaskNotify(myTask02Handle, valueToSend1, eSetValueWithoutOverwrite) == pdTRUE) {
				++valueToSend1;
				KIN1_ResetCycleCounter();
				vTaskDelay(1);
				break;
			}
			KIN1_ResetCycleCounter();
			vTaskDelay(1);
		}
//		while (xTaskNotify(myTask02Handle, valueToSend1, eSetValueWithoutOverwrite) == pdTRUE) {
//			KIN1_ResetCycleCounter();
//			vTaskDelay(1);
//			valueToSend1++;
//		}
	}




/*
	// example for testing semaphore-style direct to task notification
	for (int i = 0; i < 100; ++i) {
		//vTaskDelay(1);
		KIN1_EnableCycleCounter();
		KIN1_ResetCycleCounter();
		xTaskNotifyGive(myTask02Handle);
		vTaskDelay(1);
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//cycles[i * 2 + 1] = KIN1_GetCycleCounter();
	}
*/

/*
	 // example for testing event group notification

	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	EventBits_t uxBits;
	for (int i = 0; i < 100; ++i) {
		//vTaskDelay(1);
		KIN1_EnableCycleCounter();
		while (1) {
			uxBits = xEventGroupSetBits(eventGroup, BIT_0);
			if ((uxBits & BIT_0) == 0) {
				KIN1_ResetCycleCounter();
				vTaskDelay(1);
				break;
			}
			KIN1_ResetCycleCounter();
			vTaskDelay(1);
		}

		KIN1_ResetCycleCounter();
		vTaskDelay(1);
		//xEventGroupWaitBits(eventGroup, BIT_1, pdTRUE, pdTRUE, xTicksToWait);
		//cycles[i * 2 + 1] = KIN1_GetCycleCounter();
	}
*/



	/*
	// example for testing queue notification
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	uint32_t receivedValue = 0;
	for (int i = 0; i < 100; ++i) {
		//vTaskDelay(1);
		KIN1_EnableCycleCounter();
		while (1) {
			KIN1_ResetCycleCounter();
			if (xQueueSend(xQueue1, &valueToSend1, 1) == pdTRUE) {
				++valueToSend1;
				KIN1_ResetCycleCounter();
				vTaskDelay(1);
				break;
			}
			KIN1_ResetCycleCounter();
			vTaskDelay(1);
		}
//		xQueueSend(xQueue1, &valueToSend, xTicksToWait);
//		valueToSend++;
//		KIN1_ResetCycleCounter();
//		vTaskDelay(1);
		//xQueueReceive(xQueue1, &receivedValue, xTicksToWait);
		//cycles[i * 2 + 1] = KIN1_GetCycleCounter();
	}
	*/


  vTaskDelete( NULL );
  /*
  char buffer[40];
  	for (int i = 0; i < 50; ++i) {
  		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "task1: %u\r\n", cycles[i*2 + 1]), 500);
  	}
  	*/
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

	/*
	 // example for testing semaphore
		for (int i = 0; i < 50; ++i) {
			xSemaphoreTake(xSemaphore, portMAX_DELAY);
			cycles[i*2] = KIN1_GetCycleCounter();
			KIN1_DisableCycleCounter();
			vTaskDelay(1);
			KIN1_EnableCycleCounter();
			KIN1_ResetCycleCounter();
			xSemaphoreGive(xSemaphore);
			vTaskDelay(1);

		}
		*/


	//uint32_t valueToSend = 120;
	uint32_t receivedValue;

	// deprecated
	/*
	// uint32_t valueToSend = 100;
	for (int i = 0; i < 50; ++i) {
//		if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1000) == pdPASS) {
//			cycles[i * 2] = KIN1_GetCycleCounter();
//		}

		while (1) {
			if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1) == pdPASS) {
				cycles[i * 2] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}
		vTaskDelay(1);
		KIN1_EnableCycleCounter();
		if (xTaskNotify(blink01Handle, valueToSend, eSetValueWithoutOverwrite) == pdTRUE) {
			++valueToSend;
		}

		KIN1_ResetCycleCounter();
		vTaskDelay(1);
	}
	*/


	// example for testing normal direct to task notification
	int i = 0;
	while (1) {
		while (1) {
			KIN1_ResetCycleCounter();
			if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1) == pdPASS) {
				cycles[i++] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}

		if (receivedValue == 120) {
			break;
		}
		vTaskDelay(1);
	}



	/*
	// example for testing semaphore-style direct to task notification
	for (int i = 0; i < 100; ++i) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		cycles[i] = KIN1_GetCycleCounter();
		vTaskDelay(1);
		//KIN1_EnableCycleCounter();
		//xTaskNotifyGive(blink01Handle);
		//KIN1_ResetCycleCounter();
		//vTaskDelay(1);
	}
	*/

	/*
	int i = 0;
	while (1) {
		while (1) {
			//KIN1_ResetCycleCounter();
			uint32_t res = ulTaskNotifyTake(pdTRUE, 1);
			if (res == 1) {
				cycles[i++] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}
		if (i == 100) {
			break;
		}
		vTaskDelay(1);
	}
	*/

	/*
	// example for testing event group notification
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	for (int i = 0; i < 100; ++i) {
		xEventGroupWaitBits(eventGroup, BIT_0, pdTRUE, pdTRUE, xTicksToWait);
		cycles[i] = KIN1_GetCycleCounter();
		vTaskDelay(1);
		//KIN1_EnableCycleCounter();
		//xEventGroupSetBits(eventGroup, BIT_1);
		//KIN1_ResetCycleCounter();
		//vTaskDelay(1);
	}
	*/

	/*
	int i = 0;
	EventBits_t uxBits;
	while (1) {
		while (1) {
			uxBits = xEventGroupWaitBits(eventGroup, BIT_0, pdTRUE, pdTRUE, 1);
			if ((uxBits & BIT_0) != 0) {
				cycles[i++] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}
		if (i == 100) {
			break;
		}
		vTaskDelay(1);
	}
	*/


	/*
	// example for testing queue notification
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	for (int i = 0; i < 100; ++i) {
		xQueueReceive(xQueue1, &receivedValue, xTicksToWait);
		cycles[i] = KIN1_GetCycleCounter();
		vTaskDelay(1);
		//KIN1_EnableCycleCounter();
		//xQueueSend(xQueue1, &valueToSend, xTicksToWait);
		//valueToSend++;
		//KIN1_ResetCycleCounter();
		//vTaskDelay(1);
	}
	*/
/*
	int i = 0;
	while (1) {
		while (1) {
			if (xQueueReceive(xQueue1, &receivedValue, 1) == pdTRUE) {
				cycles[i++] = KIN1_GetCycleCounter();
				break;
			} else {
				KIN1_ResetCycleCounter();
			}
		}

		if (receivedValue == 120) {
			break;
		}
		vTaskDelay(1);
	}
	*/
	vTaskDelete( NULL );
	  /*
	  char buffer[40];
	    	for (int i = 0; i < 50; ++i) {
	    		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "task2: %u\r\n", cycles[i*2]), 500);
	    	}
	    	*/
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
