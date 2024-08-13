/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "led_display.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "SEGGER_SYSVIEW.h"
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
CAN_HandleTypeDef hcan1;
/* USER CODE BEGIN PV */
uint8_t RxData[4];
CAN_RxHeaderTypeDef   RxHeader;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
//void CAN_TransmitData(void);
void can_rx();
/* USER CODE BEGIN PFP */

//Acceptance filter Configuration
static void CAN_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define STACK_SIZE 128

void DeferredTask (void *argument);
void WasteFullLED(void *argument);

//create Queue for UART Data storage
static QueueHandle_t can1_BytesReceived = NULL;
TaskHandle_t DeferTaskHandle = NULL;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  SEGGER_SYSVIEW_Conf();

   /*##-Step1:Filter Configuration ###########################################*/
    CAN_Config();

    /*##-Step2:Start the CAN peripheral ###########################################*/
        if (HAL_CAN_Start(&hcan1) != HAL_OK)
      	  {
      		/* Start Error */
      		Error_Handler();
      	  }
    /*##-Step3:Activate CAN RX notification i.e. INTERRUPTS ########################*/
  	 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	{
  	  // Notification Error
  	  Error_Handler();
  	}
 /*##################################################### RTOS ####################################################################*/
  	 // SET PREEMPTION PRIORITY TO 6
  	  /* Set the UART for interrupt and change the preemption priority of UART to 6 */

  	 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//ensure proper priority grouping for freeRTOS


  	  //setup tasks, making sure they have been properly created before moving on
  	  assert_param(xTaskCreate(DeferredTask, "DeferredTask", STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &DeferTaskHandle) == pdPASS);
  	  assert_param(xTaskCreate(WasteFullLED, "LED_blink_low_priority", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS);

  	 can1_BytesReceived = xQueueCreate(1, sizeof(int));

  	  vTaskStartScheduler();

  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

static void CAN_Config(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void DeferredTask(void *argument)
{

	while(1)
	{
		SEGGER_SYSVIEW_PrintfHost("Waiting for mque receive\n");
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//xQueueReceive(can1_BytesReceived, &QxData, portMAX_DELAY);
		//LED_Display(RxData[0]);
		SEGGER_SYSVIEW_PrintfHost("Data: %d\n",RxData[0]);
		SEGGER_SYSVIEW_PrintfHost("Data: %d\n",RxData[1]);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,1);
		vTaskDelay(100/portTICK_PERIOD_MS);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,0);
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

void WasteFullLED(void *argument)
{
	while(1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,1);
		vTaskDelay(100/portTICK_PERIOD_MS);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,0);
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

void CAN1_RX0_IRQHandler(void)
{
	 portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; //A variable to track if a higher priority task has been woken up by the interrupt. It is initialized to pdFALSE
	SEGGER_SYSVIEW_RecordEnterISR();//This function call logs the entry of the interrupt service routine (ISR) for debugging or profiling purposes using the SEGGER SystemView tool
	uint32_t interrupt = READ_REG(hcan1.Instance->IER);

	if((interrupt & CAN_IT_RX_FIFO0_MSG_PENDING)!=0U)
	{
		if((hcan1.Instance->RF0R & CAN_RF0R_FMP0)!=0U)
		{
			 // Get the Id
			 RxHeader.IDE = (uint8_t)0x04 & hcan1.Instance->sFIFOMailBox[0].RIR;
			  if (RxHeader.IDE == 0x00)
			  {
			   RxHeader.StdId = (uint32_t)0x000007FF & (hcan1.Instance->sFIFOMailBox[0].RIR >> 21);
			  }
			  else
			  {
			   RxHeader.ExtId = (uint32_t)0x1FFFFFFF & (hcan1.Instance->sFIFOMailBox[0].RIR >> 3);
			  }

			 RxHeader.RTR = (uint8_t)0x02 & hcan1.Instance->sFIFOMailBox[0].RIR;
			  // Get the DLC
			 RxHeader.DLC = (uint8_t)0x0F & hcan1.Instance->sFIFOMailBox[0].RDTR;
			  // Get the data field
			 RxData[0] = (uint8_t)0xFF & hcan1.Instance->sFIFOMailBox[0].RDLR;
			 RxData[1] = (uint8_t)((0xFFUL << (8U)) & (hcan1.Instance->sFIFOMailBox[0].RDLR) >> 8U);
			 RxData[2] = (uint8_t)((0xFF << (16UL)) & (hcan1.Instance->sFIFOMailBox[0].RDLR) >> 16U);
			 RxData[3] = (uint8_t)((0xFF << (24UL)) & (hcan1.Instance->sFIFOMailBox[0].RDLR) >> 24U);
			 /*RxData.Data[4] = (uint8_t)0xFF & hcan1.Instance->sFIFOMailBox[0].RDHR;
			 RxData.Data[5] = (uint8_t)0xFF & (hcan1.Instance->sFIFOMailBox[0].RDHR >> 8);
			 RxData.Data[6] = (uint8_t)0xFF & (hcan1.Instance->sFIFOMailBox[0].RDHR >> 16);
			 RxData.Data[7] = (uint8_t)0xFF & (hcan1.Instance->sFIFOMailBox[0].RDHR >> 24);*/

			 // Release FIFO0
				    if (0 == CAN_RX_FIFO0) // Rx element is assigned to Rx FIFO 0
				    {
				      // Release RX FIFO 0
				      hcan1.Instance->RF0R |= (0x1UL << 5U);
				    }

			if ((RxHeader.StdId == 0xB) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
			{
				SEGGER_SYSVIEW_PrintfHost("In Interrupt\n");
				SEGGER_SYSVIEW_PrintfHost("Data: %d\n",RxData[0]);
				vTaskNotifyGiveFromISR(DeferTaskHandle, &xHigherPriorityTaskWoken);
				//xQueueSendFromISR(can1_BytesReceived, &RxData, &xHigherPriorityTaskWoken);
				SEGGER_SYSVIEW_PrintfHost("Data sent\n");
			}
		}
	}

  //Logs the exit of the ISR for debugging or profiling purposes using the SEGGER SystemView tool.
 	  SEGGER_SYSVIEW_RecordExitISR();
 	  //this xHigherPriorityTaskWoken variable will be set to true by xQueueSendFromISR() API
 	  // once data is ready in queue
 	  // portYIELD_FROM_ISR is a macro that requests a context switch from within an ISR.
 	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 	  /*If xHigherPriorityTaskWoken was set to pdTRUE by xQueueSendFromISR, this macro will request a context switch to ensure the higher priority task that was woken up can run immediately. This yields from the ISR to the higher priority task.*/

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s on line %d\r\n", file, line);
		while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
