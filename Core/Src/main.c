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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DAQ.h"
#include "Position.h"
#include "Proximity.h"
#include "IMU.h"
#include "GPS.h"
#include "Temperature.h"
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

/* USER CODE BEGIN PV */
daq_timestamp_t g_timestamp;
daq_fault_log_snapshot_t g_fault_log_snapshot;

TaskHandle_t task_handles[DAQ_NO_OF_TASKS];
SemaphoreHandle_t g_i2c_mutex;

CAN_TxHeaderTypeDef can_tx_header;

bool g_i2c_dma_flags[DAQ_NO_OF_I2C_DMA_DEVICES];
daq_i2c_dma_device_t g_i2c_dma_device = I2C_DMA_NO_DEVICE;
extern fault_record_t g_daq_fault_record;
extern DMA_HandleTypeDef hdma_tim1_ch1, hdma_tim1_ch2, hdma_tim1_ch3, hdma_tim1_ch4_trig_com;
DMA_HandleTypeDef *proximity_dma_handlers[4] = {&hdma_tim1_ch1, &hdma_tim1_ch2, &hdma_tim1_ch3, &hdma_tim1_ch4_trig_com};

imu_axis_map_t imu_axis_map = {
  .x = BNO055_AXIS_X,
  .x_sign = BNO055_AXIS_SIGN_POSITIVE,
  .y = BNO055_AXIS_Y,
  .y_sign = BNO055_AXIS_SIGN_POSITIVE,
  .z = BNO055_AXIS_Z,
  .z_sign = BNO055_AXIS_SIGN_POSITIVE
};

//size_t x,y,z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Normal_Blink(void *pvParameters)
{
    for(;;)
    {
    	vTaskDelay(75);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
/**
 * @addtogroup Trace_Module
 * @{
 */

/**
 * @brief Initializes the runtime cycle counter for profiling.
 *
 * Enables the DWT cycle counter if not already enabled. This counter
 * is used to measure CPU cycle counts for runtime statistics.
 */
#ifdef DAQ_TRACE_RECORDER
void vInitRunTimeStats(void)
{
	/* Check if Trace Enable bit is not set in the Debug Exception and Monitor Control Register (DEMCR) */
    if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //Enable trace and debug blocks by setting the TRCENA bit.
        DWT->CYCCNT = 0; // Clear the cycle count.
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //Enable the cycle counter by setting the CYCCNTENA bit in the DWT Control Register.
    }
}
/**
 * @brief Returns the current value of the runtime cycle counter.
 *
 * @return Current DWT cycle count value.
 */
uint32_t ulGetRunTimeCounterValue(void)
{
    return DWT->CYCCNT; // Return the cycle count.
}
/**
 * @brief Task that periodically stops trace recording and triggers an error handler.
 *
 * Runs indefinitely, delaying 1000 ms between iterations. Stops tracing and calls
 * `Error_Handler()`. WWDG should be disabled when using this task to prevent resets.
 * A breakpoint could also be used instead of calling the `Error_Handler()`
 *
 * @param pv Unused parameter.
 */
void vSnapshotTask(void *pv)
{
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        vTraceStop(); // The snapshot is taken.
        Error_Handler(); // Note: Disable WWDG by commenting the MX_WWDG_Init function.
    }
}
/**
 * @brief Initializes the trace recorder by setting up runtime stats and creating snapshot task.
 *
 * it initializes the runtime stats, enables tracing, and creates the snapshot task.
 */
void TraceRecorder_Init(void)
{
    vInitRunTimeStats();
    vTraceEnable(TRC_START);
    xTaskCreate(vSnapshotTask, "Trace_Task", 128, NULL, 1, NULL);
}
#endif
/** @}*/
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
#ifdef DAQ_TRACE_RECORDER
  TraceRecorder_Init();
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_WWDG_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  g_i2c_mutex  = xSemaphoreCreateMutex();
  if (g_i2c_mutex == NULL)
	  Error_Handler();
  ADC_Sensors_Init(&hadc1);
  Prox_Init(&htim1, proximity_dma_handlers);
  IMU_Init(&hi2c1, OPERATION_MODE_NDOF, imu_axis_map);
  GPS_Init(&hi2c1);
  Temp_Init(&hi2c1);

  HAL_CAN_Start(&hcan1);
  DAQ_CAN_Init(&hcan1, &can_tx_header);

  DAQ_FaultLog_Init();
  DAQ_FaultLog_Read(&g_fault_log_snapshot);

  if(g_fault_log_snapshot.current.reset_reason > DAQ_RESET_REASON_MIN &&
	 g_fault_log_snapshot.current.reset_reason < DAQ_RESET_REASON_MAX)
	  xTaskCreate(DAQ_Fault_Blink, "Fault_Blink"	, 64, NULL, 1, NULL);
  else
	  xTaskCreate(Normal_Blink	, "Blink1"		, 64, NULL, 1, NULL);
  for(uint8_t i = 0; i < DAQ_NO_OF_READ_TASKS; i++)
	  g_daq_fault_record.tasks[i].error_count = g_fault_log_snapshot.current.task_records.tasks[i].error_count;

  if(g_daq_fault_record.tasks[PROX_TASK].error_count < DAQ_MAX_ERROR_COUNT)
  	  xTaskCreate(Prox_Task	, "Prox_Task", 256	, NULL, 8, &task_handles[PROX_TASK]);
  else
  {
	  xTaskCreate(Prox_Task	, "Prox_Task", 256	, NULL, 0, &task_handles[PROX_TASK]);
	  g_daq_fault_record.tasks[PROX_TASK].task_demoted = 1;
  }
  if(g_daq_fault_record.tasks[IMU_TASK].error_count < DAQ_MAX_ERROR_COUNT)
	  xTaskCreate(IMU_Task	, "IMU_Task" , 256	, NULL, 7, &task_handles[IMU_TASK]);
  else
  {
	 xTaskCreate(IMU_Task	, "IMU_Task" , 256	, NULL, 0, &task_handles[IMU_TASK]);
	 g_daq_fault_record.tasks[IMU_TASK].task_demoted = 1;
  }
  if(g_daq_fault_record.tasks[GPS_TASK].error_count < DAQ_MAX_ERROR_COUNT)
  	  xTaskCreate(GPS_Task	, "GPS_task" , 256	, NULL, 6, &task_handles[GPS_TASK]);
  else
  {
	  xTaskCreate(GPS_Task	, "GPS_task" , 256	, NULL, 0, &task_handles[GPS_TASK]);
	  g_daq_fault_record.tasks[GPS_TASK].task_demoted = 1;
  }
  if(g_daq_fault_record.tasks[ADC_TASK].error_count < DAQ_MAX_ERROR_COUNT)
	  xTaskCreate(ADC_Task	, "ADC_Task" , 256	, NULL, 5, &task_handles[ADC_TASK]);
  else
  {
	  xTaskCreate(ADC_Task	, "ADC_Task" , 256	, NULL, 0, &task_handles[ADC_TASK]);
	  g_daq_fault_record.tasks[ADC_TASK].task_demoted = 1;
  }
  if(g_daq_fault_record.tasks[TEMP_TASK].error_count < DAQ_MAX_ERROR_COUNT)
	  xTaskCreate(Temp_Task	, "Temp_task", 256 	, NULL, 4, &task_handles[TEMP_TASK]);
  else
  {
	  xTaskCreate(Temp_Task	, "Temp_task", 256 	, NULL, 0, &task_handles[TEMP_TASK]);
	  g_daq_fault_record.tasks[TEMP_TASK].task_demoted = 1;
  }
  xTaskCreate(DAQ_CAN_Tx_Task	, "CAN_TX_TASK" , 128	, NULL, 3, &task_handles[CAN_TX_TASK]);
  xTaskCreate(DAQ_CAN_Rx_Task   , "CAN_RX_TASK", 128   , NULL, 3, &task_handles[CAN_RX_TASK]);
  xTaskCreate(DAQ_WWDG_Task	, "WWDG_task", 128	, NULL, 9, &task_handles[WWDG_TASK]);

  HAL_WWDG_Refresh(&hwwdg);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(!(g_i2c_dma_flags[g_i2c_dma_device]) && g_i2c_dma_device != I2C_DMA_NO_DEVICE)
	{
		g_i2c_dma_flags[g_i2c_dma_device] = true;
		g_i2c_dma_device = I2C_DMA_NO_DEVICE;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7)
  {
	  g_timestamp.counter++;
	  if(g_timestamp.counter==1000)
	  {
		  g_timestamp.counter=0;
	      g_timestamp.seconds++;
	      if(g_timestamp.seconds==60)
	      {
	    	  g_timestamp.seconds = 0;
	    	  g_timestamp.minutes++;
	    	  if(g_timestamp.minutes==60)
	    	  {
	    		  g_timestamp.minutes=0;
	    		  g_timestamp.hours++;
	    		  if(g_timestamp.hours==24)
	    			  g_timestamp.hours=0;
	    	  }
	      }
	  }
  }
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
  HAL_WWDG_Refresh(&hwwdg);
  fault_log_t log = {0};
  log.reset_reason = DAQ_RESET_REASON_ERRORHANDLER;
  log.task_records = g_daq_fault_record;
  log.timestamp = g_timestamp;
  DAQ_FaultLog_Write(&log);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
