/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DAQ.h"
#include "core_cm4.h"
#include "wwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
extern daq_timestamp_t g_timestamp;
extern fault_record_t g_daq_fault_record;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//void xPortSysTickHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch2;
extern DMA_HandleTypeDef hdma_tim1_ch3;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST   LR, #4          \n"
        "ITE   EQ              \n"
        "MRSEQ R0, MSP         \n"
        "MRSNE R0, PSP         \n"
        "B     DAQ_HardFault_Handler \n"
    );
}
void DAQ_HardFault_Handler(stack_registers_t *frame)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	HAL_WWDG_Refresh(&hwwdg);
	__disable_irq();
	fault_log_t log = {0};
	log.reset_reason = DAQ_RESET_REASON_HARDFAULT;
	log.fault_status = SCB->HFSR;
	log.task_records = g_daq_fault_record;
	log.timestamp = g_timestamp;
	log.stack_frame[0] = frame->r0;
	log.stack_frame[1] = frame->r1;
	log.stack_frame[2] = frame->r2;
	log.stack_frame[3] = frame->r3;
	log.stack_frame[4] = frame->r12;
	log.stack_frame[5] = frame->lr;
	log.stack_frame[6] = frame->pc;
	log.stack_frame[7] = frame->xpsr;
	DAQ_FaultLog_Write(&log);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
	/* USER CODE BEGIN W1_HardFault_IRQn 0 */
	/* USER CODE END W1_HardFault_IRQn 0 */
  }
}



/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile(
        "TST   LR, #4          \n"
        "ITE   EQ              \n"
        "MRSEQ R0, MSP         \n"
        "MRSNE R0, PSP         \n"
        "B     DAQ_MemManage_Handler \n"
    );
}
void DAQ_MemManage_Handler(stack_registers_t *frame)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	HAL_WWDG_Refresh(&hwwdg);
	__disable_irq();
	fault_log_t log = {0};
	log.reset_reason = DAQ_RESET_REASON_MEMMANAGE;
	log.fault_status = SCB->CFSR & 0x000000FF;
	log.fault_address = SCB->MMFAR;
	log.task_records = g_daq_fault_record;
	log.timestamp = g_timestamp;
	log.stack_frame[0] = frame->r0;
	log.stack_frame[1] = frame->r1;
	log.stack_frame[2] = frame->r2;
	log.stack_frame[3] = frame->r3;
	log.stack_frame[4] = frame->r12;
	log.stack_frame[5] = frame->lr;
	log.stack_frame[6] = frame->pc;
	log.stack_frame[7] = frame->xpsr;
	DAQ_FaultLog_Write(&log);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
    __asm volatile(
        "TST   LR, #4          \n"
        "ITE   EQ              \n"
        "MRSEQ R0, MSP         \n"
        "MRSNE R0, PSP         \n"
        "B     DAQ_BusFault_Handler \n"
    );
}
void DAQ_BusFault_Handler(stack_registers_t *frame)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	HAL_WWDG_Refresh(&hwwdg);
	__disable_irq();
	fault_log_t log = {0};
	log.reset_reason = DAQ_RESET_REASON_BUSFAULT;
	log.fault_status = SCB->CFSR & 0x0000FF00;
	log.fault_address = SCB->BFAR;
	log.task_records = g_daq_fault_record;
	log.timestamp = g_timestamp;
	log.stack_frame[0] = frame->r0;
	log.stack_frame[1] = frame->r1;
	log.stack_frame[2] = frame->r2;
	log.stack_frame[3] = frame->r3;
	log.stack_frame[4] = frame->r12;
	log.stack_frame[5] = frame->lr;
	log.stack_frame[6] = frame->pc;
	log.stack_frame[7] = frame->xpsr;
	DAQ_FaultLog_Write(&log);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
    __asm volatile(
        "TST   LR, #4          \n"
        "ITE   EQ              \n"
        "MRSEQ R0, MSP         \n"
        "MRSNE R0, PSP         \n"
        "B     DAQ_UsageFault_Handler \n"
    );
}
void DAQ_UsageFault_Handler(stack_registers_t *frame)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	HAL_WWDG_Refresh(&hwwdg);
	__disable_irq();
	fault_log_t log = {0};
	log.reset_reason = DAQ_RESET_REASON_USAGEFAULT;
	log.fault_status = SCB->CFSR & 0xFFFF0000;
	log.task_records = g_daq_fault_record;
	log.timestamp = g_timestamp;
	log.stack_frame[0] = frame->r0;
	log.stack_frame[1] = frame->r1;
	log.stack_frame[2] = frame->r2;
	log.stack_frame[3] = frame->r3;
	log.stack_frame[4] = frame->r12;
	log.stack_frame[5] = frame->lr;
	log.stack_frame[6] = frame->pc;
	log.stack_frame[7] = frame->xpsr;
	DAQ_FaultLog_Write(&log);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
//void SVC_Handler(void)
//{
//  /* USER CODE BEGIN SVCall_IRQn 0 */
//////////////////////////////////////////////////////
//  /* USER CODE END SVCall_IRQn 0 */
//  /* USER CODE BEGIN SVCall_IRQn 1 */
//////////////////////////////////////////////////////
//  /* USER CODE END SVCall_IRQn 1 */
//}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */
//////////////////////////////////////////////////////
//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */
//////////////////////////////////////////////////////
//  /* USER CODE END PendSV_IRQn 1 */
//}
//
///**
//  * @brief This function handles System tick timer.
//  */
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */
////////////////////////////////////////////////////
//  /* USER CODE END SysTick_IRQn 0 */
//
//  /* USER CODE BEGIN SysTick_IRQn 1 */
////////////////////////////////////////////////////
//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch1);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch2);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch4_trig_com);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch3);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
