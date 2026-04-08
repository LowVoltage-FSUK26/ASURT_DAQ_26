/*
 * DAQ.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */

#ifndef DAQ_DAQ_H_
#define DAQ_DAQ_H_

/*============================= STANDARD INCLUDES =============================*/
#include <stdint.h>
#include <assert.h>
/*=============================================================================*/

/*============================= FREERTOS INCLUDES =============================*/
#include "FreeRTOS.h"
#include "queue.h"
/*=============================================================================*/

/*============================= STM32HAL INCLUDES =============================*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
/*=============================================================================*/

#include "main.h"
#include "DAQ_Config.h"

#define PACKED __attribute__((packed))
/* CAN */

/**
 * @brief used in handling I2C DMA interrupts.
 *
 * The handling principle is similar to that of a semaphore, and uses defered processing.
 *
 */
typedef enum{
	I2C_DMA_NO_DEVICE = 0,
	I2C_DMA_GPS
}daq_i2c_dma_device_t;
/**
 * @brief Contains all main tasks
 *
 * Tasks are arranged according to priority, except for the `WWDG_Task`.
 * It's used in task handlers, fault records, and other task-related arrays.
 *
 */
typedef enum {
	PROX_TASK = 0,
	IMU_TASK,
	GPS_TASK,
	ADC_TASK,
	TEMP_TASK,
	CAN_TASK,
	WWDG_TASK
}daq_task_handle_t;

/** @addtogroup Fault_Module
 *  @{
 */

//see the group definition and details at ./docs/html/adoxes/fault.dox

/**
 * @brief Backup SRAM states.
 *
 * This state is saved as the 1st word of the backup SRAM.
 * @note Choosing 6 to be the first value of the enum is arbitrary.
 * Anything other than 0 (as it could be 0 without setting it)
 * would work fine.
 *
 */
typedef enum{
	DAQ_BKPSRAM_UNINITIALIZED = 6,
	DAQ_BKPSRAM_INITIALIZED,
}daq_bkpsram_state_t;
/**
 * @brief Fault Logging statuses.
 *
 * This status is saved as the 2nd word of the backup SRAM.
 * @note Choosing 9 to be the first value of the enum is arbitrary.
 * Anything other than 0 (as it could be 0 without setting it)
 * would work fine
 */
typedef enum{
	DAQ_FAULT_LOGGED = 9,
	DAQ_FAULT_READ
}daq_log_status_t;
/**
 * @brief Includes the various software reset reasons.
 *
 * It's saved as `fault_log_t::reset_reason`
 * @note Choosing 7 to be the first value of the enum is arbitrary.
 * Anything other than 0 (as it could be 0 without setting it)
 * would work fine
 */
typedef enum{
	DAQ_RESET_REASON_NONE = 0,
	DAQ_RESET_REASON_MIN = 6,
	DAQ_RESET_REASON_HARDFAULT,
	DAQ_RESET_REASON_MEMMANAGE,
	DAQ_RESET_REASON_BUSFAULT,
	DAQ_RESET_REASON_USAGEFAULT,
	DAQ_RESET_REASON_ERRORHANDLER,
	DAQ_RESET_REASON_TASKFAULTHANDLER,
	DAQ_RESET_REASON_MAX
}daq_reset_reason_t;
/**
 * @brief The first 2 words of the backup SRAM used in fault logging.
 *
 * @note a variable of an enum type, by default, has the same size as an `int`.
 * Therefore, each of these is a whole word.
 */
typedef struct{
	daq_bkpsram_state_t bkpsram_state;
	daq_log_status_t	log_status;
}daq_status_words_t;
/**
 * @brief Includes the types of read operations used by the `DAQ_FaultLog_Read` function.
 *
 */
typedef enum{
	DAQ_READ_PREVIOUS_LOG,
	DAQ_READ_CURRENT_LOG,
	DAQ_READ_STATUS_WORDS
}daq_bkpsram_read_type_t;
/**
 * @brief Includes the types of write operations used by the `DAQ_FaultLog_Write` function.
 *
 */
typedef enum{
	DAQ_WRITE_LOG,
	DAQ_WRITE_BKPSRAM_STATE,
	DAQ_WRITE_LOG_STATUS,
	DAQ_CLEAR_CURRENT_LOG
}daq_bkpsram_write_type_t;
/**
 * @brief Format for the DAQ system timestamp.
 * @note The counter depends on the frequency of the timer it will be used with.
 */
typedef struct{
	uint8_t  seconds;
	uint8_t  minutes;
	uint8_t  hours;
	uint16_t counter;
}daq_timestamp_t;
/**
 * @brief A struct for task stats needed for fault detection and logging.
 *
 */
typedef struct {
	uint32_t start_tick; /*!< Saves the RTOS tick the task started with (Could be used with any other tick). */
	uint32_t end_tick;/*!< Saves the RTOS tick the task ended with (Could be used with any other tick). */
	uint8_t runtime; /*!< Holds the execution time of the task in RTOS ticks. */
	uint8_t entry_count; /*!< Counts the number of times a task has been executed till its end. */
	uint8_t error_count; /*!< Counts the number of errors the task has caused. */
	uint8_t task_demoted; /*!< Set to 1 if the task was kicked from the system (ie the error count reached `DAQ_MAX_ERROR_COUNT`). */
} task_stats_t;

/* added 8/4/2026 struct to correctly offset the SP address */
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} stack_registers;
/* end */
/**
 * @brief A struct for each task's stats needed for fault detection and logging.
 *
 */
typedef struct{
	task_stats_t tasks[DAQ_NO_OF_READ_TASKS];
}daq_fault_record_t;
/**
 * @brief Contains all fault info to be logged to or read from the backup SRAM.
 *
 */
typedef struct {
    uint8_t reset_reason; /*!< One of the reset reasons in `daq_reset_reason_t`. */
    uint8_t fault_status; /*!< Stores the fault status from the register `SCB->CFSR`. */
    uint32_t fault_address; /*!< Stores the fault address from the fault address registers in `SCB`. */
    uint32_t stack_frame[8];  /*!< Suggested future improvement */
    daq_fault_record_t task_records; /*!< Stores all tasks' fault record */
    daq_timestamp_t timestamp; /*!< Stores the global timestamp of the DAQ system. */
} fault_log_t;
/**
 * @brief A buffer type for storing the previous and current faults.
 *
 */
typedef struct{
    fault_log_t prev; /*!< The last successfully captured fault on the backup SRAM. */
    fault_log_t current; /*!< The current read fault on the backup SRAM. */
}daq_fault_log_buffer_t;
/**
 * @brief Initializes the backup SRAM and Fault Handlers.
 *
 */
void DAQ_FaultLog_Init(void);
/**
 * @brief Reads the previous and current fault logs from the backup SRAM.
 * @param a pointer to the log fault buffer to write the fault log in.
 *
 * @note After the function successfully reads the current fault log, it
 * clears it so as not to be redundantly read again.
 */
void DAQ_FaultLog_Read(daq_fault_log_buffer_t* log);
/**
 * @brief Writes the received fault log as the current and previous fault logs on the backup SRAM.
 * @param a pointer to the log to be written.
 *
 * The function writes the log 2 times, one on the current, and the other on the previous.
 */
void DAQ_FaultLog_Write(fault_log_t* log);
/**
 * @brief An RTOS task responsible for checking up on the system and catching errors.
 *
 * This task is where the tasks' error detection takes place. It runs
 * once every 60ms, and if a task is found to be affecting the rest of the system
 * (e.g. blocking other tasks), this task will detect the faulty task, increase its
 * error counter, and, finally, call the @ref DAQ_Task_Fault_Handler.
 *
 * @note The WWDG gets triggered after about 110ms in this system.
 */
void DAQ_WWDG_Task(void *pvParameters);
/**
 * @brief Logs the fault info and stalls the system until a WWDG reset occurs.
 *
 */
/* added 8/4/2026 the new Hardfault handler function which includes the SP */
void DAQ_HardFault_Handler(stack_registers *frame);
/* end */

void DAQ_Task_Fault_Handler(void);
/**
 * @brief When a fault occurs, this task is created as a warning.
 *
 */
void DAQ_Fault_Blink(void *pvParameters);
/** @} */

/**
 * @addtogroup CAN_Module
 * @{
 */

/**
 * @brief Checks if the reading change exceeded minimum change
 *
 * This macro checks if the difference between current and previous readings exceeded
 * the minimum change specified in the `DAQ_Config.h` file. If so, it returns `true`.
 *
 * @param current_data the current reading
 * @param perv_data	the previous reading
 * @param min_change the minimum change, more than which readings will be sent
 * @return `true` if the reading changed enough to be transmitted on the CAN, `false` otherwise.
 */
#define DAQ_CheckChange(current_data, prev_data, min_change) (fabs(current_data - prev_data) >= min_change)
/**
 * @brief Transmission IDs for CAN messages.
 *
 * @note Receiving addresses should also be added here (ECU's Address for instance).
 */
typedef enum
{
	DAQ_CAN_ID_IMU_ANGLE = DAQ_CAN_BASE_ID,
	DAQ_CAN_ID_IMU_ACCEL,
	DAQ_CAN_ID_ADC,
	DAQ_CAN_ID_PROX_ENCODER,
	DAQ_CAN_ID_GPS,
	DAQ_CAN_ID_TEMP,
	DAQ_CAN_ID_STEERING_ENC,
} daq_can_id_t;
/**
 * @brief Message Info to be enqueued in the CAN queue.
 *
 * The `daq_can_msg_t::data` is used as the encoding container for all CAN messages.
 */
typedef struct{
    uint64_t data; /*!< Data to be sent. Maximum of 8 bytes (as specified by CAN protocol). */
    uint16_t id;   /*!< 11-bit CAN message id. Must be the least-significant 11 bits. */
    uint8_t size;  /*!< Size of the data in bytes. */
} daq_can_msg_t;
/**
 * @brief Format for encoding IMU readings for transmission via CAN.
 *
 * Linear accelerations and Euler angles are encoded separately as different `daq_can_msg_imu_t` objects.
 *
 */
typedef struct
{
    int16_t x; /*!< x-component of either linear acceleration or Euler angle. */
    int16_t y; /*!< y-component of either linear acceleration or Euler angle. */
    int16_t z; /*!< z-component of either linear acceleration or Euler angle. */
    uint16_t _padding; // Manual padding for predictable alignment.
} daq_can_msg_imu_t;
static_assert(sizeof(daq_can_msg_imu_t) == 8, "IMU CAN Message must be 8 bytes");
/**
 * @brief Format for encoding ADC Sensors readings for transmission via CAN.
 *
 * Includes both suspension linear position sensor and brake pressure sensor readings.
 * The readings are sent as percentages (from the maximum value) multiplied by
 * 100 to preserve 2 decimal places.
 */
typedef struct
{
    uint64_t suspension_front_left 	: 10;
    uint64_t suspension_front_right : 10;
    uint64_t suspension_rear_left 	: 10;
    uint64_t suspension_rear_right 	: 10;

    uint64_t pressure_rear_left 	: 10;
    uint64_t pressure_rear_right 	: 10;
    uint64_t _padding: 4; // Manual padding for predictable alignment.
} daq_can_msg_adc_t;
static_assert(sizeof(daq_can_msg_adc_t) == 8, "ADC CAN Message must be 8 bytes");
/**
 * @brief Format for encoding Proximity Speed readings for transmission via CAN.
 *
 * Contains RPM for the 4 wheels, the angle of the steering wheel from rotary encoder,
 * and the speed of the car in km/h.
 *
 * @note The steering wheel angle measurement functionality has not yet been added,
 * but its place is reserved here.
 */
typedef struct
{
    uint64_t rpm_front_left : 11;
    uint64_t rpm_front_right: 11;
    uint64_t rpm_rear_left 	: 11;
    uint64_t rpm_rear_right : 11;

    uint64_t encoder_angle 	: 10;
    uint64_t Speedkmh 		: 8;
} daq_can_msg_prox_t;
static_assert(sizeof(daq_can_msg_prox_t) == 8, "Proximity CAN Message must be 8 bytes");
/**
 * @brief Format for encoding GPS readings for transmission via CAN.
 *
 * Contains latitude and longitude as floats in decimal format.
 */
typedef struct
{
    float latitude;
    float longitude;
} daq_can_msg_gps_t;
static_assert(sizeof(daq_can_msg_gps_t) == 8, "GPS CAN Message must be 8 bytes");
/**
 * @brief Format for encoding Temperature readings for transmission via CAN.
 *
 *
 * Contains each wheel's temperature as a `uint16_t` value. The original
 * float temperature (in degrees Celsius) is multiplied by 100 and cast to
 * `uint16_t` to preserve two decimal places of accuracy during transmission.
 */
typedef struct
{
    uint16_t temp_front_left;
    uint16_t temp_front_right;
    uint16_t temp_rear_left;
    uint16_t temp_rear_right;
} daq_can_msg_temp_t;
static_assert(sizeof(daq_can_msg_temp_t) == 8, "Temperature CAN Message must be 8 bytes");
/**
 * @brief Initializes the CAN task by starting CAN communication and setting the can_tx_header.
 *
 * @param can_handle Pointer to CAN handle object.
 * @param can_tx_header Pointer to CAN TX header object.
 */
void DAQ_CAN_Init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header);
/**
 * @brief Enqueues a CAN message of type `daq_can_msg_t` to the FreeRTOS queue to be transmitted on CAN bus.
 * @param msg Pointer to CAN message object to be enqueued.
 *
 * After enqueuing, the message then dequeued by the `DAQ_CAN_Msg_Dequeue`
 * inside the `DAQ_CAN_Task` for transmitting on the CAN bus.
 */
void DAQ_CAN_Msg_Enqueue(daq_can_msg_t* msg);
/**
 * @brief Dequeues a CAN message of type `daq_can_msg_t` from the FreeRTOS CAN queue.
 *
 * @param msg Pointer to the variable in which the received CAN message (from the queue) will be stored.
 * @return `pdTRUE` if successfully executed
 * @attention This function should only be used in CAN task. If the queue is empty, it will block the task until the queue is not empty.
 *
 * After dequeuing inside the `DAQ_CAN_Task`, the message is transmitted on the CAN bus.
 */
BaseType_t DAQ_CAN_Msg_Dequeue(daq_can_msg_t* msg);
/**
 * @brief A FreeRTOS task that dequeues the CAN message and transmits it.
 *
 * This is the last station for the CAN message.
 *
 * @note This function is event based: it only gets
 * unblocked when a new message is added to the queue.
 *
 */
void DAQ_CAN_Task(void *pvParameters);
/** @} */

#endif /* DAQ_DAQ_H_ */
