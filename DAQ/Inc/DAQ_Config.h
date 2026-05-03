/*
 * DAQ_Config.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */

#ifndef DAQ_DAQ_CONFIG_H_
#define DAQ_DAQ_CONFIG_H_

#include <stdint.h>

#define DAQ_TIRE_CIRCUMFERENCE 3.1415 * 18 * 0.0254 // Used in proximity rpm calculations

#define DAQ_CAN_BASE_ID 				0x71 // Used as the base ID for CAN messages sent by the DAQ

#define DAQ_NO_OF_I2C_DMA_DEVICES 		1 + 1 // "+ 1" because the first element is unused

#define DAQ_NO_OF_TEMP_SENSORS			1 // Should be 4.

#define DAQ_IMU_RST_GPIO_PORT			GPIOA
#define DAQ_IMU_RST_PIN					GPIO_PIN_3

#define DAQ_CAN_MAX_WAIT_TICKS 			10 // Max CAN waiting time before at least a message is enqueued.

#define DAQ_CAN_TIMEOUT_MS    			500U // CAN bus receive timeout


/**
 * @addtogroup Trace_Module
 * @{
 */
/**
 * @brief Defining it enables trace recording.
 */
//#define DAQ_TRACE_RECORDER // Comment if tracing is not used.
/** @}*/

/*=========================== DAQ Minimum Changes ===========================*/
// A CAN message is sent only if the difference between the
// previous and current readings is more than the minimum change.
#define	DAQ_MIN_CHANGE_IMU_ANGLE_X 		0.01
#define	DAQ_MIN_CHANGE_IMU_ANGLE_Y 		0.01
#define	DAQ_MIN_CHANGE_IMU_ANGLE_Z 		0.1
#define	DAQ_MIN_CHANGE_IMU_ACCEL		0.1
#define	DAQ_MIN_CHANGE_PROX_RPM 		1
#define	DAQ_MIN_CHANGE_TEMP 			0.1
#define	DAQ_MIN_CHANGE_SUSPENSION 		5
#define	DAQ_MIN_CHANGE_PRESSURE 		5
/*===========================================================================*/

/**
 * @breif defines the number of different task types.
 */
typedef enum{
	DAQ_NO_OF_READ_TASKS = 5,
	DAQ_NO_OF_TASKS 	 = DAQ_NO_OF_READ_TASKS + 2 // added 2 for the WWDG and CAN tasks.
}daq_task_num_t;
/**
 * @breif defines the number of ADC related sensors.
 */
typedef enum{
	DAQ_NO_OF_SUSPENSION	= 4,
	DAQ_NO_OF_PRESSURE		= 2,
	DAQ_NO_OF_ADC_SENSORS  	= DAQ_NO_OF_SUSPENSION + DAQ_NO_OF_PRESSURE
}daq_adc_nums_t;
/**
 * @breif defines i2c-related constants (addresses and spacing between temp sensors addresses).
 */
typedef enum{
	DAQ_GPS_I2C_ADDRESS			=	0x42 << 1,
	DAQ_IMU_I2C_ADDRESS			=	0x28 << 1, 	//IMU's ADR pin is grounded.
	DAQ_TEMP_I2C_BASE_ADDRESS	=	0x5A, 		// The "<< 1" is in the Temperature.c file
	DAQ_TEMP_I2C_ADDRESS_SPACING= 	1     	// Subjected to change
}daq_i2c_related_t;
/**
 * @breif defines the number multiplied by each reading to preserve decimal places.
 */
typedef enum{
	DAQ_ACCURACY_IMU_ANGLE_X  	= 100,
	DAQ_ACCURACY_IMU_ANGLE_Y  	= 100,
	DAQ_ACCURACY_IMU_ANGLE_Z 	= 10,
	DAQ_ACCURACY_IMU_ACCEL  	= 10,
	DAQ_ACCURACY_SUSPENSION 	= 1000,
	DAQ_ACCURACY_PRESSURE 		= 1000,
	DAQ_ACCURACY_TEMP 			= 10
}daq_accuracies_t;
/**
 * @breif defines fault related limits.
 */
typedef enum{
	DAQ_MAX_ERROR_COUNT = 5,
	DAQ_MAX_RUNTIME_TICKS = 5,
	DAQ_MAX_TOTAL_TICKS_ELAPSED = 10
}daq_fault_limits_t;

#endif /* DAQ_DAQ_CONFIG_H_ */
