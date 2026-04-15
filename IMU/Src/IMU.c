/*
 * IMU.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#include "IMU.h"
#include "IMU_Private.h"
#include "wwdg.h"
#include "math.h"

#ifndef PI
#define PI 					3.14159265359
#endif /* PI */

extern SemaphoreHandle_t g_i2c_mutex;
extern fault_record_t g_daq_fault_record;

imu_reading_buffer_t imu_accels_buffer, imu_angles_buffer; // Angle and Acceleration buffers.

static imu_opmode_t imu_mode; // The global (in this file) mode of operation of the IMU.
static I2C_HandleTypeDef* GlobalConfig = NULL; //pointer to save sensor I2C handle.


void IMU_SetMode(imu_opmode_t mode)
{
  imu_mode = mode;
  HAL_I2C_Mem_Write(GlobalConfig , IMU_I2C_ADDRESS , BNO055_OPR_MODE_ADDR , 1 , &imu_mode , 1 , 20);
  HAL_Delay(30);
}

void IMU_GetVector(imu_vector_type_t vector_type , float* xyz)
{

  uint8_t buffer[6] = {0};

  int16_t x = 0, y = 0, z = 0;


  /* Read vector data (6 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , IMU_I2C_ADDRESS , (adafruit_bno055_reg_t)vector_type , 1 , buffer , 6 , 20);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type)
  {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  }
}
void IMU_WriteData(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(GlobalConfig , IMU_I2C_ADDRESS , reg , 1 , &data , 1 , 20);
}
void IMU_SetAxisMap(imu_axis_map_t axis)
{
	uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
	uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
	IMU_WriteData(BNO055_AXIS_MAP_CONFIG_ADDR, axisRemap);
	IMU_WriteData(BNO055_AXIS_MAP_SIGN_ADDR, axisMapSign);
}

void IMU_SelectRegPage(uint8_t page)
{
	if(page == 0 || page == 1)
		IMU_WriteData(BNO055_PAGE_ID_ADDR, page);
	HAL_Delay(2);
}
void IMU_Eulers_Apply_Offset(imu_vector_t* angles)
{
	angles->x -= IMU_EULER_ANGLE_OFFSET_X;
	angles->y -= IMU_EULER_ANGLE_OFFSET_Y;
}
void IMU_Transform_Accels(imu_vector_t* accels)
{
	// Calculated once at the beginning of the program.
	static const float cosx = cos(IMU_EULER_ANGLE_OFFSET_X);
	static const float cosy = cos(IMU_EULER_ANGLE_OFFSET_Y);
	static const float sinx = sin(IMU_EULER_ANGLE_OFFSET_X);
	static const float siny = sin(IMU_EULER_ANGLE_OFFSET_Y);

	// Store initial readings because there's more than 1 edit operations.
	float x_old = accels->x, y_old = accels->y, z_old = accels->z;

	// Apply the linear transformation (See the documentation for details).
	accels->x = x_old*cosy + y_old*sinx*siny + z_old*cosx*siny;
	accels->y = y_old*cosx - z_old*sinx;
	accels->z = - x_old*siny + y_old*sinx*cosy + z_old*cosx*cosy;
}
void IMU_Init(I2C_HandleTypeDef* hi2c, imu_opmode_t mode, imu_axis_map_t map)
{
	GlobalConfig = hi2c;
	uint8_t Data = 0;
	uint8_t id;
	HAL_StatusTypeDef status;

	// Reset the sensor first
	Data = 0x20;
	status = HAL_I2C_Mem_Write(GlobalConfig, IMU_I2C_ADDRESS, BNO055_SYS_TRIGGER_ADDR, 1, &Data, 1, 20);
	if (status != HAL_OK)
		return;

	// 650ms delay, but with WWDG Refresh to prevent a reset.
	for(uint8_t i = 0; i < 13; i++)
	{
		HAL_WWDG_Refresh(&hwwdg);
		HAL_Delay(50);
	}
	// Check device ID
	status = HAL_I2C_Mem_Read(GlobalConfig, IMU_I2C_ADDRESS, BNO055_CHIP_ID_ADDR, 1, &id, 1, 20);
	if (status != HAL_OK || id != IMU_ID)
		return;

	// Set to normal power mode
	IMU_WriteData(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	HAL_Delay(10);

	// Switch to register page 1
	IMU_SelectRegPage(1);
	// Set acceleration max to 16g.
	IMU_WriteData(ACCEL_CONFIG, BNO055_MAX_ACCELERATION_16G);
	HAL_Delay(10);
	// Switch to register page 0
	IMU_SelectRegPage(0);

	// Remap axes
	IMU_SetAxisMap(map);

	// Set operating mode
	IMU_SetMode(mode);
	HAL_WWDG_Refresh(&hwwdg);
}
void IMU_Task(void*pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
//	int counter = 0;
	for( ;; )
	{
//		if(g_daq_fault_record.tasks[IMU_TASK].error_count == 0 && counter++ == 250) {
//			while (1);
//		}

		g_daq_fault_record.tasks[IMU_TASK].start_tick = xTaskGetTickCount();
		if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			// Read linear acceleration and Euler angles (Tait-Bryan actually).
//			IMU_GetVector(VECTOR_EULER, (float*)&imu_angles_buffer.current);
//			IMU_GetVector(VECTOR_LINEARACCEL, (float*)&imu_accels_buffer.current);
			xSemaphoreGive(g_i2c_mutex);

			// Correct the readings according to the defined mounting offsets.
			IMU_Eulers_Apply_Offset(&imu_angles_buffer.current);
			IMU_Transform_Accels(&imu_accels_buffer.current);
		}
//		if(DAQ_CheckChange(imu_angles_buffer.current.x, imu_angles_buffer.prev.x, DAQ_MIN_CHANGE_IMU_ANGLE_X) ||
//		   DAQ_CheckChange(imu_angles_buffer.current.y, imu_angles_buffer.prev.y, DAQ_MIN_CHANGE_IMU_ANGLE_Y) ||
//		   DAQ_CheckChange(imu_angles_buffer.current.z, imu_angles_buffer.prev.z, DAQ_MIN_CHANGE_IMU_ANGLE_Z))
		{
			daq_can_msg_t can_msg_imu_angle = {0};
			daq_can_msg_imu_t encoder_msg_imu_angle = {0};

			encoder_msg_imu_angle.x = (int16_t)(imu_angles_buffer.current.x * DAQ_ACCURACY_IMU_ANGLE_X);
			encoder_msg_imu_angle.y = (int16_t)(imu_angles_buffer.current.y * DAQ_ACCURACY_IMU_ANGLE_Y);
			encoder_msg_imu_angle.z = (int16_t)(imu_angles_buffer.current.z * DAQ_ACCURACY_IMU_ANGLE_Z);

			can_msg_imu_angle.id = DAQ_CAN_ID_IMU_ANGLE;
			can_msg_imu_angle.size = 8;
			can_msg_imu_angle.data = *((uint64_t*)(&encoder_msg_imu_angle));

			DAQ_CAN_Msg_Enqueue(&can_msg_imu_angle);

			// Save current data to previous to check if it changed significantly.
			imu_angles_buffer.prev.x = imu_angles_buffer.current.x;
			imu_angles_buffer.prev.y = imu_angles_buffer.current.y;
			imu_angles_buffer.prev.z = imu_angles_buffer.current.z;
		}
//		if(DAQ_CheckChange(imu_accels_buffer.current.x, imu_accels_buffer.prev.x, DAQ_MIN_CHANGE_IMU_ACCEL) ||
//		   DAQ_CheckChange(imu_accels_buffer.current.y, imu_accels_buffer.prev.y, DAQ_MIN_CHANGE_IMU_ACCEL) ||
//		   DAQ_CheckChange(imu_accels_buffer.current.z, imu_accels_buffer.prev.z, DAQ_MIN_CHANGE_IMU_ACCEL))
		{
			daq_can_msg_t can_msg_imu_acceleration = {0};
			daq_can_msg_imu_t encoder_msg_imu_acceleration = {0};

			encoder_msg_imu_acceleration.x = (int16_t)(imu_accels_buffer.current.x * DAQ_ACCURACY_IMU_ANGLE_X);
			encoder_msg_imu_acceleration.y = (int16_t)(imu_accels_buffer.current.y * DAQ_ACCURACY_IMU_ANGLE_Y);
			encoder_msg_imu_acceleration.z = (int16_t)(imu_accels_buffer.current.z * DAQ_ACCURACY_IMU_ANGLE_Z);

			can_msg_imu_acceleration.id = DAQ_CAN_ID_IMU_ACCEL;
			can_msg_imu_acceleration.size = 8;
			can_msg_imu_acceleration.data = *((uint64_t*)(&encoder_msg_imu_acceleration));

			DAQ_CAN_Msg_Enqueue(&can_msg_imu_acceleration);

			// Save current data to previous to check if it changed significantly.
			imu_accels_buffer.prev.x = imu_accels_buffer.current.x;
			imu_accels_buffer.prev.y = imu_accels_buffer.current.y;
			imu_accels_buffer.prev.z = imu_accels_buffer.current.z;
		}
		// Update task stats.
		g_daq_fault_record.tasks[IMU_TASK].entry_count++;
		g_daq_fault_record.tasks[IMU_TASK].end_tick = xTaskGetTickCount();
		g_daq_fault_record.tasks[IMU_TASK].runtime = g_daq_fault_record.tasks[IMU_TASK].end_tick - g_daq_fault_record.tasks[IMU_TASK].start_tick;

		vTaskDelayUntil(&xLastWakeTime, 8);
	}
}
