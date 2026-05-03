/*
 * Temperature.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */
#include "Temperature.h"
#include "Temperature_Private.h"

extern SemaphoreHandle_t g_i2c_mutex;           // Global mutex to protect I2C transactions
extern fault_record_t g_daq_fault_record;   // Shared DAQ fault log structure

moving_avg_t temp_moving_avgs[TEMP_NO_OF_SENSORS]; // Per-sensor moving average filters
temp_sensor_data_t temp_sensors_data[TEMP_NO_OF_SENSORS]; // Latest readings + metadata for each sensor
temp_reading_buffer_t temp_buffer;              // Temporary buffer for raw temperature readings

static I2C_HandleTypeDef* temp_hi2c;            // Local handle to active I2C interface


float Moving_Avg(float next_num, uint8_t sensor_index)
{
  //Subtract the oldest number from the prev sum, add the new number
  temp_moving_avgs[sensor_index].sum = temp_moving_avgs[sensor_index].sum - temp_moving_avgs[sensor_index].array[temp_moving_avgs[sensor_index].counter] + next_num;
  //Assign the nextNum to the position in the array
  temp_moving_avgs[sensor_index].array[temp_moving_avgs[sensor_index].counter++] = next_num;
  if (temp_moving_avgs[sensor_index].counter >= MOVING_AVG_WINDOW_SIZE)
  {
    temp_moving_avgs[sensor_index].counter = 0;
    temp_moving_avgs[sensor_index].flag = 1;
  }
  //return the average
  if(temp_moving_avgs[sensor_index].flag == 1)
    return temp_moving_avgs[sensor_index].sum / (float)MOVING_AVG_WINDOW_SIZE;
  else
    return 0.0;
}
void Temp_Init(I2C_HandleTypeDef * hi2c)
{
	temp_hi2c = hi2c; // Store I2C handle for later use
}
void Temp_ReadRaw(uint8_t sensor_index)
{
	// Read 3 bytes from the MLX90614 object temperature register
		HAL_I2C_Mem_Read(temp_hi2c,
						 (TEMP_I2C_BASE_ADDRESS + sensor_index * TEMP_I2C_ADDRESS_SPACING) << 1,
						 MLX90614_TOBJ_ADDR,
						 I2C_MEMADD_SIZE_8BIT,
						 temp_sensors_data[sensor_index].data,
						 3,
						 HAL_MAX_DELAY);
}
float Temp_Process(uint8_t sensor_index)
{
	// Combine the two data bytes into a 16-bit value
	temp_sensors_data[sensor_index].raw =
			(temp_sensors_data[sensor_index].data[1] << 8) |
			 temp_sensors_data[sensor_index].data[0];

	// Convert raw value to Celsius (according to datasheet)
	temp_sensors_data[sensor_index].temperature =
			(temp_sensors_data[sensor_index].raw * 0.02) - 273.15;

	// Apply moving average and return filtered temperature
	return Moving_Avg(temp_sensors_data[sensor_index].temperature, sensor_index);
}
void Temp_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
    {
		g_daq_fault_record.tasks[TEMP_TASK].start_tick = xTaskGetTickCount();

		// Acquire I2C mutex before accessing bus
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
        	// Read raw data from all sensors
        	//for(uint8_t i = 0; i < TEMP_NO_OF_SENSORS; i++)
        	//	Temp_ReadRaw(i);
            xSemaphoreGive(g_i2c_mutex);

            // Process each sensor reading
            for(uint8_t i = 0; i < TEMP_NO_OF_SENSORS; i++)
            	temp_buffer.current[i] = Temp_Process(i);
        }
        // If significant change detected, prepare CAN message.
//      if(DAQ_CheckChange(temp_buffer.current[TEMP_FRONT_LEFT], temp_buffer.prev[TEMP_FRONT_LEFT], DAQ_MIN_CHANGE_TEMP)  	||
//         DAQ_CheckChange(temp_buffer.current[TEMP_FRONT_RIGHT], temp_buffer.prev[TEMP_FRONT_RIGHT], DAQ_MIN_CHANGE_TEMP) 	||
//		   DAQ_CheckChange(temp_buffer.current[TEMP_REAR_LEFT], temp_buffer.prev[TEMP_REAR_LEFT], DAQ_MIN_CHANGE_TEMP) 	  	||
//		   DAQ_CheckChange(temp_buffer.current[TEMP_REAR_RIGHT], temp_buffer.prev[TEMP_REAR_RIGHT], DAQ_MIN_CHANGE_TEMP))
        {
        	daq_can_msg_t can_msg_temp = {0};
        	daq_can_msg_temp_t encoder_msg_temp = {0};

        	encoder_msg_temp.temp_front_left = (uint16_t)(temp_buffer.current[TEMP_FRONT_LEFT] * DAQ_ACCURACY_TEMP);
        	encoder_msg_temp.temp_front_right= (uint16_t)(temp_buffer.current[TEMP_FRONT_RIGHT] * DAQ_ACCURACY_TEMP);
        	encoder_msg_temp.temp_rear_left  = (uint16_t)(temp_buffer.current[TEMP_REAR_LEFT] * DAQ_ACCURACY_TEMP);
        	encoder_msg_temp.temp_rear_right = (uint16_t)(temp_buffer.current[TEMP_REAR_RIGHT] * DAQ_ACCURACY_TEMP);

        	can_msg_temp.id = DAQ_CAN_ID_TEMP;
        	can_msg_temp.size = 8;
        	can_msg_temp.data = *((uint64_t*)&encoder_msg_temp);

        	DAQ_CAN_Tx_Msg_Enqueue(&can_msg_temp);

        	// Store current readings for change detection in next cycle
        	for(uint8_t i = 0; i < TEMP_NO_OF_SENSORS; i++)
        		temp_buffer.prev[i] = temp_buffer.current[i];
        }
        // Record task execution stats
        g_daq_fault_record.tasks[TEMP_TASK].entry_count++;
        g_daq_fault_record.tasks[TEMP_TASK].end_tick = xTaskGetTickCount();
        g_daq_fault_record.tasks[TEMP_TASK].runtime =
        	g_daq_fault_record.tasks[TEMP_TASK].end_tick - g_daq_fault_record.tasks[TEMP_TASK].start_tick;

        vTaskDelayUntil(&xLastWakeTime, 9);
    }
}
