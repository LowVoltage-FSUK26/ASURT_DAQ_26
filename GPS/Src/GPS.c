/*
 * GPS.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */
#include "GPS.h"

extern SemaphoreHandle_t g_i2c_mutex;                 // Global I2C mutex for thread-safe access across tasks
extern fault_record_t g_daq_fault_record;         // Global fault record structure for DAQ system
extern daq_i2c_dma_device_t g_i2c_dma_device;         // Global I2C-DMA device configuration/instance
extern bool g_i2c_dma_flags[DAQ_NO_OF_I2C_DMA_DEVICES]; // Flags array indicating I2C-DMA device status

char gps_i2c_buffer[45];                              // Raw GPS data buffer read over I2C (max 45 bytes)
gps_gnrmc_data_t gps_data;                            // Parsed GPS GNRMC sentence data (time, position, speed, etc.)
static I2C_HandleTypeDef* gps_hi2c;                   // Pointer to the I2C handle used for GPS communication

/*============================== STATIC FUNCTIONS ==============================*/
/**
 * @addtogroup GPS_Module
 * @{
 */

/**
 * @brief Convert a string representing a floating-point number to a double.
 *
 * Parses the string manually without using standard library functions,
 * handling integer and fractional parts.
 *
 * @param str Null-terminated string containing the number to convert.
 * @return Converted double value.
 */
static double my_atof(const char *str)
{
    double result = 0.0;
    double fraction = 1.0;
    int sign = 1;  // Sign not used in this implementation but kept for extensibility
    // Parse integer part
    while (isdigit(*str))
    {
        result = result * 10.0 + (*str - '0');
        str++;
    }
    // Parse fractional part if present
    if (*str == '.')
    {
        str++;
        while (isdigit(*str))
        {
            fraction /= 10.0;
            result += (*str - '0') * fraction;
            str++;
        }
    }

    return result * sign;
}
/**
 * @brief Converts raw GPS coordinate format (ddmm.mmmm) to decimal degrees.
 *
 * GPS raw values are formatted as degrees and minutes combined.
 * This function extracts degrees and converts the minutes portion into degrees.
 *
 * @param rawValue String containing the raw GPS coordinate.
 * @return Converted coordinate in decimal degrees.
 */
static double convertToDegrees(char *rawValue)
{
    double val = my_atof(rawValue);
    int deg = (int)(val / 100.0);        // Extract degrees portion
    double min = val - (deg * 100.0);    // Extract minutes portion
    return deg + (min / 60.0);            // Convert minutes to decimal degrees and sum
}
/** @}*/
/*===========================================================================*/

void GPS_Init(I2C_HandleTypeDef *hi2c)
{
	gps_hi2c = hi2c;
}
void GPS_ReadGNRMC(I2C_HandleTypeDef *hi2c)
{
	// Checks if the I2C DMA is free.
	if(g_i2c_dma_device == I2C_DMA_NO_DEVICE)
	{
		g_i2c_dma_device = I2C_DMA_GPS; // For the callback function to know it's the GPS.
		HAL_I2C_Master_Receive_IT(hi2c, GPS_I2C_ADDRESS, gps_i2c_buffer, sizeof(gps_i2c_buffer));
	}
}
void GPS_ParseGNRMC(gps_gnrmc_data_t *data)
{
    char *token = strtok(gps_i2c_buffer, ",");
    int field = 0;
    if (strcmp(token, "$GNRMC") != 0)
            return ; // Validate sentence header

    // Parse comma-separated fields
    while (token != NULL)
    {
        switch (field)
        {
            case 1: strncpy(data->time, token, 10); data->time[10] = '\0'; break;
            case 2: data->status = token[0];
            if (token[0]=='V') {return;} // Invalid data status, abort parsing
		    break;
            case 3: data->latitude = convertToDegrees(token); break;
            case 4: data->lat_dir = token[0]; break;
            case 5: data->longitude = convertToDegrees(token); break;
            case 6: data->lon_dir = token[0]; return;
        }
        token = strtok(NULL, ",");
        field++;
    }
}
void GPS_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
    	g_daq_fault_record.tasks[GPS_TASK].start_tick = xTaskGetTickCount(); // Record task start time for diagnostics

    	// Acquire I2C mutex before initiating GPS read
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            GPS_ReadGNRMC(gps_hi2c);
            xSemaphoreGive(g_i2c_mutex);
        }

        // Check if GPS DMA receive is complete
        if(g_i2c_dma_flags[I2C_DMA_GPS])
        {
        	g_i2c_dma_flags[I2C_DMA_GPS] = 0;
        	GPS_ParseGNRMC(&gps_data);

        	// If valid GPS data received, encode and send via CAN
        	if (gps_data.status == 'A')
        	{
        		daq_can_msg_t can_msg_gps = {0};
        		daq_can_msg_gps_t encoder_msg_gps = {0};

        		 // Adjust sign based on hemisphere indicators
        		encoder_msg_gps.latitude = (gps_data.lat_dir == 'E') ? (float)gps_data.latitude : -(float)gps_data.latitude;
        		encoder_msg_gps.longitude = (gps_data.lon_dir == 'N') ? (float)gps_data.longitude : -(float)gps_data.longitude;

        		// Prepare CAN message
        		can_msg_gps.id = DAQ_CAN_ID_GPS;
        		can_msg_gps.size = 8;
        		can_msg_gps.data = *((uint64_t*)&encoder_msg_gps);
        		DAQ_CAN_Msg_Enqueue(&can_msg_gps);
        	}
        }

        // Update fault record diagnostics for this task
        g_daq_fault_record.tasks[GPS_TASK].entry_count++;
        g_daq_fault_record.tasks[GPS_TASK].end_tick = xTaskGetTickCount();
        g_daq_fault_record.tasks[GPS_TASK].runtime =
        	g_daq_fault_record.tasks[GPS_TASK].end_tick - g_daq_fault_record.tasks[GPS_TASK].start_tick;

        vTaskDelayUntil(&xLastWakeTime, 8);
    }
}
