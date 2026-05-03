/*
 * Position.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */
#include "Position.h"

extern fault_record_t g_daq_fault_record;                  // Global DAQ fault record for diagnostics
volatile uint16_t adc_raw_values[DAQ_NO_OF_ADC_SENSORS] = {0}; // Latest raw ADC readings

adc_median_filter_t adc_filters[DAQ_NO_OF_ADC_SENSORS];        // Median filters for all ADC sensors
adc_reading_buffer_t adc_reading_buffer;                       // Buffers for current and previous readings


static void Swap_PtPt(uint16_t **a, uint16_t **b)
{
    uint16_t *temp = *a;
    *a = *b;
    *b = temp;
}
void Median_Init(median_filter_t* this, uint16_t* buffer, uint16_t** pt_buffer_sorted, uint16_t size)
{
    if (this == NULL)
    {
        return;
    }
    else if ((buffer == NULL)||(pt_buffer_sorted == NULL)||(size < 3)||(size > UINT8_MAX))
    {
        this->init=false;
        return;
    }

    this->buffer=buffer;
    this->pt_buffer_sorted=pt_buffer_sorted;
    this->size=size;
    this->index=0;
    this->initial_knuth_gap=1;
    this->init=true;

    Median_Buffer_Clear(this);
    Median_Buffer_Init(this);
    Median_SortingGap_Init(this);
}

uint16_t Median_Filter(median_filter_t* this, uint16_t input)
{
    if ((this == NULL)||(this->init==false))
        return 0;

    this->buffer[this->index] = input;

    /*increment index*/
    this->index = (this->index + 1) % this->size;

    Median_Buffer_ShellSort(this);

    return Median_ValueGet(this);
}

uint8_t Median_IterationGet(median_filter_t* this)
{
    if ((this == NULL)||(this->init==false))
        return 0;

    return this->iteration_count;
}

void Median_SortingGap_Init(median_filter_t* this)
{
    /* calculate initial gap using Knuth sequence for Shell-Sort */
    this->initial_knuth_gap=1;
    while (this->initial_knuth_gap < this->size / 3)
    {
        this->initial_knuth_gap = this->initial_knuth_gap * 3 + 1;
    }
}

uint16_t Median_ValueGet(median_filter_t* this)
{
    if (this->size % 2)
    {   /* return the middle value */
        return *this->pt_buffer_sorted[(this->size / 2)];
    }
    else
    {   /* return the average of the two middle values */
        return (*this->pt_buffer_sorted[(this->size / 2)] + *this->pt_buffer_sorted[(this->size / 2) - 1]) / 2;
    }
}

void Median_Buffer_Clear(median_filter_t* this)
{
    if (this == NULL)
        return;

    for (uint8_t i = 0; i < this->size; i++)
        this->buffer[i] = 0;
}

void Median_Buffer_Init(median_filter_t* this)
{
    for (uint8_t i = 0; i < this->size; i++)
        this->pt_buffer_sorted[i] = &this->buffer[i];
}

void Median_Buffer_ShellSort(median_filter_t* this)
{
    if (this == NULL)
        return;

    this->iteration_count = 0;

    uint8_t gap=this->initial_knuth_gap;

    while (gap > 0)
    {
        for (uint8_t  index = gap; index < this->size; index++)
        {
            uint16_t* currentElement = this->pt_buffer_sorted[index];

            /* If left element is larger, swap until correct position is found. */
            while (index >=gap && *this->pt_buffer_sorted[index - gap] > *currentElement)
            {
                Swap_PtPt(&this->pt_buffer_sorted[index - gap],&this->pt_buffer_sorted[index]);
                index-= gap;
                this->iteration_count++;
            }
            this->pt_buffer_sorted[index] = currentElement;
        }

        /* calculate next gap, using Knuth sequence */
        if (gap!=1)
        {
            gap = (gap - 1) / 3;
        }else
        {
            gap=0;
        }
    }
}
void ADC_Sensors_Init(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start_DMA(hadc, adc_raw_values, DAQ_NO_OF_ADC_SENSORS);
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		Median_Init(&adc_filters[i].filter, adc_filters[i].buffer, adc_filters[i].pt_buffer_sorted, MEDIAN_FILTER_BUFFER_SIZE);
}

void ADC_Sensors_Process(volatile uint16_t *raw_adc_values)
{
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		adc_filters[i].filtered_value =  Median_Filter(&adc_filters[i].filter, raw_adc_values[i]);
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		adc_filters[i].percent = (float)adc_filters[i].filtered_value / 4096.0;

}
void ADC_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
    {
		g_daq_fault_record.tasks[ADC_TASK].start_tick = xTaskGetTickCount();

		// Raw data is already read by DMA.
		ADC_Sensors_Process(adc_raw_values);

		// Set the current value to the filtered value.
		for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
			adc_reading_buffer.current[i] = adc_filters[i].filtered_value;

		// Transmit CAN message only if sufficient change occurs.
		if(DAQ_CheckChange(adc_reading_buffer.current[SUSPENSION_FRONT_LEFT], adc_reading_buffer.prev[SUSPENSION_FRONT_LEFT], 	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_reading_buffer.current[SUSPENSION_FRONT_RIGHT],adc_reading_buffer.prev[SUSPENSION_FRONT_RIGHT], 	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_reading_buffer.current[SUSPENSION_REAR_LEFT], 	adc_reading_buffer.prev[SUSPENSION_REAR_LEFT],	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_reading_buffer.current[SUSPENSION_REAR_RIGHT], adc_reading_buffer.prev[SUSPENSION_REAR_RIGHT],	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_reading_buffer.current[PRESSURE_REAR_LEFT], 	adc_reading_buffer.prev[PRESSURE_REAR_LEFT], 		DAQ_MIN_CHANGE_PRESSURE)   ||
		   DAQ_CheckChange(adc_reading_buffer.current[PRESSURE_REAR_RIGHT], 	adc_reading_buffer.prev[PRESSURE_REAR_RIGHT], 	DAQ_MIN_CHANGE_PRESSURE))
		{
			daq_can_msg_t can_msg_adc = {0};
			daq_can_msg_adc_t encoder_msg_adc = {0};

			encoder_msg_adc.suspension_front_left 	= (uint64_t)(adc_filters[SUSPENSION_FRONT_LEFT].percent * DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_front_right	= (uint64_t)(adc_filters[SUSPENSION_FRONT_RIGHT].percent* DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_rear_left 	= (uint64_t)(adc_filters[SUSPENSION_REAR_LEFT].percent	* DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_rear_right 	= (uint64_t)(adc_filters[SUSPENSION_REAR_RIGHT].percent * DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.pressure_rear_left 		= (uint64_t)(adc_filters[PRESSURE_REAR_LEFT].percent	* DAQ_ACCURACY_PRESSURE);
			encoder_msg_adc.pressure_rear_right 	= (uint64_t)(adc_filters[PRESSURE_REAR_RIGHT].percent	* DAQ_ACCURACY_PRESSURE);

			can_msg_adc.id = DAQ_CAN_ID_ADC;
			can_msg_adc.data = *((uint64_t*)(&encoder_msg_adc));
			can_msg_adc.size = 8;

			DAQ_CAN_Tx_Msg_Enqueue(&can_msg_adc);

			// Save current readings to previous.
			for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
				adc_reading_buffer.prev[i] = adc_reading_buffer.current[i];
		}
		// Update Task Stats
		g_daq_fault_record.tasks[ADC_TASK].entry_count++;
		g_daq_fault_record.tasks[ADC_TASK].end_tick = xTaskGetTickCount();
		g_daq_fault_record.tasks[ADC_TASK].runtime = g_daq_fault_record.tasks[ADC_TASK].end_tick - g_daq_fault_record.tasks[ADC_TASK].start_tick;

		vTaskDelayUntil(&xLastWakeTime, 7);
	}
}
