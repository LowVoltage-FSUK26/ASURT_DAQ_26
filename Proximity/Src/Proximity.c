



#include "proximity.h"

// DMA capture buffer for proximity sensor pulses (per wheel)
uint16_t prox_dma_buffer[PROX_NO_OF_WHEELS][PROX_DMA_WHEEL_BUFFER_SIZE];

// Stores calculated RPM values and related timing data
prox_rpm_buffer_t prox_rpm_buffer;

// Timer handle for proximity measurement timing
static TIM_HandleTypeDef prox_timer_handle;

// DMA channel handles for each wheel’s proximity sensor
static DMA_HandleTypeDef prox_dma_handles[PROX_NO_OF_WHEELS];

// Direct pointers to timer capture/compare registers for each wheel channel
static uint32_t timer_counters[PROX_NO_OF_WHEELS] = {&TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3, &TIM1->CCR4};

// Shared DAQ fault record structure
extern fault_record_t g_daq_fault_record;

static inline float Prox_GetTimerFreq(void)
{
    // Get peripheral clock frequency for APB1 bus
    float pclk2 = (float)HAL_RCC_GetPCLK2Freq();

    // Check APB1 prescaler; timer clocks run at double frequency if prescaler != 1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        pclk2 *= 2.0f;

    // Return timer frequency after prescaler division
    return pclk2 / ((float)TIM1->PSC + 1.0f);
}
void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[PROX_NO_OF_WHEELS])
{
    prox_timer_handle = *htim; // Save timer handle for later use

    // Save DMA handles for each wheel
    for (uint8_t i = 0; i < PROX_NO_OF_WHEELS; i++)
        prox_dma_handles[i] = *hdma[i];

    // Start input capture with DMA on all 4 timer channels
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_1, prox_dma_buffer[FRONT_LEFT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, prox_dma_buffer[FRONT_RIGHT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_3, prox_dma_buffer[REAR_LEFT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_4, prox_dma_buffer[REAR_RIGHT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
}
void Prox_Task(void *pvParameters)
{
    uint8_t slow_counter[PROX_NO_OF_WHEELS] = {0, 0, 0, 0};
    while (1)
    {
        vTaskDelay(7);

        // Record task start time for diagnostics.
        g_daq_fault_record.tasks[PROX_TASK].start_tick = xTaskGetTickCount();

        // Iterate over each wheel
        for (uint8_t wheel_no = 0; wheel_no < PROX_NO_OF_WHEELS; wheel_no++)
        {
            uint8_t last_reading_index = 0;

            // Find the first zero value in DMA buffer (unused entries)
            for (last_reading_index = 0; last_reading_index < PROX_DMA_WHEEL_BUFFER_SIZE; last_reading_index++)
                if (prox_dma_buffer[wheel_no][last_reading_index] == 0)
                    break;

            // If enough valid readings found, wheel is moving fast enough
            if (last_reading_index > 1)
            {
                slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter

                // Calculate difference between last two readings (timer counts)
                uint16_t difference = prox_dma_buffer[wheel_no][last_reading_index - 1] - prox_dma_buffer[wheel_no][last_reading_index - 2];

                // Calculate current RPM based on timer frequency and count difference
                prox_rpm_buffer.current[wheel_no] = 0.25 * 60 * (Prox_GetTimerFreq() / difference);

                // Pause DMA transfer to safely reset buffer
                HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                // Clear DMA buffer for next measurement cycle
                memset(prox_dma_buffer[wheel_no], 0, PROX_DMA_WHEEL_BUFFER_SIZE * sizeof(uint16_t));

                // Reset DMA memory pointer and count for next transfer
                prox_dma_handles[wheel_no].Instance->M0AR = prox_dma_buffer[wheel_no];
                prox_dma_handles[wheel_no].Instance->NDTR = PROX_DMA_WHEEL_BUFFER_SIZE;

                // Restart DMA transfer
                HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], prox_dma_buffer[wheel_no], PROX_DMA_WHEEL_BUFFER_SIZE);
            }
            else
            {
            	// If slow reading count below threshold, increment counter
                if (slow_counter[wheel_no] <= 20)
                    slow_counter[wheel_no]++;
                else
                {
                	// Wheel considered stopped, reset RPM to zero
                    prox_rpm_buffer.current[wheel_no] = 0;
                    slow_counter[wheel_no] = 0;

                    // Pause DMA to reset buffer safely
                    HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                    // Clear buffer
                    memset(prox_dma_buffer[wheel_no], 0, PROX_DMA_WHEEL_BUFFER_SIZE * sizeof(uint16_t));

                    // Reset DMA pointers and counters for next cycle
                    prox_dma_handles[wheel_no].Instance->M0AR = prox_dma_buffer[wheel_no];
                    prox_dma_handles[wheel_no].Instance->NDTR = PROX_DMA_WHEEL_BUFFER_SIZE;

                    // Restart DMA transfer
                    HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], prox_dma_buffer[wheel_no], PROX_DMA_WHEEL_BUFFER_SIZE);
                }
            }
        }

        // Check if RPM values changed enough to warrant sending CAN message
        if(DAQ_CheckChange(prox_rpm_buffer.current[FRONT_LEFT_BUFF], prox_rpm_buffer.prev[FRONT_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
           DAQ_CheckChange(prox_rpm_buffer.current[FRONT_RIGHT_BUFF], prox_rpm_buffer.prev[FRONT_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_rpm_buffer.current[REAR_LEFT_BUFF], prox_rpm_buffer.prev[REAR_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_rpm_buffer.current[REAR_RIGHT_BUFF], prox_rpm_buffer.prev[REAR_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM))
        {
        	daq_can_msg_t can_msg_prox = {0};
        	daq_can_msg_prox_t encoder_msg_prox = {0};

        	can_msg_prox.id = DAQ_CAN_ID_PROX_ENCODER;
        	can_msg_prox.size = 8;

        	// Cast current RPM values to uint64_t for CAN encoding
        	encoder_msg_prox.rpm_front_left  = (uint64_t)prox_rpm_buffer.current[FRONT_LEFT_BUFF];
        	encoder_msg_prox.rpm_front_right = (uint64_t)prox_rpm_buffer.current[FRONT_RIGHT_BUFF];
        	encoder_msg_prox.rpm_rear_left   = (uint64_t)prox_rpm_buffer.current[REAR_LEFT_BUFF];
        	encoder_msg_prox.rpm_rear_right  = (uint64_t)prox_rpm_buffer.current[REAR_RIGHT_BUFF];
        	//encoder_msg_prox.ENCODER_angle = (uint64_t)ENCODER_get_angle();

        	// Calculate and assign vehicle speed based on front wheel RPMs
        	encoder_msg_prox.Speedkmh = (uint64_t) PROX_CALCULATE_SPEED(prox_rpm_buffer.current[FRONT_LEFT_BUFF], prox_rpm_buffer.current[FRONT_RIGHT_BUFF]);

        	// Pack and enqueue CAN message
        	can_msg_prox.data = *((uint64_t*)&encoder_msg_prox);
        	DAQ_CAN_Tx_Msg_Enqueue(&can_msg_prox);

        	// Update previous RPM values to current ones for change detection
        	for(uint8_t i = 0; i < PROX_NO_OF_WHEELS; i++)
        		prox_rpm_buffer.prev[i] = prox_rpm_buffer.current[i];
        }
        // Update fault record for this task's execution.
        g_daq_fault_record.tasks[PROX_TASK].entry_count++;
        g_daq_fault_record.tasks[PROX_TASK].end_tick = xTaskGetTickCount();
        g_daq_fault_record.tasks[PROX_TASK].runtime = g_daq_fault_record.tasks[PROX_TASK].end_tick - g_daq_fault_record.tasks[PROX_TASK].start_tick;
    }
}
