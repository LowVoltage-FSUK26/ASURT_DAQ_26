/*
 * DAQ_BKPSRAM_Private.h
 *
 *  Created on: Aug 11, 2025
 *      Author: Belal
 */

#ifndef SRC_DAQ_BKPSRAM_PRIVATE_H_
#define SRC_DAQ_BKPSRAM_PRIVATE_H_

#include "DAQ.h"

/**
 * @addtogroup Fault_Module
 * @{
 */

/**
 * @brief defines backup SRAM addresses according to the memory layout mentioned in this group.
 */
typedef enum{
	DAQ_BKPSRAM_BASE_ADDR 			= 0x40024000,
	DAQ_BKPSRAM_STATE_ADDR 			= DAQ_BKPSRAM_BASE_ADDR,
	DAQ_BKPSRAM_LOG_STATUS_ADDR 	= DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_bkpsram_state_t),
	DAQ_BKPSRAM_CURRENT_LOG_ADDR 	= DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_words_t),
	DAQ_BKPSRAM_BUFFER_LOG_ADDR     = DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_words_t) + sizeof(fault_log_t),
	DAQ_BKPSRAM_PREVIOUS_LOG_ADDR 	= DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_words_t) + 2 * sizeof(fault_log_t)
}daq_bkpsram_addr_t;
/** @} */

#endif /* SRC_DAQ_BKPSRAM_PRIVATE_H_ */
