/*
 *  DMA_Map.c
 *
 *  Created on: Aug 31, 2025
 *  Author: Adel Faki @ Hexabitz
 */

#include "dma_map.h"


// External UART handles (IMU Module Ports)
//extern UART_HandleTypeDef huart1;  // P4
//extern UART_HandleTypeDef huart2;  // P2
//extern UART_HandleTypeDef huart3;  // P6
//extern UART_HandleTypeDef huart4;  // P1
//extern UART_HandleTypeDef huart5;  // P5
//extern UART_HandleTypeDef huart6;  // P3



static size_t map_index = 0;

uint8_t staging_buf[100] ={0};
uint8_t port_mem_buf[100][NUM_OF_PORTS] ={0};

// Example TX map:
//const map_entry_t module_tx_map[] = {
//        // P1 -> P4
//		{ FWD_TO_PORT, .src = UART_PORT1, .dst = UART_PORT4, .size = 10 },
//		// P3 -> MEM
//		{ PORT_TO_MEM, .src = UART_PORT3, .mem = port_mem_buf[P3], .size = 10 },
//		// P6 -> P1
//		{ FWD_TO_PORT, .src = UART_PORT6, .dst = UART_PORT1, .size = 10 },
//		// P6 -> P3
//		{ FWD_TO_PORT, .src = UART_PORT6, .dst = UART_PORT3, .size = 10 } };

// -----------------------------------------------------
// Start map execution
// -----------------------------------------------------
void MapTx_Start(void) {
	map_index = 0;
	MapTx_Setup();
}

// -----------------------------------------------------
// Advance to next entry
// -----------------------------------------------------
void MapTx_Setup(void) {
	if (map_index >= module_tx_map_len) {
		// Done
		return;
	}

	const map_entry_t *e = &module_tx_map[map_index];

	switch (e->type) {
	case PORT_TO_MEM:
		HAL_UARTEx_ReceiveToIdle_DMA(e->src, e->mem, e->size);
		__HAL_DMA_DISABLE_IT(e->src->hdmarx, DMA_IT_HT);
		break;

	case FWD_TO_PORT:
		HAL_UARTEx_ReceiveToIdle_DMA(e->src, staging_buf, e->size);
		__HAL_DMA_DISABLE_IT(e->src->hdmarx, DMA_IT_HT);
		break;

	case MEM_TO_PORT:
		HAL_UART_Transmit_DMA(e->dst, e->mem, e->size);
		break;

	default:
		break;
	}
}

// -------------------------------------------------
// IDLE Line Event Callback
// -------------------------------------------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	const map_entry_t *e = &module_tx_map[map_index];

	if (e->type == FWD_TO_PORT && e->src == huart) {
		HAL_UART_Transmit_DMA(e->dst, staging_buf, Size);
	}

    // For store-to-mem, you can advance immediately
	else if (e->type == PORT_TO_MEM && e->src == huart) {

		// Move to next entry
        map_index++;
        MapTx_Setup();
    }

//	map_index++;
//	MapTx_Setup();
}

/* only advance once TX really finishes. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    const map_entry_t *e = &module_tx_map[map_index];

    if (e->type == FWD_TO_PORT && e->dst == huart) {
        // Now safe to advance
        map_index++;
        MapTx_Setup();
    }
}

