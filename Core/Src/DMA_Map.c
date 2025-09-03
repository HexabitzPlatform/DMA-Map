/*
 *  DMA_Map.c
 *
 *  Created on: Aug 31, 2025
 *  Author: Adel Faki @ Hexabitz
 */

#include "dma_map.h"

// Example buffers
uint8_t staging_buf[100];

uint8_t port_mem_buf[100][NUM_OF_PORTS];

// External UART handles (from CubeMX init)
extern UART_HandleTypeDef huart1;  // P1
extern UART_HandleTypeDef huart2;  // P2
extern UART_HandleTypeDef huart3;  // P3
extern UART_HandleTypeDef huart4;  // P4
extern UART_HandleTypeDef huart5;  // P5
extern UART_HandleTypeDef huart6;  // P6

// -----------------------------------------------------
// Example TX map:
// -----------------------------------------------------
const map_entry_t module_tx_map[] = {
  // P1 -> P4
  { FWD_TO_PORT, .src=&huart1, .dst=&huart4, .size=10 },
  // P3 -> MEM
  { PORT_TO_MEM, .src=&huart3, .mem=port_mem_buf[P3], .size=10 },
  // P6 -> P1
  { FWD_TO_PORT, .src=&huart6, .dst=&huart1, .size=10 },
  // P6 -> P3
  { FWD_TO_PORT, .src=&huart6, .dst=&huart3, .size=10 }
};
const size_t module_tx_map_len = sizeof(module_tx_map)/sizeof(module_tx_map[0]);

static size_t map_index = 0;

// -----------------------------------------------------
// Start map execution
// -----------------------------------------------------
void MapTx_Start(void)
{
	map_index = 0;
	MapTx_Setup();
}

// -----------------------------------------------------
// Advance to next entry
// -----------------------------------------------------
void MapTx_Setup(void)
{
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
        // when Idle occurs, we’ll forward to e->dst
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    const map_entry_t *e = &module_tx_map[map_index];

    if (e->type == PORT_TO_MEM && e->src == huart) {
        // Already in e->mem
    }
    else if (e->type == FWD_TO_PORT && e->src == huart) {
        HAL_UART_Transmit_DMA(e->dst, staging_buf, Size);
    }

    // Move to next entry
    map_index++;
    MapTx_Setup();
}

