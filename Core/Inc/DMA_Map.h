/*
 *  DMA_Map.h
 *
 *  Created on: Aug 31, 2025
 *  Author: Adel Faki @ Hexabitz
 */

#ifndef DMA_MAP_H
#define DMA_MAP_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define NUM_OF_PORTS 6

#define UART_PORT1 &huart4
#define UART_PORT2 &huart2
#define UART_PORT3 &huart6
#define UART_PORT4 &huart1
#define UART_PORT5 &huart5
#define UART_PORT6 &huart3

enum Port_e {
	P1 = 0, P2, P3, P4, P5, P6
};

typedef enum {
	PORT_TO_MEM,     // UART -> memory
	FWD_TO_PORT,     // UART -> UART
	MEM_TO_PORT      // memory -> UART (Rx Phase)
} map_action_t;

typedef struct {
	map_action_t type;
	UART_HandleTypeDef *src;   // source UART (if peripheral)
	UART_HandleTypeDef *dst;   // dest UART (if peripheral)
	size_t size;               // number of bytes (M)
	uint8_t *mem;              // pointer to the staging memory buffer
} map_entry_t;

extern const map_entry_t module_tx_map[];
extern const size_t module_tx_map_len;

extern uint8_t port_mem_buf[100][NUM_OF_PORTS];
extern uint8_t staging_buf[100];

void MapTx_Start(void);
void MapTx_Setup(void);

#endif // DMA_MAP_H
