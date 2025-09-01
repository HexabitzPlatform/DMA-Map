#ifndef DMA_MAP_H
#define DMA_MAP_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stddef.h>

// Data source/destination enums
typedef enum {
    SRC_MEMORY,
    SRC_PERIPHERAL
} dma_src_t;

typedef enum {
    DST_MEMORY,
    DST_PERIPHERAL
} dma_dst_t;

// Generic DMA setup function
void SetupDMA(dma_src_t srcType, void *src,
              dma_dst_t dstType, void *dst,
              size_t size);

// Example buffers
extern uint8_t uart_rx_buf[1024];
extern uint8_t uart_tx_buf[1024];

#endif // DMA_MAP_H
