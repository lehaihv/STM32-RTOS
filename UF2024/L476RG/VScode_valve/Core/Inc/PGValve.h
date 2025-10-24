#ifndef PGVALVE_H
#define PGVALVE_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define PGV_FRAME_STX 0xCC
#define PGV_FRAME_ETX 0xDD
#define PGV_FRAME_SIZE 8

typedef struct {
    UART_HandleTypeDef *huart;  // RS-485 UART
    GPIO_TypeDef *dePort;       // DE pin, optional (set NULL if using auto-DE)
    uint16_t dePin;
    uint8_t addr;
} PGValve_HandleTypeDef;

// Public functions
bool PGV_Init(PGValve_HandleTypeDef *hvalve, UART_HandleTypeDef *huart,
              uint8_t addr, GPIO_TypeDef *dePort, uint16_t dePin);
bool PGV_Reset(PGValve_HandleTypeDef *hvalve);
bool PGV_SwitchTo(PGValve_HandleTypeDef *hvalve, uint8_t port);
bool PGV_GetCurrentPosition(PGValve_HandleTypeDef *hvalve, uint8_t *port);

// Optional helper: move and wait until port reached
bool PGV_MoveToPortAndWait(PGValve_HandleTypeDef *hvalve, uint8_t port, uint32_t timeoutMs);

#endif
