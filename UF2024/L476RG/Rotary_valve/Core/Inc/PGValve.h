#ifndef STM32_PGVALVE_H
#define STM32_PGVALVE_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define PGV_FRAME_STX 0xCC
#define PGV_FRAME_ETX 0xDD
#define PGV_FRAME_SIZE 8

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dePort;   // optional DE pin
    uint16_t dePin;
    uint8_t addr;
} PGValve_HandleTypeDef;

// basic functions
bool PGV_Init(PGValve_HandleTypeDef *hvalve, UART_HandleTypeDef *huart, uint8_t addr,
              GPIO_TypeDef *dePort, uint16_t dePin);
bool PGV_Reset(PGValve_HandleTypeDef *hvalve);
bool PGV_SwitchTo(PGValve_HandleTypeDef *hvalve, uint8_t port);
bool PGV_GetCurrentPosition(PGValve_HandleTypeDef *hvalve, uint8_t *port);

// internal helpers
static void PGV_SetDE(PGValve_HandleTypeDef *hvalve, bool tx);
static void PGV_ComputeChecksum(const uint8_t *frame6, uint8_t *ckl, uint8_t *ckh);
static uint8_t PGV_SendFrame(PGValve_HandleTypeDef *hvalve, uint8_t func, uint8_t plo,
                             uint8_t phi, uint8_t *resp, uint32_t timeout);

#endif
