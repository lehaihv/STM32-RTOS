#include "PGValve.h"
#include "stdio.h"

static void PGV_SetDE(PGValve_HandleTypeDef *hvalve, bool tx) {
    if (hvalve->dePort && hvalve->dePin != 0xFFFF) {
        HAL_GPIO_WritePin(hvalve->dePort, hvalve->dePin, tx ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (tx) HAL_Delay(1); // allow RS-485 DE settling
    }
}

static void PGV_ComputeChecksum(const uint8_t *frame6, uint8_t *ckl, uint8_t *ckh) {
    uint16_t sum = 0;
    for (int i = 0; i < 6; ++i) sum += frame6[i];
    *ckl = sum & 0xFF;
    *ckh = (sum >> 8) & 0xFF;
}

static uint8_t PGV_SendFrame(PGValve_HandleTypeDef *hvalve, uint8_t func, uint8_t plo,
                             uint8_t phi, uint8_t *resp, uint32_t timeoutMs) {
    uint8_t frame6[6] = { PGV_FRAME_STX, hvalve->addr, func, plo, phi, PGV_FRAME_ETX };
    uint8_t ckl, ckh;
    PGV_ComputeChecksum(frame6, &ckl, &ckh);

    uint8_t frame[PGV_FRAME_SIZE] = { frame6[0], frame6[1], frame6[2], frame6[3],
                                      frame6[4], frame6[5], ckl, ckh };

    // flush UART
    uint8_t tmp;
    while (HAL_UART_Receive(hvalve->huart, &tmp, 1, 1) == HAL_OK) {}

    // send frame
    PGV_SetDE(hvalve, true);
    if (HAL_UART_Transmit(hvalve->huart, frame, PGV_FRAME_SIZE, 1000) != HAL_OK) {
        PGV_SetDE(hvalve, false);
        return 1;
    }
    PGV_SetDE(hvalve, false);

    if (!resp) return 0; // no response requested

    // wait for STX
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeoutMs) {
        if (HAL_UART_Receive(hvalve->huart, &tmp, 1, 10) == HAL_OK) {
            if (tmp == PGV_FRAME_STX) {
                resp[0] = tmp;
                for (int i = 1; i < PGV_FRAME_SIZE; i++) {
                    if (HAL_UART_Receive(hvalve->huart, &resp[i], 1, timeoutMs) != HAL_OK)
                        return 4; // timeout
                }
                // validate
                uint8_t rckl, rckh;
                PGV_ComputeChecksum(resp, &rckl, &rckh);
                if (resp[6] != rckl || resp[7] != rckh) return 1;
                if (resp[5] != PGV_FRAME_ETX) return 1;
                return 0; // success
            }
        }
    }
    return 4; // timeout
}

// ---------------- Public API ----------------

bool PGV_Init(PGValve_HandleTypeDef *hvalve, UART_HandleTypeDef *huart, uint8_t addr,
              GPIO_TypeDef *dePort, uint16_t dePin) {
    hvalve->huart = huart;
    hvalve->addr = addr;
    hvalve->dePort = dePort;
    hvalve->dePin = dePin;
    if (dePort && dePin != 0xFFFF) HAL_GPIO_WritePin(dePort, dePin, GPIO_PIN_RESET);
    return true;
}

bool PGV_Reset(PGValve_HandleTypeDef *hvalve) {
    uint8_t resp[PGV_FRAME_SIZE];
    return PGV_SendFrame(hvalve, 0x45, 0x00, 0x00, resp, 5000) == 0;
}

bool PGV_SwitchTo(PGValve_HandleTypeDef *hvalve, uint8_t port) {
    uint8_t resp[PGV_FRAME_SIZE];
    return PGV_SendFrame(hvalve, 0x44, port, 0x00, resp, 1000) == 0;
}

bool PGV_GetCurrentPosition(PGValve_HandleTypeDef *hvalve, uint8_t *port) {
    uint8_t resp[PGV_FRAME_SIZE];
    if (PGV_SendFrame(hvalve, 0x3E, 0x00, 0x00, resp, 500) != 0) return false;
    *port = resp[3];
    return true;
}

bool PGV_MoveToPortAndWait(PGValve_HandleTypeDef *hvalve, uint8_t port, uint32_t timeoutMs) {
    if (!PGV_SwitchTo(hvalve, port)) return false;

    uint32_t start = HAL_GetTick();
    uint8_t cur;
    while (HAL_GetTick() - start < timeoutMs) {
        if (PGV_GetCurrentPosition(hvalve, &cur)) {
            if (cur == port) return true;
        }
        HAL_Delay(100);
    }
    return false;
}
