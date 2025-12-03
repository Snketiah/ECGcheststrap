/*
 * max30003.h
 *
 *  Created on: Nov 1, 2025
 *      Author: stevenketiah
 */

/**
  ******************************************************************************
  * @file    max30003.h
  * @brief   MAX30003 ECG AFE driver for STM32WB05
  ******************************************************************************
  */

#ifndef __MAX30003_H
#define __MAX30003_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t timeout;
} MAX30003_Handle_t;

typedef enum {
    MAX30003_OK = 0,
    MAX30003_ERROR,
    MAX30003_TIMEOUT,
    MAX30003_INVALID_PARAM
} MAX30003_Status_t;

typedef enum {
    MAX30003_GAIN_20V = 0x00,
    MAX30003_GAIN_40V = 0x01,
    MAX30003_GAIN_80V = 0x02,
    MAX30003_GAIN_160V = 0x03
} MAX30003_Gain_t;

typedef enum {
    MAX30003_RATE_512SPS = 0x00,
    MAX30003_RATE_256SPS = 0x01,
    MAX30003_RATE_128SPS = 0x02,
    MAX30003_RATE_500SPS = 0x00,
    MAX30003_RATE_250SPS = 0x01,
    MAX30003_RATE_125SPS = 0x02,
    MAX30003_RATE_200SPS = 0x02,
    MAX30003_RATE_199_8SPS = 0x02
} MAX30003_DataRate_t;

typedef enum {
    MAX30003_LEADBIAS_50M = 0x00,
    MAX30003_LEADBIAS_100M = 0x01,
    MAX30003_LEADBIAS_200M = 0x02
} MAX30003_LeadBias_t;

typedef struct {
    MAX30003_Gain_t gain;
    MAX30003_DataRate_t dataRate;
    uint8_t masterFreq;  // 0: 32768Hz, 1: 32000Hz, 2: 32000Hz, 3: 31968Hz
    bool enableECG;
    bool enableRtoR;
    bool enableLeadOff;
    bool enableLeadBias;
    uint8_t leadBiasValue;  // 0: 50MΩ, 1: 100MΩ, 2: 200MΩ
} MAX30003_Config_t;

typedef struct {
    uint32_t ecgSample;
    uint8_t etag;
    uint8_t ptag;
} MAX30003_ECG_Data_t;

/* Register Addresses --------------------------------------------------------*/
#define MAX30003_REG_NO_OP          0x00
#define MAX30003_REG_STATUS         0x01
#define MAX30003_REG_EN_INT         0x02
#define MAX30003_REG_EN_INT2        0x03
#define MAX30003_REG_MNGR_INT       0x04
#define MAX30003_REG_MNGR_DYN       0x05
#define MAX30003_REG_SW_RST         0x08
#define MAX30003_REG_SYNCH          0x09
#define MAX30003_REG_FIFO_RST       0x0A
#define MAX30003_REG_INFO           0x0F
#define MAX30003_REG_CNFG_GEN       0x10
#define MAX30003_REG_CNFG_CAL       0x12
#define MAX30003_REG_CNFG_EMUX      0x14
#define MAX30003_REG_CNFG_ECG       0x15
#define MAX30003_REG_CNFG_RTOR1     0x1D
#define MAX30003_REG_CNFG_RTOR2     0x1E
#define MAX30003_REG_ECG_FIFO_BURST 0x20
#define MAX30003_REG_ECG_FIFO       0x21
#define MAX30003_REG_RTOR           0x25

/* Status Register Bits ------------------------------------------------------*/
#define MAX30003_STATUS_EINT        (1 << 23)
#define MAX30003_STATUS_EOVF        (1 << 22)
#define MAX30003_STATUS_FSTINT      (1 << 21)
#define MAX30003_STATUS_DCLOFFINT   (1 << 20)
#define MAX30003_STATUS_LONINT      (1 << 11)
#define MAX30003_STATUS_RRINT       (1 << 10)
#define MAX30003_STATUS_SAMP        (1 << 9)
#define MAX30003_STATUS_PLLINT      (1 << 8)
#define MAX30003_STATUS_LDOFF_PH    (1 << 3)
#define MAX30003_STATUS_LDOFF_PL    (1 << 2)
#define MAX30003_STATUS_LDOFF_NH    (1 << 1)
#define MAX30003_STATUS_LDOFF_NL    (1 << 0)

/* ETAG Values ---------------------------------------------------------------*/
#define MAX30003_ETAG_VALID         0x00
#define MAX30003_ETAG_FAST_MODE     0x01
#define MAX30003_ETAG_VALID_EOF     0x02
#define MAX30003_ETAG_FAST_EOF      0x03
#define MAX30003_ETAG_EMPTY         0x06
#define MAX30003_ETAG_OVERFLOW      0x07

/* Exported functions --------------------------------------------------------*/
MAX30003_Status_t MAX30003_Init(MAX30003_Handle_t *handle, MAX30003_Config_t *config);
MAX30003_Status_t MAX30003_Reset(MAX30003_Handle_t *handle);
MAX30003_Status_t MAX30003_ReadStatus(MAX30003_Handle_t *handle, uint32_t *status);
MAX30003_Status_t MAX30003_ReadECGFIFO(MAX30003_Handle_t *handle, MAX30003_ECG_Data_t *data);
MAX30003_Status_t MAX30003_ReadECGFIFOBurst(MAX30003_Handle_t *handle, MAX30003_ECG_Data_t *data, uint8_t count);
MAX30003_Status_t MAX30003_ReadRtoR(MAX30003_Handle_t *handle, uint32_t *interval);
MAX30003_Status_t MAX30003_EnableInterrupts(MAX30003_Handle_t *handle, uint32_t intMask);
MAX30003_Status_t MAX30003_DisableInterrupts(MAX30003_Handle_t *handle, uint32_t intMask);
MAX30003_Status_t MAX30003_StartConversion(MAX30003_Handle_t *handle);
MAX30003_Status_t MAX30003_StopConversion(MAX30003_Handle_t *handle);
MAX30003_Status_t MAX30003_FIFOReset(MAX30003_Handle_t *handle);
MAX30003_Status_t MAX30003_Synch(MAX30003_Handle_t *handle);
MAX30003_Status_t MAX30003_ConfigureLeadOff(MAX30003_Handle_t *handle, bool enable, uint8_t current, uint8_t threshold);
MAX30003_Status_t MAX30003_ConfigureRtoR(MAX30003_Handle_t *handle, bool enable, uint8_t gain, uint8_t threshold);
MAX30003_Status_t MAX30003_ReadRegister(MAX30003_Handle_t *handle, uint8_t regAddr, uint32_t *data);
MAX30003_Status_t MAX30003_WriteRegister(MAX30003_Handle_t *handle, uint8_t regAddr, uint32_t data);
MAX30003_Status_t MAX30003_ConfigureLeadBias(MAX30003_Handle_t *handle, MAX30003_LeadBias_t bias);
MAX30003_Status_t MAX30003_EnableFastRecovery(MAX30003_Handle_t *handle, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30003_H */
