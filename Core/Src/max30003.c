/*
 * max30003.c
 *
 *  Created on: Nov 1, 2025
 *      Author: stevenketiah
 */


/**
  ******************************************************************************
  * @file    max30003.c
  * @brief   MAX30003 ECG AFE driver implementation for STM32WB05
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "max30003.h"

/* Private defines -----------------------------------------------------------*/
#define MAX30003_SPI_TIMEOUT    1000
#define MAX30003_WRITE_BIT      0x00
#define MAX30003_READ_BIT       0x01

/* Private function prototypes -----------------------------------------------*/
static void MAX30003_CS_Low(MAX30003_Handle_t *handle);
static void MAX30003_CS_High(MAX30003_Handle_t *handle);
static MAX30003_Status_t MAX30003_SPITransmitReceive(MAX30003_Handle_t *handle,
                                                      uint8_t *txData,
                                                      uint8_t *rxData,
                                                      uint16_t size);
static void MAX30003_Delay(uint32_t ms);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Pull CS pin low
  */
static void MAX30003_CS_Low(MAX30003_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Pull CS pin high
  */
static void MAX30003_CS_High(MAX30003_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}

/**
  * @brief  SPI transmit and receive
  */
static MAX30003_Status_t MAX30003_SPITransmitReceive(MAX30003_Handle_t *handle,
                                                      uint8_t *txData,
                                                      uint8_t *rxData,
                                                      uint16_t size)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_SPI_TransmitReceive(handle->hspi, txData, rxData, size, handle->timeout);

    if (hal_status == HAL_OK) {
        return MAX30003_OK;
    } else if (hal_status == HAL_TIMEOUT) {
        return MAX30003_TIMEOUT;
    } else {
        return MAX30003_ERROR;
    }
}

/**
  * @brief  Delay in milliseconds
  */
static void MAX30003_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Read a register from MAX30003
  */
MAX30003_Status_t MAX30003_ReadRegister(MAX30003_Handle_t *handle, uint8_t regAddr, uint32_t *data)
{
    uint8_t txData[4] = {0};
    uint8_t rxData[4] = {0};
    MAX30003_Status_t status;

    // Prepare command byte (address + read bit)
    txData[0] = (regAddr << 1) | MAX30003_READ_BIT;

    // Select the device
    MAX30003_CS_Low(handle);

    // Send command and receive data
    status = MAX30003_SPITransmitReceive(handle, txData, rxData, 4);

    // Deselect the device
    MAX30003_CS_High(handle);

    if (status == MAX30003_OK) {
        // Combine received bytes into 24-bit data (ignore first byte which is command echo)
        *data = ((uint32_t)rxData[1] << 16) | ((uint32_t)rxData[2] << 8) | rxData[3];
    }

    return status;
}

/**
  * @brief  Write a register to MAX30003
  */
MAX30003_Status_t MAX30003_WriteRegister(MAX30003_Handle_t *handle, uint8_t regAddr, uint32_t data)
{
    uint8_t txData[4];
    uint8_t rxData[4] = {0};
    MAX30003_Status_t status;

    // Prepare command byte (address + write bit)
    txData[0] = (regAddr << 1) | MAX30003_WRITE_BIT;

    // Prepare data bytes (24-bit)
    txData[1] = (data >> 16) & 0xFF;
    txData[2] = (data >> 8) & 0xFF;
    txData[3] = data & 0xFF;

    // Select the device
    MAX30003_CS_Low(handle);

    // Send command and data
    status = MAX30003_SPITransmitReceive(handle, txData, rxData, 4);

    // Deselect the device
    MAX30003_CS_High(handle);

    return status;
}

/**
  * @brief  Software reset the MAX30003
  */
MAX30003_Status_t MAX30003_Reset(MAX30003_Handle_t *handle)
{
    MAX30003_Status_t status;

    // Write 0x000000 to SW_RST register
    status = MAX30003_WriteRegister(handle, MAX30003_REG_SW_RST, 0x000000);

    if (status == MAX30003_OK) {
        // Wait for reset to complete
        MAX30003_Delay(10);
    }

    return status;
}

/**
  * @brief  Initialize the MAX30003
  */
MAX30003_Status_t MAX30003_Init(MAX30003_Handle_t *handle, MAX30003_Config_t *config)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Set default timeout if not specified
    if (handle->timeout == 0) {
        handle->timeout = MAX30003_SPI_TIMEOUT;
    }

    // Software reset
    status = MAX30003_Reset(handle);
    if (status != MAX30003_OK) return status;

    // Wait for device to be ready
    MAX30003_Delay(50);

    // Read INFO register to verify communication
    status = MAX30003_ReadRegister(handle, MAX30003_REG_INFO, &regVal);
    if (status != MAX30003_OK) return status;

    // Configure general settings (CNFG_GEN register)
    regVal = 0;
    regVal |= ((uint32_t)config->masterFreq & 0x03) << 20;  // FMSTR[1:0]
    regVal |= (config->enableECG ? 1 : 0) << 19;           // EN_ECG
    regVal |= (config->enableLeadOff ? 1 : 0) << 12;       // EN_DCLOFF[1:0]
    regVal |= (config->enableLeadBias ? 1 : 0) << 4;       // EN_RBIAS[1:0]
    regVal |= (config->leadBiasValue & 0x03) << 2;         // RBIASV[1:0]
    regVal |= 0x03;  // Enable bias on both ECGP and ECGN

    status = MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_GEN, regVal);
    if (status != MAX30003_OK) return status;

    // Configure ECG channel (CNFG_ECG register)
    regVal = 0;
    regVal |= ((uint32_t)config->dataRate & 0x03) << 22;    // RATE[1:0]
    regVal |= ((uint32_t)config->gain & 0x03) << 16;        // GAIN[1:0]
    regVal |= (1 << 14);  // DHPF = 1 (0.5Hz HPF)
    regVal |= (1 << 12);  // DLPF[1:0] = 01 (40Hz LPF)

    status = MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_ECG, regVal);
    if (status != MAX30003_OK) return status;

    // Configure input MUX (CNFG_EMUX register)
    // Close input switches to connect electrodes
    regVal = 0x000000;  // OPENP=0, OPENN=0 (switches closed)
    status = MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_EMUX, regVal);
    if (status != MAX30003_OK) return status;

    // Configure R-to-R detection if enabled (CNFG_RTOR1 register)
    if (config->enableRtoR) {
		// Set default R-to-R parameters in CNFG_RTOR1, with Auto-Gain
		regVal = (0x0B << 20) | // WNDW = 11 (96ms)
				 (0x0F << 16) | // GAIN = Auto-Scale
				 (1 << 15)    | // EN_RTOR = enabled
				 (0x02 << 12) | // PAVG = 8
				 (0x03 << 8);   // PTSF = 4/16
		status = MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_RTOR1, regVal);
		if (status != MAX30003_OK) return status;

		// Configure CNFG_RTOR2 with default settings for timing
		// HOFF = 32 (~256ms), RAVG = 8, RHSF = 4/8
		regVal = (0x20 << 16) | (0x02 << 12) | (0x04 << 8);
		status = MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_RTOR2, regVal);
		if (status != MAX30003_OK) return status;
    }


    // Configure interrupt management (MNGR_INT register)
    regVal = 0;
    regVal |= (0x0F << 19);  // EFIT[4:0] = 15 (16 samples threshold)
    regVal |= (0x01 << 4);   // CLR_RRINT[1:0] = 01 (clear on RTOR read)
    regVal |= (0x01 << 2);   // CLR_SAMP = 1 (self-clear)
    status = MAX30003_WriteRegister(handle, MAX30003_REG_MNGR_INT, regVal);
    if (status != MAX30003_OK) return status;

    // Configure dynamic management (MNGR_DYN register)
    regVal = 0x3F0000;  // Fast recovery disabled, threshold at max
    status = MAX30003_WriteRegister(handle, MAX30003_REG_MNGR_DYN, regVal);
    if (status != MAX30003_OK) return status;

    return MAX30003_OK;
}

/**
  * @brief  Start ECG conversion
  */
MAX30003_Status_t MAX30003_StartConversion(MAX30003_Handle_t *handle)
{
    // Write to SYNCH register to start synchronized conversion
    return MAX30003_WriteRegister(handle, MAX30003_REG_SYNCH, 0x000000);
}

/**
  * @brief  Stop ECG conversion
  */
MAX30003_Status_t MAX30003_StopConversion(MAX30003_Handle_t *handle)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read CNFG_GEN register
    status = MAX30003_ReadRegister(handle, MAX30003_REG_CNFG_GEN, &regVal);
    if (status != MAX30003_OK) return status;

    // Clear EN_ECG bit
    regVal &= ~(1 << 19);

    // Write back to CNFG_GEN register
    return MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_GEN, regVal);
}

/**
  * @brief  Read status register
  */
MAX30003_Status_t MAX30003_ReadStatus(MAX30003_Handle_t *handle, uint32_t *status)
{
    return MAX30003_ReadRegister(handle, MAX30003_REG_STATUS, status);
}

/**
  * @brief  Read single ECG sample from FIFO
  */
MAX30003_Status_t MAX30003_ReadECGFIFO(MAX30003_Handle_t *handle, MAX30003_ECG_Data_t *data)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read ECG FIFO register
    status = MAX30003_ReadRegister(handle, MAX30003_REG_ECG_FIFO, &regVal);
    if (status != MAX30003_OK) return status;

    // Extract ECG data (18-bit signed)
    data->ecgSample = (regVal >> 6) & 0x3FFFF;

    // Sign extend if negative
    if (data->ecgSample & 0x20000) {
        data->ecgSample |= 0xFFFC0000;
    }

    // Extract tags
    data->etag = (regVal >> 3) & 0x07;
    data->ptag = regVal & 0x07;

    return MAX30003_OK;
}

/**
  * @brief  Read multiple ECG samples from FIFO in burst mode
  */
MAX30003_Status_t MAX30003_ReadECGFIFOBurst(MAX30003_Handle_t *handle, MAX30003_ECG_Data_t *data, uint8_t count)
{
    MAX30003_Status_t status;
    uint32_t sample;

    if (count == 0) {
        return MAX30003_INVALID_PARAM;
    }

    // Calculate total transaction size: 4 bytes for first sample + 3 bytes for each additional
    uint16_t totalSize = 4 + ((count - 1) * 3);
    uint8_t txData[totalSize];
    uint8_t rxData[totalSize];

    // Clear buffers
    for (int i = 0; i < totalSize; i++) {
        txData[i] = 0;
        rxData[i] = 0;
    }

    // Prepare burst read command (first byte only)
    txData[0] = (MAX30003_REG_ECG_FIFO_BURST << 1) | MAX30003_READ_BIT;

    // Select the device - keep CS low for entire burst
    MAX30003_CS_Low(handle);

    // Perform single continuous SPI transaction for burst read
    // First 4 bytes: command + 3 dummy bytes for first sample
    // Subsequent 3-byte groups for additional samples
    status = MAX30003_SPITransmitReceive(handle, txData, rxData, totalSize);

    // Deselect the device
    MAX30003_CS_High(handle);

    if (status == MAX30003_OK) {
        // Parse first sample from 4-byte response
        sample = ((uint32_t)rxData[1] << 16) |
                ((uint32_t)rxData[2] << 8) |
                rxData[3];

        // Extract ECG data (18-bit signed)
        data[0].ecgSample = (sample >> 6) & 0x3FFFF;

        // Sign extend if negative
        if (data[0].ecgSample & 0x20000) {
            data[0].ecgSample |= 0xFFFC0000;
        }

        // Extract tags
        data[0].etag = (sample >> 3) & 0x07;
        data[0].ptag = sample & 0x07;

        // Parse subsequent samples (3 bytes each)
        for (int i = 1; i < count; i++) {
            int offset = 4 + ((i - 1) * 3);

            // Combine 3 bytes into 24-bit sample
            sample = ((uint32_t)rxData[offset] << 16) |
                    ((uint32_t)rxData[offset + 1] << 8) |
                    rxData[offset + 2];

            // Extract ECG data (18-bit signed)
            data[i].ecgSample = (sample >> 6) & 0x3FFFF;

            // Sign extend if negative
            if (data[i].ecgSample & 0x20000) {
                data[i].ecgSample |= 0xFFFC0000;
            }

            // Extract tags
            data[i].etag = (sample >> 3) & 0x07;
            data[i].ptag = sample & 0x07;
        }
    }

    return status;
}

/**
  * @brief  Read R-to-R interval
  */
MAX30003_Status_t MAX30003_ReadRtoR(MAX30003_Handle_t *handle, uint32_t *interval)
{
	MAX30003_Status_t status;
	uint32_t statusReg;
	uint32_t regVal;

	// First check if RRINT status bit is asserted
	status = MAX30003_ReadRegister(handle, MAX30003_REG_STATUS, &statusReg);
	if (status != MAX30003_OK) return status;

	// Check if R-to-R interrupt is asserted (bit 10)
	if (!(statusReg & MAX30003_STATUS_RRINT)) {
		// No valid R-to-R interval available
		return MAX30003_ERROR;
	}

	// Read RTOR register
	status = MAX30003_ReadRegister(handle, MAX30003_REG_RTOR, &regVal);
	if (status != MAX30003_OK) return status;

	// Extract 14-bit R-to-R interval (left justified)
	*interval = (regVal >> 10) & 0x3FFF;

	return MAX30003_OK;
}

/**
  * @brief  Reset FIFO
  */
MAX30003_Status_t MAX30003_FIFOReset(MAX30003_Handle_t *handle)
{
    return MAX30003_WriteRegister(handle, MAX30003_REG_FIFO_RST, 0x000000);
}

/**
  * @brief  Synchronize device
  */
MAX30003_Status_t MAX30003_Synch(MAX30003_Handle_t *handle)
{
    return MAX30003_WriteRegister(handle, MAX30003_REG_SYNCH, 0x000000);
}

/**
  * @brief  Enable interrupts
  */
MAX30003_Status_t MAX30003_EnableInterrupts(MAX30003_Handle_t *handle, uint32_t intMask)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read current interrupt configuration
    status = MAX30003_ReadRegister(handle, MAX30003_REG_EN_INT, &regVal);
    if (status != MAX30003_OK) return status;

    // Set interrupt enable bits
    regVal |= intMask;

    // Set INTB type to open-drain with pullup
    regVal |= 0x03;  // INTB_TYPE[1:0] = 11

    // Write back to register
    return MAX30003_WriteRegister(handle, MAX30003_REG_EN_INT, regVal);
}

/**
  * @brief  Disable interrupts
  */
MAX30003_Status_t MAX30003_DisableInterrupts(MAX30003_Handle_t *handle, uint32_t intMask)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read current interrupt configuration
    status = MAX30003_ReadRegister(handle, MAX30003_REG_EN_INT, &regVal);
    if (status != MAX30003_OK) return status;

    // Clear interrupt enable bits
    regVal &= ~intMask;

    // Write back to register
    return MAX30003_WriteRegister(handle, MAX30003_REG_EN_INT, regVal);
}

/**
  * @brief  Configure lead-off detection
  */
MAX30003_Status_t MAX30003_ConfigureLeadOff(MAX30003_Handle_t *handle, bool enable,
                                            uint8_t current, uint8_t threshold)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read CNFG_GEN register
    status = MAX30003_ReadRegister(handle, MAX30003_REG_CNFG_GEN, &regVal);
    if (status != MAX30003_OK) return status;

    // Clear all lead-off related bits: EN_DCLOFF[13:12], IPOL[11], IMAG[10:8], VTH[7:6]
    regVal &= ~((0x03 << 12) | (0x01 << 11) | (0x07 << 8) | (0x03 << 6));

    // Set lead-off configuration
    if (enable) {
        regVal |= (0x01 << 12);              // EN_DCLOFF[1:0] = 01 (enable)
        regVal |= (0x00 << 11);              // DCLOFF_IPOL = 0 (default polarity)
        regVal |= ((current & 0x07) << 8);   // DCLOFF_IMAG[2:0]
        regVal |= ((threshold & 0x03) << 6); // DCLOFF_VTH[1:0]
    }

    // Write back to register
    return MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_GEN, regVal);
}

/**
  * @brief  Configure R-to-R detection
  */
MAX30003_Status_t MAX30003_ConfigureRtoR(MAX30003_Handle_t *handle, bool enable,
                                         uint8_t gain, uint8_t threshold)
{
    uint32_t regVal;

    // Configure CNFG_RTOR1 register
    regVal = 0;
    regVal |= (0x03 << 20);  // WNDW[3:0] = 3 (default window)
    regVal |= ((gain & 0x0F) << 16);  // GAIN[3:0]
    regVal |= (enable ? 1 : 0) << 15;  // EN_RTOR
    regVal |= (0x02 << 12);  // PAVG[1:0] = 2
    regVal |= ((threshold & 0x0F) << 8);  // PTSF[3:0]

    return MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_RTOR1, regVal);
}

/**
  * @brief  Configure the resistive lead bias value.
  * @note   This function assumes lead bias is already enabled in the init config.
  */
MAX30003_Status_t MAX30003_ConfigureLeadBias(MAX30003_Handle_t *handle, MAX30003_LeadBias_t bias)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read the current CNFG_GEN register
    status = MAX30003_ReadRegister(handle, MAX30003_REG_CNFG_GEN, &regVal);
    if (status != MAX30003_OK) {
        return status;
    }

    // Clear the RBIASV bits [3:2]
    regVal &= ~(0x03 << 2);

    // Set the new bias value
    regVal |= ((uint32_t)bias & 0x03) << 2;

    // Write the modified value back to the register
    return MAX30003_WriteRegister(handle, MAX30003_REG_CNFG_GEN, regVal);
}

/**
  * @brief  Enable or disable the Automatic Fast Recovery feature.
  */
MAX30003_Status_t MAX30003_EnableFastRecovery(MAX30003_Handle_t *handle, bool enable)
{
    MAX30003_Status_t status;
    uint32_t regVal;

    // Read the current MNGR_DYN register
    status = MAX30003_ReadRegister(handle, MAX30003_REG_MNGR_DYN, &regVal);
    if (status != MAX30003_OK) {
        return status;
    }

    // Clear the FAST_MODE bits [23:22]
    regVal &= ~(0x03 << 22);

    if (enable) {
        // Set FAST[1:0] to 0b10 for Automatic Fast Recovery Mode
        regVal |= (0x02 << 22);
    }

    // Write the modified value back to the register
    return MAX30003_WriteRegister(handle, MAX30003_REG_MNGR_DYN, regVal);
}
