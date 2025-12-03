/*
 * usb_uart_driver.c
 *
 *  Created on: Nov 8, 2025
 *      Author: stevenketiah
 */


#include "usb_uart_driver.h"
#include <string.h>

/**
 * @brief Initialize the USB-UART driver with interrupt-based reception
 * @param handle Pointer to USB_UART_Handle structure
 * @param huart Pointer to UART_HandleTypeDef from HAL
 * @return USB_UART_Status
 */
USB_UART_Status USB_UART_Init(USB_UART_Handle* handle, UART_HandleTypeDef* huart) {
    if (handle == NULL || huart == NULL) {
        return USB_UART_ERROR;
    }

    // Initialize handle structure
    handle->huart = huart;
    handle->rx_index = 0;
    handle->message_ready = false;
    handle->rx_callback = NULL;

    // Clear receive buffer
    memset((void*)handle->rx_buffer, 0, UART_RX_BUFFER_SIZE);

    // Start interrupt-based reception
    if (HAL_UART_Receive_IT(handle->huart, (uint8_t*)&handle->rx_byte, 1) == HAL_OK) {
        return USB_UART_OK;
    } else {
        return USB_UART_ERROR;
    }
}

/**
 * @brief Deinitialize the USB-UART driver
 * @param handle Pointer to USB_UART_Handle structure
 * @return USB_UART_Status
 */
USB_UART_Status USB_UART_Deinit(USB_UART_Handle* handle) {
    if (handle == NULL) {
        return USB_UART_ERROR;
    }

    // Abort ongoing reception
    if (HAL_UART_AbortReceive_IT(handle->huart) == HAL_OK) {
        return USB_UART_OK;
    } else {
        return USB_UART_ERROR;
    }
}

/**
 * @brief Transmit data over UART (blocking mode)
 * @param handle Pointer to USB_UART_Handle structure
 * @param data Pointer to data buffer
 * @param size Number of bytes to transmit
 * @return USB_UART_Status
 */
USB_UART_Status USB_UART_Transmit(USB_UART_Handle* handle, uint8_t* data, uint16_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return USB_UART_ERROR;
    }

    HAL_StatusTypeDef hal_status = HAL_UART_Transmit(handle->huart, data, size, UART_TIMEOUT);

    switch (hal_status) {
        case HAL_OK:
            return USB_UART_OK;
        case HAL_BUSY:
            return USB_UART_BUSY;
        case HAL_TIMEOUT:
            return USB_UART_TIMEOUT;
        default:
            return USB_UART_ERROR;
    }
}

/**
 * @brief Transmit null-terminated string over UART
 * @param handle Pointer to USB_UART_Handle structure
 * @param str Pointer to null-terminated string
 * @return USB_UART_Status
 */
USB_UART_Status USB_UART_TransmitString(USB_UART_Handle* handle, const char* str) {
    if (handle == NULL || str == NULL) {
        return USB_UART_ERROR;
    }

    uint16_t length = strlen(str);
    return USB_UART_Transmit(handle, (uint8_t*)str, length);
}

/**
 * @brief Register a callback function for received messages
 * @param handle Pointer to USB_UART_Handle structure
 * @param callback Function pointer to callback (receives data pointer and length)
 */
void USB_UART_RegisterCallback(USB_UART_Handle* handle, void (*callback)(uint8_t*, uint32_t)) {
    if (handle != NULL) {
        handle->rx_callback = callback;
    }
}

/**
 * @brief Check if a complete message is ready
 * @param handle Pointer to USB_UART_Handle structure
 * @return true if message is ready, false otherwise
 */
bool USB_UART_IsMessageReady(USB_UART_Handle* handle) {
    if (handle == NULL) {
        return false;
    }
    return handle->message_ready;
}

/**
 * @brief Get the received message
 * @param handle Pointer to USB_UART_Handle structure
 * @param buffer Destination buffer
 * @param max_length Maximum buffer size
 * @return Number of bytes copied
 */
uint32_t USB_UART_GetMessage(USB_UART_Handle* handle, uint8_t* buffer, uint32_t max_length) {
    if (handle == NULL || buffer == NULL || !handle->message_ready) {
        return 0;
    }

    uint32_t length = strlen((const char*)handle->rx_buffer);
    if (length > max_length - 1) {
        length = max_length - 1;
    }

    memcpy(buffer, (const void*)handle->rx_buffer, length);
    buffer[length] = '\0';

    // Clear message ready flag
    handle->message_ready = false;

    return length;
}

/**
 * @brief UART interrupt handler - Call this from HAL_UART_RxCpltCallback
 * @param handle Pointer to USB_UART_Handle structure
 * @param huart Pointer to UART_HandleTypeDef that triggered the interrupt
 */
void USB_UART_IRQHandler(USB_UART_Handle* handle, UART_HandleTypeDef *huart) {
    if (handle == NULL || huart == NULL) {
        return;
    }

    // Check if this is our UART instance
    if (huart->Instance != handle->huart->Instance) {
        return;
    }

    // Check for newline character (end of message)
    if (handle->rx_byte == '\n') {
        // Null-terminate the string
        handle->rx_buffer[handle->rx_index] = '\0';

        // Set message ready flag
        handle->message_ready = true;

        // Call registered callback if available
        if (handle->rx_callback != NULL) {
            handle->rx_callback((uint8_t*)handle->rx_buffer, handle->rx_index);
        }

        // Reset index for next message
        handle->rx_index = 0;
    }
    else {
        // Store received byte if buffer not full
        if (handle->rx_index < UART_RX_BUFFER_SIZE - 1) {
            handle->rx_buffer[handle->rx_index++] = handle->rx_byte;
        }
        else {
            // Buffer overflow - reset and discard
            handle->rx_index = 0;
        }
    }

    // Re-enable interrupt to receive next byte
    HAL_UART_Receive_IT(handle->huart, (uint8_t*)&handle->rx_byte, 1);
}
