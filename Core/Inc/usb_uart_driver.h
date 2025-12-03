/*
 * usb_uart_driver.h
 *
 *  Created on: Nov 8, 2025
 *      Author: stevenketiah
 */

#ifndef INC_USB_UART_DRIVER_H_
#define INC_USB_UART_DRIVER_H_

#include "stm32wb0x_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Configuration */
#define UART_RX_BUFFER_SIZE 256
#define UART_TIMEOUT 100

/* Status enumeration */
typedef enum {
    USB_UART_OK = 0,
    USB_UART_ERROR = 1,
    USB_UART_BUSY = 2,
    USB_UART_TIMEOUT = 3
} USB_UART_Status;

/* USB-UART Handle Structure */
typedef struct {
    UART_HandleTypeDef *huart;
    volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    volatile uint32_t rx_index;
    volatile uint8_t rx_byte;
    volatile bool message_ready;
    void (*rx_callback)(uint8_t* data, uint32_t length);
} USB_UART_Handle;

/* Function Prototypes */
USB_UART_Status USB_UART_Init(USB_UART_Handle* handle, UART_HandleTypeDef* huart);
USB_UART_Status USB_UART_Deinit(USB_UART_Handle* handle);
USB_UART_Status USB_UART_Transmit(USB_UART_Handle* handle, uint8_t* data, uint16_t size);
USB_UART_Status USB_UART_TransmitString(USB_UART_Handle* handle, const char* str);
void USB_UART_RegisterCallback(USB_UART_Handle* handle, void (*callback)(uint8_t*, uint32_t));
bool USB_UART_IsMessageReady(USB_UART_Handle* handle);
uint32_t USB_UART_GetMessage(USB_UART_Handle* handle, uint8_t* buffer, uint32_t max_length);
void USB_UART_IRQHandler(USB_UART_Handle* handle, UART_HandleTypeDef *huart);


#endif /* INC_USB_UART_DRIVER_H_ */
