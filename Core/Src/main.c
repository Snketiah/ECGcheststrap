/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ble.h"
#include "max30003.h"
#include "usb_uart_driver.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

PKA_HandleTypeDef hpka;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Global handles */
MAX30003_Handle_t max30003;
MAX30003_Config_t max30003_config;
USB_UART_Handle usb_uart;

/* Teleplot buffer */
char teleplot_buffer[128];

/* Statistics */
uint32_t sample_count = 0;
uint32_t error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RADIO_Init(void);
static void MX_RADIO_TIMER_Init(void);
static void MX_PKA_Init(void);
/* USER CODE BEGIN PFP */

void Send_To_Teleplot_Int(const char *variable, int32_t value) {
  int len = snprintf(teleplot_buffer, sizeof(teleplot_buffer), ">%s:%ld\n",
                     variable, (long)value);
  USB_UART_Transmit(&usb_uart, (uint8_t *)teleplot_buffer, len);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  MAX30003_Status_t status;
  MAX30003_ECG_Data_t ecg_data;
  uint32_t device_status;
  uint32_t rtor_interval;
  int32_t heart_rate_bpm; // Heart rate as integer
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_RADIO_Init();
  MX_RADIO_TIMER_Init();
  MX_PKA_Init();
  /* USER CODE BEGIN 2 */

  /* Send startup message */
  USB_UART_Init(&usb_uart, &huart1);

  HAL_Delay(100);

  USB_UART_TransmitString(&usb_uart, "Starting MAX30003 ECG Stream...\n");

  /* Configure MAX30003 handle */
  max30003.hspi = &hspi3;
  max30003.cs_port = MAX30003_CS_GPIO_Port;
  max30003.cs_pin = MAX30003_CS_Pin; // Adjust to your CS pin
  max30003.timeout = 1000;

  /* Configure MAX30003 settings */
  max30003_config.gain = MAX30003_GAIN_80V;        // 20V/V gain
  max30003_config.dataRate = MAX30003_RATE_128SPS; // 128 samples/sec
  max30003_config.masterFreq = 0;                  // 32768Hz
  max30003_config.enableECG = true;
  max30003_config.enableRtoR = true;
  max30003_config.enableLeadOff = true;
  max30003_config.enableLeadBias = true;
  max30003_config.leadBiasValue = (uint8_t)MAX30003_LEADBIAS_200M; // 100MΩ

  /* Initialize MAX30003 */
  USB_UART_TransmitString(&usb_uart, "Initializing MAX30003...\n");
  status = MAX30003_Init(&max30003, &max30003_config);
  if (status != MAX30003_OK) {
    USB_UART_TransmitString(&usb_uart, "ERROR: MAX30003 init failed!\n");
    Error_Handler();
  }
  USB_UART_TransmitString(&usb_uart, "MAX30003 initialized successfully\n");

  // Configure Lead-Off Detection
  USB_UART_TransmitString(&usb_uart, "Configuring Lead-Off Detection...\n");
  uint8_t lead_off_current = 0x02;   // 10nA
  uint8_t lead_off_threshold = 0x01; // +/- 400mV
  status = MAX30003_ConfigureLeadOff(&max30003, true, lead_off_current,
                                     lead_off_threshold);
  if (status != MAX30003_OK) {
    USB_UART_TransmitString(&usb_uart, "WARNING: Lead-Off config failed\n");
  }

  // Enable Automatic Fast Recovery Mode
  USB_UART_TransmitString(&usb_uart, "Enabling Fast Recovery Mode...\n");
  status = MAX30003_EnableFastRecovery(&max30003, true);
  if (status != MAX30003_OK) {
    USB_UART_TransmitString(&usb_uart,
                            "WARNING: Fast Recovery config failed\n");
  }

  /* Enable interrupts for ECG FIFO and R-to-R */
  MAX30003_EnableInterrupts(&max30003,
                            MAX30003_STATUS_EINT |          // ECG FIFO
                                MAX30003_STATUS_RRINT |     // R-to-R
                                MAX30003_STATUS_DCLOFFINT); // Lead-off

  /* Start ECG conversion */
  status = MAX30003_StartConversion(&max30003);
  if (status != MAX30003_OK) {
    USB_UART_TransmitString(&usb_uart, "ERROR: Failed to start conversion!\n");
    Error_Handler();
  }

  /* Configure Teleplot colors */
  USB_UART_TransmitString(&usb_uart, ">ecg|g\n");        // ECG in green
  USB_UART_TransmitString(&usb_uart, ">heart_rate|r\n"); // Heart rate in red
  USB_UART_TransmitString(&usb_uart, ">lead_off|y\n");   // Lead-off in yellow

  USB_UART_TransmitString(&usb_uart, "Starting ECG stream...\n");

  /* USER CODE END 2 */

  /* Init code for STM32_BLE */
  MX_APPE_Init(NULL);

  /* USER CODE BEGIN 3 */
  /* Start BLE advertising to make device discoverable */
  APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
  USB_UART_TransmitString(&usb_uart,
                          "BLE Advertising started - Device discoverable as "
                          "'Ryder Heart Rate Monitor'\n");
  /* USER CODE END 3 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* Read device status */
    status = MAX30003_ReadStatus(&max30003, &device_status);

    if (status == MAX30003_OK) {

      /* Check for ECG FIFO data available */
      if (device_status & MAX30003_STATUS_EINT) {

        /* Read ECG sample from FIFO */
        status = MAX30003_ReadECGFIFO(&max30003, &ecg_data);

        if (status == MAX30003_OK) {
          /* Process based on ETAG */
          switch (ecg_data.etag) {
          case MAX30003_ETAG_VALID:
          case MAX30003_ETAG_VALID_EOF:
            /* Valid ECG sample - send raw ADC value directly */
            /* This is the most efficient - just send the raw 18-bit signed
             * value */
            Send_To_Teleplot_Int("ecg", ecg_data.ecgSample);

            sample_count++;
            break;

          case MAX30003_ETAG_FAST_MODE:
          case MAX30003_ETAG_FAST_EOF:
            /* Fast recovery mode - skip sample */
            break;

          case MAX30003_ETAG_OVERFLOW:
            /* FIFO overflow - reset FIFO */
            USB_UART_TransmitString(&usb_uart, "FIFO overflow! Resetting...\n");
            MAX30003_FIFOReset(&max30003);
            error_count++;
            break;

          case MAX30003_ETAG_EMPTY:
            /* FIFO empty - normal condition */
            break;
          }
        }
      }

      /* Check for R-to-R interval detection */
      if (device_status & MAX30003_STATUS_RRINT) {

        /* Read R-to-R interval (already validates RRINT in our driver) */
        status = MAX30003_ReadRtoR(&max30003, &rtor_interval);

        if (status == MAX30003_OK && rtor_interval > 0) {
          /* Calculate heart rate as integer */
          /* HR = 60000 / (interval * 8) */
          /* Using integer math: HR = 7500 / interval */
          heart_rate_bpm = 7500 / rtor_interval;

          /* Sanity check heart rate (30-200 bpm) */
          if (heart_rate_bpm >= 30 && heart_rate_bpm <= 200) {
            Send_To_Teleplot_Int("heart_rate", heart_rate_bpm);
          }
        }
      }

      /* Check for lead-off detection */
      if (device_status & MAX30003_STATUS_DCLOFFINT) {
        /* Lead-off detected */
        Send_To_Teleplot_Int("lead_off", 1);

        /* Check which electrode */
        if (device_status & MAX30003_STATUS_LDOFF_PH) {
          USB_UART_TransmitString(&usb_uart, "Lead-off: ECGP high\n");
        }
        if (device_status & MAX30003_STATUS_LDOFF_PL) {
          USB_UART_TransmitString(&usb_uart, "Lead-off: ECGP low\n");
        }
        if (device_status & MAX30003_STATUS_LDOFF_NH) {
          USB_UART_TransmitString(&usb_uart, "Lead-off: ECGN high\n");
        }
        if (device_status & MAX30003_STATUS_LDOFF_NL) {
          USB_UART_TransmitString(&usb_uart, "Lead-off: ECGN low\n");
        }
      } else {
        /* Electrodes connected properly */
        static uint32_t last_leadoff_update = 0;
        if (HAL_GetTick() - last_leadoff_update > 1000) {
          Send_To_Teleplot_Int("lead_off", 0);
          last_leadoff_update = HAL_GetTick();
        }
      }

      /* Print statistics every 5 seconds */
      static uint32_t last_stats_time = 0;
      if (HAL_GetTick() - last_stats_time > 5000) {
        char stats_buffer[100];
        snprintf(stats_buffer, sizeof(stats_buffer),
                 "Stats: %lu samples, %lu errors\n", sample_count, error_count);
        USB_UART_TransmitString(&usb_uart, stats_buffer);
        last_stats_time = HAL_GetTick();
      }
    }

    /* Small delay - at 128 sps, new sample every ~7.8ms */
    /* Check more frequently to avoid FIFO overflow */
    HAL_Delay(5);

    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE_BYPASS;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSEBYPASSState = RCC_LSE_BYPASS_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the SYSCLKSource and SYSCLKDivider
   */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief PKA Initialization Function
 * @param None
 * @retval None
 */
static void MX_PKA_Init(void) {

  /* USER CODE BEGIN PKA_Init 0 */

  /* USER CODE END PKA_Init 0 */

  /* USER CODE BEGIN PKA_Init 1 */

  /* USER CODE END PKA_Init 1 */
  hpka.Instance = PKA;
  if (HAL_PKA_Init(&hpka) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN PKA_Init 2 */

  /* USER CODE END PKA_Init 2 */
}

/**
 * @brief RADIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_RADIO_Init(void) {

  /* USER CODE BEGIN RADIO_Init 0 */

  /* USER CODE END RADIO_Init 0 */

  RADIO_HandleTypeDef hradio = {0};

  /* USER CODE BEGIN RADIO_Init 1 */

  /* USER CODE END RADIO_Init 1 */

  if (__HAL_RCC_RADIO_IS_CLK_DISABLED()) {
    /* Radio Peripheral reset */
    __HAL_RCC_RADIO_FORCE_RESET();
    __HAL_RCC_RADIO_RELEASE_RESET();

    /* Enable Radio peripheral clock */
    __HAL_RCC_RADIO_CLK_ENABLE();
  }
  hradio.Instance = RADIO;
  HAL_RADIO_Init(&hradio);
  /* USER CODE BEGIN RADIO_Init 2 */

  /* USER CODE END RADIO_Init 2 */
}

/**
 * @brief RADIO_TIMER Initialization Function
 * @param None
 * @retval None
 */
static void MX_RADIO_TIMER_Init(void) {

  /* USER CODE BEGIN RADIO_TIMER_Init 0 */

  /* USER CODE END RADIO_TIMER_Init 0 */

  RADIO_TIMER_InitTypeDef RADIO_TIMER_InitStruct = {0};

  /* USER CODE BEGIN RADIO_TIMER_Init 1 */

  /* USER CODE END RADIO_TIMER_Init 1 */

  if (__HAL_RCC_RADIO_IS_CLK_DISABLED()) {
    /* Radio Peripheral reset */
    __HAL_RCC_RADIO_FORCE_RESET();
    __HAL_RCC_RADIO_RELEASE_RESET();

    /* Enable Radio peripheral clock */
    __HAL_RCC_RADIO_CLK_ENABLE();
  }
  /* Wait to be sure that the Radio Timer is active */
  while (LL_RADIO_TIMER_GetAbsoluteTime(WAKEUP) < 0x10)
    ;
  RADIO_TIMER_InitStruct.XTAL_StartupTime = 320;
  RADIO_TIMER_InitStruct.enableInitialCalibration = FALSE;
  RADIO_TIMER_InitStruct.periodicCalibrationInterval = 0;
  HAL_RADIO_TIMER_Init(&RADIO_TIMER_InitStruct);
  /* USER CODE BEGIN RADIO_TIMER_Init 2 */

  /* USER CODE END RADIO_TIMER_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MAX30003_CS_Pin | LED_SWITCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAX30003_CS_Pin LED_SWITCH_Pin */
  GPIO_InitStruct.Pin = MAX30003_CS_Pin | LED_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SWDIO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin | INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_BTN_Pin CHARGE_Pin */
  GPIO_InitStruct.Pin = BLE_BTN_Pin | CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_MCO;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_2 | PWR_GPIO_BIT_14 |
                                              PWR_GPIO_BIT_4);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_2 | PWR_GPIO_BIT_14 |
                                                PWR_GPIO_BIT_4);

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2);

  /*RT DEBUG GPIO_Init */
  RT_DEBUG_GPIO_Init();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
