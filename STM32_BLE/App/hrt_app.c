/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    HRT_app.c
 * @author  MCD Application Team
 * @brief   HRT_app application definition.
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
#include "hrt_app.h"
#include "app_ble.h"
#include "app_common.h"
#include "ble.h"
#include "hrt.h"
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum {
  Hrm_NOTIFICATION_OFF,
  Hrm_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */

  /* USER CODE END Service1_APP_SendInformation_t */
  HRT_APP_SENDINFORMATION_LAST
} HRT_APP_SendInformation_t;

typedef struct {
  HRT_APP_SendInformation_t Hrm_Notification_Status;
  /* USER CODE BEGIN Service1_APP_Context_t */

  /* USER CODE END Service1_APP_Context_t */
  uint16_t ConnectionHandle;
} HRT_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static HRT_APP_Context_t HRT_APP_Context;

uint8_t a_HRT_UpdateCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void HRT_Hrm_SendNotification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void HRT_Notification(HRT_NotificationEvt_t *p_Notification) {
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch (p_Notification->EvtOpcode) {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

  case HRT_HRM_NOTIFY_ENABLED_EVT:
    /* USER CODE BEGIN Service1Char1_NOTIFY_ENABLED_EVT */

    /* USER CODE END Service1Char1_NOTIFY_ENABLED_EVT */
    break;

  case HRT_HRM_NOTIFY_DISABLED_EVT:
    /* USER CODE BEGIN Service1Char1_NOTIFY_DISABLED_EVT */

    /* USER CODE END Service1Char1_NOTIFY_DISABLED_EVT */
    break;

  default:
    /* USER CODE BEGIN Service1_Notification_default */

    /* USER CODE END Service1_Notification_default */
    break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void HRT_APP_EvtRx(HRT_APP_ConnHandleNotEvt_t *p_Notification) {
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch (p_Notification->EvtOpcode) {
  /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

  /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
  case HRT_CONN_HANDLE_EVT:
    HRT_APP_Context.ConnectionHandle = p_Notification->ConnectionHandle;
    /* USER CODE BEGIN Service1_APP_CENTR_CONN_HANDLE_EVT */

    /* USER CODE END Service1_APP_CENTR_CONN_HANDLE_EVT */
    break;
  case HRT_DISCON_HANDLE_EVT:
    HRT_APP_Context.ConnectionHandle = 0xFFFF;
    /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */

    /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
    break;

  default:
    /* USER CODE BEGIN Service1_APP_EvtRx_default */

    /* USER CODE END Service1_APP_EvtRx_default */
    break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

void HRT_APP_Init(void) {
  HRT_APP_Context.ConnectionHandle = 0xFFFF;
  HRT_Init();

  /* USER CODE BEGIN Service1_APP_Init */

  /* USER CODE END Service1_APP_Init */
  return;
}

void HRT_APP_Process(void) {
  /* USER CODE BEGIN HRT_APP_Process_1 */

  /* USER CODE END HRT_APP_Process_1 */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void HRT_Hrm_SendNotification(void) /* Property Notification */
{
  HRT_APP_SendInformation_t notification_on_off = Hrm_NOTIFICATION_OFF;
  HRT_Data_t hrt_notification_data;

  hrt_notification_data.p_Payload = (uint8_t *)a_HRT_UpdateCharData;
  hrt_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char1_NS_1*/

  /* USER CODE END Service1Char1_NS_1*/

  if (notification_on_off != Hrm_NOTIFICATION_OFF &&
      HRT_APP_Context.ConnectionHandle != 0xFFFF) {
    HRT_NotifyValue(HRT_HRM, &hrt_notification_data,
                    HRT_APP_Context.ConnectionHandle);
  }

  /* USER CODE BEGIN Service1Char1_NS_Last*/

  /* USER CODE END Service1Char1_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
