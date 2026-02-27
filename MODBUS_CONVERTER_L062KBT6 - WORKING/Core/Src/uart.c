/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : 
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "uart.h"
#include <string.h>

/* External UART Handles */
extern UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private Variables */
static uint8_t uart1_rx_byte;
static uint8_t uart2_rx_byte;


/* ================= CR95HF BUFFERS ================= */
uint8_t cr95hf_tx_buf[32];
uint8_t cr95hf_rx_buf[32];

uint8_t cr95hf_full_uid[10];
uint8_t cr95hf_uid_length = 0;




/* USART2 */
static uint8_t uart2_rx_buffer[UART2_RX_BUFFER_SIZE];
static uint16_t uart2_rx_index = 0;
static uint16_t uart2_expected_len = 0;
static uint8_t uart2_frame_ready = 0;

/* ================= TRANSACTION CONTROL ================= */

static uint8_t  cr95hf_transaction_done = 0;
static uint32_t cr95hf_timeout_ms = 0;
static uint32_t cr95hf_start_tick = 0;
static uint16_t cr95hf_expected_len = 0;


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}


/* ================= UART START ================= */
void UART_StartReception(void)
{
    const char msg1[] = "UART1 WELCOME\r\n";
    const char msg2[] = "UART2 WELCOME\r\n";

    /* Send welcome */
    //HAL_UART_Transmit(&huart1, (uint8_t*)msg1, sizeof(msg1)-1, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2)-1, 100);

    /* Start interrupt reception */
   // HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}



/* ================= USART1 SEND ================= */
void UART1_Send(uint8_t *data, uint16_t len)
{
//    uart1_expected_len = len;
//    uart1_rx_index = 0;
//    uart1_frame_ready = 0;

//    HAL_UART_Transmit(&huart1, data, len, 100);
}


/* ================= RX CALLBACK ================= */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    /* ================= USART1 ================= */
//    if (huart->Instance == USART1)
//    {
//        uart1_rx_buffer[uart1_rx_index++] = uart1_rx_byte;

//        if (uart1_expected_len > 0 &&
//            uart1_rx_index >= uart1_expected_len)
//        {
//            uart1_frame_ready = 1;
//            uart1_rx_index = 0;
//        }

//        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
//    }

//    /* ================= USART2 (Modbus Ready) ================= */
//    if (huart->Instance == USART2)
//    {
//        uart2_rx_buffer[uart2_rx_index++] = uart2_rx_byte;

//        /* Decide expected length after 2nd byte */
//        if (uart2_rx_index == 2)
//        {
//            if (uart2_rx_buffer[1] == 0x03)
//                uart2_expected_len = 8;  // Modbus Read Holding Registers
//            else
//                uart2_expected_len = 2;
//        }

//        if (uart2_expected_len > 0 &&
//            uart2_rx_index >= uart2_expected_len)
//        {
//            uart2_frame_ready = 1;
//            uart2_rx_index = 0;
//            uart2_expected_len = 0;
//        }

//        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
//    }
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    /* ================= USART1 ================= */
//    if (huart->Instance == USART1)
//    {
//        /* Echo back received byte */
//        HAL_UART_Transmit(&huart1, &uart1_rx_byte, 1, 100);

//        /* Store safely (optional) */
//        if (uart1_rx_index < UART1_RX_BUFFER_SIZE)
//        {
//            uart1_rx_buffer[uart1_rx_index++] = uart1_rx_byte;
//        }

//        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
//    }

//    /* ================= USART2 ================= */
//    else if (huart->Instance == USART2)
//    {
//        /* Echo back received byte */
//        HAL_UART_Transmit(&huart2, &uart2_rx_byte, 1, 100);

//        /* Store safely (optional) */
//        if (uart2_rx_index < UART2_RX_BUFFER_SIZE)
//        {
//            uart2_rx_buffer[uart2_rx_index++] = uart2_rx_byte;
//        }

//        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
//    }
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    /* ================= USART1 (CR95HF) ================= */
//    if (huart->Instance == USART1)
//    {
//			rxDone = 1;
//        if (uart1_rx_index < UART1_RX_BUFFER_SIZE)
//        {
//            uart1_rx_buffer[uart1_rx_index++] = uart1_rx_byte;
//        }

//        /* ---- PRINT RECEIVED BYTE IN HEX ON USART2 ---- */
//        char dbg[6];
//        sprintf(dbg, "%02X ", uart1_rx_byte);
//        HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), 100);

//        /* If expected length reached ? transaction done */
//        if (cr95hf_expected_len > 0 &&
//            uart1_rx_index >= cr95hf_expected_len)
//        {
//            cr95hf_transaction_done = 1;

//            /* New line after full frame */
//            char nl[] = "\r\n";
//            HAL_UART_Transmit(&huart2, (uint8_t*)nl, sizeof(nl)-1, 100);
//        }

//        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
//    }

//    /* ================= USART2 ================= */
//    else if (huart->Instance == USART2)
//    {
//        if (uart2_rx_index < UART2_RX_BUFFER_SIZE)
//        {
//            uart2_rx_buffer[uart2_rx_index++] = uart2_rx_byte;
//        }

//        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
//    }
//}



uint8_t CR95HF_Send(uint8_t *data, uint8_t len)
{
//    CR95HF_SendCommand(data, len, 200);   // 200ms timeout

//    while(!CR95HF_IsTransactionDone())
//		{
//				// allow background processing
//		}


//    if(uart1_rx_index >= 2)
//        return 1;

//    return 0;
}

//void CR95HF_SendCommand(uint8_t *cmd,
//                        uint16_t expected_rx_len,
//                        uint32_t timeout_ms)
//{
//    uart1_rx_index = 0;
//    cr95hf_transaction_done = 0;

//    cr95hf_expected_len = expected_rx_len;
//    cr95hf_timeout_ms   = timeout_ms;
//    cr95hf_start_tick   = HAL_GetTick();

//    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), 100);
//}

uint8_t CR95HF_IsTransactionDone(void)
{
    /* Timeout */
    if((HAL_GetTick() - cr95hf_start_tick) > cr95hf_timeout_ms)
    {
        return 1;
    }

    return cr95hf_transaction_done;
}




//uint8_t CR95HF_Echo(void)
//{
//    uint8_t retry = 50;
//    uint8_t echo_cmd[] = {0x55};
//	
//	const char msg2[] = "CR95HF RX:-\r\n";
//	const char msg3[] = "\r\n";
//	char dbg[6];

//    /* Send welcome */
//    //HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2)-1, 100);

//    while(retry--)
//    {
//       // HAL_UART_Transmit(&huart1, &echo_cmd, 1, 100);
//			CR95HF_Send(echo_cmd, 1);

////        if(HAL_UART_Receive(&huart1, uart1_rx_buffer, 1, 100) == HAL_OK)			
////        {
//					sprintf(dbg, "%02X ", uart1_rx_byte);
//					HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2)-1, 100);
//					HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), 100);
//					HAL_UART_Transmit(&huart2, (uint8_t*)msg3, sizeof(msg3)-1, 100);
//					if(uart1_rx_buffer[0] == 0x55)
//							return 1;
////			}
//    }
//    return 0;
//}

uint8_t CR95HF_SelectProtocol_ISO14443A(void)
{
    uint8_t cmd[] = {0x02,0x05,0x02,0x00,0x10,0x00};
    return CR95HF_Send(cmd, 2);
}

//uint8_t CR95HF_RF_On(void)
//{
//    uint8_t cmd[] = {0x09,0x01,0x01};
//    return CR95HF_Send(cmd, sizeof(cmd));
//}

uint8_t CR95HF_REQA(void)
{
    uint8_t cmd[] = {0x04,0x02,0x26,0x07};
    return CR95HF_Send(cmd, 7);
}

uint8_t CR95HF_Anticollision(uint8_t cascade_level, uint8_t *uid_part)
{
//    uint8_t cmd[5];

//    cmd[0] = 0x04;
//    cmd[1] = 0x03;

//    cmd[2] = (cascade_level == 1) ? 0x93 : 0x95;

//    cmd[3] = 0x20;
//    cmd[4] = 0x08;

//    if(!CR95HF_Send(cmd,10))
//        return 0;

//    memcpy(uid_part, &uart1_rx_buffer[2], 6);
//    return 1;
}

uint8_t CR95HF_Select(uint8_t cascade_level, uint8_t *uid_part)
{
//    uint8_t bcc = uid_part[0] ^ uid_part[1] ^
//                  uid_part[2] ^ uid_part[3];

//    cr95hf_tx_buf[0] = 0x04;
//    cr95hf_tx_buf[1] = 0x09;
//    cr95hf_tx_buf[2] = (cascade_level == 1) ? 0x93 : 0x95;
//    cr95hf_tx_buf[3] = 0x70;

//    memcpy(&cr95hf_tx_buf[4], uid_part, 6);

//    //cr95hf_tx_buf[8] = bcc;
//   // cr95hf_tx_buf[9] = 0x28;

//    if(!CR95HF_Send(cr95hf_tx_buf,8))
//        return 0;

//    return uart1_rx_buffer[2];   // SAK
}

uint8_t CR95HF_GetUID(void)
{
    uint8_t retry = 10;

    while(retry--)
    {
        if(!CR95HF_REQA())
            continue;

        uint8_t uid_cl1[4];
        if(!CR95HF_Anticollision(1, uid_cl1))
            continue;

        uint8_t sak = CR95HF_Select(1, uid_cl1);

        if(sak & 0x04)
        {
            uint8_t uid_cl2[4];
            if(!CR95HF_Anticollision(2, uid_cl2))
                continue;

            sak = CR95HF_Select(2, uid_cl2);

            cr95hf_full_uid[0] = uid_cl1[1];
            cr95hf_full_uid[1] = uid_cl1[2];
            cr95hf_full_uid[2] = uid_cl1[3];

            memcpy(&cr95hf_full_uid[3], uid_cl2, 4);

            cr95hf_uid_length = 7;
            return 1;
        }
        else
        {
            memcpy(cr95hf_full_uid, uid_cl1, 4);
            cr95hf_uid_length = 4;
            return 1;
        }
    }

    return 0;
}



/* ================= GETTERS ================= */
//uint8_t UART1_IsFrameReceived(void)
//{
//    return uart1_frame_ready;
//}

//uint8_t* UART1_GetBuffer(void)
//{
//    uart1_frame_ready = 0;
//    return uart1_rx_buffer;
//}

uint16_t UART1_GetLength(void)
{
    //return uart1_expected_len;
}


uint8_t UART2_IsFrameReceived(void)
{
    return uart2_frame_ready;
}

uint8_t* UART2_GetBuffer(void)
{
    uart2_frame_ready = 0;
    return uart2_rx_buffer;
}

uint16_t UART2_GetLength(void)
{
    return uart2_expected_len == 0 ? 8 : uart2_expected_len;
}


