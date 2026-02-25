/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : Header for uart.c file.
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

#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <stdint.h>

/* Buffer Sizes */
#define UART1_RX_BUFFER_SIZE   128
#define UART2_RX_BUFFER_SIZE   128

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void UART_StartReception(void);


/* USART1 Send */
void UART1_Send(uint8_t *data, uint16_t len);
void UART2_Send(uint8_t *data, uint16_t len);

/* ================= CR95HF API ================= */

uint8_t CR95HF_Echo(void);
uint8_t CR95HF_SelectProtocol_ISO14443A(void);
uint8_t CR95HF_RF_On(void);
uint8_t CR95HF_REQA(void);
uint8_t CR95HF_Anticollision(uint8_t cascade_level, uint8_t *uid_part);
uint8_t CR95HF_Select(uint8_t cascade_level, uint8_t *uid_part);
uint8_t CR95HF_GetUID(void);

void CR95HF_SendCommand(uint8_t *cmd, uint16_t expected_rx_len, uint32_t timeout_ms);

uint8_t CR95HF_IsTransactionDone(void);


/* UID Output */
extern uint8_t cr95hf_full_uid[10];
extern uint8_t cr95hf_uid_length;



/* RX Flags */
uint8_t UART1_IsFrameReceived(void);
uint8_t UART2_IsFrameReceived(void);

/* RX Buffers */
uint8_t* UART1_GetBuffer(void);
uint16_t UART1_GetLength(void);

uint8_t* UART2_GetBuffer(void);
uint16_t UART2_GetLength(void);

#endif
