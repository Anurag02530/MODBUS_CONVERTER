#ifndef CR95HF_SIMPLE_H
#define CR95HF_SIMPLE_H

#include "main.h"

#define UART1_RX_BUFFER_SIZE   128

// ================= GLOBALS =================
//extern UART_HandleTypeDef hlpuart3;  // CR95HF UART
//extern uint8_t rxBuf[32];            // Last received data
//extern uint8_t rxLen;                // Number of bytes expected
//extern uint8_t rxDone;               // RX complete flag
//extern uint8_t uid[10];              // UID storage
//extern uint8_t uidLen;               // UID length

// ================= FUNCTIONS =================
void CR95HF_UART_Init(void);   // Initialize LPUART3 for CR95HF
static void CR95HF_Send(uint8_t *cmd, uint8_t tx, uint8_t rx);  // Send command and start RX
void CR95HF_Process(void);     // Call repeatedly in main loop to handle state machine

#endif // CR95HF_SIMPLE_H
