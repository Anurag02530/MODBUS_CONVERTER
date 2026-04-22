#ifndef CR95HF_SIMPLE_H
#define CR95HF_SIMPLE_H

#include "main.h"

#define UART1_RX_BUFFER_SIZE   600
#define UART2_RX_BUFFER_SIZE   600

//typedef struct
//{
//    

//    // ?? Time fields (2 bytes each)
//    uint16_t second;
//    uint16_t minute;
//    uint16_t hour;
//    uint16_t date;
//    uint16_t month;
//    uint16_t year;
//		uint16_t base_unit;
//		double total_volume;
//		double forward_volume;
//    double reverse_volume;
//    float flow;
//		float temperature;
//		uint32_t serial_no;
//		uint16_t crc;
//    

//} MeterData_t;


   
		
    

    

// ================= GLOBALS =================
//extern UART_HandleTypeDef hlpuart3;  // CR95HF UART
//extern uint8_t rxBuf[32];            // Last received data
//extern uint8_t rxLen;                // Number of bytes expected
//extern uint8_t rxDone;               // RX complete flag
//extern uint8_t uid[10];              // UID storage
//extern uint8_t uidLen;               // UID length

// ================= FUNCTIONS =================
void CR95HF_UART_Init(void);   // USART1 for CR95HF
void DEBUG_UART_Init(void);		//USART2 for RS485/DEBUG
static void CR95HF_Send(uint8_t *cmd, uint8_t tx, uint8_t rx);  // Send command and start RX
void CR95HF_Process(void);     // Call repeatedly in main loop to handle state machine
void Process_Frame(uint8_t *rxBuffer, uint16_t rxLen);
void DBG_Print(const char *s);
void DBG_PrintHex(uint8_t *data, uint16_t len);
void Read_Meter(void);
void CR95HF_Process1(void);
void CR95HF_Read_Memory(void);
void DBG_PrintASCII(uint8_t *buf, uint16_t len);
void Decode_Data(void);
void build_frame(uint8_t *tx);

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

#endif // CR95HF_SIMPLE_H
