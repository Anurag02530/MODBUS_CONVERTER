/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cr95hf.c
  * @brief          : cr95hf program body
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
/* Includes ------------------------------------------------------------------*/


#include <string.h>
#include "cr95hf.h"
#include "stdint.h"
#include <stdio.h>

//static MeterData_t meter;
uint16_t second;
uint16_t minute;
uint16_t hour;
uint16_t date;
uint16_t month;
uint16_t year;
uint16_t base_unit;
double total_volume;
double forward_volume;
double reverse_volume;
float flow;
float temperature;
uint32_t serial_no;
uint8_t modbus_address;
uint16_t crc;



uint8_t tx[105]; // full frame
int A5_5A_Err=0, A5_5A_Err_Count=0;

#define FLASH_ADDR   0x08007C00   // last page (adjust if needed)


// Store Meter ID and CRC Init persistently
static uint32_t saved_meter_id = 0;
static uint16_t saved_crc_init = 0;

#define COMBINATION_1 0
#define COMBINATION_2 1
#define COMBINATION_3 2

uint8_t FE_frame[9] = { 0x04, 0x07, 0xA2, 0xFE, 0x00, 0x00, 0x2C, 0x17, 0x28 };
uint8_t FF_frame[9] = { 0x04, 0x07, 0xA2, 0xFF, 0xA5, 0x5A, 0x01, 0x03, 0x28 };

uint8_t targetSeq[14] = { 0x67, 0x6F, 0x4C, 0x65, 0x6D, 0x75, 0x6C, 0x6F, 0x56, 0x79, 0x6C, 0x69, 0x61, 0x44 };

uint8_t combination_state = COMBINATION_3;  // start with combination 1

UART_HandleTypeDef huart1;  // CR95HF
UART_HandleTypeDef huart2;  // Debug

#define TOTAL_MEMORY_SIZE 512

uint8_t full_memory[TOTAL_MEMORY_SIZE];
uint16_t mem_offset = 0;

/* ================= GLOBALS ================= */

//static uint8_t uart1_rx_buffer[128];
static volatile uint8_t rxDone = 0;
static volatile uint32_t rxTick = 0;
static uint8_t rxLen = 0;

static uint8_t uid[10];
static uint8_t uidLen = 0;

static uint8_t state = 0;

int XX = 3, Read_Done = 0;
int retry = 10;

/* USART1 */
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
uint16_t uart1_rx_index = 0;
uint8_t uart2_rx_buffer[UART2_RX_BUFFER_SIZE];
uint16_t uart2_rx_index = 0;
uint16_t uart1_expected_len = 0;
uint8_t uart1_frame_ready = 0;
uint8_t uart2_frame_ready = 0;

static uint8_t rxByte;  // single byte buffer for IT
uint8_t Debug_rxByte;

void handle_modbus_write(uint8_t *rx)
{
    uint8_t slave = rx[0];
    uint8_t func  = rx[1];

    DBG_Print("Func: ");
    DBG_PrintHex(&func, 1);
    DBG_Print("\r\n");

    if (slave != modbus_address && slave != 0x00)
    {
        DBG_Print("Wrong Slave Address\r\n");
        return;
    }

    // CRC check
    uint16_t rx_crc = (rx[6] << 8) | rx[7];
    uint16_t calc_crc = modbus_crc(rx, 6);
		DBG_Print("Expected CRC: ");
    char msg[30];
		sprintf(msg, "CRC calc: 0x%04X\r\n", calc_crc);
		DBG_Print(msg);

    if (rx_crc != calc_crc)
    {
        DBG_Print("CRC Error\r\n");
        return;
    }

    if (func == 0x06)
    {
        DBG_Print("Function 06\r\n");

        uint16_t reg = (rx[2] << 8) | rx[3];
        uint16_t val = (rx[4] << 8) | rx[5];

        if (reg == 0x0001)
        {
            DBG_Print("Register OK\r\n");

            if (val >= 1 && val <= 247)
            {
                DBG_Print("Writing new address\r\n");

                write_modbus_address((uint8_t)val);
                modbus_address = (uint8_t)val;

                // echo response
                uint16_t crc = modbus_crc(rx, 6);
                rx[6] = crc & 0xFF;
                rx[7] = (crc >> 8);

                HAL_UART_Transmit(&huart1, rx, 8, 100);

                DBG_Print("New Addr Set: ");
                DBG_PrintHex(&modbus_address, 1);
                DBG_Print("\r\n");
            }
        }
    }
}


void write_modbus_address(uint8_t addr)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_ADDR;
    erase.NbPages = 1;

    HAL_FLASHEx_Erase(&erase, &page_error);

    // write full 32-bit word (important!)
    uint32_t data = addr;

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR, data);

    HAL_FLASH_Lock();
}

uint8_t read_modbus_address(void)
{
    uint8_t addr = *(uint8_t*)FLASH_ADDR;

    if (addr == 0xFF || addr == 0x00 || addr > 247)
        return 1;   // default

    return addr;
}

/* ================= CRC FUNCTIONS ================= */

// ====================== CRC-16/CCITT Function ======================
// Same algorithm as online calculator (poly 0x1021, non-reflected, no final XOR)
uint16_t crc16_ccitt(const uint8_t *data, uint8_t len, uint16_t init_val)
{
    uint16_t crc = init_val;
    const uint16_t poly = 0x1021;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }

		FE_frame[7] = (uint8_t)(crc & 0xFF);        // Low byte
    FE_frame[6] = (uint8_t)((crc>> 8) & 0xFF); // High byte
//		DBG_Print("CRC: ");
//    DBG_PrintHex(FE_frame, 8);
    return crc;
}



void reverse_buffer(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len / 2; i++)
    {
        uint8_t temp = data[i];
        data[i] = data[len - 1 - i];
        data[len - 1 - i] = temp;
    }
}


// ================= CRC =================
uint16_t modbus_crc(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++) {  // chnaged here int to uint16_t
        crc ^= buf[pos];

        for (int i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void build_frame(uint8_t *tx)
{
    uint16_t idx = 0;
    float f;
	
		uint8_t addr = read_modbus_address();

		if (addr == 0xFF || addr == 0x00) {
				addr = 1;  // default Modbus ID
		}

    // ---- 16-bit (BIG ENDIAN) ----
    #define PUT_U16(v)  do { tx[idx++] = (v)>>8; tx[idx++] = (v)&0xFF; } while(0)

    // ---- 32-bit (BIG ENDIAN, NO SWAP) ----
    #define PUT_U32(v) do {                      \
        uint32_t val = *(uint32_t*)&(v);        \
        tx[idx++] = (val >> 24) & 0xFF;         \
        tx[idx++] = (val >> 16) & 0xFF;         \
        tx[idx++] = (val >> 8)  & 0xFF;         \
        tx[idx++] = (val >> 0)  & 0xFF;         \
    } while(0)

    // ---- TIME ----
    PUT_U16(second);
    PUT_U16(minute);
    PUT_U16(hour);
    PUT_U16(date);
    PUT_U16(month);
    PUT_U16(year);
    PUT_U16(base_unit);

    // ---- DOUBLE ? FLOAT ----
    f = (float)total_volume;
    PUT_U32(f);

    f = (float)forward_volume;
    PUT_U32(f);

    f = (float)reverse_volume;
    PUT_U32(f);

    // ---- FLOAT ----
    PUT_U32(flow);
    PUT_U32(temperature);

    // ---- SERIAL ----
    PUT_U32(serial_no);

    // ---- DUMMY ----
    tx[idx++] = 0x00;
    tx[idx++] = 0x00;

    // ---- PAD TO 100 BYTES ----
    while (idx < 100)
        tx[idx++] = 0x00;

    // ---- CRC ----
    uint16_t crc = modbus_crc(tx, 100);

    tx[idx++] = crc & 0xFF;        // LSB
    tx[idx++] = (crc >> 8) & 0xFF; // MSB

    DBG_Print("\r\nTX string to modbus: ");
    DBG_PrintHex(tx, idx);
}

//// ============== MAIN PACKER ==============
//void build_frame(uint8_t *tx)
//{
//    idx = 0;
//    // ===== PACK AS-IS (NO CHANGE IN VALUES) =====

//    memcpy(&tx[idx], &second, 2); idx += 2;
//    memcpy(&tx[idx], &minute, 2); idx += 2;
//    memcpy(&tx[idx], &hour,   2); idx += 2;
//    memcpy(&tx[idx], &date,   2); idx += 2;
//    memcpy(&tx[idx], &month,  2); idx += 2;
//    memcpy(&tx[idx], &year,   2); idx += 2;

//    memcpy(&tx[idx], &base_unit, 2); idx += 2;

//    memcpy(&tx[idx], &total_volume, 4); idx += 4;
//    memcpy(&tx[idx], &forward_volume, 4); idx += 4;
//    memcpy(&tx[idx], &reverse_volume, 4); idx += 4;

//    memcpy(&tx[idx], &flow, 4); idx += 4;
//    memcpy(&tx[idx], &temperature, 4); idx += 4;

//    memcpy(&tx[idx], &serial_no, 4); idx += 4;

//    // ===== ADD DUMMY AFTER SERIAL =====
//    tx[idx++] = 0x00;
//    tx[idx++] = 0x00;

//    // ===== PAD TILL 100 BYTES =====
//    while (idx < 100) {
//        tx[idx++] = 0x00;
//    }

//    // ===== CRC =====
//    uint16_t crc = modbus_crc(tx, 100);

//    tx[idx++] = crc & 0xFF;        // LSB
//    tx[idx++] = (crc >> 8) & 0xFF; // MSB
//		DBG_Print("\r\nTX string to modbus: ");
//		DBG_PrintHex(tx, idx);
//		
//}

/* ================= FAST DEBUG PRINT ================= */

void Decode_Data(void)
{	
//	int idx = 0;
//	uint8_t tx[105]; // full frame
	int frame_recieved=0;
	reverse_buffer(uart1_rx_buffer, uart1_rx_index);
	
	DBG_Print("\nReversed data PAGE READ F0-FF RX: ");
  DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
	char buf[100];
	if (uart1_rx_index < 40) // basic safety check
	{
			DBG_Print("Invalid data length\r\n");
			return;
	}



	// ---- Extract values (based on your data order) ----
	if(uart1_rx_buffer[6] == 0x01){
		frame_recieved=1;
		memcpy(&serial_no, &uart1_rx_buffer[14], 4);
//		serial_no_data = meter.serial_no;
//		
		sprintf(buf, "\r\nRaw Serial Bytes: %02X %02X %02X %02X \r\n",
        uart1_rx_buffer[17],
        uart1_rx_buffer[16],
        uart1_rx_buffer[15],
        uart1_rx_buffer[14]);
		DBG_Print(buf);
	}
	
	if(uart1_rx_buffer[6] == 0x2C){
		frame_recieved=1;
			// ?? Time fields (1 byte ? automatically 0x00ss)
		second = (uint16_t)uart1_rx_buffer[12];
		minute = (uint16_t)uart1_rx_buffer[11];
		hour   = (uint16_t)uart1_rx_buffer[10];
		date   = (uint16_t)uart1_rx_buffer[9];
		month  = (uint16_t)uart1_rx_buffer[8];
		year   = (uint16_t)uart1_rx_buffer[7];
		
//		sprintf(buf, "\r\nRaw Time Bytes: %02X %02X %02X %02X %02X %02X\r\n",
//        uart1_rx_buffer[7],
//        uart1_rx_buffer[8],
//        uart1_rx_buffer[9],
//        uart1_rx_buffer[10],
//        uart1_rx_buffer[11],
//        uart1_rx_buffer[12]);
//		DBG_Print(buf);
	}

	if(uart1_rx_buffer[6] == 0x29){
		frame_recieved=1;
		// Volumes & float values
		memcpy(&temperature,     &uart1_rx_buffer[42], 4);
		memcpy(&flow,            &uart1_rx_buffer[38], 4);
		memcpy(&reverse_volume,  &uart1_rx_buffer[30], 8);
		memcpy(&forward_volume,  &uart1_rx_buffer[22], 8);
		memcpy(&total_volume,    &uart1_rx_buffer[14], 8);

		base_unit = (uint16_t)uart1_rx_buffer[13];



//		// ---- Print values ----
//		DBG_Print("------ UART DATA ------\r\n");
//		
//		// ---- Print Serial Number ----
////		sprintf(buf, "Serial No: %08X \r\n",
////        meter.serial_no);
////		DBG_Print(buf);
//		// Serial
////		sprintf(buf, "Serial No: %02X %02X %02X %02X\r\n",
////						((uint8_t*)&meter.serial_no)[3],
////						((uint8_t*)&meter.serial_no)[2],
////						((uint8_t*)&meter.serial_no)[1],
////						((uint8_t*)&meter.serial_no)[0]);
////		DBG_Print(buf);
////		
////		uint8_t *p = (uint8_t*)&meter.serial_no;

////		sprintf(buf, "Serial No: %02X %02X %02X %02X\r\n",
////						p[3], p[2], p[1], p[0]);   // MSB first
////		DBG_Print(buf);
//		
//		sprintf(buf, "Serial No	: %08lX\r\n", serial_no);
//		DBG_Print(buf);
//		
//		// ---- Print Time ----
//		sprintf(buf, "Time       : %02d:%02d:%02d\r\n",
//						hour, minute, second);
//		DBG_Print(buf);

//		// ---- Print Date ----
//		sprintf(buf, "Date       : %02d-%02d-20%02d\r\n",
//						date, month, year);
//		DBG_Print(buf);

//		// ---- Print Base Unit ----
//		sprintf(buf, "Base Unit  : %d\r\n", base_unit);
//		DBG_Print(buf);

//		// ---- Print Temperature ----
//		sprintf(buf, "Temperature: %.2f\r\n", temperature);
//		DBG_Print(buf);

//		// ---- Print Flow ----
//		sprintf(buf, "Flow       : %.2f\r\n", flow);
//		DBG_Print(buf);
//		

//		// ---- Print Volumes ----
//		forward_volume= (float)forward_volume;
//		sprintf(buf, "Forward Vol: %.6lf\r\n", forward_volume);
//		DBG_Print(buf);
//		
//		total_volume= (float)total_volume;
//		sprintf(buf, "Total Vol  : %.6lf\r\n", total_volume);
//		DBG_Print(buf);
//		
//		reverse_volume= (float)reverse_volume;
//		sprintf(buf, "Reverse Vol: %.6lf\r\n", reverse_volume);
//		DBG_Print(buf);

//		DBG_Print("-----------------------\r\n");
		}
	
	if(frame_recieved==1){
		// ?? Base unit
		



		// ---- Print values ----
		DBG_Print("------ UART DATA ------\r\n");
		
		// ---- Print Serial Number ----
//		sprintf(buf, "Serial No: %08X \r\n",
//        meter.serial_no);
//		DBG_Print(buf);
		// Serial
//		sprintf(buf, "Serial No: %02X %02X %02X %02X\r\n",
//						((uint8_t*)&meter.serial_no)[3],
//						((uint8_t*)&meter.serial_no)[2],
//						((uint8_t*)&meter.serial_no)[1],
//						((uint8_t*)&meter.serial_no)[0]);
//		DBG_Print(buf);
//		
//		uint8_t *p = (uint8_t*)&meter.serial_no;

//		sprintf(buf, "Serial No: %02X %02X %02X %02X\r\n",
//						p[3], p[2], p[1], p[0]);   // MSB first
//		DBG_Print(buf);
		
		sprintf(buf, "Serial No  : %08lX\r\n", serial_no);
		DBG_Print(buf);
		
		// ---- Print Time ----
		sprintf(buf, "Time       : %02d:%02d:%02d\r\n",
						hour, minute, second);
		DBG_Print(buf);

		// ---- Print Date ----
		sprintf(buf, "Date       : %02d-%02d-20%02d\r\n",
						date, month, year);
		DBG_Print(buf);

		// ---- Print Base Unit ----
		sprintf(buf, "Base Unit  : %d\r\n", base_unit);
		DBG_Print(buf);

		// ---- Print Temperature ----
		sprintf(buf, "Temperature: %.2f\r\n", temperature);
		DBG_Print(buf);

		// ---- Print Flow ----
		sprintf(buf, "Flow       : %.2f\r\n", flow);
		DBG_Print(buf);
		

		// ---- Print Volumes ----
		forward_volume= (float)forward_volume;
		sprintf(buf, "Forward Vol: %.6lf\r\n", forward_volume);
		DBG_Print(buf);
		
		total_volume= (float)total_volume;
		sprintf(buf, "Total Vol  : %.6lf\r\n", total_volume);
		DBG_Print(buf);
		
		reverse_volume= (float)reverse_volume;
		sprintf(buf, "Reverse Vol: %.6lf\r\n", reverse_volume);
		DBG_Print(buf);

		DBG_Print("-----------------------\r\n");
		frame_recieved=0;
		build_frame(tx);		//chnged from build_frame(&tx[idx]);
	}
}

//

// ================= HEX TO ASCII =================
// Converts a byte array to ASCII hex string
// inBuf: input bytes
// len: number of bytes
// outBuf: output char buffer, must be at least len*3+1
void HexToAscii(uint8_t *inBuf, uint8_t len, char *outBuf)
{
    for (uint8_t i = 0; i < len; i++)
    {
        sprintf(&outBuf[i*3], "%02X ", inBuf[i]);
    }
    outBuf[len*3] = 0; // null-terminate
}

// Copy RX buffer to null-terminated ASCII string
// Non-printable bytes are replaced with '.'
void RxToAscii(uint8_t *rxBuf, uint16_t rxLen, char *asciiStr, uint16_t maxLen)
{
    uint8_t i;
    for(i = 0; i < rxLen && i < maxLen-1; i++)
    {
        if(rxBuf[i] >= 32 && rxBuf[i] <= 126) // printable ASCII
            asciiStr[i] = rxBuf[i];
        else if(rxBuf[i] == 0x0D)
            asciiStr[i] = '\r';
        else if(rxBuf[i] == 0x0A)
            asciiStr[i] = '\n';
        else
            asciiStr[i] = '.'; // replace non-printable bytes
    }
    asciiStr[i] = 0; // null-terminate
		HAL_UART_Transmit(&huart2, (uint8_t*)asciiStr, strlen(asciiStr), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}




void DBG_Print(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

void DBG_PrintHex(uint8_t *data, uint16_t len) {
  static const char hex[] = "0123456789ABCDEF";
  char out[3];

  for (uint16_t i = 0; i < len; i++) {
    out[0] = hex[(data[i] >> 4) & 0x0F];
    out[1] = hex[data[i] & 0x0F];
    out[2] = ' ';
    HAL_UART_Transmit(&huart2, (uint8_t *)out, 3, 100);
  }
  DBG_Print("\r\n");
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void) {

  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void) {

  HAL_UART_IRQHandler(&huart2);
}

/* ================= UART INIT ================= */

void CR95HF_UART_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;

  HAL_UART_Init(&huart1);
	
	modbus_address = read_modbus_address();
}

void DEBUG_UART_Init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  //		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  //		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  //		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USART2 interrupt Init */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  HAL_UART_Receive_IT(&huart2, &Debug_rxByte, 1);
}

/* ================= RX CALLBACK ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uint8_t byte = rxByte;

    uart1_rx_buffer[uart1_rx_index++] = byte;

    if (uart1_rx_index >= UART1_RX_BUFFER_SIZE)
      uart1_rx_index = 0;
		if (uart1_rx_index >= uart1_expected_len)
		{
			rxDone = 1;
		}
    // RESTART RX IMMEDIATELY (FIRST THING)
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  } else if (huart->Instance == USART2) {
    uint8_t byte = Debug_rxByte;
    uart2_frame_ready = 0;

    //        if(uart2_frame_ready == 0)  // prevent overwrite
    //        {
    uart2_rx_buffer[uart2_rx_index++] = byte;

    if (uart2_rx_index >= UART2_RX_BUFFER_SIZE)
      uart2_rx_index = 0;

    //            if (byte == '#' )
    if (uart2_rx_index >= 5) {
      uart2_frame_ready = 1;
      //HAL_UART_Transmit(&huart2, uart2_rx_buffer, uart2_rx_index, 100);
    }
    //        }

    HAL_UART_Receive_IT(&huart2, &Debug_rxByte, 1);
  }
}



/* ================= SEND ================= */

static void CR95HF_Send1(uint8_t *cmd, uint8_t txLen, uint8_t expectedRx) {
  rxDone = 0;
  rxLen = expectedRx;
	uart1_expected_len = expectedRx;
  uart1_rx_index = 0;

  memset(uart1_rx_buffer, 0, sizeof(uart1_rx_buffer));

  HAL_UART_Transmit(&huart1, cmd, txLen, 100);

  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

  rxTick = HAL_GetTick();
}


/* ================= TIMEOUT CHECK ================= */

static uint8_t WaitRx(uint32_t timeout) {
  if (!rxDone) {
    if (HAL_GetTick() - rxTick > timeout) {
      state = 0;
    }
    return 0;
  }

  rxDone = 0;
  //    __HAL_UART_FLUSH_DRREGISTER(&huart1);
  return 1;
}


void Read_Meter(void) {
  char buf[32];
  retry = 10;
  state = 0;
  Read_Done = 0;
	uart2_frame_ready = 0;
	memset(full_memory, 0, sizeof(full_memory));
	mem_offset = 0;
  while (retry != 0 && Read_Done == 0) {
		
   
		if (Read_Done == 1){
			//DBG_Print("Read done.\r\n\r\n");
		break;
		}
//		CR95HF_Read_Memory();
		CR95HF_Process();
		if (Read_Done == 1){
			DBG_Print("Read done.\r\n\r\n");
		return;
		}

    //		DBG_Print("RETRY : ");
    //		DBG_Print(retry);
    //		DBG_Print("   STATE : ");
    //		DBG_Print((uint8_t ) state);
    if (state == 0) {
      sprintf(buf, "RETRY : %d   STATE : %d\r\n\r\n", retry, Read_Done);
      DBG_Print(buf);
      DBG_Print("\r\n\r\n");
    }
		HAL_Delay(1);
  }
  //retry=10;
}

/* ================= STATE MACHINE ================= */

void CR95HF_Process1(void) {
  switch (state) {
    case 0:  // ECHO
      {
        uint8_t cmd = 0x55;
        DBG_Print("STATE=ECHO\r\n");
        CR95HF_Send1(&cmd, 1, 1);
        state = 1;
        retry--;
        break;
      }

    case 1:
      //            if(!WaitRx(1000)) break;
      WaitRx(1000);

      DBG_Print("ECHO RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      if (uart1_rx_buffer[0] != 0x55 && uart1_rx_buffer[0] != 0x80)
        DBG_Print("ECHO ERROR\r\n");

      state = 2;
      break;

    case 2:  // PROTOCOL
      {
        uint8_t cmd[] = { 0x02, 0x05, 0x02, 0x00, 0x00, 0x10, 0x00 };
        DBG_Print("STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        state = 3;
        break;
      }

    case 3:
      //            if(!WaitRx(1000)) break;
      WaitRx(1000);

      DBG_Print("PROTO RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      state = 4;
      break;

    case 4:  // RF ON
      {
        uint8_t cmd[] = { 0x09, 0x04, 0x3A, 0x00, 0x58, 0x04 };
        DBG_Print("STATE=RFON\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        state = 5;
        break;
      }

    case 5:
      //            if(!WaitRx(1000)) break;
      WaitRx(1000);

      DBG_Print("RFON RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
      HAL_Delay(10);
      state = 6;
      break;

    case 6:  // REQA
      {
        uint8_t cmd[] = { 0x04, 0x02, 0x26, 0x07 };
        DBG_Print("STATE=REQA\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 7);
        HAL_Delay(40);
        state = 7;
        break;
      }

    case 7:
      //if(!WaitRx(1500)) break;
      WaitRx(1000);

      if (uart1_rx_buffer[0] != 0x80) {
        state = 0;
        break;
      }

      DBG_Print("REQA RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      state = 8;
      break;

    case 8:  // ANTICOLLISION CL1
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x93, 0x20, 0x08 };
        DBG_Print("STATE=ANTICOLL CL1\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 10);
        HAL_Delay(40);
        state = 9;
        break;
      }

    case 9:
      //            if(!WaitRx(1500)) break;
      WaitRx(1000);

      DBG_Print("CL1 RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      if (rxLen >= 7) {
        uidLen = 4;
        memcpy(uid, &uart1_rx_buffer[3], 4);
        DBG_Print("UID PART1: ");
        DBG_PrintHex(uid, 4);
      }

      state = 10;
      break;

    case 10:  // SELECT CL2
      {
        uint8_t cmd[] = { 0x04, 0x08, 0x93, 0x70,
                          uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                          uart1_rx_buffer[6], uart1_rx_buffer[7] };
        DBG_Print("STATE=SELECT CL1\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 8);
        HAL_Delay(40);
        state = 11;
        break;
      }

    case 11:
      //            if(!WaitRx(1500)) break;
      WaitRx(1000);

      DBG_Print("SEL CL1 RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      state = 12;
      break;

    case 12:  // ANTICOLLISION CL2
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x95, 0x20, 0x08 };
        DBG_Print("STATE=ANTICOLL CL2\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 10);
        HAL_Delay(40);
        state = 13;
        break;
      }

    case 13:
      //            if(!WaitRx(1500)) break;
      WaitRx(1000);

      DBG_Print("CL2 RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      if (rxLen >= 7) {
        uidLen = 4;
        memcpy(uid, &uart1_rx_buffer[3], 4);
        DBG_Print("UID PART2: ");
        DBG_PrintHex(uid, 4);
      }

      state = 14;
      break;

    case 14:  // SELECT CL1
      {
        uint8_t cmd[] = { 0x04, 0x08, 0x95, 0x70,
                          uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                          uart1_rx_buffer[6], uart1_rx_buffer[7] };
        DBG_Print("STATE=SELECT CL2\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 8);
        HAL_Delay(40);
        state = 15;
        break;
      }

    case 15:
      //            if(!WaitRx(1500)) break;
      WaitRx(1000);

      DBG_Print("SEL CL2 RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      state = 16;
      break;


    case 16:  // READ PAGE
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x0D, 0x1B, 0x28 };
        DBG_Print("STATE=READ PAGE\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 20);
        HAL_Delay(40);
        state = 17;
        break;
      }

    case 17:
      //            if(!WaitRx(1500)) break;
      WaitRx(1000);

      DBG_Print("PAGE RX: ");
      DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      state = 0;
      break;

    default:
      state = 0;
      break;
  }
}





void CR95HF_Process(void) {
  switch (state) {
    case 0:  // ECHO
      {
        uint8_t cmd = 0x55;
        DBG_Print("0 STATE=ECHO\r\n");
        CR95HF_Send1(&cmd, 1, 1);
        state = 1;
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        DBG_Print("ECHO RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x55) {
          DBG_Print("ECHO ERROR\r\n");
					retry--;
          state = 0;
          break;
        }
        state = 1;
        retry--;
        break;
      }

    case 1:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("1 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x83) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 2;
        break;
      }
    case 2:  // PROTOCOL SELECT
      {
        uint8_t cmd[] = { 0x02, 0x05, 0x02, 0x00, 0x00, 0x10, 0x00 };
        //DBG_Print("2 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x00 && uart1_rx_buffer[1] != 0x00) {
          //DBG_Print("PROTCOL SELECT ERROR\r\n");
          state = 0;
          break;
        }
        state = 3;
        break;
      }
    case 3:  // REQA
      {
        uint8_t cmd[] = { 0x04, 0x02, 0x26, 0x07 };
        //DBG_Print("3 STATE=REQA\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 7);
        HAL_Delay(40);
        //if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("REQA RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 4;
        break;
      }
    case 4:  // ANTICOLLISION CL1
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x93, 0x20, 0x08 };
        //DBG_Print("4 STATE=ANTICOLL CL1\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 10);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("ANTICOLLISION CL1 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

        if (rxLen >= 7) {
          uidLen = 4;
          memcpy(uid, &uart1_rx_buffer[3], 4);
          //DBG_Print("UID PART1: ");
          //DBG_PrintHex(uid, 4);
          //								if(uart1_rx_buffer[2] != 0x88)		// DO IT LATER. IF CASCADED TAG, THEN HAVE MORE 4 BYTES OF UID
          //								{
          //										state = 5;
          //										break;
          //								}else{
          //									state = 5;
          //								}
        } else {
          state = 0;
          break;
        }
      }

    case 5:  // SELECT CL1
      {
        uint8_t cmd[] = { 0x04, 0x08, 0x93, 0x70,
                          uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                          uart1_rx_buffer[6], uart1_rx_buffer[7] };
        //DBG_Print("5 STATE=SELECT CL1\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 8);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SEL CL1 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 6;
        break;
      }
    case 6:  // ANTICOLLISION CL2
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x95, 0x20, 0x08 };
        //DBG_Print("6 STATE=ANTICOLL CL2\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 10);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("CL2 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (rxLen >= 7) {
          uidLen = 4;
          memcpy(uid, &uart1_rx_buffer[3], 4);
          //DBG_Print("UID PART2: ");
          //DBG_PrintHex(uid, 4);
        } else {
          state = 0;
          break;
        }
        state = 7;
        break;
      }
    case 7:  // SELECT CL2
      {
        uint8_t cmd[] = { 0x04, 0x08, 0x95, 0x70,
                          uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                          uart1_rx_buffer[6], uart1_rx_buffer[7] };
        //DBG_Print("7 STATE=SELECT CL2\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 8);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SEL CL2 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 8;
        break;
      }
    case 8:  // GET VERSION
      {
        uint8_t cmd[] = { 0x04, 0x02, 0x60, 0x28 };
        DBG_Print("8 GET VERSION\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 15);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("GET VERSION RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 9;
        break;
      }
    case 9:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("9 STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 6);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("STATE RESET RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 10;
        break;
      }
    case 10:  // SECTOR SELECT
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x03, 0x00, 0x00, 0x00, 0x28 };
        //DBG_Print("10 SECTOR SELECT\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SECTOR SELECT RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87) {
          state = 0;
          break;
        }
        state = 11;
        break;
      }
    case 11:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x30, 0xF8, 0x28 };//changing here to check the direct command is working or not
        DBG_Print("11 PAGE READ F8\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F8 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }

        state = 12;

        //HAL_Delay(1000);
        break;
      }

    case 12:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("12 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }

        state = 13;
        break;
      }

    case 13:  // GAURD DELAY
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x01, 0x00, 0x00, 0x00, 0x28 };
				if (A5_5A_Err){
					cmd[2] = 0x01;
					//A5_5A_Err = 0;
				}else{
					cmd[2] = 0x00;
				}
        //DBG_Print("13 GAURD DELAY\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("GAURD DELAY RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 14;
        break;
      }


    case 14:  // PAGE WRITE FE
      {	
				
        uint8_t cmd[] = { 0x04, 0x07, 0xA2, 0xFE, 0x1E, 0x06, 0x13, 0x84, 0x28 };
        DBG_Print("14 PAGE WRITE FE\r\n");
				Process_Frame(uart1_rx_buffer, uart1_rx_index);
        CR95HF_Send1(FE_frame, sizeof(FE_frame), 6);
        HAL_Delay(40);
        //if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE WRITE FE RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 15;
        //HAL_Delay(1000);
        break;
      }


    case 15:  // PAGE WRITE FF
      {
        uint8_t cmd[] = { 0x04, 0x07, 0xA2, 0xFF, 0xA5, 0x5A, 0x29, 0x03, 0x28 };
        DBG_Print("15 PAGE WRITE FF\r\n");
        CR95HF_Send1(FF_frame, sizeof(FF_frame), 6);
        HAL_Delay(40);
        //if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE WRITE FF RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 16;
        //HAL_Delay(2000);
        break;
      }


    case 16:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        DBG_Print("16 STATE= STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        DBG_Print("STATE RESET RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 17;
        break;
      }

    case 17:  // SECTOR SELECT
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x03, 0x00, 0x00, 0x00, 0x28 };
        //DBG_Print("17 SECTOR SELECT\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SECTOR SELECT RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 18;
        break;
      }
    case 18:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x30, 0xF8, 0x28 };
        DBG_Print("18 PAGE READ F8\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F8 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 19;
        //HAL_Delay(1000);
        break;
      }

    case 19:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("19 STATE=STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("STATE RESET RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 20;
        break;
      }

    case 20:  // GAURD DELAY
      {	
				uint8_t cmd[] = { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x28 };
				if (A5_5A_Err){
					cmd[2] = 0x01;
					//A5_5A_Err = 0;
				}else{
					cmd[2] = 0x00;
				}
        //DBG_Print("20 GAURD DELAY\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("GAURD DELAY RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 21;
        break;
      }

    case 21:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0xF0, 0xFF, 0x28 };
        DBG_Print("21 PAGE READ F0-FF\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F0-FF RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
				//DBG_PrintHex(uart1_rx_buffer[uart1_rx_index]-6, 1);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
				
				if (uart1_rx_buffer[62] == 0xA5  && uart1_rx_buffer[63] == 0x5A ) {
					DBG_Print("\nCombination 3 reset due to A5 and 5A\n ");
					combination_state=COMBINATION_3;
					
					A5_5A_Err_Count++;
					if (A5_5A_Err_Count>2){
						A5_5A_Err=~A5_5A_Err;
						A5_5A_Err_Count=0;
					}
					state = 22;
					break;
				}
				
				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C && uart1_rx_buffer[64] != 0x01) {
					DBG_Print("\nCombination 3 reset state 21 __ 7\n ");
					combination_state=COMBINATION_3;
				}
				
//				if (uart1_rx_buffer[62] == 0xA5  && uart1_rx_buffer[63] == 0x5A ) {
//					DBG_Print("\nCombination 3 reset due to A5 and 5A\n ");
//					combination_state=COMBINATION_3;
//					
//					A5_5A_Err_Count++;
//					if (A5_5A_Err_Count>2){
//						A5_5A_Err=1;
//					}
//					
//				}else{
//				
//				}
				
				
				
				
//				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C) {
//					DBG_Print("\nCombination 3 reset state 21__6\n ");
//					combination_state=COMBINATION_3;
//				}
//				if (uart1_rx_buffer[65] != 0x29  && uart1_rx_buffer[64] != 0x2C) {
//					DBG_Print("\nCombination 3 reset state 21__5\n ");
//					combination_state=COMBINATION_3;
//				}
        Process_Frame(uart1_rx_buffer, uart1_rx_index);
				Decode_Data();
        state = 22;
        //HAL_Delay(3000);
        break;
      }

      /***************** READING AGAIN ***************************/
    case 22:  // ECHO
      {
        uint8_t cmd = 0x55;
        DBG_Print("22 STATE=ECHO\r\n");
        CR95HF_Send1(&cmd, 1, 1);
        state = 1;
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        DBG_Print("ECHO RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x55) {
          DBG_Print("ECHO ERROR\r\n");
          state = 0;
          break;
        }
        state = 23;
        retry--;
        break;
      }

    case 23:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("23 STATE=	STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 24;
        break;
      }
    case 24:  // SECTOR SELECT
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x03, 0x00, 0x00, 0x00, 0x28 };
        //DBG_Print("24 SECTOR SELECT\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SECTOR SELECT RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 25;
        break;
      }
    case 25:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x30, 0xF8, 0x28 };
        DBG_Print("25 PAGE READ F8\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F8 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 26;
        //HAL_Delay(1000);
        break;
      }

    case 26:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        DBG_Print("26 STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 6);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("STATE RESET RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 27;
        break;
      }
    case 27:  // SECTOR SELECT
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x03, 0x00, 0x00, 0x00, 0x28 };
        DBG_Print("27 SECTOR SELECT\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SECTOR SELECT RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87) {
          state = 0;
          break;
        }
        state = 28;
        break;
      }
    case 28:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x30, 0xF8, 0x28 };
        DBG_Print("28 PAGE READ F8\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F8 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 29;

        //HAL_Delay(1000);
        break;
      }

    case 29:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("29 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }

        state = 30;
        break;
      }

    case 30:  // GAURD DELAY
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x01, 0x00, 0x00, 0x00, 0x28 };
				if (A5_5A_Err){
					cmd[2] = 0x01;
					//A5_5A_Err = 0;
				}else{
					cmd[2] = 0x00;
				}
        //DBG_Print("30 GAURD DELAY\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("GAURD DELAY RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 31;
        break;
      }


    case 31:  // PAGE WRITE FE
      {
        uint8_t cmd[] = { 0x04, 0x07, 0xA2, 0xFE, 0x1E, 0x06, 0xF8, 0x74, 0x28 };
        DBG_Print("31 PAGE WRITE FE\r\n");

        CR95HF_Send1(FE_frame, sizeof(FE_frame), 6);
        HAL_Delay(40);
        //if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE WRITE FE RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 32;
        //HAL_Delay(1000);
        break;
      }


    case 32:  // PAGE WRITE FF
      {
        uint8_t cmd[] = { 0x04, 0x07, 0xA2, 0xFF, 0xA5, 0x5A, 0x2C, 0x03, 0x28 };
        DBG_Print("32 PAGE WRITE FF\r\n");
        CR95HF_Send1(FF_frame, sizeof(FF_frame), 6);
        HAL_Delay(40);
        //if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE WRITE FF RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 33;
        //HAL_Delay(2000);
        break;
      }


    case 33:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("33 STATE= STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("STATE RESET RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 34;
        break;
      }

    case 34:  // SECTOR SELECT
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x03, 0x00, 0x00, 0x00, 0x28 };
        //DBG_Print("34 SECTOR SELECT\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("SECTOR SELECT RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 35;
        break;
      }
    case 35:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x03, 0x30, 0xF8, 0x28 };
        DBG_Print("35 PAGE READ F8\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F8 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 36;
        //HAL_Delay(1000);
        break;
      }

    case 36:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("36 STATE=STATE RESET\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("STATE RESET RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x90) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        state = 37;
        break;
      }

    case 37:  // GAURD DELAY
      {
        uint8_t cmd[] = { 0x04, 0x05, 0x01, 0x00, 0x00, 0x00, 0x28 };
				if (A5_5A_Err){
					cmd[2] = 0x01;
					//A5_5A_Err = 0;
				}else{
					cmd[2] = 0x00;
				}
        //DBG_Print("37 GAURD DELAY\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("GAURD DELAY RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        state = 38;
        break;
      }

    case 38:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0xF0, 0xFF, 0x28 };
        DBG_Print("38 PAGE READ F0-FF\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        DBG_Print("PAGE READ F0-FF RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
				
				if (uart1_rx_buffer[62] == 0xA5  && uart1_rx_buffer[63] == 0x5A ) {
					DBG_Print("\nCombination 3 reset due to A5 and 5A\n ");
					combination_state=COMBINATION_3;
					
					A5_5A_Err_Count++;
					if (A5_5A_Err_Count>2){
						A5_5A_Err=~A5_5A_Err;
						A5_5A_Err_Count=0;
					}
					state = 39;
					break;
					
				}
				
				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C && uart1_rx_buffer[64] != 0x01 ) {
					DBG_Print("\nCombination 3 reset state 21 __ 7\n ");
					combination_state=COMBINATION_3;
					
				}else {
					combination_state=COMBINATION_1;
				}
        Process_Frame(uart1_rx_buffer, uart1_rx_index);
				Decode_Data();
				
				//Parse_UART1_Data();
			//	Print_Serial_Number();
        state = 39;
        //HAL_Delay(3000);
        break;
      }
    case 39:  // PROTOCOL SELECT
      {
        uint8_t cmd[] = { 0x02, 0x02, 0x00, 0x00 };
        //DBG_Print("39 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        //            if(!WaitRx(1000)) break;
        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x00 && uart1_rx_buffer[1] != 0x00) {
          DBG_Print("PROTCOL SELECT ERROR\r\n");
          state = 0;
          break;
        }
        Read_Done = 1;
				return;

        break;
      }

    default:
      state = 40;
      break;
  }
}
// Print what was actually read from the tag (useful for checking)
void Print_Read_Data(void) {
    DBG_Print("\r\n=== METER READ DATA ===\r\n");
    DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);   // Last received frame (F0-FF usually)
    DBG_Print("=======================\r\n");
    
    // Optional: Parse basic info if you want
    // Parse_UART1_Data();        // Uncomment if you want temperature/volume parsing
    // Print_Serial_Number();     // Uncomment if you want serial number
}


// ====================== MAIN STATE MACHINE (with full debug) ======================
void CR95HF_Process2(void) {
    switch (state) {
        case 0:  // ECHO
            DBG_Print("STATE 0: ECHO\r\n");
            CR95HF_Send1((uint8_t*)"\x55", 1, 1);
            if (!WaitRx(1000) || uart1_rx_buffer[0] != 0x55) {
                DBG_Print("ECHO ERROR\r\n");
                state = 0; break;
            }
            DBG_Print("ECHO RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            state = 1;
            retry--;
            break;

        case 1:  // Reset + Protocol + Inventory flow
            DBG_Print("STATE 1: Reset + Protocol Select + Inventory\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x03\xC2\xFF\x28", 5, 2); WaitRx(1000);
            CR95HF_Send1((uint8_t*)"\x02\x05\x02\x00\x00\x10\x00", 7, 2); WaitRx(1000);
            CR95HF_Send1((uint8_t*)"\x04\x02\x26\x07", 4, 7);
            if (!WaitRx(1000) || uart1_rx_buffer[0] != 0x80) {
                DBG_Print("REQA Failed\r\n"); state = 0; break;
            }
            DBG_Print("REQA OK\r\n");
            state = 8;
            break;

        case 8:  // Get Version
            DBG_Print("STATE 8: GET VERSION\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x02\x60\x28", 4, 15);
            if (!WaitRx(1000) || uart1_rx_buffer[0] != 0x80) {
                DBG_Print("GET VERSION Failed\r\n"); state = 0; break;
            }
            DBG_Print("GET VERSION RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            state = 10;
            break;

        case 10: // Sector Select
            DBG_Print("STATE 10: SECTOR SELECT\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x05\x03\x00\x00\x00\x28", 7, 2); WaitRx(1000);
            state = 11;
            break;

        case 11:
            DBG_Print("STATE 11: PAGE READ F8\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x03\x30\xF8\x28", 5, 2); WaitRx(1000);
            DBG_Print("F8 RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            state = 12;
            break;

        case 12: // Guard Delay
            DBG_Print("STATE 12: GUARD DELAY\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x05\x00\x00\x00\x00\x28", 7, 2); WaitRx(1000);
            state = 14;
            break;

        case 14: // Write FE
            DBG_Print("STATE 14: PAGE WRITE FE\r\n");
            Process_Frame(uart1_rx_buffer, uart1_rx_index);
            CR95HF_Send1(FE_frame, 9, 6);
            WaitRx(1000);
            DBG_Print("WRITE FE RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            state = 15;
            break;

        case 15: // Write FF
            DBG_Print("STATE 15: PAGE WRITE FF\r\n");
            CR95HF_Send1(FF_frame, 9, 6);
            WaitRx(1000);
            DBG_Print("WRITE FF RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            state = 16;
            break;

        case 16: // Final Read F0-FF + Done
            DBG_Print("STATE 16: FINAL RESET + READ F0-FF\r\n");
            CR95HF_Send1((uint8_t*)"\x04\x03\xC2\xFF\x28", 5, 2); WaitRx(1000);
            CR95HF_Send1((uint8_t*)"\x04\x04\x3A\xF0\xFF\x28", 6, 200);   // large enough buffer
            if (WaitRx(2000) && uart1_rx_buffer[0] == 0x80) {
                DBG_Print("FINAL READ F0-FF SUCCESS\r\n");
                DBG_Print("FINAL READ RX: "); DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
                Process_Frame(uart1_rx_buffer, uart1_rx_index);
                Print_Read_Data();          // <--- This shows you exactly what was read
                Read_Done = 1;
                state = 0;
                return;
            }
            DBG_Print("FINAL READ Failed\r\n");
            state = 0;
            break;

        default:
            state = 0;
            break;
    }
	}



void CR95HF_Read_Memory(void) {
	char buf[32];
  switch (state) {
    case 0:  // ECHO
      {
        uint8_t cmd = 0x55;
        //DBG_Print("0 STATE=ECHO\r\n");
        CR95HF_Send1(&cmd, 1, 1);
        state = 1;
        if(!WaitRx(1000)) break;
//        WaitRx(1000);
        //DBG_Print("ECHO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x55) {
          //DBG_Print("ECHO ERROR\r\n");
          state = 0;
          break;
        }
        state = 1;
        retry--;
        break;
      }

    case 1:  // STATE RESET
      {
        uint8_t cmd[] = { 0x04, 0x03, 0xC2, 0xFF, 0x28 };
        //DBG_Print("1 STATE=PROTOCOL\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 2);
        HAL_Delay(40);
        if(!WaitRx(1000)) break;
//        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x83) {
          //DBG_Print("STATE RESET ERROR\r\n");
          state = 0;
          break;
        }
        // PROTOCOL SELECT
        {
          uint8_t cmd[] = { 0x02, 0x05, 0x02, 0x00, 0x00, 0x10, 0x00 };
          //DBG_Print("2 STATE=PROTOCOL\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 2);
        }
        HAL_Delay(40);
				if(!WaitRx(1000)) break;
//        WaitRx(1000);
        //DBG_Print("PROTO RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x00 && uart1_rx_buffer[1] != 0x00) {
          //DBG_Print("PROTCOL SELECT ERROR\r\n");
          state = 0;
          break;
        }
        // REQA
        {
          uint8_t cmd[] = { 0x04, 0x02, 0x26, 0x07 };
          //DBG_Print("3 STATE=REQA\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 7);
        }
        HAL_Delay(40);
        if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("REQA RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        // ANTICOLLISION CL1
        {
          uint8_t cmd[] = { 0x04, 0x03, 0x93, 0x20, 0x08 };
          //DBG_Print("4 STATE=ANTICOLL CL1\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 10);
        }
        HAL_Delay(40);
       if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("ANTICOLLISION CL1 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

        if (rxLen >= 7) {
          uidLen = 4;
          memcpy(uid, &uart1_rx_buffer[3], 4);
          //DBG_Print("UID PART1: ");
          //DBG_PrintHex(uid, 4);
          //								if(uart1_rx_buffer[2] != 0x88)		// DO IT LATER. IF CASCADED TAG, THEN HAVE MORE 4 BYTES OF UID
          //								{
          //										state = 5;
          //										break;
          //								}else{
          //									state = 5;
          //								}
        } else {
          state = 0;
          break;
        }
        // SELECT CL1
        {
          uint8_t cmd[] = { 0x04, 0x08, 0x93, 0x70,
                            uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                            uart1_rx_buffer[6], uart1_rx_buffer[7] };
          //DBG_Print("5 STATE=SELECT CL1\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 8);
        }
        HAL_Delay(40);
				if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("SEL CL1 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        // ANTICOLLISION CL2
        {
          uint8_t cmd[] = { 0x04, 0x03, 0x95, 0x20, 0x08 };
          //DBG_Print("6 STATE=ANTICOLL CL2\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 10);
        }
        HAL_Delay(40);
        if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("CL2 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (rxLen >= 7) {
          uidLen = 4;
          memcpy(uid, &uart1_rx_buffer[3], 4);
          //DBG_Print("UID PART2: ");
          //DBG_PrintHex(uid, 4);
        } else {
          state = 0;
          break;
        }
        // SELECT CL2
        {
          uint8_t cmd[] = { 0x04, 0x08, 0x95, 0x70,
                            uart1_rx_buffer[2], uart1_rx_buffer[3], uart1_rx_buffer[4], uart1_rx_buffer[5],
                            uart1_rx_buffer[6], uart1_rx_buffer[7] };
          //DBG_Print("7 STATE=SELECT CL2\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 8);
        }
        HAL_Delay(40);
        if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("SEL CL2 RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        // GET VERSION
        {
          uint8_t cmd[] = { 0x04, 0x02, 0x60, 0x28 };
          //DBG_Print("8 GET VERSION\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 15);
        }
        HAL_Delay(40);
        if(!WaitRx(1500)) break;
//        WaitRx(1000);
        //DBG_Print("GET VERSION RX: ");
        //DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          break;
        }
        // PAGE READ
        {
          uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x05, 0x20, 0x28 };
          //DBG_Print("Direct PAGE READ 05-20\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 119);
        }
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("PAGE READ 05-20 RX: ");
       // DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
				if (uart1_rx_buffer[0] == 0x80)
				{
						uint8_t payload_len = uart1_rx_buffer[1]-5;

						if ((mem_offset + payload_len) < TOTAL_MEMORY_SIZE)
						{
								memcpy(&full_memory[mem_offset], &uart1_rx_buffer[2], payload_len);
								mem_offset += payload_len;
						}
						else
						{
								DBG_Print("BUFFER OVERFLOW\r\n");
								state = 0;
								return;
						}
//						DBG_Print("\r\nFULL MEMORY:\r\n");
//						DBG_PrintHex(full_memory, mem_offset);
//						DBG_Print("\r\nFULL MEMORY 1Done\r\n");
				}else {
					//state = 0;
					state = 0;
					break;
				}
        // PAGE READ
        {
          uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x20, 0x40, 0x28 };
          //DBG_Print("Direct PAGE READ 20-40\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 139);
        }
        HAL_Delay(40);
        //            if(!WaitRx(1500)) break;
        WaitRx(1000);
        //DBG_Print("PAGE READ 20-40 RX: ");
       // DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] == 0x80)
				{
						uint8_t payload_len = uart1_rx_buffer[1]-5;

						if ((mem_offset + payload_len) < TOTAL_MEMORY_SIZE)
						{
								memcpy(&full_memory[mem_offset], &uart1_rx_buffer[2], payload_len);
								mem_offset += payload_len;
						}
						else
						{
								DBG_Print("BUFFER OVERFLOW\r\n");
								state = 0;
								return;
						}
//					DBG_Print("\r\nFULL MEMORY:\r\n");
//					DBG_PrintHex(full_memory, mem_offset);
//						DBG_Print("\r\nFULL MEMORY 2Done\r\n");
				}else {
					//state = 0;
					state = 0;
					break;
				}
        // PAGE READ
        {
          uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x40, 0x60, 0x28 };
          //DBG_Print("Direct PAGE READ 40-60\r\n");
          CR95HF_Send1(cmd, sizeof(cmd), 139);
        }
        HAL_Delay(100);
        //            if(!WaitRx(1500)) break;
        WaitRx(5000);
        //DBG_Print("PAGE READ 40-60 RX: ");
      //  DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] == 0x80)
				{
						uint8_t payload_len = uart1_rx_buffer[1]-5;

						if ((mem_offset + payload_len) < TOTAL_MEMORY_SIZE)
						{
								memcpy(&full_memory[mem_offset], &uart1_rx_buffer[2], payload_len);
//							DBG_Print("\r\nFULL MEMORY length before:\r\n");
//						 sprintf(buf, "RETRY : %d  \r\n\r\n",  mem_offset);
//							DBG_Print(buf);
//							DBG_Print("\r\n\r\n");
						//DBG_Print("\r\nFULL MEMORY 3Done\r\n");	
							mem_offset += payload_len;
							full_memory[mem_offset] = '\0';
//							DBG_Print("\r\nFULL MEMORY length after:\r\n");
//						 sprintf(buf, "RETRY : %d  SIZE : %d\r\n\r\n",  mem_offset, strlen(full_memory));
//							DBG_Print(buf);
//							DBG_Print("\r\n\r\n");
						//DBG_Print("\r\nFULL MEMORY 3Done\r\n");
						}
						else
						{
								DBG_Print("BUFFER OVERFLOW\r\n");
								state = 0;
								return;
						}
						DBG_Print("\r\nFULL MEMORY:\r\n");
						DBG_PrintHex(full_memory, mem_offset);
						DBG_Print("\r\nFULL MEMORY 3Done\r\n");
						
								char asciiStr[128];

						// After receiving RX from CR95HF
						//HexToAscii(rxBuf, rxLen, asciiStr);
						RxToAscii(full_memory, mem_offset, asciiStr, sizeof(asciiStr));
						
						//DBG_PrintASCII(full_memory, mem_offset);
//						UART_PrintASCII(&huart1, full_memory, mem_offset);
						//UART_PrintASCII_Fast(&huart1, full_memory, mem_offset);
						Read_Done = 1;
						return;
				}else {
					state = 0;
					break;
				}
      }


    default:
      state = 0;
      break;
  }
}








void Process_Frame(uint8_t *rxBuffer, uint16_t rxLen) {
  uint8_t SS=0;
  uint8_t MM=0;


  // Safety check
  if (rxLen < 12)
    return;

  // Extract SS and MM (12th and 11th from end)
  SS = rxBuffer[rxLen - 12];
  MM = rxBuffer[rxLen - 11];

  // ---------------- FE FRAME ----------------
  FE_frame[0] = 0x04;
  FE_frame[1] = 0x07;
  FE_frame[2] = 0xA2;
  FE_frame[3] = 0xFE;
  FE_frame[4] = SS;
  FE_frame[5] = MM;

  // ---------------- FF FRAME ----------------
  FF_frame[0] = 0x04;
  FF_frame[1] = 0x07;
  FF_frame[2] = 0xA2;
  FF_frame[3] = 0xFF;
  FF_frame[4] = 0xA5;
  FF_frame[5] = 0x5A;


if (combination_state == COMBINATION_3) {

      // FE tail
			FE_frame[4] = 0x00;
			FE_frame[5] = 0x00;
      FE_frame[6] = 0x2C;
      FE_frame[7] = 0x17;
			FF_frame[8] = 0x28;
		
      // FF tail
      FF_frame[6] = 0x01;
      FF_frame[7] = 0x03;
			FE_frame[8] = 0x28;

    }

// ====================== TRY TO EXTRACT METER ID (Only when available) ======================
    if (rxLen >= 45 && combination_state == COMBINATION_3)
    {
			combination_state = COMBINATION_1;
			uint32_t meter_id = ((uint32_t)rxBuffer[41] << 24) |
													((uint32_t)rxBuffer[42] << 16) |
													((uint32_t)rxBuffer[43] <<  8) |
													 rxBuffer[44];

			// If we got a valid new Meter ID, save it and calculate CRC init
			if (meter_id != 0 && meter_id != saved_meter_id)
			{
					saved_meter_id = meter_id;
					uint16_t high = (meter_id >> 16) & 0xFFFF;
					uint16_t low  = meter_id & 0xFFFF;
					saved_crc_init = high ^ low;

					// Optional: Print for debugging
					// printf("Meter ID saved: 0x%08X, CRC Init: 0x%04X\n", saved_meter_id, saved_crc_init);
			}
			return;
	}
		
		
		
		

    if (combination_state == COMBINATION_1) {
      // FE tail
//      FE_frame[6] = 0x4E;
//      FE_frame[7] = 0x92;

      // FF tail
      FF_frame[6] = 0x2C;
      FF_frame[7] = 0x03;
			uint8_t command[] = {FF_frame[7], FF_frame[6], 0x5A, 0xA5};

    // Calculate CRC
			uint16_t crc_value = crc16_ccitt(command, 4, saved_crc_init);

      combination_state = COMBINATION_2;
    } else if (combination_state == COMBINATION_2){
      // FE tail
//      FE_frame[6] = 0xA5;
//      FE_frame[7] = 0x62;

      // FF tail
      FF_frame[6] = 0x29;
      FF_frame[7] = 0x03;
			
			uint8_t command[] = {FF_frame[7], FF_frame[6], 0x5A, 0xA5};

    // Calculate CRC
			uint16_t crc_value = crc16_ccitt(command, 4, saved_crc_init);

      combination_state = COMBINATION_1;
    }



  FE_frame[8] = 0x28;
  FF_frame[8] = 0x28;

  //		DBG_Print("Extracted SS MM: ");		// UNCOMMENT TO DEBUG THE FRAMES
  //		uint8_t temp[2] = {SS, MM};
  //		DBG_PrintHex(temp, 2);
  //		DBG_Print("FE FRAME: \r\n");
  //		DBG_PrintHex(FE_frame, 8);
  //		DBG_Print("FF FRAME: \r\n");
  //		DBG_PrintHex(FF_frame, 8);
  //		DBG_Print("\r\n");

  // Now transmit FE_frame and FF_frame using your existing UART code
  // HAL_UART_Transmit(&huartX, FE_frame, 9, 100);
  // HAL_UART_Transmit(&huartX, FF_frame, 9, 100);
}
