#include <string.h>
#include "cr95hf.h"
#include "stdint.h"
#include <string.h>
#include <stdio.h>

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
//		FE_frame[6] = 0x4E;
//    FE_frame[7] = 0x92;
		FE_frame[7] = (uint8_t)(crc & 0xFF);        // Low byte
    FE_frame[6] = (uint8_t)((crc>> 8) & 0xFF); // High byte
		DBG_Print("CRC: ");
    DBG_PrintHex(FE_frame, 8);
    return crc;
}


// ====================== MAIN FUNCTION TO INTEGRATE ======================
/*
 * Function: Prepare_Meter_Command
 * Description: Extracts Meter ID from 80 45 response, calculates CRC init,
 *              computes CRC for command 03 29 5A A5, and builds final frame.
 * 
 * Parameters:
 *   response     : Pointer to received data buffer (starting with 0x80 0x45)
 *   resp_len     : Length of response buffer
 *   final_cmd    : Output buffer (must be at least 6 bytes)
 * 
 * Return:
 *   1 = Success (final_cmd is filled)
 *   0 = Failed (invalid response length)
 */
uint8_t Prepare_Meter_Command(const uint8_t *response, uint16_t resp_len, uint8_t *final_cmd)
{
    uint32_t meter_id = 0;
    uint16_t crc_init = 0;
    uint16_t crc_value = 0;

    // Check minimum length (we need at least byte 44 for Meter ID)
    if (resp_len < 45) {
        return 0;                    // Invalid response
    }

    // Extract Meter ID (bytes 41 to 44) - Big Endian
    meter_id = ((uint32_t)response[41] << 24) |
               ((uint32_t)response[42] << 16) |
               ((uint32_t)response[43] <<  8) |
                response[44];

    // Step 1: Calculate CRC Initial Value (exact method as in example)
    uint16_t high = (meter_id >> 16) & 0xFFFF;
    uint16_t low  = meter_id & 0xFFFF;
    crc_init = high ^ low;

    // Step 2: Command bytes
    const uint8_t command[4] = {0x03, 0x29, 0x5A, 0xA5};

    // Step 3: Calculate CRC
    crc_value = crc16_ccitt(command, 4, crc_init);

    // Step 4: Build final command frame (Low byte first, then High byte)
    final_cmd[0] = 0x03;
    final_cmd[1] = 0x29;
    final_cmd[2] = 0x5A;
    final_cmd[3] = 0xA5;
    final_cmd[4] = (uint8_t)(crc_value & 0xFF);        // Low byte
    final_cmd[5] = (uint8_t)((crc_value >> 8) & 0xFF); // High byte

    return 1;   // Success
}




/* ================= FAST DEBUG PRINT ================= */

void Parse_UART1_Data(void)
{
	char buf[100];
    if (uart1_rx_index < 40) // basic safety check
    {
        DBG_Print("Invalid data length\r\n");
        return;
    }

    double forward_volume = 0;
    double total_volume = 0;
    double reverse_volume = 0;
    float temperature = 0;
    float flow = 0;

    // ---- Extract values (based on your data order) ----
    // Adjust offsets if required

    memcpy(&temperature,     &uart1_rx_buffer[24], 4);
    memcpy(&reverse_volume,  &uart1_rx_buffer[28], 8);
    memcpy(&total_volume,    &uart1_rx_buffer[36], 8);
    memcpy(&forward_volume,  &uart1_rx_buffer[44], 8);
    memcpy(&flow,            &uart1_rx_buffer[52], 4);

    // ---- Print values ----
    DBG_Print("------ UART DATA ------\r\n");

 // ---- Print Temperature ----
    sprintf(buf, "Temperature: %.2f\r\n", temperature);
    DBG_Print(buf);

    // ---- Print Flow ----
    sprintf(buf, "Flow       : %.2f\r\n", flow);
    DBG_Print(buf);

    // ---- Print Volumes ----
    sprintf(buf, "Forward Vol: %.6lf\r\n", forward_volume);
    DBG_Print(buf);

    sprintf(buf, "Total Vol  : %.6lf\r\n", total_volume);
    DBG_Print(buf);

    sprintf(buf, "Reverse Vol: %.6lf\r\n", reverse_volume);
    DBG_Print(buf);

    DBG_Print("-----------------------\r\n");
}

void Print_Serial_Number(void)
{
    char buf[50];
    char serial[15];
    char serial_rev[15];

    memcpy(serial, &uart1_rx_buffer[8], 14);
    serial[14] = '\0';

    for (int i = 0; i < 14; i++)
    {
        serial_rev[i] = serial[13 - i];
    }
    serial_rev[14] = '\0';

    sprintf(buf, "Serial No (Original): %s\r\n", serial);
    DBG_Print(buf);

    sprintf(buf, "Serial No (Reverse) : %s\r\n", serial_rev);
    DBG_Print(buf);
}

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

    case 40:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x05, 0x20, 0x28 };
        DBG_Print("Direct PAGE READ 00-50\r\n");
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
        //Process_Frame(uart1_rx_buffer, uart1_rx_index);
        state = 41;
        //HAL_Delay(3000);
        break;
      }

    case 41:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x20, 0x40, 0x28 };
        DBG_Print("Direct PAGE READ 00-50\r\n");
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
        //Process_Frame(uart1_rx_buffer, uart1_rx_index);
        state = 42;
        //HAL_Delay(3000);
        break;
      }

    case 42:  // PAGE READ
      {
        uint8_t cmd[] = { 0x04, 0x04, 0x3A, 0x40, 0x60, 0x28 };
        DBG_Print("Direct PAGE READ 00-50\r\n");
        CR95HF_Send1(cmd, sizeof(cmd), 200);
        HAL_Delay(100);
        //            if(!WaitRx(1500)) break;
        WaitRx(5000);
        DBG_Print("PAGE READ 25-91 RX: ");
        DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
        if (uart1_rx_buffer[0] != 0x80) {
          state = 0;
          Read_Done = 1;
          break;
        }
        //Process_Frame(uart1_rx_buffer, uart1_rx_index);
        state = 43;
        Read_Done = 1;

        //HAL_Delay(3000);
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
        uint8_t cmd[] = { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x28 };
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
				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C && uart1_rx_buffer[64] != 0x01) {
					DBG_Print("\nCombination 3 reset state 21 __ 7\n ");
					combination_state=COMBINATION_3;
				}
//				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C) {
//					DBG_Print("\nCombination 3 reset state 21__6\n ");
//					combination_state=COMBINATION_3;
//				}
//				if (uart1_rx_buffer[65] != 0x29  && uart1_rx_buffer[64] != 0x2C) {
//					DBG_Print("\nCombination 3 reset state 21__5\n ");
//					combination_state=COMBINATION_3;
//				}
        Process_Frame(uart1_rx_buffer, uart1_rx_index);
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
        uint8_t cmd[] = { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x28 };
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
        uint8_t cmd[] = { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x28 };
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
				if (uart1_rx_buffer[64] != 0x29  && uart1_rx_buffer[64] != 0x2C && uart1_rx_buffer[64] != 0x01 ) {
					DBG_Print("\nCombination 3 reset state 21 __ 7\n ");
					combination_state=COMBINATION_3;
					
				}else {
					combination_state=COMBINATION_1;
				}
        Process_Frame(uart1_rx_buffer, uart1_rx_index);
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


      //				case 12:    // READ PAGE
      //        {
      //            uint8_t cmd[] = {0x04,0x04,0x3A,0x0D,0x1B,0x28};
      //            DBG_Print("STATE=READ PAGE\r\n");
      //            CR95HF_Send1(cmd,sizeof(cmd),20);
      //						HAL_Delay(40);
      //            state = 13;
      //            break;
      //        }

      //        case 13:
      ////            if(!WaitRx(1500)) break;
      //						WaitRx(1000);

      //            DBG_Print("PAGE RX: ");
      //            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

      //            state = 0;
      //            break;

    default:
      state = 40;
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

///*143587374f
//	kulk
	

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
	


//  // Alternate combination
//  if (memcmp(&rxBuffer[3], targetSeq, 14) != 0) {
//    SS = rxBuffer[rxLen - 12];
//    MM = rxBuffer[rxLen - 11];
//    FE_frame[4] = SS;
//    FE_frame[5] = MM;

//    if (combination_state == COMBINATION_1) {

//      // FE tail
//      FE_frame[6] = 0xF8;
//      FE_frame[7] = 0x74;

//      // FF tail
//      FF_frame[6] = 0x2C;
//      FF_frame[7] = 0x03;

//      combination_state = COMBINATION_2;
//    } else if (combination_state == COMBINATION_2){

//      // FE tail
//      FE_frame[6] = 0x13;
//      FE_frame[7] = 0x84;

//      // FF tail
//      FF_frame[6] = 0x29;
//      FF_frame[7] = 0x03;

//      combination_state = COMBINATION_1;
//    } else if (combination_state == COMBINATION_3) {

//      // FE tail
//			FE_frame[4] = 0x00;
//			FE_frame[5] = 0x00;
//      FE_frame[6] = 0x2C;
//      FE_frame[7] = 0x17;

//      // FF tail
//      FF_frame[6] = 0x01;
//      FF_frame[7] = 0x03;

//      combination_state = COMBINATION_1;
//    }
//  } else {
//    SS = rxBuffer[rxLen - 12];
//    MM = rxBuffer[rxLen - 11];
//    FE_frame[4] = SS;
//    FE_frame[5] = MM;

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
