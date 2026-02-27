#include <string.h>
#include "cr95hf.h"

UART_HandleTypeDef huart1;   // CR95HF
extern UART_HandleTypeDef huart2;   // Debug

/* ================= GLOBALS ================= */

//static uint8_t uart1_rx_buffer[128];
static volatile uint8_t rxDone = 0;
static volatile uint32_t rxTick = 0;
static uint8_t rxLen = 0;

static uint8_t uid[10];
static uint8_t uidLen = 0;

static uint8_t state = 0;

/* USART1 */
static uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
static uint16_t uart1_rx_index = 0;
static uint16_t uart1_expected_len = 0;
static uint8_t uart1_frame_ready = 0;

static uint8_t rxByte;   // single byte buffer for IT


/* ================= FAST DEBUG PRINT ================= */

static void DBG_Print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
}

static void DBG_PrintHex(uint8_t *data, uint8_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    char out[3];

    for(uint8_t i = 0; i < len; i++)
    {
        out[0] = hex[(data[i] >> 4) & 0x0F];
        out[1] = hex[data[i] & 0x0F];
        out[2] = ' ';
        HAL_UART_Transmit(&huart2, (uint8_t*)out, 3, 100);
    }
    DBG_Print("\r\n");
}

/* ================= UART INIT ================= */

void CR95HF_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate   = 57600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;

    HAL_UART_Init(&huart1);
}

/* ================= RX CALLBACK ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t byte = rxByte;

        uart1_rx_buffer[uart1_rx_index++] = byte;

        if (uart1_rx_index >= UART1_RX_BUFFER_SIZE)
            uart1_rx_index = 0;

        // RESTART RX IMMEDIATELY (FIRST THING)
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)
//    {
//				/* ?? CLEAR OVERRUN ERROR FIRST */
//        if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
//        {
//            __HAL_UART_CLEAR_OREFLAG(&huart1);
//        }	
//			
//        uart1_rx_buffer[uart1_rx_index++] = rxByte;

//        if (uart1_rx_index >= rxLen)
//        {
//            rxDone = 1;
//						HAL_UART_Receive_IT(&huart1, &rxByte, 1);
//        }
//        else
//        {
//            HAL_UART_Receive_IT(&huart1, &rxByte, 1);
//        }
//    }
//}

/* ================= SEND ================= */

static void CR95HF_Send1(uint8_t *cmd, uint8_t txLen, uint8_t expectedRx)
{
    rxDone = 0;
    rxLen  = expectedRx;
    uart1_rx_index = 0;

    memset(uart1_rx_buffer, 0, sizeof(uart1_rx_buffer));

    HAL_UART_Transmit(&huart1, cmd, txLen, 100);

    HAL_UART_Receive_IT(&huart1, &rxByte, 1);

    rxTick = HAL_GetTick();
}


/* ================= TIMEOUT CHECK ================= */

static uint8_t WaitRx(uint32_t timeout)
{
    if (!rxDone)
    {
        if (HAL_GetTick() - rxTick > timeout)
        {
            state = 0;
        }
        return 0;
    }

		rxDone = 0;
//    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    return 1;
}

/* ================= STATE MACHINE ================= */

void CR95HF_Process1(void)
{
    switch(state)
    {
        case 0:     // ECHO
        {
            uint8_t cmd = 0x55;
            DBG_Print("STATE=ECHO\r\n");
						CR95HF_Send1(&cmd,1,1);
            state = 1;
            break;
        }

        case 1:
//            if(!WaitRx(1000)) break;
						WaitRx(1000);

            DBG_Print("ECHO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

            if(uart1_rx_buffer[0] != 0x55 && uart1_rx_buffer[0] != 0x80)
                DBG_Print("ECHO ERROR\r\n");

            state = 2;
            break;

        case 2:     // PROTOCOL
        {
            uint8_t cmd[] = {0x02,0x05,0x02,0x00,0x00,0x10,0x00};
            DBG_Print("STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
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

        case 4:     // RF ON
        {
            uint8_t cmd[] = {0x09,0x04,0x3A,0x00,0x58,0x04};
            DBG_Print("STATE=RFON\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
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

        case 6:     // REQA
        {
            uint8_t cmd[] = {0x04,0x02,0x26,0x07};
            DBG_Print("STATE=REQA\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),7);
						HAL_Delay(40); 
            state = 7;
            break;
        }

        case 7:
            //if(!WaitRx(1500)) break;
						WaitRx(1000);

            if(uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }

            DBG_Print("REQA RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

            state = 8;
            break;

        case 8:     // ANTICOLLISION CL1
        {
            uint8_t cmd[] = {0x04,0x03,0x93,0x20,0x08};
            DBG_Print("STATE=ANTICOLL CL1\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),10);
						HAL_Delay(40); 
            state = 9;
            break;
        }

        case 9:
//            if(!WaitRx(1500)) break;
						WaitRx(1000);

            DBG_Print("CL1 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

            if(rxLen >= 7)
            {
                uidLen = 4;
                memcpy(uid,&uart1_rx_buffer[3],4);
                DBG_Print("UID PART1: ");
                DBG_PrintHex(uid,4);
            }

            state = 10;
            break;

        case 10:    // SELECT CL2
        {
            uint8_t cmd[] = {0x04,0x08,0x93,0x70,
                             uart1_rx_buffer[2],uart1_rx_buffer[3],uart1_rx_buffer[4],uart1_rx_buffer[5],
                             uart1_rx_buffer[6],uart1_rx_buffer[7]};
            DBG_Print("STATE=SELECT CL1\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),8);
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

        case 12:     // ANTICOLLISION CL2
        {
            uint8_t cmd[] = {0x04,0x03,0x95,0x20,0x08};
            DBG_Print("STATE=ANTICOLL CL2\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),10);
						HAL_Delay(40); 
            state = 13;
            break;
        }

        case 13:
//            if(!WaitRx(1500)) break;
						WaitRx(1000);

            DBG_Print("CL2 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

            if(rxLen >= 7)
            {
                uidLen = 4;
                memcpy(uid,&uart1_rx_buffer[3],4);
                DBG_Print("UID PART2: ");
                DBG_PrintHex(uid,4);
            }

            state = 14;
            break;

        case 14:    // SELECT CL1
        {
            uint8_t cmd[] = {0x04,0x08,0x95,0x70,
                             uart1_rx_buffer[2],uart1_rx_buffer[3],uart1_rx_buffer[4],uart1_rx_buffer[5],
                             uart1_rx_buffer[6],uart1_rx_buffer[7]};
            DBG_Print("STATE=SELECT CL2\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),8);
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
				
				
				case 16:    // READ PAGE
        {
            uint8_t cmd[] = {0x04,0x04,0x3A,0x0D,0x1B,0x28};
            DBG_Print("STATE=READ PAGE\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),20);
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





void CR95HF_Process(void)
{
    switch(state)
    {
        case 0:     // ECHO
        {
            uint8_t cmd = 0x55;
            DBG_Print("0 STATE=ECHO\r\n");
						CR95HF_Send1(&cmd,1,1);
            state = 1;
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("ECHO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            if(uart1_rx_buffer[0] != 0x55 ){
                DBG_Print("ECHO ERROR\r\n");
								state = 0;
								break;
						}
            state = 1;
            break;
					}
        
				case 1:     // STATE RESET
        {
            uint8_t cmd[] = {0x04,0x03,0xC2,0xFF,0x28};
            DBG_Print("1 STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40);
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("PROTO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x87 ){
                DBG_Print("STATE RESET ERROR\r\n");
								state = 0;
								break;
						}
            state = 2;
            break;
						
				}	
				case 2:     // PROTOCOL SELECT
        {
            uint8_t cmd[] = {0x02,0x05,0x02,0x00,0x00,0x10,0x00};
            DBG_Print("2 STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40);
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("PROTO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x00 && uart1_rx_buffer[1] != 0x00 ){
                DBG_Print("PROTCOL SELECT ERROR\r\n");
								state = 0;
								break;
						}
            state = 3;
            break;

				}	
        case 3:     // REQA
        {
            uint8_t cmd[] = {0x04,0x02,0x26,0x07};
            DBG_Print("3 STATE=REQA\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),7);
						HAL_Delay(40); 
            //if(!WaitRx(1500)) break;
						WaitRx(1000);
						DBG_Print("REQA RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            if(uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 4;
            break;
				}	
        case 4:     // ANTICOLLISION CL1
        {
            uint8_t cmd[] = {0x04,0x03,0x93,0x20,0x08};
            DBG_Print("4 STATE=ANTICOLL CL1\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),10);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("ANTICOLLISION CL1 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);

            if(rxLen >= 7)
            {
                uidLen = 4;
                memcpy(uid,&uart1_rx_buffer[3],4);
                DBG_Print("UID PART1: ");
                DBG_PrintHex(uid,4);
//								if(uart1_rx_buffer[2] != 0x88)		// DO IT LATER. IF CASCADED TAG, THEN HAVE MORE 4 BYTES OF UID
//								{
//										state = 5;
//										break;
//								}else{
//									state = 5;
//								}
            }else {
								state = 0;
                break;
						}
				}	

        case 5:    // SELECT CL1
        {
            uint8_t cmd[] = {0x04,0x08,0x93,0x70,
                             uart1_rx_buffer[2],uart1_rx_buffer[3],uart1_rx_buffer[4],uart1_rx_buffer[5],
                             uart1_rx_buffer[6],uart1_rx_buffer[7]};
            DBG_Print("5 STATE=SELECT CL1\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),8);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("SEL CL1 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 6;
            break;
				}	
        case 6:     // ANTICOLLISION CL2
        {
            uint8_t cmd[] = {0x04,0x03,0x95,0x20,0x08};
            DBG_Print("6 STATE=ANTICOLL CL2\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),10);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("CL2 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
            if(rxLen >= 7)
            {
                uidLen = 4;
                memcpy(uid,&uart1_rx_buffer[3],4);
                DBG_Print("UID PART2: ");
                DBG_PrintHex(uid,4);
            }else {
								state = 0;
                break;
						}
            state = 7;
            break;
				}	
        case 7:    // SELECT CL2
        {
            uint8_t cmd[] = {0x04,0x08,0x95,0x70,
                             uart1_rx_buffer[2],uart1_rx_buffer[3],uart1_rx_buffer[4],uart1_rx_buffer[5],
                             uart1_rx_buffer[6],uart1_rx_buffer[7]};
            DBG_Print("7 STATE=SELECT CL2\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),8);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("SEL CL2 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 8;
            break;
				}					
				case 8:    // GET VERSION
        {
            uint8_t cmd[] = {0x04,0x02,0x60,0x28};
            DBG_Print("8 GET VERSION\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),15);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("GET VERSION RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 9;
            break;
				
				}					
				case 9:    // STATE RESET
        {
            uint8_t cmd[] = {0x04,0x03,0xC2,0xFF,0x28};
            DBG_Print("9 STATE RESET\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),6);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("STATE RESET RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 10;
            break;
				}							
				case 10:    // SECTOR SELECT
        {
            uint8_t cmd[] = {0x04,0x05,0x03,0x00,0x00,0x00,0x28};
            DBG_Print("10 SECTOR SELECT\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("SECTOR SELECT RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 11;
            break;
					}						
				case 11:    // PAGE READ
        {
            uint8_t cmd[] = {0x04,0x03,0x30,0xF8,0x28};
            DBG_Print("11 PAGE READ F8\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("PAGE READ F8 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 12;
						HAL_Delay(1000);
            break;
					}						
						
				case 12:     // STATE RESET
        {
            uint8_t cmd[] = {0x04,0x03,0xC2,0xFF,0x28};
            DBG_Print("12 STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40);
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("PROTO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 ){
                DBG_Print("STATE RESET ERROR\r\n");
								state = 0;
								break;
						}
            state = 13;
            break;	
				}	
				
				case 13:    // GAURD DELAY
        {
            uint8_t cmd[] = {0x04,0x05,0x00,0x00,0x00,0x00,0x28};
            DBG_Print("13 GAURD DELAY\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("GAURD DELAY RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 14;
            break;
					}	


				case 14:    // PAGE WRITE 
        {
            uint8_t cmd[] = {0x04,0x07,0xA2,0xFE,0x1E,0x06,0x13,0x84,0x28};
            DBG_Print("14 PAGE WRITE FE\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),6);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("PAGE WRITE FE RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 15;
            break;
				}						
				

				case 15:    // PAGE WRITE 
        {
            uint8_t cmd[] = {0x04,0x07,0xA2,0xFF,0xA5,0x5A,0x29,0x03,0x28};
            DBG_Print("15 PAGE WRITE FF\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),6);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("PAGE WRITE FF RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 16;
            break;
				}
	

				case 16:     // STATE RESET
        {
            uint8_t cmd[] = {0x04,0x03,0xC2,0xFF,0x28};
            DBG_Print("16 STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40);
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("PROTO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 ){
                DBG_Print("STATE RESET ERROR\r\n");
								state = 0;
								break;
						}
            state = 17;
            break;	
				}		

				case 17:    // SECTOR SELECT
        {
            uint8_t cmd[] = {0x04,0x05,0x03,0x00,0x00,0x00,0x28};
            DBG_Print("17 SECTOR SELECT\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("SECTOR SELECT RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 18;
            break;
					}						
				case 18:    // PAGE READ
        {
            uint8_t cmd[] = {0x04,0x03,0x30,0xF8,0x28};
            DBG_Print("18 PAGE READ F8\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("PAGE READ F8 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 19;
						HAL_Delay(1000);
            break;
					}		

				case 19:     // STATE RESET
        {
            uint8_t cmd[] = {0x04,0x03,0xC2,0xFF,0x28};
            DBG_Print("19 STATE=PROTOCOL\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40);
//            if(!WaitRx(1000)) break;
						WaitRx(1000);
            DBG_Print("PROTO RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x90 ){
                DBG_Print("STATE RESET ERROR\r\n");
								state = 0;
								break;
						}
            state = 20;
            break;	
				}
				
				case 20:    // GAURD DELAY
        {
            uint8_t cmd[] = {0x04,0x05,0x00,0x00,0x00,0x00,0x28};
            DBG_Print("20 GAURD DELAY\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("GAURD DELAY RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x87 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 21;
            break;
					}	

				case 21:    // PAGE READ
        {
            uint8_t cmd[] = {0x04,0x04,0x3A,0xF0,0xFF,0x28};
            DBG_Print("21 PAGE READ F0-FF\r\n");
            CR95HF_Send1(cmd,sizeof(cmd),2);
						HAL_Delay(40); 
//            if(!WaitRx(1500)) break;
						WaitRx(1000);
            DBG_Print("PAGE READ F8 RX: ");
            DBG_PrintHex(uart1_rx_buffer, uart1_rx_index);
						if(uart1_rx_buffer[0] != 0x80 && uart1_rx_buffer[0] != 0x80)
            {
                state = 0;
                break;
            }
            state = 11;
						HAL_Delay(1000);
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
            state = 0;
            break;
    }
}
