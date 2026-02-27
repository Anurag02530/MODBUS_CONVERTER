
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cr95hf.h"
#include "uart.h"
#include "modbus.h"
#include <stdio.h>



void SystemClock_Config(void);
static void MX_GPIO_Init(void);

//extern UART_HandleTypeDef huart2;

/* printf redirect */
//int fputc(int ch, FILE *f)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
//    return ch;
//}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
   // MX_USART1_UART_Init();
		CR95HF_UART_Init();
    MX_USART2_UART_Init();
	
		HAL_Delay(100);

 //   UART_StartReception();

    /* Example send */
//    uint8_t tx_data[] = {0x01,0x02,0x03,0x04};
//    UART1_Send(tx_data, sizeof(tx_data));
	
		/* ================= CR95HF INIT SEQUENCE ================= */
	
	while(1){
		CR95HF_Process();
	}

//		if(!CR95HF_Echo())
//		{
//				printf("CR95HF ECHO FAILED\r\n");
//		}
//		else
//		{
//				printf("CR95HF ECHO OK\r\n");

//				if(CR95HF_SelectProtocol_ISO14443A())
//						printf("Protocol Set OK\r\n");

////				if(CR95HF_RF_On())
////						printf("RF Field ON\r\n");
//		}

		
//		if(!CR95HF_Echo())
//    {
//        printf("Echo Failed\r\n");
//        while(1);
//    }

//    CR95HF_SelectProtocol_ISO14443A();
//    CR95HF_RF_ON();
//		CR95HF_REQA();
//		CR95HF_Anticollision(0);
		
		

//    while (1)
//    {
//			
//				CR95HF_Process();
//			
//				if(CR95HF_GetUID())
//        {
//            printf("Card Detected: ");

//            for(uint8_t i=0;i<cr95hf_uid_length;i++)
//                printf("%02X ", cr95hf_full_uid[i]);

//            printf("\r\n");

//            HAL_Delay(1000);
//        }
//        /* USART1 */
//        if (UART1_IsFrameReceived())
//        {
//            uint8_t *buf = UART1_GetBuffer();
//            uint16_t len = UART1_GetLength();

//            printf("USART1 RX: ");
//            for (int i = 0; i < len; i++)
//                printf("%02X ", buf[i]);
//            printf("\r\n");
//        }

//        /* USART2 (Modbus) */
//        if (UART2_IsFrameReceived())
//        {
//            uint8_t *buf = UART2_GetBuffer();

//            printf("USART2 RX: ");
//            for (int i = 0; i < 8; i++)
//                printf("%02X ", buf[i]);
//            printf("\r\n");
//        }
//    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_Pin_RS485_GPIO_Port, DE_Pin_RS485_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DE_Pin_RS485_Pin */
  GPIO_InitStruct.Pin = DE_Pin_RS485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_Pin_RS485_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
