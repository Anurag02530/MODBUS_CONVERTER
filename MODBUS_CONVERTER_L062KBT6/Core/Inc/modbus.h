/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus.h
  * @brief          : Header for modbus.c file.
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

#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);

#endif
