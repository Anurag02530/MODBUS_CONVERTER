Code By: Anurag
Date: 22/04/26
Modbus address support has been added in this version, along with the ability to change the device address at runtime.

Currently, the Modbus address can be modified using:
Function Code: 0x06 (Write Single Register)
Register Address: 0x0001

This implementation is temporary and will be updated in the next version to align with a proper Modbus register map and standard configuration practices.
02 06 00 01 00 01 F9 19
where:
02 is slave address 
06 is function code
00 01 is register address 
00 01 is new address 


Dated: 14/04/2026
THIS CODE STATE MACHINE WORKING PERFECTLY FOR CR95HF NFC READER WITH THE STM32L062KBT6 
CONNECTIONS ARE :
USART1 AF0 TX PB6 <-----> TX CR95HF 
	   RX PB7 <-----> RX CR95HF  

USART2 AF4 TX PA2 <-----> RX MAX3485 DI PIN 1 
	   RX PA3 <-----> TX MAX3485 DO PIN 4

This Code also have the CRC function and working very well with the meters.

additional code is written for the error of not working in some meters and sowing the all values 00 initial command are also not working in these type of meters (01 03).For this issue did the changes in the Guard Delay command (04 05 00 00 00 00 28 or 04 05 01 00 00 00 28 exchange between these guard delay commands in the case 13,20,30,37)

