/*
* DWIN DGUS DWIN Library for STM32 
* @author Alexandr Ribtsov 
* light Remake of
* Library Supports all Basic Function
* Created by Tejeet ( tejeet@dwin.com.cn ) 
* Please Checkout Latest Offerings FROM DWIN 
* Here : https://www.dwin-global.com/
*/
// DWIN.h
#ifndef DWIN_H // include guard
#define DWIN_H

//#include <stdio.h>
//#include <stdint.h>
#include "stm32f4xx_hal.h"

#define DWIN_DEFAULT_BAUD_RATE      115200
#define ARDUINO_RX_PIN              10
#define ARDUINO_TX_PIN              11

#define REQUEST_FRAME_BUFFER 7
#define WRITE_FRAME_BUFFER 8
#define READ_FRAME_BUFFER_DEFAULT 9


#define CMD_HEAD1           0x5A
#define CMD_HEAD2           0xA5
#define CMD_WRITE           0x82
#define CMD_READ            0x83

#define MIN_ASCII           32
#define MAX_ASCII           255

#define CMD_READ_TIMEOUT    50
#define CMD_SEND_TIMEOUT    50
#define READ_TIMEOUT        100


/*
// set Current Page ID
void DWIN_setPage(UART_HandleTypeDef *huart, uint8_t pageID);
// get Current Page ID
uint8_t DWIN_getPage();
*/
void DWIN_setPage(uint8_t pageID);
uint8_t DWIN_getPage();


#endif /* DWIN_H */
