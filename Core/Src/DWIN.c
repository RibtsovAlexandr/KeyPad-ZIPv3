/******************************************************************************
* DWIN Display Library for my STM32 project
* @author Alexandr Ribtsov 
* light Remake of
* Library Supports all Basic Function
* Created by Tejeet ( tejeet@dwin.com.cn ) 
* Here : https://www.dwin-global.com/
*******************************************************************************/
#include "DWIN.h"

/*
// Restart DWIN HMI
void DWIN::restartHMI(){  // HEX(5A A5 07 82 00 04 55 aa 5a a5 )
    byte sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x04, 0x55, 0xaa, CMD_HEAD1, CMD_HEAD2};
    _dwinSerial->write(sendBuffer, sizeof(sendBuffer)); 
    delay(10);
    readDWIN();
}
*/

// Change Page 
void DWIN_setPage(UART_HandleTypeDef *huart, uint8_t pageID){
    //5A A5 07 82 00 84 5a 01 00 02
    uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, pageID};
    HAL_UART_Transmit(huart, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
}


// Get Current Page ID
uint8_t DWIN_getPage(UART_HandleTypeDef *huart){
    uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x04, CMD_READ, 0x00 , 0x14, 0x01};
    //_dwinSerial->write(sendBuffer, sizeof(sendBuffer)); 
    HAL_UART_Transmit(huart, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
    //return readCMDLastByte();
    return 0;
}
