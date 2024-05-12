/******************************************************************************

Welcome to GDB Online.
GDB online is an online compiler and debugger tool for C, C++, Python, Java, PHP, Ruby, Perl,
C#, OCaml, VB, Swift, Pascal, Fortran, Haskell, Objective-C, Assembly, HTML, CSS, JS, SQLite, Prolog.
Code, Compile, Run and Debug online from anywhere in world.

*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "DWIN.h"

// variables
    // Идентификатор отображаемого пункта меню.
#define FIRST_MENU_PARAMETRE    MeasureParams           //начальный пункт: меню ввода параметров измерения
#define LAST_MENU_PARAMETRE     SettingsManufacture     //конечный пункт: меню настроек производителя
#define TOTAL_MENU_PARAMETRES   8                       //Общее кол-во пунктов меню
typedef enum {
        FIRST_MENU_PARAMETRE   = 0,    //Параметр взят из макроса: меню ввода параметров измерения
        MeasureHandle,      // меню измерений вручную
        MeasureAuto,        // меню измерений Автоматическое
        LogView,            // меню просмотра журнала
        DiagramView,        // меню отображения графиков измерений
        Settings,           // меню настроек
        // оставленные позиции для возможных пары пунктов меню
        SettingsAdvanced,    // меню инженерных настроек
        LAST_MENU_PARAMETRE  // Параметр взят из макроса: меню настроек производителя
    } MenuItem;
MenuItem CurrentMenuItem = FIRST_MENU_PARAMETRE;
//запрет на вход в Инженерные режимы
bool AccessAdvanced = false, AccessManufacture = false;
// матрица резрешенных переходов
const uint8_t MenuItemRouting [TOTAL_MENU_PARAMETRES][TOTAL_MENU_PARAMETRES] = {
//                           0  1  2  3  4  5  6  7  8
/*      MeasureParams 0*/  { 0, 1, 1, 1, 0, 1, 0, 0 },
/*      MeasureHandle 1*/  { 1, 0, 1, 1, 0, 0, 0, 0 },
/*      MeasureAuto   2*/  { 1, 1, 0, 1, 0, 0, 0, 0 },
/*      LogView       3*/  { 1, 1, 1, 0, 1, 1, 0, 0 },
/*      DiagramView   4*/  { 1, 0, 0, 1, 0, 0, 0, 0 },
/*      Settings      5*/  { 1, 0, 0, 1, 0, 0, 1, 1 },
/*   SettingsAdvanced 6*/  { 0, 0, 0, 0, 0, 1, 0, 0 },
/*SettingsManufacture 7*/  { 0, 0, 0, 0, 0, 1, 0, 0 },
};

// prototypes
void MenuItemSwitch (MenuItem newItem);
void ShowMenu (MenuItem Item);
void DWIN_Menu_Logic_InitProcedure (void);
bool MayChangeMenu (MenuItem newItem);
void DWIN_setPage(uint8_t page);
uint8_t DWIN_getPage();

int DWIN_menu_logic_main()
{
    int scannedMenuItem;
    
    DWIN_Menu_Logic_InitProcedure();
    
    for (int i=0;i<15;i++){
        printf ("try# %d Enter next number of menu item: ",i);
        scanf ("%d", &scannedMenuItem);
        MenuItemSwitch((MenuItem)scannedMenuItem);
    }
   
    return 0;
}

bool MayChangeMenu (MenuItem newItem){
    if ((FIRST_MENU_PARAMETRE<=newItem)&&(newItem<=LAST_MENU_PARAMETRE)&&   //проверка по граничным значениям
        (MenuItemRouting[CurrentMenuItem][newItem]==(uint8_t)true)){ // проверка по матрице переходов
        return true;
    }else{
        printf("Menu isn't changed.\n");
        return false;
    }
}

void DWIN_Menu_Logic_InitProcedure (void){
    CurrentMenuItem = MeasureParams;
    ShowMenu(CurrentMenuItem);
    DWIN_setPage(CurrentMenuItem);
}

void ShowMenu(MenuItem Item){
    printf (">> Show ");
    switch (Item){
        case MeasureParams:
            printf ("Set parameters of Measurements");
            break;
        case MeasureHandle:      // меню измерений вручную
            printf ("Handle mode measurements");
            break;
        case MeasureAuto:       // меню измерений Автоматическое
            printf ("Automatic mode measurements");
            break;
        case LogView:           // меню просмотра журнала
            printf ("View Logs");
            break;
        case DiagramView:        // меню отображения графиков измерений
            printf ("Diagram view");
            break;
        case Settings:           // меню настроек
            printf ("Settings");
            break;
        case SettingsAdvanced:    // меню инженерных настроек
            printf ("Advanced settings");
            break;
        case SettingsManufacture:   // меню настроек производителя
            printf ("Manufacturer settings");
            break;
        default:
            printf ("error: NOT defined");
    }
    printf(" menu.\n");
}

void MenuItemSwitch(MenuItem newItem){
    if (MayChangeMenu(newItem)){
        CurrentMenuItem=newItem;
        ShowMenu(CurrentMenuItem);
        DWIN_setPage(CurrentMenuItem);
        //printf(" switched.\n");
    }
}

// // DWIN functions section

// // Change Page 
// void DWIN_setPage(uint8_t page){
//     //5A A5 07 82 00 84 5a 01 00 02
//     uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, page};
//     HAL_UART_Transmit(&huart5, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
//     //readDWIN();
// }

// // Get Current Page ID
// uint8_t DWIN_getPage(){
//     uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x04, CMD_READ, 0x00 , 0x14, 0x01};
//     HAL_UART_Transmit(&huart5, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
//     //return readCMDLastByte();
//     return 0;
// }
