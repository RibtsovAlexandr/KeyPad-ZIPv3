/******************************************************************************
* DWIN Menu logic Library for my STM32 project
* @author Alexandr Ribtsov 
*******************************************************************************/

// DWIN_menu_logic.h
#if !defined DWIN_MENU_LOGIC_H // include guard
#define DWIN_MENU_LOGIC_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "DWIN.h"

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

// prototypes
void MenuItemSwitch (MenuItem newItem);
void ShowMenu (MenuItem Item);
void DWIN_Menu_Logic_InitProcedure (void);
bool MayChangeMenu (MenuItem newItem);
//void DWIN_setPage(uint8_t page);
uint8_t DWIN_getPage();

// Using DWIN display attached at UART5
extern UART_HandleTypeDef huart5;

#endif /* DWIN_MENU_LOGIC_H */