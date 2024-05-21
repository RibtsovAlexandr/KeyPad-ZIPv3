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
#include "DWIN_menu_logic.h"

// variables

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

// // prototypes
// void MenuItemSwitch (MenuItem newItem);
// void ShowMenu (MenuItem Item);
// void DWIN_Menu_Logic_InitProcedure (void);
// bool MayChangeMenu (MenuItem newItem);
// void DWIN_setPage(uint8_t page);
// uint8_t DWIN_getPage();

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

