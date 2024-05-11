# KeyPad communication board 
## Software project v3

This project must give Codes of keys pushed in custom KeyPad

    Весь stdout перенаправлен в USART3, далее на Bluetooth модуль hc-06.
Журналы будут сыпаться туда.

    DWIN Дисплей подключаем к 5-Вольтовому разъёму USART5

## Переведено на FreeRTOS:
 * Сканирование клавиатуры,
 * Контрольная индикация дежурной лампы.
