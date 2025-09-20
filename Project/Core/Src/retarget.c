#include "usart.h"
#include <stdio.h>

// This function replaces low-level putchar used by printf
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
