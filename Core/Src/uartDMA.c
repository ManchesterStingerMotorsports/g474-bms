/*
 * uartDMA.c
 *
 *  Created on: Jan 25, 2025
 *      Author: amrlxyz
 */

#include "uartDMA.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "main.h"

#define uartHandle hlpuart1
#define BUFFER_SIZE 2048


char buffer[BUFFER_SIZE] = {0};
volatile int head = 0;
volatile int tail = 0;

bool isFull = false;
bool isWrapped = false;

int tailDma = 0; // Stores the tail index of the buffer being sent



int getDataLen(void)
{
    int tempTail = tail;

    if (head >= tempTail)
    {
        return head - tempTail; // Data is in a single contiguous block
    }
    else
    {
        return BUFFER_SIZE - tempTail + head; // Data wraps around the buffer
    }
}


void startUartDmaTx(void)
{
    // Get the data len up to the buffer size only
    int dataLen = isWrapped ? BUFFER_SIZE - tail : head - tail;

    // get pointer to the tail data
    uint8_t* dataPtr = (uint8_t*)(buffer + tail);

    tailDma = tail + dataLen;

    HAL_UART_Transmit_DMA(&uartHandle, dataPtr, dataLen);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
    if (huart == &uartHandle)
    {
        tail = tailDma;
        if (tail == BUFFER_SIZE)
        {
            tail = 0;
            isWrapped = false;
        }
        if (head > tail)
        {
            startUartDmaTx();
        }
    }
}


// Function to append formatted data to the ring buffer
int printfDma(const char *format, ...)
{
    const int TEMP_BUFF_SIZE = 256;

    char temp_buffer[TEMP_BUFF_SIZE];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(temp_buffer, TEMP_BUFF_SIZE, format, args);
    va_end(args);

    if (written < 0)
    {
        return -1; // Error in formatting
    }
    else if (written > TEMP_BUFF_SIZE)
    {
        // Over limit of temp buffer
        Error_Handler();
    }
    else if (getDataLen() + written > BUFFER_SIZE)
    {
        // Buffer full
        Error_Handler();
    }

    for (int i = 0; i < written; i++)
    {
        buffer[head++] = temp_buffer[i];

        if (head == BUFFER_SIZE)
        {
            isWrapped = true;
            head = 0;
        }

//        // Overflow handling (overwrite oldest data)
//        if (is_full)
//        {
//            tail = (tail + 1) % BUFFER_SIZE;
//        }

        if (head == tail)
        {
            Error_Handler();
        }
    }

    if (HAL_UART_GetState(&uartHandle) == HAL_UART_STATE_READY)
    {
        startUartDmaTx();
    }

    return written;
}
