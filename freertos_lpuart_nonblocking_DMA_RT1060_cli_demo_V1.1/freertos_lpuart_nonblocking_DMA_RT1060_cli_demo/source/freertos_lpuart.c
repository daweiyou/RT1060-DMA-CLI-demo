/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

//#include "fsl_lpuart_freertos.h"
#include "private_lpuart_freertos.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "lpuart_freertos_cfg.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 2)
#define uart_operation_task_PRIORITY (configMAX_PRIORITIES - 1)
#define cli_task_PRIORITY (configMAX_PRIORITIES - 1)

SemaphoreHandle_t xMutex_uart_wr;
SemaphoreHandle_t xMutex_uart_rd;

#define NORMAL_STRING (1 << 0)
#define HARDWARE_ERROR (1 << 1)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern xComPortHandle* xSerialPortInitMinimal(unsigned int baudrate, unsigned int queuelen);
/*******************************************************************************
 * Code
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_txBuffer[UART_TX_BUFFER_LENGTH]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rxBuffer[UART_RX_BUFFER_LENGTH]) = {0};

static EventGroupHandle_t event_group = NULL;

const char *to_send               = "FreeRTOS LPUART driver example!\r\n";
const char *send_hardware_error = "\r\nHardware error occurs!\r\n";

//const char *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
//const char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";

uint8_t background_buffer[32];
//uint8_t recv_buffer[4];
uint32_t recv_buffer_len;
lpuart_rtos_handle_t lpuart_rtos_handle;
struct _lpuart_handle t_handle;

lpuart_rtos_config_t lpuart_config = {
    .baudrate    = 115200,
    .parity      = kLPUART_ParityDisabled,
    .stopbits    = kLPUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

//return lpuart_rtos_handle, if return 0, fail init
xComPortHandle* xSerialPortInitMinimal(unsigned int baudrate, unsigned int queuelen)
{
	xComPortHandle* port = NULL;

	lpuart_config.srcclk = DEMO_LPUART_CLK_FREQ;
	lpuart_config.base	 = DEMO_LPUART;
	lpuart_config.baudrate = baudrate;

	if (kStatus_Success != LPUART_RTOS_Init_Private(&lpuart_rtos_handle, &t_handle, &lpuart_config))
	{
        port = NULL;
	}
	else
	{
        port = &lpuart_rtos_handle;
	}

	return port;
}

void vSerialPutString(xComPortHandle* pxPort, uint8_t *tx_buffer, short tx_buffer_len)
{
    short tx_len = 0;
    
    tx_len = (tx_buffer_len < UART_TX_BUFFER_LENGTH)?tx_buffer_len:UART_TX_BUFFER_LENGTH;
        
    //use non cache buffer for DMA operation
    memcpy(g_txBuffer,tx_buffer,tx_len);
	if (kStatus_Success != LPUART_RTOS_Send_Private(pxPort, g_txBuffer, tx_len))
	{
		;
	}
}

BaseType_t xSerialGetChar(xComPortHandle* pxPort, uint8_t* p_rx_char, unsigned int delay)
{
	int error;
	size_t n = 0;

	error = LPUART_RTOS_Receive_Private(pxPort, g_rxBuffer, UART_RX_BUFFER_LENGTH, &n);
	if (error == kStatus_LPUART_Error)
	{
		return pdFAIL;
	}

    assert(n>0);
    
    if(n>0)
	    *p_rx_char = g_rxBuffer[0];

	return pdPASS;
}

void xSerialPutChar( xComPortHandle* pxPort, uint8_t tx_char, unsigned int delay )
{
    //use non cache buffer for DMA operation
    g_txBuffer[0] = tx_char;
	if (kStatus_Success != LPUART_RTOS_Send_Private(pxPort, g_txBuffer, 1))
	{
		;
	}
}


static void uart_tx_task(void *pvParameters)
{

	int error;
    EventBits_t event_bits;

	if (xSemaphoreTake(xMutex_uart_wr, portMAX_DELAY) != pdTRUE)
	{
		PRINTF("Failed to take xMutex_uart_wr semaphore.\r\n");
	}

	do
	{
	
		event_bits = xEventGroupWaitBits(event_group,	 /* The event group handle. */
										 NORMAL_STRING | HARDWARE_ERROR,		 /* The bit pattern the event group is waiting for. */
										 pdTRUE,		 /* BIT_0 and BIT_4 will be cleared automatically. */
										 pdFALSE,		 /* Don't wait for both bits, either bit unblock task. */
										 portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */
		

		if ((event_bits & NORMAL_STRING) == NORMAL_STRING)
		{
			/* send back the received data */
			if (kStatus_Success != LPUART_RTOS_Send_Private(&lpuart_rtos_handle, (uint8_t *)g_txBuffer, recv_buffer_len))
			{
				break;
			}
		}
		else if ((event_bits & HARDWARE_ERROR) == HARDWARE_ERROR)
		{
			/* Notify about hardware buffer overrun */
			LPUART_RTOS_Send_Private(&lpuart_rtos_handle, (uint8_t *)send_hardware_error, strlen(send_hardware_error));
		}
	}while(event_bits & NORMAL_STRING);

	xSemaphoreGive(xMutex_uart_wr);
	vTaskSuspend(NULL);
}
static void uart_rx_task(void *pvParameters)
{
	int error;
	size_t n = 0;

	if (xSemaphoreTake(xMutex_uart_rd, portMAX_DELAY) != pdTRUE)
	{
		PRINTF("Failed to take xMutex_uart_wr semaphore.\r\n");
	}

	/* Receive user input and send it back to terminal. */
	do
	{
		error = LPUART_RTOS_Receive_Private(&lpuart_rtos_handle, g_rxBuffer, UART_RX_BUFFER_LENGTH, &n);
		if (error == kStatus_LPUART_Error)
		{
			xEventGroupSetBits(event_group, HARDWARE_ERROR);
		}
		if (error == kStatus_Success)
		{
		    recv_buffer_len = n;
		    memcpy(g_txBuffer,g_rxBuffer,recv_buffer_len);
			xEventGroupSetBits(event_group, NORMAL_STRING);
		}
	} while (kStatus_Success == error);

	xSemaphoreGive(xMutex_uart_rd);
	vTaskSuspend(NULL);
}


static void uart_init_task(void *pvParameters)
{
	int error;
	size_t n = 0;

    xMutex_uart_wr = xSemaphoreCreateMutex();
    xMutex_uart_rd = xSemaphoreCreateMutex();
	event_group = xEventGroupCreate();

	lpuart_config.srcclk = DEMO_LPUART_CLK_FREQ;
	lpuart_config.base	 = DEMO_LPUART;

	if (kStatus_Success != LPUART_RTOS_Init_Private(&lpuart_rtos_handle, &t_handle, &lpuart_config))
	{
		vTaskSuspend(NULL);
	}
	
	/* Send introduction message. */
	if (kStatus_Success != LPUART_RTOS_Send_Private(&lpuart_rtos_handle, (uint8_t *)to_send, strlen(to_send)))
	{
		vTaskSuspend(NULL);
	}

    if (xTaskCreate(uart_tx_task, "uart_tx_task", configMINIMAL_STACK_SIZE + 100, NULL, uart_operation_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(uart_rx_task, "uart_rx_task", configMINIMAL_STACK_SIZE + 100, NULL, uart_operation_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

	if (xSemaphoreTake(xMutex_uart_wr, portMAX_DELAY) != pdTRUE)
	{
		PRINTF("Failed to take xMutex_uart_wr semaphore.\r\n");
	}
	if (xSemaphoreTake(xMutex_uart_rd, portMAX_DELAY) != pdTRUE)
	{
		PRINTF("Failed to take xMutex_uart_rd semaphore.\r\n");
	}

	PRINTF("Read to exit\r\n");

	LPUART_RTOS_Deinit_Private(&lpuart_rtos_handle);
	vTaskSuspend(NULL);
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    lpuart_config_t lpuartConfig;
    edma_config_t config;
    lpuart_transfer_t xfer;
#if (ENABLE_RTS_TRANSCEIVER == 0)
    gpio_pin_config_t rts_gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
#endif
    
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    initRTSPin();
    BOARD_InitBootClocks();

    
	vUARTCommandConsoleStart(configMINIMAL_STACK_SIZE + 500, cli_task_PRIORITY);
	/* Register commands with the FreeRTOS+CLI command interpreter. */
	vRegisterCLICommands();

    
//    if (xTaskCreate(uart_init_task, "uart_init_task", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
//    {
//        PRINTF("Task creation failed!.\r\n");
//        while (1)
//            ;
//    }
    vTaskStartScheduler();
    for (;;)
        ;
}

