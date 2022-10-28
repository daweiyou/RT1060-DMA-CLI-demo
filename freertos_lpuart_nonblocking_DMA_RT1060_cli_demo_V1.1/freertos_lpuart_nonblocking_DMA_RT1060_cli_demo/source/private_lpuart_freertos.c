/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "private_lpuart_freertos.h"
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>

#include "lpuart_freertos_cfg.h"

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


/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.lpuart_freertos"
#endif

lpuart_edma_handle_t g_lpuartEdmaHandle;
edma_handle_t g_lpuartTxEdmaHandle;
edma_handle_t g_lpuartRxEdmaHandle;
lpuart_transfer_t sendXfer;
lpuart_transfer_t receiveXfer;

uint32_t received_size = 0;
volatile bool reciveFrame                                          = false;
UART_STATE uart_state = UART_RX;

lpuart_rtos_handle_t* Local_handle = NULL;

/* LPUART user callback */
void LPUART_DMA_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);

#if (ENABLE_RTS_TRANSCEIVER == 0)
/*Switch RS485 direction pin*/
void Set_RS485_Receive()
{
	uart_state = UART_RX;
	GPIO_PinWrite(RTS_GPIO_PORT, RTS_GPIO_PIN, 0U);//transceiver direction control output low

}
void Set_RS485_Transimit()
{
	uart_state = UART_TX;
	GPIO_PinWrite(RTS_GPIO_PORT, RTS_GPIO_PIN, 1U);//transceiver direction control output high

}
#endif


void EDMA_AbortTransfer_bugfix(edma_handle_t *handle)
{
    handle->base->TCD[handle->channel].BITER_ELINKNO = 0;
    handle->base->TCD[handle->channel].CITER_ELINKNO = 0;
    handle->base->TCD[handle->channel].BITER_ELINKYES = 0;
    handle->base->TCD[handle->channel].CITER_ELINKYES = 0;
}
void LPUART_TransferAbortReceiveEDMA_bugfix(LPUART_Type *base, lpuart_edma_handle_t *handle)
{
    assert(NULL != handle);
    assert(NULL != handle->rxEdmaHandle);

    /* Stop transfer. */
    EDMA_AbortTransfer_bugfix(handle->rxEdmaHandle);
}
/*LPUART ISR handler, mainly handle TC and receive IDLE interrupt*/
void DEMO_LPUART_IRQHandler(void)
{
    uint8_t data;
	volatile uint32_t stat = LPUART_GetStatusFlags(DEMO_LPUART);
    uint32_t rx_count;

    BaseType_t xHigherPriorityTaskWoken, xResult;

    xHigherPriorityTaskWoken = pdFALSE;
    xResult                  = pdFAIL;

    
    /* If new data arrived. */
    if ((kLPUART_IdleLineFlag)&stat)
    {
        LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_IdleLineFlag);

        /*Fetch Data from DMA RX buffer*/
		if (kStatus_NoTransferInProgress ==
			LPUART_TransferGetReceiveCountEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &rx_count))
		{
			rx_count = 0;
		}
        if(rx_count > 0)
        {
            //memcpy(g_txBuffer,g_rxBuffer,rx_count);/*save received data*/
            received_size = rx_count;
		    reciveFrame = true;/*Assume receive entire frame*/
		    LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
			LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);
			xResult =
            xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_RX_COMPLETE, &xHigherPriorityTaskWoken);
        }

        /*
        Reset RX Buffer: Here call SDK API to implement reset
        Then prepare continue to receive new data
        */
//#if (ENABLE_RTS_TRANSCEIVER == 0)
//		Set_RS485_Receive();
//#endif
//		LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
//        LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);
//		LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);
    }

    else if((kLPUART_RxOverrunFlag)&stat)
    {
		LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_RxOverrunFlag);
		xResult =
            xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_HARDWARE_BUFFER_OVERRUN, &xHigherPriorityTaskWoken);
    }
    else if((kLPUART_NoiseErrorFlag)&stat)
    {
		LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_NoiseErrorFlag);
		xResult =
    	xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_HARDWARE_NOISE_ERROR, &xHigherPriorityTaskWoken);

    }
    else if((kLPUART_FramingErrorFlag)&stat)
    {
		LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_FramingErrorFlag);
		xResult =
    	xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_HARDWARE_FRAME_ERROR, &xHigherPriorityTaskWoken);

    }
    else if((kLPUART_ParityErrorFlag)&stat)
    {
		LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_ParityErrorFlag);
		xResult =
    	xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_HARDWARE_PARITY_ERROR, &xHigherPriorityTaskWoken);

    }

    /*if TC interrupt is enabled*/
	if(LPUART_GetEnabledInterrupts(DEMO_LPUART) & kLPUART_TransmissionCompleteFlag)
	{
		if((kLPUART_TransmissionCompleteFlag)&stat)/*TC interrupt flag is set*/
		{
			/*Since TC occurs, disable TC interrupt now*/
			LPUART_DisableInterrupts(DEMO_LPUART, kLPUART_TransmissionCompleteInterruptEnable);

			xResult = xEventGroupSetBitsFromISR(Local_handle->txEvent, RTOS_LPUART_TX_COMPLETE, &xHigherPriorityTaskWoken);

			/*Enable RS485 receive again*/
//#if (ENABLE_RTS_TRANSCEIVER == 0)
//			Set_RS485_Receive();
//#endif
//			LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
//            LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);
//			LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);
		
		}
	}

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif

}
/* 
LPUART user callback
Note: this is DMA interrupt callback, not LPUART interrupt callback
*/

void LPUART_DMA_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    xHigherPriorityTaskWoken = pdFALSE;
    xResult                  = pdFAIL;

    userData = userData;

    /*
      The status is from LPUART-DMA driver.
      DMA tramsmit is finished. Then recovery to receive mode, and reset DMA register and status
    */
    if (kStatus_LPUART_TxIdle == status)
    {
//#if (SWO_PRINTF == 1)
//        printf("DMA tx done, begin start receive\r\n");
//#endif
        /*TX DMA is finished, then enable TC interrupt, while TC int occurs, which indicate all data are sent out*/
		LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_TransmissionCompleteInterruptEnable);//ÉèÖÃTCÖÐ¶Ï
    }
    
	/*
	The status is from LPUART-DMA driver.
	RX buffer should be defined big enough, make sure DMA receive buffer never became full
	once this full condition occurs, should treat it as excepation
	*/
    if (kStatus_LPUART_RxIdle == status)
    {
//#if (SWO_PRINTF == 1)
//        printf("DMA rx full\r\n");
//#endif

        /*End user should carefully define DMA receive buffer length, avoid enter this state.
          Once such situation occurs for some possible bus error, end user should handler this situation by real case.
          in this demo, only skip existing buffer and restart DMA receiver again*/
	    LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
        LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);
	    //LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);

        xResult =
            xEventGroupSetBitsFromISR(Local_handle->rxEvent, RTOS_LPUART_RING_BUFFER_OVERRUN, &xHigherPriorityTaskWoken);
    }

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void initRTSPin()
{
#if (ENABLE_RTS_TRANSCEIVER == 1)
  IOMUXC_SetPinMux(
	IOMUXC_GPIO_AD_B0_15_LPUART1_RTS_B,		/* GPIO_AD_B0_15 is configured as LPUART1_RTS_B */
	0U);									 /* Software Input On Field: Input Path is determined by functionality */


  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_15_LPUART1_RTS_B,        /* GPIO_AD_B0_15 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */				
#else
  IOMUXC_SetPinMux(
    IOMUXC_GPIO_AD_B0_15_GPIO1_IO15,	  /* GPIO_AD_B0_15 is configured as LPUART1_RTS_B */
    0U);									   /* Software Input On Field: Input Path is determined by functionality */
  
  
  IOMUXC_SetPinConfig(
  	IOMUXC_GPIO_AD_B0_15_GPIO1_IO15, 	   /* GPIO_AD_B0_15 PAD functional properties : */
  	0x10B0u);								/* Slew Rate Field: Slow Slew Rate
  											   Drive Strength Field: R0/6
  											   Speed Field: medium(100MHz)
  											   Open Drain Enable Field: Open Drain Disabled
  											   Pull / Keep Enable Field: Pull/Keeper Enabled
  											   Pull / Keep Select Field: Keeper
  											   Pull Up / Down Config. Field: 100K Ohm Pull Down
  											   Hyst. Enable Field: Hysteresis Disabled */	  

#endif 
    
}


//static void LPUART_RTOS_Callback_Private(LPUART_Type *base, lpuart_handle_t *state, status_t status, void *param)
//{
//    lpuart_rtos_handle_t *handle = (lpuart_rtos_handle_t *)param;
//    BaseType_t xHigherPriorityTaskWoken, xResult;
//
//    xHigherPriorityTaskWoken = pdFALSE;
//    xResult                  = pdFAIL;
//
//    if (status == kStatus_LPUART_RxIdle)
//    {
//        xResult = xEventGroupSetBitsFromISR(handle->rxEvent, RTOS_LPUART_COMPLETE, &xHigherPriorityTaskWoken);
//    }
//    else if (status == kStatus_LPUART_TxIdle)
//    {
//        xResult = xEventGroupSetBitsFromISR(handle->txEvent, RTOS_LPUART_COMPLETE, &xHigherPriorityTaskWoken);
//    }
//    else if (status == kStatus_LPUART_RxRingBufferOverrun)
//    {
//        xResult =
//            xEventGroupSetBitsFromISR(handle->rxEvent, RTOS_LPUART_RING_BUFFER_OVERRUN, &xHigherPriorityTaskWoken);
//    }
//    else if (status == kStatus_LPUART_RxHardwareOverrun)
//    {
//        /* Clear Overrun flag (OR) in LPUART STAT register */
//        LPUART_ClearStatusFlags(base, kLPUART_RxOverrunFlag);
//        xResult =
//            xEventGroupSetBitsFromISR(handle->rxEvent, RTOS_LPUART_HARDWARE_BUFFER_OVERRUN, &xHigherPriorityTaskWoken);
//    }
//
//    if (xResult != pdFAIL)
//    {
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Init
 * Description   : Initializes the LPUART instance for application
 *
 *END**************************************************************************/
/*!
 * brief Initializes an LPUART instance for operation in RTOS.
 *
 * param handle The RTOS LPUART handle, the pointer to an allocated space for RTOS context.
 * param t_handle The pointer to an allocated space to store the transactional layer internal state.
 * param cfg The pointer to the parameters required to configure the LPUART after initialization.
 * return kStatus_Success, others failed
 */
int LPUART_RTOS_Init_Private(lpuart_rtos_handle_t *handle, lpuart_handle_t *t_handle, const lpuart_rtos_config_t *cfg)
{
	lpuart_config_t lpuartConfig;
	edma_config_t config;
	lpuart_transfer_t xfer;
#if (ENABLE_RTS_TRANSCEIVER == 0)
	gpio_pin_config_t rts_gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
#endif

    if (NULL == handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == t_handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg->base)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->srcclk)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->baudrate)
    {
        return kStatus_InvalidArgument;
    }

    Local_handle = handle;
    
    handle->base    = cfg->base;
    handle->t_state = t_handle;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    handle->txSemaphore = xSemaphoreCreateMutexStatic(&handle->txSemaphoreBuffer);
#else
    handle->txSemaphore = xSemaphoreCreateMutex();
#endif
    if (NULL == handle->txSemaphore)
    {
        return kStatus_Fail;
    }
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    handle->rxSemaphore = xSemaphoreCreateMutexStatic(&handle->rxSemaphoreBuffer);
#else
    handle->rxSemaphore = xSemaphoreCreateMutex();
#endif
    if (NULL == handle->rxSemaphore)
    {
        vSemaphoreDelete(handle->txSemaphore);
        return kStatus_Fail;
    }
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    handle->txEvent = xEventGroupCreateStatic(&handle->txEventBuffer);
#else
    handle->txEvent     = xEventGroupCreate();
#endif
    if (NULL == handle->txEvent)
    {
        vSemaphoreDelete(handle->rxSemaphore);
        vSemaphoreDelete(handle->txSemaphore);
        return kStatus_Fail;
    }
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    handle->rxEvent = xEventGroupCreateStatic(&handle->rxEventBuffer);
#else
    handle->rxEvent     = xEventGroupCreate();
#endif
    if (NULL == handle->rxEvent)
    {
        vEventGroupDelete(handle->txEvent);
        vSemaphoreDelete(handle->rxSemaphore);
        vSemaphoreDelete(handle->txSemaphore);
        return kStatus_Fail;
    }


	/* Initialize the LPUART. */
	/*
	 * lpuartConfig.baudRate_Bps = 115200U;
	 * lpuartConfig.parityMode = kLPUART_ParityDisabled;
	 * lpuartConfig.stopBitCount = kLPUART_OneStopBit;
	 * lpuartConfig.txFifoWatermark = 0;
	 * lpuartConfig.rxFifoWatermark = 0;
	 * lpuartConfig.enableTx = false;
	 * lpuartConfig.enableRx = false;
	 */
	LPUART_GetDefaultConfig(&lpuartConfig);
	lpuartConfig.baudRate_Bps = CLI_UART_BAUDRATE;
	lpuartConfig.enableTx	  = true;
	lpuartConfig.enableRx	  = true;
	/*set RX IDLE parameters*/
	lpuartConfig.rxIdleType = kLPUART_IdleTypeStopBit;
	lpuartConfig.rxIdleConfig = kLPUART_IdleCharacter16;

	/*set LPUART tx rx fifo, receive doesn't use FIFO*/
	lpuartConfig.rxFifoWatermark = 0;
	/*
		If doesn't use tramsmit fifo, set txFifoWatermark=0
		Must be care, the maximum fifo depth doesn't exceed allowed
	*/
	lpuartConfig.txFifoWatermark = FSL_FEATURE_LPUART_FIFO_SIZEn(DEMO_LPUART) - 1;

	LPUART_Init(DEMO_LPUART, &lpuartConfig, DEMO_LPUART_CLK_FREQ);


	//initialize direction control pin and function
#if (ENABLE_RTS_TRANSCEIVER == 0)
	GPIO_PinInit(RTS_GPIO_PORT, RTS_GPIO_PIN, &rts_gpio_config);
#else
	DEMO_LPUART->MODIR &= ~(LPUART_MODIR_TXRTSE(1) | LPUART_MODIR_TXRTSPOL(1));
	//enable transmitter RTS signal
	DEMO_LPUART->MODIR |= LPUART_MODIR_TXRTSE(1);
	//Transmitter RTS is active high.
	DEMO_LPUART->MODIR |= LPUART_MODIR_TXRTSPOL(1);
#endif


	/* Enable UART interrupt. */
	/*Enable LPUART initialized interrupt:
	  1.idle line
	  2.all error interrupt: if error interrupts are not handled properly, the LPUART will stop work
	*/
	LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_IdleLineInterruptEnable|
										 kLPUART_RxOverrunInterruptEnable|
										 kLPUART_NoiseErrorInterruptEnable|
										 kLPUART_FramingErrorInterruptEnable|
										 kLPUART_ParityErrorInterruptEnable);
										 /*kLPUART_TxFifoOverflowInterruptEnable|
										 kLPUART_RxFifoUnderflowInterruptEnable);*/
	/*Disable TC interrupt*/
	LPUART_DisableInterrupts(DEMO_LPUART, kLPUART_TransmissionCompleteInterruptEnable);
			
	EnableIRQ(DEMO_LPUART_IRQn);

#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
	/* Init DMAMUX */
	DMAMUX_Init(EXAMPLE_LPUART_DMAMUX_BASEADDR);
	/* Set channel for LPUART */
	DMAMUX_SetSource(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_TX_DMA_CHANNEL, LPUART_TX_DMA_REQUEST);
	DMAMUX_SetSource(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL, LPUART_RX_DMA_REQUEST);
	DMAMUX_EnableChannel(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_TX_DMA_CHANNEL);
	DMAMUX_EnableChannel(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL);
#endif
	/* Init the EDMA module */
	EDMA_GetDefaultConfig(&config);
	EDMA_Init(EXAMPLE_LPUART_DMA_BASEADDR, &config);
	EDMA_CreateHandle(&g_lpuartTxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_TX_DMA_CHANNEL);
	EDMA_CreateHandle(&g_lpuartRxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_RX_DMA_CHANNEL);

	/* Create LPUART DMA handle. */
	/*
		This demo disable UART RX FIFO
	*/
	/*
		For Callbacks, there are two levels callbacks:
		1st:LPUART_SendEDMACallback/LPUART_ReceiveEDMACallback, saved in instance g_lpuartRxEdmaHandle/g_lpuartTxEdmaHandle
		2nd:user defined callback: saved in g_lpuartEdmaHandle
	*/
	LPUART_TransferCreateHandleEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, LPUART_DMA_UserCallback, NULL, &g_lpuartTxEdmaHandle,
									&g_lpuartRxEdmaHandle);

//	sendXfer.data		 = g_txBuffer;
//	sendXfer.dataSize	 = UART_TX_BUFFER_LENGTH;
//	
//	receiveXfer.data	 = g_rxBuffer;
//	receiveXfer.dataSize = UART_RX_BUFFER_LENGTH;

    NVIC_SetPriority(DEMO_LPUART_IRQn, 5);
    NVIC_SetPriority(DMA0_DMA16_IRQn, 6);
    NVIC_SetPriority(DMA1_DMA17_IRQn, 6);
    
    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Deinit
 * Description   : Deinitializes the LPUART instance and frees resources
 *
 *END**************************************************************************/
/*!
 * brief Deinitializes an LPUART instance for operation.
 *
 * This function deinitializes the LPUART module, sets all register value to the reset value,
 * and releases the resources.
 *
 * param handle The RTOS LPUART handle.
 */
int LPUART_RTOS_Deinit_Private(lpuart_rtos_handle_t *handle)
{
    LPUART_Deinit(handle->base);

    vEventGroupDelete(handle->txEvent);
    vEventGroupDelete(handle->rxEvent);

    /* Give the semaphore. This is for functional safety */
    xSemaphoreGive(handle->txSemaphore);
    xSemaphoreGive(handle->rxSemaphore);

    vSemaphoreDelete(handle->txSemaphore);
    vSemaphoreDelete(handle->rxSemaphore);

    /* Invalidate the handle */
    handle->base    = NULL;
    handle->t_state = NULL;

    Local_handle = NULL;
    
    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Send
 * Description   : Initializes the UART instance for application
 *
 *END**************************************************************************/
/*!
 * brief Sends data in the background.
 *
 * This function sends data. It is an synchronous API.
 * If the hardware buffer is full, the task is in the blocked state.
 *
 * param handle The RTOS LPUART handle.
 * param buffer The pointer to buffer to send.
 * param length The number of bytes to send.
 */
int LPUART_RTOS_Send_Private(lpuart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length)
{
    EventBits_t ev;
    int retval = kStatus_Success;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        return kStatus_Success;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    if (pdFALSE == xSemaphoreTake(handle->txSemaphore, 0))
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

//    handle->txTransfer.data     = (uint8_t *)buffer;
//    handle->txTransfer.dataSize = (uint32_t)length;
//
//    /* Non-blocking call */
//    LPUART_TransferSendNonBlocking(handle->base, handle->t_state, &handle->txTransfer);

	sendXfer.data		 = (uint8_t *)buffer;
	sendXfer.dataSize	 = length;
	LPUART_SendEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &sendXfer);

    ev = xEventGroupWaitBits(handle->txEvent, RTOS_LPUART_TX_COMPLETE, pdTRUE, pdFALSE, portMAX_DELAY);
    if (!(ev & RTOS_LPUART_TX_COMPLETE))
    {
        retval = kStatus_Fail;
    }

    if (pdFALSE == xSemaphoreGive(handle->txSemaphore))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    return retval;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Recv
 * Description   : Receives chars for the application
 *
 *END**************************************************************************/
/*!
 * brief Receives data.
 *
 * This function receives data from LPUART. It is an synchronous API. If any data is immediately available
 * it is returned immediately and the number of bytes received.
 *
 * param handle The RTOS LPUART handle.
 * param buffer The pointer to buffer where to write received data.
 * param length The number of bytes to receive.
 * param received The pointer to a variable of size_t where the number of received data is filled.
 */
int LPUART_RTOS_Receive_Private(lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received)
{
    EventBits_t ev;
    size_t n              = 0;
    int retval            = kStatus_Fail;
    size_t local_received = 0;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        if (received != NULL)
        {
            *received = n;
        }
        return kStatus_Success;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    /* New transfer can be performed only after current one is finished */
    if (pdFALSE == xSemaphoreTake(handle->rxSemaphore, portMAX_DELAY))
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

//    handle->rxTransfer.data     = buffer;
//    handle->rxTransfer.dataSize = (uint32_t)length;
//
//    /* Non-blocking call */
//    LPUART_TransferReceiveNonBlocking(handle->base, handle->t_state, &handle->rxTransfer, &n);


	/*
	  Set to receive satus again
	  The receive will be termined by:
	  1.Receive DMA finished interrupt: this is be thought as error, because no such 2KB length frame actually, handle this error in EDMA_HandleIRQ()
	  2.UART Idle interrupt: the frame is received, get data from DMA rx buffer, and handle this frame
	*/
//#if (ENABLE_RTS_TRANSCEIVER == 0)
//	  Set_RS485_Receive();
//#endif
	receiveXfer.data	 = buffer;
	receiveXfer.dataSize = (uint32_t)length;//real length parameter may long enough, for example UART_RX_BUFFER_LENGTH

	LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
	LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);
	LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);
		

    ev = xEventGroupWaitBits(
        handle->rxEvent, RTOS_LPUART_RX_COMPLETE | RTOS_LPUART_RING_BUFFER_OVERRUN | RTOS_LPUART_HARDWARE_BUFFER_OVERRUN | RTOS_LPUART_HARDWARE_NOISE_ERROR | RTOS_LPUART_HARDWARE_FRAME_ERROR | RTOS_LPUART_HARDWARE_PARITY_ERROR,
        pdTRUE, pdFALSE, portMAX_DELAY);
    if (ev & (RTOS_LPUART_RING_BUFFER_OVERRUN | RTOS_LPUART_HARDWARE_BUFFER_OVERRUN | RTOS_LPUART_HARDWARE_NOISE_ERROR | RTOS_LPUART_HARDWARE_FRAME_ERROR | RTOS_LPUART_HARDWARE_PARITY_ERROR))
    {
        /* Stop data transfer to application buffer, ring buffer is still active */
        //LPUART_TransferAbortReceive(handle->base, handle->t_state);
        
	    LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
        LPUART_TransferAbortReceiveEDMA_bugfix(DEMO_LPUART, &g_lpuartEdmaHandle);

        
        /* Prevent false indication of successful transfer in next call of LPUART_RTOS_Receive.
           RTOS_LPUART_COMPLETE flag could be set meanwhile overrun is handled */
        xEventGroupClearBits(handle->rxEvent, RTOS_LPUART_RX_COMPLETE);
        retval         = kStatus_LPUART_Error;
        local_received = 0;
    }
//    else if (ev & RTOS_LPUART_RING_BUFFER_OVERRUN)
//    {
//        /* Stop data transfer to application buffer, ring buffer is still active */
//        LPUART_TransferAbortReceive(handle->base, handle->t_state);
//        /* Prevent false indication of successful transfer in next call of LPUART_RTOS_Receive.
//           RTOS_LPUART_COMPLETE flag could be set meanwhile overrun is handled */
//        xEventGroupClearBits(handle->rxEvent, RTOS_LPUART_COMPLETE);
//        retval         = kStatus_LPUART_RxRingBufferOverrun;
//        local_received = 0;
//    }
    else if (ev & RTOS_LPUART_RX_COMPLETE)
    {
        retval         = kStatus_Success;
        local_received = received_size;
    }

    /* Prevent repetitive NULL check */
    if (received != NULL)
    {
        *received = local_received;
    }

    /* Enable next transfer. Current one is finished */
    if (pdFALSE == xSemaphoreGive(handle->rxSemaphore))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }
    return retval;
}
