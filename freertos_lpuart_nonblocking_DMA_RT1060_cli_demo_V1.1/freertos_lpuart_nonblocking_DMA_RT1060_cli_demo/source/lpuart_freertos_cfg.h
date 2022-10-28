#ifndef _LPUART_FREERTOS_CFG_H_
#define _LPUART_FREERTOS_CFG_H_

#define DEMO_LPUART LPUART1
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn LPUART1_IRQn
#define DEMO_LPUART_IRQHandler LPUART1_IRQHandler

#define LPUART_TX_DMA_CHANNEL 0U
#define LPUART_RX_DMA_CHANNEL 1U
#define LPUART_TX_DMA_REQUEST kDmaRequestMuxLPUART1Tx
#define LPUART_RX_DMA_REQUEST kDmaRequestMuxLPUART1Rx
#define EXAMPLE_LPUART_DMAMUX_BASEADDR DMAMUX
#define EXAMPLE_LPUART_DMA_BASEADDR DMA0

#define UART_RX_BUFFER_LENGTH 2048
#define UART_TX_BUFFER_LENGTH 2048

#define CLI_UART_BAUDRATE (115200)

typedef enum{
    UART_RX,
    UART_TX,
}UART_STATE;
/*
ENABLE_RTS_TRANSCEIVER to control 
whether use GPIO_AD_B0_15 as LPUART1_RTS_B signal to drive direction of external transceiver

ENABLE_RTS_TRANSCEIVER = 1, use LPUART_RTS mode as signal
ENABLE_RTS_TRANSCEIVER = 0, use GPIO to drive extenal transceiver direction

*/
/*GPIO1_IO15 GPIO_AD_B0_15 ALT5*/
#define RTS_GPIO_PORT GPIO1
#define RTS_GPIO_PIN (15U)

#define xComPortHandle lpuart_rtos_handle_t //dawei add

#endif
