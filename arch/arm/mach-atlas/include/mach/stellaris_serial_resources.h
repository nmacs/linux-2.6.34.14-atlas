#ifndef MACH_STELLARIS_SERIAL_RESOURCES_H
#define MACH_STELLARIS_SERIAL_RESOURCES_H

#include <mach/pins.h>

#ifdef CONFIG_STELLARIS_DMA
#define UART0_DMA_BUFFER_SIZE 256
#define UART1_DMA_BUFFER_SIZE 256
#define UART2_DMA_BUFFER_SIZE 256

char __sramdata uart0_dma_tx_buffer[UART0_DMA_BUFFER_SIZE];
char __sramdata uart0_dma_rx_buffer[UART0_DMA_BUFFER_SIZE];

char __sramdata uart1_dma_tx_buffer[UART1_DMA_BUFFER_SIZE];
char __sramdata uart1_dma_rx_buffer[UART1_DMA_BUFFER_SIZE];

char __sramdata uart2_dma_tx_buffer[UART2_DMA_BUFFER_SIZE];
char __sramdata uart2_dma_rx_buffer[UART2_DMA_BUFFER_SIZE];
#endif

static struct stellaris_platform_uart stellaris_uarts[] = {
  {
    .mapbase    = STLR_UART0_BASE,
    .irq        = STLR_UART0_IRQ,
    .uart_index = 0,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART0_RX,
		.dma_tx_channel = DMA_CHANNEL_UART0_TX,
		.dma_tx_buffer = uart0_dma_tx_buffer,
		.dma_rx_buffer = uart0_dma_rx_buffer,
		.dma_buffer_size = UART0_DMA_BUFFER_SIZE,
#endif
		.dtr_gpio = GPIO_UART0_DTR,
		.rts_gpio = GPIO_UART0_RTS,
		.flags = STLR_UART_HAS_RTS | STLR_UART_HAS_CTS | STLR_UART_HAS_DTR |
		         STLR_UART_INVERT_RTS | STLR_UART_INVERT_DTR,
  },
  {
    .mapbase    = STLR_UART1_BASE,
    .irq        = STLR_UART1_IRQ,
    .uart_index = 1,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART1_RX,
		.dma_tx_channel = DMA_CHANNEL_UART1_TX,
		.dma_tx_buffer = uart1_dma_tx_buffer,
		.dma_rx_buffer = uart1_dma_rx_buffer,
		.dma_buffer_size = UART1_DMA_BUFFER_SIZE,
#endif
  },
  {
    .mapbase    = STLR_UART2_BASE,
    .irq        = STLR_UART2_IRQ,
    .uart_index = 2,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART2_RX,
		.dma_tx_channel = DMA_CHANNEL_UART2_TX,
		.dma_tx_buffer = uart2_dma_tx_buffer,
		.dma_rx_buffer = uart2_dma_rx_buffer,
		.dma_buffer_size = UART2_DMA_BUFFER_SIZE,
#endif
  },
	{
    .mapbase    = STLR_UART6_BASE,
    .irq        = STLR_UART6_IRQ,
    .uart_index = 6,
	}
};

#define STLR_NACTIVEUARTS ARRAY_SIZE(stellaris_uarts)

#endif /* MACH_STELLARIS_SERIAL_RESOURCES_H */