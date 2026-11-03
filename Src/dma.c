#include "dma.h"

/* CubeMX DMA request: USART3_RX, DMA1 Stream1 Channel4. */
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* The DBUS receiver uses USART IDLE, not DMA transfer interrupts. */
}
