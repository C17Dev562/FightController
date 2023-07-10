#include "driver/uart.h"
#include "stm32f4xx_hal_cortex.h"

UART_HandleTypeDef UART1_Handler;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

void uart_init(void) {
  UART1_Handler.Instance = USART1;
  UART1_Handler.Init.BaudRate = 420000;
  UART1_Handler.Init.WordLength = UART_WORDLENGTH_8B;
  UART1_Handler.Init.StopBits = UART_STOPBITS_1;
  UART1_Handler.Init.Parity = UART_PARITY_NONE;
  UART1_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART1_Handler.Init.Mode = UART_MODE_TX_RX;
  UART1_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UART1_Handler);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_DISABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
      Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

  }
}

// void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
// {
//   if(huart->Instance==USART1)
//   {
//   /* USER CODE BEGIN USART1_MspDeInit 0 */

//   /* USER CODE END USART1_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART1_CLK_DISABLE();

//     /**USART1 GPIO Configuration
//     PA9     ------> USART1_TX
//     PA10     ------> USART1_RX
//     */
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

//     /* USART1 DMA DeInit */
//     HAL_DMA_DeInit(huart->hdmarx);
//     HAL_DMA_DeInit(huart->hdmatx);
//   /* USER CODE BEGIN USART1_MspDeInit 1 */

//   /* USER CODE END USART1_MspDeInit 1 */
//   }

// }
