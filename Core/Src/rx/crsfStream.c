#include "crsfStream.h"
#include "portmacro.h"
#include "projdefs.h"
#include <stddef.h>

extern UART_HandleTypeDef UART1_Handler;
uint8_t receiveCurrentCRSFValue = 0;
uint8_t BufferCRSFValue = 0;
StreamBufferHandle_t xStreamBuffer = NULL;
void vCRSFStreamTask(void)
{

  const size_t xStreamBufferSizeBytes = 1024, xTriggerLevel = 128 ;
  xStreamBuffer = xStreamBufferCreate(xStreamBufferSizeBytes, xTriggerLevel);
}

void vCRSFSendToBuffer(void *pvParameters)
{
  size_t xBytesSent;
  const TickType_t x10ms = pdMS_TO_TICKS(10);

  xBytesSent = xStreamBufferSend(xStreamBuffer, &receiveCurrentCRSFValue, 1, x10ms);
  if (xBytesSent != 1) {
      
  }
}

void ReceiveCRSFdataFromUart(void)
{
    HAL_UART_Receive_DMA(&UART1_Handler,&receiveCurrentCRSFValue,1);
    // crsfDataReceive(currentValue);
    // crsfFrameStatus();

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   xStreamBufferSendFromISR(xStreamBuffer, &receiveCurrentCRSFValue, 1, NULL);
}


