#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "config.h"
#include "types.h"
#include "protocol_uart.h"
#include "antenna_controller.h"
QueueHandle_t requests,acknowledgements;
State sharedState={};
uint32_t rxValid=0,rxInvalid=0,rxOverflow=0;
void setup() {
  Serial.begin(115200);
  sharedState.stale=true;
  requests=xQueueCreate(8,sizeof(Request)); acknowledgements=xQueueCreate(4,sizeof(Ack));
  if(!requests || !acknowledgements) for(;;) {}
  if(xTaskCreate(controlTask,"motor",640,nullptr,1,nullptr)!=pdPASS) for(;;) {}
  if(xTaskCreate(uartTask,"uart",1024,nullptr,1,nullptr)!=pdPASS) for(;;) {}
}
void loop() {}
