#pragma once
#include "Arduino.h"
void yieldTest();
#define taskYIELD() yieldTest()
#define taskENTER_CRITICAL() do{}while(0)
#define taskEXIT_CRITICAL() do{}while(0)
#define pdPASS 1
