#pragma once
#include <stdint.h>
struct Request {
  uint8_t kind; // 0 telemetry, 1 calibrate, 2 disarm
  uint32_t seq;
  float az,el;
  bool azValid,elValid;
};
struct State {
  uint32_t seq,lastRx;
  float cmdAz,cmdEl;
  bool hasSource,azKnown,elKnown,armed,stale;
  uint8_t fault; // 1 config, 2 uncalibrated, 3 unreachable, 4 queue
};
struct Ack { uint32_t seq; bool ok; uint8_t reason; };
extern QueueHandle_t requests, acknowledgements;
extern State sharedState;
extern uint32_t rxValid,rxInvalid,rxOverflow;
