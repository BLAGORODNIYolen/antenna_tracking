#pragma once
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <algorithm>
using std::min; using std::max; using std::abs;
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define OUTPUT 1
#define HIGH 1
#define LOW 0
extern uint32_t clockUs;
inline uint32_t micros(){return clockUs;}
inline uint32_t millis(){return clockUs/1000;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline void delayMicroseconds(int){}
struct SerialMock {
 int available(){return 0;} int read(){return -1;} int availableForWrite(){return 64;}
 size_t write(const uint8_t*,size_t n){return n;}
};
extern SerialMock Serial;
