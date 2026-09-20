#pragma once
#include <cstdint>
extern uint32_t test_now;
inline uint32_t millis(){return test_now;}
inline uint32_t micros(){return test_now*1000;}
inline void delay(uint32_t n){test_now+=n;}
inline void delayMicroseconds(uint32_t){}
constexpr int INPUT_PULLUP=1, OUTPUT_OPEN_DRAIN=2, LOW=0, HIGH=1;
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return HIGH;}
struct ESPMock {void restart(){}};
inline ESPMock ESP;
