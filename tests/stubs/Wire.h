#pragma once
#include "Arduino.h"
struct WireMock {
 void end(){} bool begin(int,int){return true;} void setTimeOut(int){} void setClock(int){}
 void beginTransmission(int){} int endTransmission(){return 0;}
};
inline WireMock Wire;
