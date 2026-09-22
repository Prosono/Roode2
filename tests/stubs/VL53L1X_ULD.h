#pragma once
#include <cstdint>
using VL53L1_Error=int;
constexpr int VL53L1_ERROR_NONE=0, VL53L1_ERROR_TIME_OUT=-7, VL53L1_ERROR_CONTROL_INTERFACE=-1;
constexpr int VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND=8;
constexpr int VL53L1_RESULT__OSC_CALIBRATE_VAL=0xDE, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD=0x6C;
enum EDistanceMode {Short=1,Long=2};
enum ERangeStatus {RangeValid=0,SigmaFail,SignalFail,MinRangeFail,PhaseOutOfLimit,HardwareFail,RangeValidNoWrapCheck,WrapTargetFail};
struct VL53L1X_Result_t {uint16_t Distance{2000},SigPerSPAD{100},Ambient{0},NumSPADs{20}; uint8_t Status{0};};
inline VL53L1X_Result_t mock_result{};
inline bool mock_ready=true;
inline int mock_error=0;
inline unsigned mock_calls=0;
inline uint32_t mock_guard_ticks=0;
inline int VL53L1_WrByte(int,int,int){++mock_calls;return mock_error;}
inline int VL53L1_RdWord(int,int,uint16_t *v){*v=1000;return mock_error;}
inline int VL53L1_WrDWord(int,int,uint32_t v){mock_guard_ticks=v;return mock_error;}
inline int VL53L1_RdDWord(int,int,uint32_t *v){*v=mock_guard_ticks;return mock_error;}
struct VL53L1X_ULD {
 uint8_t address=0x52;
 int GetBootState(uint8_t *v){*v=mock_ready;return mock_error;}
 int GetI2CAddress(){return address;}
 int SetI2CAddress(uint8_t v){address=v;return mock_error;}
 int SetROI(int,int){return mock_error;} int SetROICenter(int){return mock_error;}
 int SetDistanceMode(EDistanceMode){return mock_error;} int SetTimingBudgetInMs(int){return mock_error;}
 int SetInterMeasurementInMs(int){return mock_error;}
 int StartRanging(){return mock_error;} int StopRanging(){return mock_error;}
 int CheckForDataReady(uint8_t *v){*v=mock_ready;return mock_error;}
 int ClearInterrupt(){return mock_error;} int GetResult(VL53L1X_Result_t *v){*v=mock_result;return mock_error;}
};
