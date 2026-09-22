#pragma once
#include <cstdint>
#include <string>
#include <vector>
using VL53L1_Error = int;
constexpr int VL53L1_ERROR_NONE = 0, VL53L1_ERROR_TIME_OUT = -7, VL53L1_ERROR_CONTROL_INTERFACE = -1;
constexpr int VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 8;
constexpr int VL53L1_RESULT__OSC_CALIBRATE_VAL = 0xDE;
constexpr int VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD = 0x6C;
enum EDistanceMode { Short = 1, Long = 2 };
enum ERangeStatus { RangeValid = 0, SigmaFail, SignalFail, MinRangeFail, PhaseOutOfLimit,
                    HardwareFail, RangeValidNoWrapCheck, WrapTargetFail };
struct VL53L1X_Result_t {
  uint16_t Distance{700}, SigPerSPAD{100}, Ambient{0}, NumSPADs{20};
  uint8_t Status{0};
};
inline VL53L1X_Result_t mock_result{};
inline bool mock_ready = true;
inline std::string mock_failed_operation;
inline std::string mock_delayed_operation;
inline uint32_t mock_delay_ms = 0;
inline uint16_t mock_clock_pll = 1000;
inline uint32_t mock_guard_ticks = 0;
inline bool mock_corrupt_guard_readback = false;
extern uint32_t test_now;
inline std::vector<std::string> mock_operations;
inline int mock_operation(const std::string &name) {
  mock_operations.push_back(name);
  if (name == mock_delayed_operation) test_now += mock_delay_ms;
  return name == mock_failed_operation ? VL53L1_ERROR_CONTROL_INTERFACE : 0;
}
inline int VL53L1_WrByte(int, int, int) { return mock_operation("write"); }
inline int VL53L1_RdWord(int, int, uint16_t *v) {
  *v = mock_clock_pll; return mock_operation("clock_read");
}
inline int VL53L1_WrDWord(int, int, uint32_t value) {
  const int result = mock_operation("guard_write");
  if (!result) mock_guard_ticks = value;
  return result;
}
inline int VL53L1_RdDWord(int, int, uint32_t *v) {
  *v = mock_guard_ticks ^ (mock_corrupt_guard_readback ? 1U : 0U);
  return mock_operation("guard_read");
}
struct VL53L1X_ULD {
  uint8_t address{0x52};
  uint16_t intermeasurement{0};
  int GetBootState(uint8_t *v) { *v = mock_ready; return mock_operation("boot"); }
  int GetI2CAddress() { return address; }
  int SetI2CAddress(uint8_t v) { address = v; return mock_operation("address"); }
  int SetROI(int, int) { return mock_operation("roi"); }
  int SetROICenter(int) { return mock_operation("center"); }
  int SetDistanceMode(EDistanceMode) { return mock_operation("mode"); }
  int SetTimingBudgetInMs(int) { return mock_operation("budget"); }
  int SetInterMeasurementInMs(int value) { intermeasurement = value; return mock_operation("interval"); }
  int StartRanging() { return mock_operation("start"); }
  int StopRanging() { return mock_operation("stop"); }
  int CheckForDataReady(uint8_t *v) { *v = mock_ready; return mock_operation("ready"); }
  int ClearInterrupt() { return mock_operation("clear"); }
  int GetResult(VL53L1X_Result_t *v) { *v = mock_result; return mock_operation("result"); }
};
