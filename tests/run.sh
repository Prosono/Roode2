#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. tests/counting_core_test.cpp -o "$build_dir/counting_core_test"
"$build_dir/counting_core_test"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/stubs -I. tests/firmware_test.cpp components/tof_overdoor_counter/tof_overdoor_counter.cpp -o "$build_dir/firmware_test"
"$build_dir/firmware_test"
for test in calibration_migration acquisition; do
  "${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/acquisition_stubs -Itests/stubs -I. "tests/${test}_test.cpp" components/tof_overdoor_counter/tof_overdoor_counter.cpp -o "$build_dir/${test}_test"
  "$build_dir/${test}_test"
done
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/stubs -I. tests/runtime_stability_test.cpp components/tof_overdoor_counter/tof_overdoor_counter.cpp -o "$build_dir/runtime_stability_test"
"$build_dir/runtime_stability_test"
for test in runtime_guard ui_bridge; do
  "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -pthread -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. "tests/${test}_test.cpp" -o "$build_dir/${test}_test"
  "$build_dir/${test}_test"
done
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -DUSE_WIFI -Itests/wifi_stubs -Itests/stubs -I. tests/wifi_recovery_test.cpp components/tof_overdoor_counter/tof_overdoor_counter.cpp -o "$build_dir/wifi_recovery_test"
"$build_dir/wifi_recovery_test"
# ESPHome downloads the real ArduinoJson headers during firmware generation.
# Set ARDUINOJSON_INCLUDE to that library's src directory to exercise serialization.
if [[ -n "${ARDUINOJSON_INCLUDE:-}" ]]; then
  "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. -I"$ARDUINOJSON_INCLUDE" tests/ui_json_test.cpp -o "$build_dir/ui_json_test"
  "$build_dir/ui_json_test"
else
  echo 'SKIP: UI JSON test (set ARDUINOJSON_INCLUDE to the ArduinoJson src directory)'
fi
node tests/ui_polling_test.js
python3 tests/replay_capture_test.py
python3 tests/capture_trace_test.py
