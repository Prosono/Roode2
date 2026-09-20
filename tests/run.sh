#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. tests/counting_core_test.cpp -o "$build_dir/counting_core_test"
"$build_dir/counting_core_test"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/stubs -I. tests/firmware_test.cpp components/tof_overdoor_counter/tof_overdoor_counter.cpp -o "$build_dir/firmware_test"
"$build_dir/firmware_test"
