// Diagnostic replay of sampled browser traces. This intentionally does not
// assert a perfect count: omitted measurements and physical ROI mix-ups cannot
// be repaired by replay, and expected directions are external ground truth.
#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

uint32_t test_now = 100;
using namespace esphome;
struct ZoneSample {
  unsigned stamp{}, raw{}, filtered{}, baseline{}, trigger{}, status{};
};
struct Row {
  unsigned now{}, healthy{}, valid{}, fresh{}, active{};
  std::array<ZoneSample, 8> zones{};
};
std::vector<Row> read_rows(const char *path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot open replay fixture");
  std::vector<Row> rows;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream values(line);
    Row row;
    values >> row.now >> row.healthy >> row.valid >> row.fresh >> row.active;
    for (auto &zone : row.zones)
      values >> zone.stamp >> zone.raw >> zone.filtered >> zone.baseline >> zone.trigger >> zone.status;
    if (!values) throw std::runtime_error("Incomplete replay fixture row");
    rows.push_back(row);
  }
  if (rows.empty()) throw std::runtime_error("Empty replay fixture");
  return rows;
}
const char *decision_name(counting_core::Decision decision) {
  switch (decision) {
    case counting_core::Decision::IN: return "IN";
    case counting_core::Decision::OUT: return "OUT";
    case counting_core::Decision::UNSURE_IN: return "UNSURE_IN";
    case counting_core::Decision::UNSURE_OUT: return "UNSURE_OUT";
    case counting_core::Decision::REJECTED: return "CANCELLED";
    default: return "NONE";
  }
}
void observed_replay(const std::vector<Row> &rows) {
  std::cerr << "Observed mode uses legacy active-mask evidence; integration-time direction gates are NOT exercised.\n";
  counting_core::Fusion fusion;
  fusion.required = 2;
  fusion.required_health = 3;
  fusion.clear_ms = 90;
  fusion.minimum_ms = 25;
  fusion.agreement_ms = 3000;
  for (const auto &row : rows) {
    std::array<uint8_t, 4> state{}, fresh{};
    for (unsigned i = 0; i < 4; ++i) {
      state[i] = (row.active >> (i*2)) & 3;
      fresh[i] = ((row.fresh & row.valid) >> (i*2)) & 3;
    }
    // The old trace does not include pending debounce candidates; therefore
    // even this replay can differ slightly from the device's original events.
    const auto result = fusion.update(row.now, row.healthy, state, fresh);
    if (result.decision != counting_core::Decision::NONE)
      std::cout << row.now << '\t' << decision_name(result.decision) << '\t'
                << unsigned(result.in_votes) << '\t' << unsigned(result.out_votes) << '\n';
  }
}

class FirmwareReplay : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  explicit FirmwareReplay(const Row &initial, bool strict_timing) {
    test_now = initial.now;
    channels_.resize(4);
    calibration_active_ = false;
    startup_clear_validated_ = true;
    sampling_size_ = 1;
    trigger_threshold_mm_ = 200;
    clear_threshold_mm_ = 80;
    debounce_ms_ = 25;
    cooldown_ms_ = 80;
    direction_window_ms_ = 90;
    detection_timeout_ms_ = 3000;
    min_event_sensors_ = 2;
    min_valid_sensors_ = 3;
    require_timing_evidence_ = strict_timing;
    capture_start_ms_ = initial.now;
    for (unsigned i = 0; i < 4; ++i) {
      auto &channel = channels_[i];
      channel.sensor = std::make_unique<VL53L1X_ULD>();
      channel.sensor_label = "U" + std::to_string(i);
      channel.initialized = channel.ranging_started = channel.has_reading = channel.calibrated = true;
      channel.stale = false;
      for (unsigned z = 0; z < 2; ++z) {
        auto &zone = channel.zones[z];
        const auto &sample = initial.zones[i*2+z];
        zone.calibrated = zone.has_reading = true;
        zone.valid_measurement = (initial.valid & (1U << (i*2+z))) != 0;
        zone.raw_distance = sample.raw;
        zone.filtered_distance = sample.filtered;
        zone.last_update_ms = sample.stamp;
        zone.last_good_read_ms = zone.valid_measurement ? sample.stamp : 0;
        if (sample.stamp) previous_channel_sample_[i] = std::max(previous_channel_sample_[i], sample.stamp);
      }
    }
    metadata(initial);
  }
  void metadata(const Row &row) {
    for (unsigned i = 0; i < 4; ++i) {
      auto &channel = channels_[i];
      // A zeroed baseline in this trace means the channel is being recovered.
      // Do not manufacture measurements during that missing interval.
      const bool available = row.zones[i*2].baseline && row.zones[i*2+1].baseline;
      channel.initialized = channel.ranging_started = channel.calibrated = available;
      for (unsigned z = 0; z < 2; ++z) {
        auto &zone = channel.zones[z];
        const auto &sample = row.zones[i*2+z];
        zone.calibrated = available;
        zone.baseline = sample.baseline;
        // Recorded trigger = max(200, 80+6*noise). The floor hides the actual
        // lower noise, so this is its upper bound, not an exact reconstruction.
        zone.noise = sample.trigger > 80 ? (sample.trigger-80)/6.0f : 0;
      }
    }
  }
  void clear_fresh() {
    for (auto &channel : channels_) for (auto &zone : channel.zones) zone.fresh = false;
  }
  void sample(unsigned index, const ZoneSample &sample, bool recorded_filter) {
    auto &channel = channels_[index/2];
    auto &zone = channel.zones[index%2];
    if (!channel.initialized || sample.status == 255) return;
    // The capture supplies an already completed result, not hardware readiness
    // polling. Bypass only the new live acquisition overrun/drain scheduling:
    // otherwise sparse historical samples would be discarded a second time.
    channel.ranging_started = true;
    channel.settling_after_stop = false;
    channel.ranging_started_ms = test_now - timing_budget_ms_;
    channel.current_zone = index%2;
    mock_result.Distance = sample.raw;
    mock_result.Status = sample.status;
    if (!read_channel_(channel)) throw std::runtime_error("Mock sensor read failed");
    if (!zone.fresh) throw std::runtime_error("Replay result was not consumed");
    // The old capture has no integration-start timestamp. Widen the lower
    // bound to one timing budget before the previous same-sensor result: the
    // old autonomous pipeline may already have integrated before it was read.
    // This is an explicit reconstruction assumption, not a measured start.
    const auto previous = previous_channel_sample_[index/2];
    const auto bound = previous ? previous : capture_start_ms_;
    zone.sample_started_ms = bound > timing_budget_ms_ ? bound - timing_budget_ms_ : 0;
    previous_channel_sample_[index/2] = sample.stamp;
    if (recorded_filter) zone.filtered_distance = sample.filtered;
  }
  void step() {
    const auto before = confirmed_in_count_ + confirmed_out_count_ + unsure_in_count_ + unsure_out_count_ + rejected_count_;
    update_sensor_health_();
    update_sensor_states_();
    update_blocked_state_();
    update_detection_state_machine_();
    const auto after = confirmed_in_count_ + confirmed_out_count_ + unsure_in_count_ + unsure_out_count_ + rejected_count_;
    if (after != before) std::cout << test_now << '\t' << last_direction_ << '\t' << last_reason_ << '\n';
  }
 private:
  std::array<uint32_t, 4> previous_channel_sample_{};
  uint32_t capture_start_ms_{0};
};

void firmware_replay(const std::vector<Row> &rows, bool recorded_filter, bool strict_timing) {
  std::cerr << "Integration timing: " << (strict_timing ? "required (opt-in)" : "diagnostic only (default)") << '\n';
  struct Event { unsigned at, row, zone; bool sample; };
  std::vector<Event> events;
  std::array<unsigned, 8> previous{};
  for (unsigned r = 0; r < rows.size(); ++r) {
    events.push_back({rows[r].now, r, 0, false});
    for (unsigned z = 0; z < 8; ++z) {
      const auto stamp = rows[r].zones[z].stamp;
      if (stamp && stamp != previous[z] && stamp >= rows.front().now)
        events.push_back({stamp, r, z, true});
      previous[z] = stamp;
    }
  }
  std::stable_sort(events.begin(), events.end(), [](const Event &a, const Event &b) { return a.at < b.at; });
  FirmwareReplay firmware(rows.front(), strict_timing);
  for (const auto &event : events) {
    test_now = event.at;
    firmware.clear_fresh();
    if (event.sample) firmware.sample(event.zone, rows[event.row].zones[event.zone], recorded_filter);
    else firmware.metadata(rows[event.row]);
    firmware.step();
  }
}

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: replay_capture observed|raw|filtered|raw-strict|filtered-strict fixture.tsv");
    const auto rows = read_rows(argv[2]);
    std::cerr << "Diagnostic replay: " << rows.size() << " sampled snapshots; omitted samples cannot be reconstructed.\n";
    const std::string mode(argv[1]);
    if (mode == "observed") observed_replay(rows);
    else if (mode == "raw" || mode == "filtered" || mode == "raw-strict" || mode == "filtered-strict")
      firmware_replay(rows, mode == "filtered" || mode == "filtered-strict", mode == "raw-strict" || mode == "filtered-strict");
    else throw std::runtime_error("Unknown replay mode");
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
