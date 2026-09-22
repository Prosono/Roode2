#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace counting_core {

inline unsigned bits(uint8_t mask) {
  unsigned n = 0;
  for (; mask; mask >>= 1) n += mask & 1;
  return n;
}

// Welford statistics: bounded memory, no integer-square overflow.
struct Statistics {
  unsigned count{0};
  double mean{0}, m2{0};
  float low{0}, high{0};
  void add(float value) {
    if (!std::isfinite(value)) return;
    if (count == 0) low = high = value;
    low = std::min(low, value);
    high = std::max(high, value);
    const double delta = value - mean;
    mean += delta / ++count;
    m2 += delta * (value - mean);
  }
  float deviation() const { return count ? std::sqrt(std::max(0.0, m2 / count)) : 0; }
};

// The crossing happened after the last sample on the old side of a threshold
// and no later than the first sample on the new side. Poll/confirmation times
// alone cannot establish direction when two alternating ROIs overlap in time.
struct Crossing {
  uint32_t lower{0}, upper{0};
  bool valid{false};
};

inline uint8_t crossing_order(const Crossing &a, const Crossing &b) {
  if (!a.valid || !b.valid) return 0;
  if (static_cast<int32_t>(b.lower - a.upper) > 0) return 1;
  if (static_cast<int32_t>(a.lower - b.upper) > 0) return 2;
  return 0;  // Touching/overlapping intervals do not establish an order.
}

// Call only for a NEW valid measurement. A lone outlier cannot become a hit
// merely because the scheduler has run many times while waiting for the sensor.
struct Debounce {
  bool active{false}, candidate{false};
  uint32_t since{0};
  unsigned hits{0};
  Crossing crossing;
  uint32_t last_stable{0}, candidate_lower{0};
  bool have_stable{false}, candidate_bounded{false};
  bool update(bool target, uint32_t now, uint32_t hold_ms) {
    return update(target, now, hold_ms, now);
  }
  bool update(bool target, uint32_t now, uint32_t hold_ms, uint32_t sample_started) {
    if (target == active) { hits = 0; last_stable = sample_started; have_stable = true; return false; }
    if (!hits || target != candidate) {
      candidate = target; since = now; hits = 1;
      candidate_lower = last_stable; candidate_bounded = have_stable;
      return false;
    }
    ++hits;
    if (now - since < hold_ms) return false;
    active = target;
    crossing = {candidate_lower, since, candidate_bounded};
    last_stable = sample_started;
    have_stable = true;
    hits = 0;
    return true;
  }
};

// State bits are the two physical ROI fields. Direction is the starting side.
// Keep the whole episode until the doorway is clear, including late reversals.
struct Path {
  uint8_t first{0}, last{0}, seen{0}, previous{0};
  bool ambiguous{false}, cleared{false}, visited{false};
  uint32_t started{0}, finished{0};
  std::array<Crossing, 2> rises{}, falls{};
  bool timed{false};
  void observe(uint8_t state, uint32_t now, const std::array<Crossing, 2> *crossings = nullptr) {
    if (state == previous) return;
    if (crossings != nullptr) {
      timed = true;
      for (unsigned z = 0; z < 2; ++z) {
        if (!((state ^ previous) & (1U << z))) continue;
        if (state & (1U << z)) rises[z] = (*crossings)[z];
        else falls[z] = (*crossings)[z];
      }
    }
    previous = state;
    if (!state) {
      if (visited) { cleared = true; finished = now; }
      return;
    }
    if (cleared) ambiguous = true; // More than one track in an unseparated episode.
    if (!visited) {
      visited = true;
      started = now;
      if (state == 3) ambiguous = true;
      else first = state;
    }
    seen |= state;
    if (state != 3) last = state;
  }
  uint8_t geometric_direction(uint32_t minimum_ms) const {
    return visited && cleared && !ambiguous && first && last && first != last && seen == 3 &&
                   finished - started >= minimum_ms ? first : 0;
  }
  uint8_t direction(uint32_t minimum_ms) const {
    const auto vote = geometric_direction(minimum_ms);
    if (!vote || !timed) return vote;
    const auto entering = crossing_order(rises[0], rises[1]);
    const auto leaving = crossing_order(falls[0], falls[1]);
    if ((!entering && !leaving) || (entering && entering != vote) || (leaving && leaving != vote)) return 0;
    return vote;
  }
};

enum class Decision : uint8_t { NONE, IN, OUT, UNSURE_IN, UNSURE_OUT, REJECTED };
struct Result {
  Decision decision{Decision::NONE};
  uint8_t in_votes{0}, out_votes{0}, participants{0};
  uint8_t uncertain_timing{0};
  uint32_t duration_ms{0};
};

// An episode is bounded by a confirmed empty doorway. Quorum never decreases
// with sensor loss. Health changes discard that sensor's entire track; losing
// quorum disarms the counter until a fresh clear interval has been observed.
class Fusion {
 public:
  uint8_t required{3};  // Matching complete direction tracks.
  uint8_t required_health{0};  // Optional higher availability requirement.
  uint32_t clear_ms{90}, minimum_ms{25}, agreement_ms{3000};
  bool invert{false};
  // Keep uncertain timing diagnostic until a given installation has enough
  // temporal separation. Enabling this on undersampled data can reject every
  // passage, including correct ones; it is not a universal accuracy upgrade.
  bool require_timing_evidence{false};
  void reset() { *this = Fusion{}; }
  bool active() const { return active_; }
  uint32_t started() const { return started_; }
  const Path &path(unsigned i) const { return paths_[i]; }
  Result update(uint32_t now, uint8_t healthy, const std::array<uint8_t, 4> &states,
                const std::array<uint8_t, 4> &fresh, bool unsettled = false,
                const std::array<std::array<Crossing, 2>, 4> *crossings = nullptr) {
    Result result;
    uint8_t occupied = 0;
    for (unsigned i = 0; i < 4; ++i) {
      if ((healthy & (1U << i)) && states[i]) occupied |= 1U << i;
      if (!(healthy & (1U << i)) || !(previous_healthy_ & (1U << i))) paths_[i] = Path{};
    }
    previous_healthy_ = healthy;
    if (active_) eligible_ &= healthy;
    if (bits(healthy) < std::max(required, required_health)) {
      if (active_) { result.decision = Decision::REJECTED; result.duration_ms = now - started_; }
      active_ = false;
      armed_ = false;
      clear_pending_ = false;
      paths_ = {};
      return result;
    }
    if (occupied) {
      clear_pending_ = false;
      clear_samples_ = 0;
      if (!armed_) return result;
      if (!active_) { active_ = true; started_ = now; paths_ = {}; eligible_ = healthy; }
    }
    if (active_) {
      for (unsigned i = 0; i < 4; ++i)
        if (eligible_ & (1U << i)) paths_[i].observe(states[i], now, crossings ? &(*crossings)[i] : nullptr);
    }
    if (occupied || unsettled) {
      clear_pending_ = false;
      clear_samples_ = 0;
      return result;
    }
    if (!clear_pending_) { clear_pending_ = true; clear_since_ = now; clear_samples_ = 0; }
    for (unsigned i = 0; i < 4; ++i) clear_samples_ |= (fresh[i] & 3U) << (2 * i);
    uint8_t needed = 0;
    for (unsigned i = 0; i < 4; ++i) if (healthy & (1U << i)) needed |= 3U << (2 * i);
    if (now - clear_since_ < clear_ms || (clear_samples_ & needed) != needed) return result;
    armed_ = true;
    if (!active_) return result;
    uint32_t earliest = 0, latest = 0;
    bool have_vote = false;
    for (unsigned i = 0; i < 4; ++i) {
      if (!(eligible_ & healthy & (1U << i))) continue;
      const auto &p = paths_[i];
      const uint8_t timed_vote = p.direction(minimum_ms);
      const uint8_t geometric_vote = p.geometric_direction(minimum_ms);
      if (!timed_vote && p.timed && geometric_vote) ++result.uncertain_timing;
      const uint8_t vote = require_timing_evidence ? timed_vote : geometric_vote;
      if (!vote) continue;
      result.participants |= 1U << i;
      // OUT ROI first historically maps to OUT unless installation is inverted.
      const bool is_in = invert ? vote == 1 : vote == 2;
      if (is_in) ++result.in_votes; else ++result.out_votes;
      const uint32_t offset = p.finished - started_;
      if (!have_vote) earliest = latest = offset;
      earliest = std::min(earliest, offset);
      latest = std::max(latest, offset);
      have_vote = true;
    }
    result.duration_ms = now - started_;
    if ((result.in_votes && result.out_votes) || (have_vote && latest - earliest > agreement_ms))
      result.decision = Decision::REJECTED;
    else if (result.in_votes >= required) result.decision = Decision::IN;
    else if (result.out_votes >= required) result.decision = Decision::OUT;
    else if (result.in_votes) result.decision = Decision::UNSURE_IN;
    else if (result.out_votes) result.decision = Decision::UNSURE_OUT;
    else result.decision = Decision::REJECTED;
    active_ = false;
    paths_ = {};
    return result;
  }
 private:
  std::array<Path, 4> paths_{};
  bool active_{false}, armed_{false}, clear_pending_{false};
  uint32_t started_{0}, clear_since_{0};
  uint8_t previous_healthy_{0}, clear_samples_{0}, eligible_{0};
};
}  // namespace counting_core
}  // namespace esphome
