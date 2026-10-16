#include "lunar_incremental_navigation_core/rolling_update_policy.hpp"
#include <cmath>
#include <stdexcept>
namespace lunar::incremental_navigation {
RollingUpdatePolicy::RollingUpdatePolicy(double period_s, double distance_m)
    : period_s_(period_s), distance_m_(distance_m) {
  if (!std::isfinite(period_s) || period_s < 0. ||
      !std::isfinite(distance_m) || distance_m < 0.)
    throw std::invalid_argument("rolling update period and distance must be finite and nonnegative");
}
bool RollingUpdatePolicy::ShouldStart(double now_s, std::uint64_t revision, double remaining_m) const {
  if (period_s_ == 0. || busy_ || frozen_ || !std::isfinite(now_s)) return false;
  return now_s - last_started_s_ >= period_s_ ||
      (std::isfinite(remaining_m) && remaining_m >= 0. && remaining_m <= distance_m_ && revision != early_revision_);
}
void RollingUpdatePolicy::Started(double now_s, std::uint64_t revision) {
  last_started_s_ = now_s; early_revision_ = revision; busy_ = true;
}
void RollingUpdatePolicy::Published(std::uint64_t revision, double remaining_m) noexcept {
  if (remaining_m <= distance_m_) early_revision_ = revision;
}
void RollingUpdatePolicy::Reset() noexcept {
  last_started_s_ = -std::numeric_limits<double>::infinity();
  early_revision_ = 0; busy_ = false; frozen_ = false;
}
}
