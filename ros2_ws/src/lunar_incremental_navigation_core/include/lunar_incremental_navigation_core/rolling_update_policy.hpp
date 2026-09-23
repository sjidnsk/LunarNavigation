#pragma once
#include <cstdint>
#include <limits>
namespace lunar::incremental_navigation {
class RollingUpdatePolicy final {
 public:
  RollingUpdatePolicy(double period_s, double distance_m);
  [[nodiscard]] bool ShouldStart(double now_s, std::uint64_t revision, double remaining_m) const;
  void Started(double now_s, std::uint64_t revision);
  // A replacement already inside the threshold is not another threshold crossing.
  void Published(std::uint64_t revision, double remaining_m) noexcept;
  void Finished() noexcept { busy_ = false; }
  void SetFrozen(bool frozen) noexcept { frozen_ = frozen; }
  void Reset() noexcept;
 private:
  double period_s_, distance_m_;
  double last_started_s_{-std::numeric_limits<double>::infinity()};
  std::uint64_t early_revision_{};
  bool busy_{}, frozen_{};
};
}
