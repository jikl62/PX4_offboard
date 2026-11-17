#pragma once
// 测距趋势只读核对：PX4 dist_bottom 是地形距离估计，可能与 z 共用 EKF 数据，
// RANGE 位不认证具体 MTF 设备，也不能证明独立测量。此类绝不返回控制许可或改写地面 z0。
#include <cmath>
#include <cstdint>

namespace fc01::attitude {
struct RangeTrendEvidence {
  bool available{}, compared{}, same_direction{};
  double distance_m{}, variance_m2{}, fused_rise_m{}, range_rise_m{}, interval_s{};
  std::uint64_t sample_stamp{};
};
class RangeTrendDiagnostics {
public:
  // stamp为local原始样本µs；z为NED下正m，distance为正向离地距离m，variance为m²。
  // 仅处理已由接收层接受的消息。新鲜度由读取方另外检查，重复样本不延长比较窗口。
  void observe(std::uint64_t stamp, bool valid, double z, double distance, double variance,
               std::uint8_t z_reset, std::uint8_t distance_reset, double max_gap_s) {
    const bool usable = stamp && valid && std::isfinite(z) && std::isfinite(distance) &&
                        distance >= 0 && std::isfinite(variance) && variance >= 0;
    if (!usable || (last_stamp_ && stamp < last_stamp_)) {
      e_ = {};
      anchored_ = false;
      return;
    }
    if (stamp == last_stamp_) {
      // 同戳改数也不能当成另一个测量；保持水位但撤销当前诊断。
      if (z != last_z_ || distance != e_.distance_m || variance != e_.variance_m2 ||
          z_reset != z_reset_ || distance_reset != distance_reset_) {
        e_ = {};
        anchored_ = false;
      }
      return;
    }
    const bool continuous = anchored_ && z_reset == z_reset_ && distance_reset == distance_reset_ &&
                            double(stamp - last_stamp_) * 1e-6 <= max_gap_s;
    last_stamp_ = stamp;
    last_z_ = z;
    z_reset_ = z_reset;
    distance_reset_ = distance_reset;
    if (!continuous) {
      e_ = {};
      anchor_stamp_ = stamp;
      anchor_z_ = z;
      anchor_distance_ = distance;
      anchored_ = true;
    }
    e_.available = true;
    e_.sample_stamp = stamp;
    e_.distance_m = distance;
    e_.variance_m2 = variance;
    // 200ms仅是诊断差分窗口，降低单帧量化噪声影响；不是离地/到高判据或硬一致性阈值。
    if (stamp - anchor_stamp_ >= 200000) {
      e_.compared = true;
      e_.interval_s = double(stamp - anchor_stamp_) * 1e-6;
      e_.fused_rise_m = anchor_z_ - z;
      e_.range_rise_m = distance - anchor_distance_;
      e_.same_direction = (e_.fused_rise_m > 0 && e_.range_rise_m > 0) ||
                          (e_.fused_rise_m < 0 && e_.range_rise_m < 0) ||
                          (e_.fused_rise_m == 0 && e_.range_rise_m == 0);
      anchor_stamp_ = stamp;
      anchor_z_ = z;
      anchor_distance_ = distance;
    }
  }
  // fresh必须同时来自信封、原始样本和同步时钟；失效返回空值，不能展示旧比较为当前有效。
  RangeTrendEvidence evidence(bool fresh) const {
    return fresh ? e_ : RangeTrendEvidence{};
  }

private:
  RangeTrendEvidence e_;
  bool anchored_{};
  std::uint64_t last_stamp_{}, anchor_stamp_{};
  std::uint8_t z_reset_{}, distance_reset_{};
  double last_z_{}, anchor_z_{}, anchor_distance_{};
};
} // namespace fc01::attitude
