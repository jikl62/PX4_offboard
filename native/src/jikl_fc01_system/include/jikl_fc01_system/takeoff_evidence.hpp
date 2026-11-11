#pragma once
// 离地证据拼合：分别累计 land detector 与实际上升运动，再要求两种证据同时新鲜。
// height 是相对已锁定地面的上正高度 m；up_speed 是上正 m/s（上层由 PX4 NED vz 取负）。
// 源戳为 µs，now 和 dwell 为单调秒；一个 !landed 或单帧高 z/速度都不能单独证明离地。
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc01::attitude {
// 分别报告最终确认、检测器确认、上升确认及各自样本/源戳，避免把两个异步来源混成一个计数。
struct LiftoffEvidence {
  bool confirmed{}, detector_confirmed{}, rise_confirmed{};
  unsigned detector_samples{}, local_samples{};
  std::uint64_t first_land_stamp{}, last_land_stamp{}, first_local_stamp{}, last_local_stamp{};
};

// 检测器与位置消息独立到达，各自保留证据：先到的!landed不因本地高度尚未跨阈值而被吃掉。
// 单独检测器序列或一个高度/速度尖峰都不算离地；即使两者一致，也不能排除共同估计误差。
class TakeoffEvidence {
public:
  void reset() {
    // *this=临时默认对象把本任务的全部证据与时钟水位一起清空；只能由外层任务/失效路径明确触发。
    *this = TakeoffEvidence{};
  }
  const LiftoffEvidence& evidence() const {
    // 借用成员证据以减少复制；调用observe后内容会改变，不是一个被冻结的历史快照。
    return e_;
  }
  // 每次消费真实新样本；land_baseline 为本任务起飞前基线，避免旧空中检测复用。
  template <class C>
  bool observe(const C& c, double now, std::uint64_t land_baseline, std::uint64_t land_stamp,
               bool landed, std::uint64_t local_stamp, double height, double up_speed) {
    // 本轮先撤销最终confirmed，再重算两个来源的交集；不能让上一次true自动沿用到失效输入。
    e_.confirmed = false;
    if (!std::isfinite(now) || !std::isfinite(height) || !std::isfinite(up_speed)) {
      reset();
      return false;
    }
    // 检测器允许间隔取 land_sample_max_gap 与 land_timeout 一半的较小值，避免稀疏消息伪造连续 dwell。
    const double detector_gap = std::min(c.land_sample_max_gap, c.land_timeout * .5);
    // 任一源戳或单调时钟回退都清全部证据；不能继续累加两个不相干的时间段。
    if ((land_seen_ && land_stamp < land_seen_) || (local_seen_ && local_stamp < local_seen_) ||
        (last_observed_ >= 0 && now < last_observed_)) {
      reset();
      return false;
    }
    last_observed_ = now;
    // 检测器回到地面先撤销 detector；若此前真见过空中，则同时撤销早先上升片段，区分起落弹跳。
    if (landed) {
      clear_detector();
      // 曾观察非地面后又landed相当于起落弹跳，旧上升片段须作废，不能拼成下一次离地证据。
      if (detector_was_airborne_)
        clear_local();
      detector_was_airborne_ = false;
    }
    // 只在检测器源戳推进时累计，且必须晚于本任务基线；重复外壳不算新样本。
    if (land_stamp > land_seen_) {
      land_seen_ = land_stamp;
      if (!landed && land_stamp > land_baseline) {
        if (land_last_time_ >= 0 && (now - land_last_time_ > detector_gap ||
                                     double(land_stamp - e_.last_land_stamp) * 1e-6 > detector_gap))
          clear_detector();
        if (!e_.detector_samples) {
          land_first_time_ = now;
          e_.first_land_stamp = land_stamp;
        }
        land_last_time_ = now;
        e_.last_land_stamp = land_stamp;
        ++e_.detector_samples;
        detector_was_airborne_ = true;
        e_.detector_confirmed =
            e_.detector_samples >= 2 && land_stamp > e_.first_land_stamp &&
            now - land_first_time_ + 1e-9 >= c.liftoff_dwell_seconds &&
            double(land_stamp - e_.first_land_stamp) * 1e-6 + 1e-9 >= c.liftoff_dwell_seconds;
        // 主机经过时间和源戳经过时间都要满dwell，防止一阵延迟后补送旧样本瞬间凑够持续时间。
      }
    }
    if (land_last_time_ >= 0 && now - land_last_time_ > detector_gap)
      clear_detector();
    // 本地状态独立累计：先检查源/接收间隔，然后才接受本帧高度和实际向上运动。
    if (local_stamp > local_seen_) {
      if (local_last_time_ >= 0 && (now - local_last_time_ > c.local_timeout ||
                                    double(local_stamp - local_seen_) * 1e-6 > c.local_timeout))
        clear_local();
      local_seen_ = local_stamp;
      // 检测器稍晚到达时可在有界窗口内拼合已确认上升；高度不再增长的新信封不能无限续期旧运动。
      if (e_.rise_confirmed && (motion_at_ < 0 || now - motion_at_ > c.land_timeout ||
                                double(local_stamp - motion_stamp_) * 1e-6 > c.land_timeout))
        clear_rise();
      // 上升更新必须同时满足高度、速度及高度真正增长；新外壳停在同高度不能无限续期。
      if (height >= c.liftoff_height_m && up_speed >= c.liftoff_speed_mps &&
          height > latest_height_ + 1e-9) {
        motion_at_ = now;
        motion_stamp_ = local_stamp;
      }
      local_last_time_ = now;
      latest_height_ = height;
      if (height < c.liftoff_height_m)
        clear_rise();
      if (!e_.rise_confirmed) {
        if (height < c.liftoff_height_m || up_speed < c.liftoff_speed_mps)
          clear_rise();
        else {
          if (!e_.local_samples) {
            local_first_time_ = now;
            first_height_ = height;
            e_.first_local_stamp = local_stamp;
          }
          e_.last_local_stamp = local_stamp;
          ++e_.local_samples;
          // 至少两帧、源时间和单调时间都满 dwell，还需实际高度增长，避免单一速度尖峰证明离地。
          e_.rise_confirmed =
              e_.local_samples >= 2 && local_stamp > e_.first_local_stamp &&
              now - local_first_time_ + 1e-9 >= c.liftoff_dwell_seconds &&
              double(local_stamp - e_.first_local_stamp) * 1e-6 + 1e-9 >= c.liftoff_dwell_seconds &&
              height - first_height_ + 1e-9 >= c.liftoff_speed_mps * c.liftoff_dwell_seconds;
        }
      } else
        e_.last_local_stamp = local_stamp;
    }
    if (local_last_time_ >= 0 && now - local_last_time_ > c.local_timeout)
      clear_local();
    // 制动后只能在实测上升运动的land_timeout内桥接新本地观测，同时核对源域和主机域的期限。
    // 最终取两证据交集：当前非地面、检测器和上升均确认，高度达标，最新各源未超时。
    e_.confirmed = !landed && e_.detector_confirmed && e_.rise_confirmed &&
                   latest_height_ >= c.liftoff_height_m && local_last_time_ >= 0 &&
                   now - local_last_time_ <= c.local_timeout && land_last_time_ >= 0 &&
                   now - land_last_time_ <= detector_gap;
    return e_.confirmed;
  }

private:
  // 只清检测器片段；clear_rise 清上升片段，clear_local 另清本地到达寿命，各来源证据独立失效。
  void clear_detector() {
    e_.detector_confirmed = false;
    e_.detector_samples = 0;
    e_.first_land_stamp = e_.last_land_stamp = 0;
    land_first_time_ = land_last_time_ = -1;
  }
  void clear_rise() {
    e_.rise_confirmed = false;
    e_.local_samples = 0;
    e_.first_local_stamp = e_.last_local_stamp = 0;
    local_first_time_ = motion_at_ = -1;
    motion_stamp_ = 0;
  }
  void clear_local() {
    clear_rise();
    local_last_time_ = -1;
  }
  LiftoffEvidence e_;
  // seen是各源最新已观察戳；motion是最近真实向上运动戳。它们与主机秒的*_time_不可直接相减。
  std::uint64_t land_seen_{}, local_seen_{}, motion_stamp_{};
  double land_first_time_{-1}, land_last_time_{-1}, local_first_time_{-1}, local_last_time_{-1},
      last_observed_{-1};
  double first_height_{}, latest_height_{}, motion_at_{-1};
  bool detector_was_airborne_{};
};
} // namespace fc01::attitude
