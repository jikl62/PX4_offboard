#pragma once
// 垂向反馈一致性观察器：输入均来自 PX4 同一估计器，只检查内部证据，不能证明实际飞行高度正确。
// vz 与 z_deriv 的速度差仅作诊断；输入/时钟/位置积分不一致仍保留原拒绝路径，绝不在此换高度源或重建零点。
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fc01::attitude {
// 部署配置显式开启监测；纯离线旧调用可以保持关闭，不伪造原始 PX4 采样时间。
struct VerticalFeedbackConfig {
  // Opt in explicitly in the deployed YAML; legacy synthetic Core callers
  // need not manufacture a PX4 z_deriv or a source timestamp.
  bool enabled{};
  // Velocity-output differences are diagnostic only. These existing values
  // also bound the independent position/derivative consistency window.
  // rate_tolerance_mps 为 m/s 残差阈值；dwell_seconds 为秒窗口，同时用于速度差诊断与独立位置积分检查。
  double rate_tolerance_mps{.20},dwell_seconds{.50};
  // 只接受有限的正阈值及有界窗口；配置异常直接抛出，避免运行时窗口永久不结束。
  void validate() const{
    if(!std::isfinite(rate_tolerance_mps)||rate_tolerance_mps<=0||rate_tolerance_mps>1||
       !std::isfinite(dwell_seconds)||dwell_seconds<.2||dwell_seconds>2)
      throw std::invalid_argument("INVALID_VERTICAL_FEEDBACK_GUARD_PARAMETERS");
  }
};
// checked 表示尝试检查；healthy/reason 表示硬故障；suspect/rate_warning 不单独授权停机。
struct VerticalFeedbackEvidence {
  // While healthy, suspect/rate_warning report velocity-output disagreement.
  // Hard input, clock and position-consistency faults use healthy/reason.
  bool checked{},healthy{true},suspect{},position_window_checked{};
  bool rate_warning{};
  // 两种误差均为 m/s：前者 |vz-z_deriv|，后者为位置变化减导数积分后除窗口时长；持续时间单位 s。
  double rate_error_mps{},position_rate_error_mps{},disagreement_seconds{};
  // 原始 PX4 timestamp_sample，单位 µs；不是主机回调时间或被新消息刷新的接收时间。
  std::uint64_t sample_stamp{};
  std::uint32_t disagreement_samples{};
  std::string reason;
};

// Consistency only: z, vz and z_deriv share an estimator. Agreement cannot
// prove actual altitude or rule out common-mode drift. Never select a new
// height/velocity source or rebase a flight reference here.
// 持有相邻新样本和积分窗口；硬故障锁存到任务外显式 reset，正常尾帧不能在空中自行清除。
class VerticalFeedbackGuard {
 public:
  // 保存并校验窗口参数；不读取系统时钟，不推测实际传感器精度。
  explicit VerticalFeedbackGuard(VerticalFeedbackConfig config={}):c_(config){c_.validate();}
  // 由外层新任务/地面恢复流程重建整个观察器；不是每次收到健康帧就调用。
  void reset(){*this=VerticalFeedbackGuard(c_);}
  // 只读访问当前累计证据，供状态展示与故障归因。
  const VerticalFeedbackEvidence& evidence() const{return e_;}
  // stamp 为 PX4 微秒采样时间；z 为 NED 向下 m，vz/z_deriv 均向下为正 m/s；max_gap_seconds 来自本地反馈时限。
  const VerticalFeedbackEvidence& observe(std::uint64_t stamp,double z,double vz,
                                         double z_deriv,double max_gap_seconds){
    // 关闭时保留默认证据，不制造位置导数要求；这是显式配置行为。
    if(!c_.enabled)return e_;
    // 硬故障已经发生后不再用后续正常数据覆盖原 reason，恢复交给地面任务管理。
    if(!e_.healthy)return e_; // a fault cannot self-clear in flight
    e_.checked=true;e_.sample_stamp=stamp;
    // 缺失采样时间、非有限状态或非法最大间隔都不能用于积分。
    if(!stamp||!std::isfinite(z)||!std::isfinite(vz)||!std::isfinite(z_deriv)||
       !std::isfinite(max_gap_seconds)||max_gap_seconds<=0)
      return fail("VERTICAL_FEEDBACK_INPUT_INVALID");
    // 源采样时间回退意味着因果顺序失效，禁止用负 dt 更新窗口。
    if(last_stamp_&&stamp<last_stamp_)return fail("VERTICAL_FEEDBACK_TIME_REGRESSION");
    // 重复样本不提供新的时间、积分面积或持续异常计数。
    if(stamp==last_stamp_)return e_; // cached input supplies no duration/evidence
    // 只在严格递增采样间计算秒数；首帧 dt=0，只建立窗口锚点。
    const double dt=last_stamp_?double(stamp-last_stamp_)*1e-6:0.;
    // 跨过允许采样间隔后不能假装中间轨迹已知，保持样本缺失故障。
    if(last_stamp_&&dt>max_gap_seconds)return fail("VERTICAL_FEEDBACK_SAMPLE_GAP");
    e_.sample_stamp=stamp;
    // 两个输出虽同为向下速度，却可能使用不同修正算法；差值用于观察，不能当成独立高度真值。
    e_.rate_error_mps=std::abs(vz-z_deriv);
    e_.suspect=e_.rate_error_mps>c_.rate_tolerance_mps;
    // 仅连续越限的新样本累计持续时间和数量；至少三帧并满足 dwell 才给 rate_warning。
    if(e_.suspect){
      if(!bad_since_)bad_since_=stamp;
      ++e_.disagreement_samples;
      e_.disagreement_seconds=double(stamp-bad_since_)*1e-6;
      // PX4 produces vz and z_deriv with different correction algorithms.
      // Keep the observation without treating their difference as a fault.
      e_.rate_warning=e_.disagreement_seconds>=c_.dwell_seconds&&e_.disagreement_samples>=3;
    }else{
      // 回到阈值内就重置速度差诊断窗口；这不清除已经锁存的硬故障。
      bad_since_=0;e_.disagreement_samples=0;e_.disagreement_seconds=0;e_.rate_warning=false;
    }
    // Integrate the position derivative over strictly new PX4 samples.
    // vz is a different EKF output, monitored above; integrating vz
    // here would double-count that disagreement and position corrections.
    // A window-average
    // residual tolerates ordinary sample noise without differentiating z at
    // the host loop frequency. Estimator resets are rejected by Core first.
    // 首个可信新样本保存窗口起点 z；之后积分 z_deriv，避免把 vz 的独立修正重复计入。
    if(!last_stamp_){window_stamp_=stamp;window_z_=z;window_samples_=1;}
    else{
      // 梯形积分：相邻 z_deriv 的均值乘 dt，得到本窗口 NED 向下位移 m。
      integrated_rate_+=(last_rate_+z_deriv)*.5*dt;
      ++window_samples_;
      const double window=double(stamp-window_stamp_)*1e-6;
      // 有足够时长且至少三帧才比较位置变化与积分，不在主机循环频率上对噪声位置差分。
      if(window>=c_.dwell_seconds&&window_samples_>=3){
        e_.position_window_checked=true;
        // 窗口平均残差 = |实际 z 变化 - z_deriv 积分|/窗口秒数；它与前面的 vz 差诊断是两个独立证据。
        e_.position_rate_error_mps=std::abs((z-window_z_)-integrated_rate_)/window;
        // 位置和其导数长时间不一致仍保持原硬故障；本轮中文化不放宽该保护。
        if(e_.position_rate_error_mps>c_.rate_tolerance_mps)
          return fail("VERTICAL_POSITION_RATE_INCONSISTENT");
        // 完成一轮后从当前帧开启下个窗口，积分清零；相邻帧的 last_rate_ 稍后才更新。
        window_stamp_=stamp;window_z_=z;integrated_rate_=0;window_samples_=1;
      }
    }
    // 最后提交本帧采样水位和导数；前面拒绝的帧不会成为下一次积分的合法起点。
    last_stamp_=stamp;last_rate_=z_deriv;return e_;
  }
 private:
  // 集中锁存不健康、可疑及首个故障原因；调用后 observe 的早退防止正常尾帧洗掉证据。
  const VerticalFeedbackEvidence& fail(const char* reason){
    e_.healthy=false;e_.suspect=true;e_.reason=reason;return e_;
  }
  VerticalFeedbackConfig c_;VerticalFeedbackEvidence e_;
  // 分别记录上次积分样本、速度差首次越限、位置积分窗口起点的 PX4 微秒时间。
  std::uint64_t last_stamp_{},bad_since_{},window_stamp_{};
  // 上次向下导数 m/s、窗口起点向下位置 m、窗口积分向下位移 m；三者必须在同一估计器参考中。
  double last_rate_{},window_z_{},integrated_rate_{};
  std::uint32_t window_samples_{};
};
} // namespace fc01::attitude
