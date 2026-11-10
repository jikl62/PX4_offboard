#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fc01::attitude {
struct VerticalFeedbackConfig {
  // Opt in explicitly in the deployed YAML; legacy synthetic Core callers
  // need not manufacture a PX4 z_deriv or a source timestamp.
  bool enabled{};
  // Velocity-output differences are diagnostic only. These existing values
  // also bound the independent position/derivative consistency window.
  double rate_tolerance_mps{.20},dwell_seconds{.50};
  void validate() const{
    if(!std::isfinite(rate_tolerance_mps)||rate_tolerance_mps<=0||rate_tolerance_mps>1||
       !std::isfinite(dwell_seconds)||dwell_seconds<.2||dwell_seconds>2)
      throw std::invalid_argument("INVALID_VERTICAL_FEEDBACK_GUARD_PARAMETERS");
  }
};
struct VerticalFeedbackEvidence {
  // While healthy, suspect/rate_warning report velocity-output disagreement.
  // Hard input, clock and position-consistency faults use healthy/reason.
  bool checked{},healthy{true},suspect{},position_window_checked{};
  bool rate_warning{};
  double rate_error_mps{},position_rate_error_mps{},disagreement_seconds{};
  std::uint64_t sample_stamp{};
  std::uint32_t disagreement_samples{};
  std::string reason;
};

// Consistency only: z, vz and z_deriv share an estimator. Agreement cannot
// prove actual altitude or rule out common-mode drift. Never select a new
// height/velocity source or rebase a flight reference here.
class VerticalFeedbackGuard {
 public:
  explicit VerticalFeedbackGuard(VerticalFeedbackConfig config={}):c_(config){c_.validate();}
  void reset(){*this=VerticalFeedbackGuard(c_);}
  const VerticalFeedbackEvidence& evidence() const{return e_;}
  const VerticalFeedbackEvidence& observe(std::uint64_t stamp,double z,double vz,
                                         double z_deriv,double max_gap_seconds){
    if(!c_.enabled)return e_;
    if(!e_.healthy)return e_; // a fault cannot self-clear in flight
    e_.checked=true;e_.sample_stamp=stamp;
    if(!stamp||!std::isfinite(z)||!std::isfinite(vz)||!std::isfinite(z_deriv)||
       !std::isfinite(max_gap_seconds)||max_gap_seconds<=0)
      return fail("VERTICAL_FEEDBACK_INPUT_INVALID");
    if(last_stamp_&&stamp<last_stamp_)return fail("VERTICAL_FEEDBACK_TIME_REGRESSION");
    if(stamp==last_stamp_)return e_; // cached input supplies no duration/evidence
    const double dt=last_stamp_?double(stamp-last_stamp_)*1e-6:0.;
    if(last_stamp_&&dt>max_gap_seconds)return fail("VERTICAL_FEEDBACK_SAMPLE_GAP");
    e_.sample_stamp=stamp;
    e_.rate_error_mps=std::abs(vz-z_deriv);
    e_.suspect=e_.rate_error_mps>c_.rate_tolerance_mps;
    if(e_.suspect){
      if(!bad_since_)bad_since_=stamp;
      ++e_.disagreement_samples;
      e_.disagreement_seconds=double(stamp-bad_since_)*1e-6;
      // PX4 produces vz and z_deriv with different correction algorithms.
      // Keep the observation without treating their difference as a fault.
      e_.rate_warning=e_.disagreement_seconds>=c_.dwell_seconds&&e_.disagreement_samples>=3;
    }else{
      bad_since_=0;e_.disagreement_samples=0;e_.disagreement_seconds=0;e_.rate_warning=false;
    }
    // Integrate the position derivative over strictly new PX4 samples.
    // vz is a different EKF output, monitored above; integrating vz
    // here would double-count that disagreement and position corrections.
    // A window-average
    // residual tolerates ordinary sample noise without differentiating z at
    // the host loop frequency. Estimator resets are rejected by Core first.
    if(!last_stamp_){window_stamp_=stamp;window_z_=z;window_samples_=1;}
    else{
      integrated_rate_+=(last_rate_+z_deriv)*.5*dt;
      ++window_samples_;
      const double window=double(stamp-window_stamp_)*1e-6;
      if(window>=c_.dwell_seconds&&window_samples_>=3){
        e_.position_window_checked=true;
        e_.position_rate_error_mps=std::abs((z-window_z_)-integrated_rate_)/window;
        if(e_.position_rate_error_mps>c_.rate_tolerance_mps)
          return fail("VERTICAL_POSITION_RATE_INCONSISTENT");
        window_stamp_=stamp;window_z_=z;integrated_rate_=0;window_samples_=1;
      }
    }
    last_stamp_=stamp;last_rate_=z_deriv;return e_;
  }
 private:
  const VerticalFeedbackEvidence& fail(const char* reason){
    e_.healthy=false;e_.suspect=true;e_.reason=reason;return e_;
  }
  VerticalFeedbackConfig c_;VerticalFeedbackEvidence e_;
  std::uint64_t last_stamp_{},bad_since_{},window_stamp_{};
  double last_rate_{},window_z_{},integrated_rate_{};
  std::uint32_t window_samples_{};
};
} // namespace fc01::attitude
