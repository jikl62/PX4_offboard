#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc01::attitude {
struct LiftoffEvidence {
  bool confirmed{},detector_confirmed{},rise_confirmed{};
  unsigned detector_samples{},local_samples{};
  std::uint64_t first_land_stamp{},last_land_stamp{},first_local_stamp{},last_local_stamp{};
};

// The detector and local position arrive independently. Keep each source's
// evidence separately: an early !landed callback is not consumed merely
// because its companion local callback has not crossed the height threshold.
// Neither a detector sequence nor a single high position/velocity can certify
// liftoff. Agreement still cannot exclude common-mode estimator drift.
class TakeoffEvidence {
 public:
  void reset(){*this=TakeoffEvidence{};}
  const LiftoffEvidence& evidence()const{return e_;}
  template<class C>bool observe(const C& c,double now,std::uint64_t land_baseline,
      std::uint64_t land_stamp,bool landed,std::uint64_t local_stamp,double height,double up_speed){
    e_.confirmed=false;
    if(!std::isfinite(now)||!std::isfinite(height)||!std::isfinite(up_speed)){
      reset();return false;
    }
    const double detector_gap=std::min(c.land_sample_max_gap,c.land_timeout*.5);
    if((land_seen_&&land_stamp<land_seen_)||(local_seen_&&local_stamp<local_seen_)||
       (last_observed_>=0&&now<last_observed_)){
      reset();return false;
    }
    last_observed_=now;
    if(landed){
      clear_detector();
      // A real bounce after !landed invalidates the earlier rise episode.
      if(detector_was_airborne_)clear_local();
      detector_was_airborne_=false;
    }
    if(land_stamp>land_seen_){
      land_seen_=land_stamp;
      if(!landed&&land_stamp>land_baseline){
        if(land_last_time_>=0&&(now-land_last_time_>detector_gap||
            double(land_stamp-e_.last_land_stamp)*1e-6>detector_gap))clear_detector();
        if(!e_.detector_samples){land_first_time_=now;e_.first_land_stamp=land_stamp;}
        land_last_time_=now;e_.last_land_stamp=land_stamp;++e_.detector_samples;
        detector_was_airborne_=true;
        e_.detector_confirmed=e_.detector_samples>=2&&land_stamp>e_.first_land_stamp&&
          now-land_first_time_+1e-9>=c.liftoff_dwell_seconds&&
          double(land_stamp-e_.first_land_stamp)*1e-6+1e-9>=c.liftoff_dwell_seconds;
      }
    }
    if(land_last_time_>=0&&now-land_last_time_>detector_gap)clear_detector();
    if(local_stamp>local_seen_){
      if(local_last_time_>=0&&(now-local_last_time_>c.local_timeout||
          double(local_stamp-local_seen_)*1e-6>c.local_timeout))clear_local();
      local_seen_=local_stamp;
      // The bounded join permits a slow detector to arrive after braking,
      // but new envelopes at a fixed height cannot renew an old rise forever.
      if(e_.rise_confirmed&&(motion_at_<0||now-motion_at_>c.land_timeout||
          double(local_stamp-motion_stamp_)*1e-6>c.land_timeout))clear_rise();
      if(height>=c.liftoff_height_m&&up_speed>=c.liftoff_speed_mps&&height>latest_height_+1e-9){
        motion_at_=now;motion_stamp_=local_stamp;
      }
      local_last_time_=now;latest_height_=height;
      if(height<c.liftoff_height_m)clear_rise();
      if(!e_.rise_confirmed){
        if(height<c.liftoff_height_m||up_speed<c.liftoff_speed_mps)clear_rise();
        else{
          if(!e_.local_samples){local_first_time_=now;first_height_=height;e_.first_local_stamp=local_stamp;}
          e_.last_local_stamp=local_stamp;++e_.local_samples;
          e_.rise_confirmed=e_.local_samples>=2&&local_stamp>e_.first_local_stamp&&
            now-local_first_time_+1e-9>=c.liftoff_dwell_seconds&&
            double(local_stamp-e_.first_local_stamp)*1e-6+1e-9>=c.liftoff_dwell_seconds&&
            height-first_height_+1e-9>=c.liftoff_speed_mps*c.liftoff_dwell_seconds;
        }
      }else e_.last_local_stamp=local_stamp;
    }
    if(local_last_time_>=0&&now-local_last_time_>c.local_timeout)clear_local();
    // Fresh local observations may bridge braking only within land_timeout
    // of measured upward motion, using both source and receiver time.
    e_.confirmed=!landed&&e_.detector_confirmed&&e_.rise_confirmed&&
      latest_height_>=c.liftoff_height_m&&local_last_time_>=0&&
      now-local_last_time_<=c.local_timeout&&land_last_time_>=0&&now-land_last_time_<=detector_gap;
    return e_.confirmed;
  }
 private:
  void clear_detector(){e_.detector_confirmed=false;e_.detector_samples=0;
    e_.first_land_stamp=e_.last_land_stamp=0;land_first_time_=land_last_time_=-1;}
  void clear_rise(){e_.rise_confirmed=false;e_.local_samples=0;
    e_.first_local_stamp=e_.last_local_stamp=0;local_first_time_=motion_at_=-1;motion_stamp_=0;}
  void clear_local(){clear_rise();local_last_time_=-1;}
  LiftoffEvidence e_;
  std::uint64_t land_seen_{},local_seen_{},motion_stamp_{};
  double land_first_time_{-1},land_last_time_{-1},local_first_time_{-1},local_last_time_{-1},last_observed_{-1};
  double first_height_{},latest_height_{},motion_at_{-1};
  bool detector_was_airborne_{};
};
} // namespace fc01::attitude
