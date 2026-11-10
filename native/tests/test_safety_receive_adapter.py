"""Run the real node callback, receive scheduler and tick with queued samples.

The ROS API, clocks and DDS queue are synthetic. This exercises the production
local subscription callback, Safety observer, publication and reprepare gates.
It does not execute ROS/DDS, connect to a device, or validate real flight.
"""
from pathlib import Path
import json
import os
import subprocess
import tempfile
import unittest

import test_rc_recovery_adapter as recovery


HARNESS = recovery.HARNESS.split('\nint main(){', 1)[0] + r'''
#include <deque>
#include <limits>

struct SafetyReceiveNode:RecoveryNode {
  int payload_case{};
  bool resting{},takeover_reported{},height_excursion{};
  std::uint64_t raw_override{};
  std::deque<Local> local_queue;
  std::deque<Attitude> attitude_queue;
  unsigned passes{},retained_events{};
  bool old_core_hint{},payload_event{};
  fc01::SourceSlot timing_slot{fc01::SourceSlot::Count};
  int timing_case{};
  std::string timing_source_seen,timing_reason_seen;

  std::uint64_t tail_stamp(fc01::SourceSlot slot)const{
    return timing_slot==slot&&timing_case==5?stamp+100002:stamp;
  }
  void timing_head(){
    if(!timing_case)return;
    const auto expired_us=std::uint64_t(node.source_timeout(timing_slot)*1000000)+100000;
    std::uint64_t envelope=stamp-110000,sample=envelope;
    if(timing_case==1)sample=0;
    if(timing_case==2)sample=envelope+1;
    if(timing_case==3)envelope=0;
    if(timing_case==4)sample=envelope=stamp-expired_us;
    if(timing_case==5)envelope=stamp+100001;
    if(timing_case==6)sample=stamp-expired_us;
    if(timing_slot==fc01::SourceSlot::Local){
      auto value=node.local_;value.timestamp=envelope;value.timestamp_sample=sample;
      node.local_sub_->callback(std::make_shared<Local>(value),rclcpp::MessageInfo{});
    }else if(timing_slot==fc01::SourceSlot::Attitude){
      auto value=node.attitude_;value.timestamp=envelope;value.timestamp_sample=sample;
      node.receive_sampled(value,node.attitude_,node.attitude_stamp_,node.attitude_sample_,
        fc01::SourceSlot::Attitude,rclcpp::MessageInfo{});
    }else if(timing_slot==fc01::SourceSlot::Status){
      auto value=node.status_;value.timestamp=envelope;
      node.status_sub_->callback(std::make_shared<Status>(value),rclcpp::MessageInfo{});
    }else if(timing_slot==fc01::SourceSlot::Land){
      auto value=node.land_;value.timestamp=envelope;
      node.land_sub_->callback(std::make_shared<Land>(value),rclcpp::MessageInfo{});
    }else if(timing_slot==fc01::SourceSlot::Flags){
      auto value=node.flags_;value.timestamp=envelope;
      node.flags_sub_->callback(std::make_shared<Flags>(value),rclcpp::MessageInfo{});
    }else if(timing_slot==fc01::SourceSlot::Battery){
      auto value=node.battery_;value.timestamp=envelope;
      node.battery_sub_->callback(std::make_shared<Battery>(value),rclcpp::MessageInfo{});
    }
    // A following callback can be valid 100 us later while its timestamp
    // advances past the initially-too-far-future envelope. No clock rewind.
    if(timing_case==5){recovery_test_steady+=.0001;recovery_test_wall_ns+=100000;}
  }

  void enqueue(std::uint64_t raw,bool valid=true){
    Local value{};value.timestamp=raw;value.timestamp_sample=raw_override?raw_override:raw;
    value.z=payload_case==1?1.f:2.f;value.z_valid=valid;value.v_z_valid=true;
    if(payload_case==2)value.x=value.y=value.vx=value.vy=std::numeric_limits<float>::quiet_NaN();
    Attitude pose{};pose.timestamp=raw;pose.timestamp_sample=raw_override?raw_override:raw;pose.q={1,0,0,0};
    local_queue.push_back(value);attitude_queue.push_back(pose);
  }
  void context(int unsafe){
    Status status{};status.timestamp=tail_stamp(fc01::SourceSlot::Status);status.system_id=status.component_id=1;
    status.arming_state=Status::ARMING_STATE_DISARMED;status.nav_state=Status::NAVIGATION_STATE_STAB;
    status.pre_flight_checks_pass=status.power_input_valid=true;
    status.failsafe_and_user_took_over=takeover_reported;
    status.can_set_nav_states_mask=1U<<Status::NAVIGATION_STATE_STAB;
    if(unsafe==3||unsafe==4){
      auto bad=status;bad.timestamp=stamp-1000;
      if(unsafe==3)bad.nav_state=1;else bad.system_id=2;
      node.receive_event(bad,node.status_,node.status_stamp_,fc01::SourceSlot::Status,
        rclcpp::MessageInfo{},fc01::status_receive_event<Status>);
    }
    node.receive_event(status,node.status_,node.status_stamp_,fc01::SourceSlot::Status,
      rclcpp::MessageInfo{},fc01::status_receive_event<Status>);
    Land land{};land.timestamp=tail_stamp(fc01::SourceSlot::Land);land.landed=true;land.at_rest=resting;
    if(unsafe==1){
      auto bad=land;bad.timestamp=stamp-1000;bad.landed=false;
      node.receive_event(bad,node.land_,node.land_stamp_,fc01::SourceSlot::Land,
        rclcpp::MessageInfo{},fc01::land_receive_event<Land>);
    }
    node.receive_event(land,node.land_,node.land_stamp_,fc01::SourceSlot::Land,
      rclcpp::MessageInfo{},fc01::land_receive_event<Land>);
    Battery battery{};battery.timestamp=tail_stamp(fc01::SourceSlot::Battery);battery.id=1;battery.connected=true;battery.voltage_v=7.6f;
    node.receive_event(battery,node.battery_,node.battery_stamp_,fc01::SourceSlot::Battery,
      rclcpp::MessageInfo{},fc01::battery_receive_event<Battery>);
    Flags flags{};flags.timestamp=tail_stamp(fc01::SourceSlot::Flags);
    node.receive_event(flags,node.flags_,node.flags_stamp_,fc01::SourceSlot::Flags,
      rclcpp::MessageInfo{},fc01::flags_receive_event<Flags>);
    Manual manual{};manual.timestamp=manual.timestamp_sample=stamp;manual.valid=true;
    manual.data_source=Manual::SOURCE_RC;manual.throttle=-1;node.receive_manual(manual);
    Switches switches{};switches.timestamp=switches.timestamp_sample=stamp;
    switches.mode_slot=6;switches.arm_switch=switches.kill_switch=3;node.receive_switches(switches);
  }
  void receive_cycle(bool force_old_schedule=false,int unsafe=0){
    const bool expiry=timing_case==4||timing_case==6;
    const double elapsed=expiry?node.source_timeout(timing_slot)+.2:.05;
    now+=elapsed;stamp+=std::uint64_t(elapsed*1000000);
    recovery_test_steady=now;recovery_test_wall_ns=std::int64_t(stamp)*1000;
    node.begin_cycle(now);
    // A 20 Hz producer and a depth-ten queue with 100 ms before queueing.
    // One callback per 20 Hz control tick cannot consume the initial backlog.
    if(local_queue.empty()&&!node.local_stamp_.value){
      for(unsigned i=0;i<10;++i)enqueue(stamp-550000+i*50000);
    }else{
      if(unsafe==2){enqueue(node.local_.timestamp+10000,false);enqueue(node.local_.timestamp+20000);}
      if(height_excursion){enqueue(stamp-125000);local_queue.back().z+=.20f;}
      enqueue(stamp-100000);
    }
    while(local_queue.size()>10){local_queue.pop_front();attitude_queue.pop_front();}
    fc01::ReceivePassSchedule receive;
    receive.begin(now,node.receive_budget_seconds(),node.received_callbacks(),node.receive_events(),
      force_old_schedule||node.stop_receive_on_events());
    bool first=true;retained_events=0;
    while(receive.next_pass(recovery_test_steady,node.received_callbacks(),node.receive_events())){
      if(first){timing_head();context(unsafe);first=false;}
      if(!local_queue.empty()){
        auto value=local_queue.front();local_queue.pop_front();
        if(timing_slot==fc01::SourceSlot::Local&&timing_case==5)
          value.timestamp=tail_stamp(fc01::SourceSlot::Local);
        const bool vertical_only=node.config_.profile=="attitude_altitude"&&
          !node.config_.attitude.visual.enabled&&!node.config_.attitude.px4_land_fallback_confirmed;
        payload_event=fc01::local_receive_event(node.local_,value,node.config_.attitude,vertical_only);
        old_core_hint=node.runtime_->attitude_core().local_requires_control_step(
          value.timestamp_sample,value.z,value.vz,value.z_deriv);
        const auto before=node.receive_events();
        // Invoke the exact callback registered by SystemNode's constructor.
        node.local_sub_->callback(std::make_shared<Local>(value),rclcpp::MessageInfo{});
        retained_events+=unsigned(node.receive_events()-before);
        auto pose=attitude_queue.front();attitude_queue.pop_front();
        if(timing_slot==fc01::SourceSlot::Attitude&&timing_case==5)
          pose.timestamp=tail_stamp(fc01::SourceSlot::Attitude);
        node.receive_sampled(pose,node.attitude_,node.attitude_stamp_,node.attitude_sample_,
          fc01::SourceSlot::Attitude,rclcpp::MessageInfo{});
      }
      node.receive_pass_done();recovery_test_steady+=.0001;recovery_test_wall_ns+=100000;
    }
    passes=unsigned(receive.pass_count());node.receive_stopped(receive);
    timing_source_seen=node.receive_timing_block_source_==fc01::SourceSlot::Count?"":
      fc01::source_slot_name(node.receive_timing_block_source_);
    timing_reason_seen=node.receive_timing_block_reason_;
    node.callbacks_done(recovery_test_steady);assert(node.tick());
    node.observe_cycle(now,recovery_test_steady,recovery_test_steady);
    char output[32768];while(::recv(observer,output,sizeof(output),0)>0){}
    assert(node.command_pub_->calls==0&&!fc01::attitude::nonzero(node.last_attitude_output_.thrust));
  }
};

static void backlog_reproduces_old_stall_and_catches_up_only_in_stopped_safety(){
  for(int payload:{0,1,2})for(bool force_old:{false,true}){
    SafetyReceiveNode n;n.payload_case=payload;
    unsigned one_pass=0,retained=0;
    for(unsigned tick=0;tick<80;++tick){
      n.receive_cycle(force_old);
      assert(n.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::Safety);
      assert(!n.node.last_attitude_output_.publish&&n.node.runtime_->rc_attempts_completed()==0);
      if(tick>=5){one_pass+=n.passes==1;retained+=n.retained_events;}
    }
    auto snapshot=n.node.snapshot(n.now);
    const double age=double(n.stamp-n.node.local_.timestamp)*1e-6;
    assert(n.node.config_.attitude.local_timeout==.5&&n.node.config_.attitude.attitude_timeout==.5);
    if(force_old&&payload==1){
      assert(one_pass==75&&retained==75&&n.local_queue.size()==9);
      assert(age>=.55-1e-9&&!snapshot.attitude.source_clock_valid&&!snapshot.local_fresh);
    }else{
      assert(one_pass==0&&n.local_queue.empty()&&age<.11&&snapshot.attitude.source_clock_valid);
    }
    if(payload==1)assert(n.old_core_hint&&!n.payload_event);
    if(payload==2)assert(!n.old_core_hint&&!n.payload_event);
    assert(!n.node.receive_event_overflow_&&n.node.clock_event_count_==0);
    // Landed alone cannot establish rest, even after the queue is current.
    assert(!n.node.land_.at_rest&&!n.node.runtime_->attitude_core().rc_fault_recovery_ready());
    n.takeover_reported=force_old&&payload==2;n.receive_cycle(force_old);
    std::cout<<n.node.state(n.node.snapshot(n.now),n.now)<<std::endl;
    std::cout<<"SAFETY_BACKLOG_PASS payload="<<payload<<" old_schedule="<<force_old<<"\n";
  }
}

static void every_unsafe_payload_still_vetoes_recovery_behind_a_healthy_tail(){
  for(int unsafe:{1,2,3,4}){
    SafetyReceiveNode n;n.payload_case=1;
    for(unsigned tick=0;tick<10;++tick)n.receive_cycle();
    n.resting=true;
    for(unsigned tick=0;tick<45;++tick)n.receive_cycle();
    assert(n.node.runtime_->attitude_core().rc_recovery_diagnostics().window_active);
    n.receive_cycle(false,unsafe);
    const auto d=n.node.runtime_->attitude_core().rc_recovery_diagnostics();
    assert(n.passes>1&&n.node.land_.landed&&n.node.land_.at_rest&&n.node.local_.z_valid);
    assert(d.event_blocked&&!d.ready&&!d.window_active&&n.node.runtime_->rc_attempts_completed()==0);
    const char* reasons[]={"", "RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_LANDED_AT_REST_REQUIRED",
      "RC_RECOVERY_QUEUED_EVENT_Z_VZ_STALE_OR_INVALID",
      "RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_STABILIZED_REQUIRED",
      "RC_RECOVERY_QUEUED_EVENT_PUBLISHER_OR_TELEMETRY_CONFLICT"};
    assert(d.block_reason==reasons[unsafe]);
    n.receive_cycle();assert(!n.node.runtime_->attitude_core().rc_recovery_diagnostics().event_blocked);
  }
  std::cout<<"SAFETY_CATCHUP_UNSAFE_PAYLOAD_VETO_PASS\n";
}

static void unsafe_timing_cannot_borrow_a_healthy_tails_clock(){
  using Slot=fc01::SourceSlot;
  for(const auto slot:{Slot::Local,Slot::Attitude,Slot::Status,Slot::Land,Slot::Flags,Slot::Battery}){
    for(const int kind:{1,2,3,4,5,6}){
      const bool sampled=slot==Slot::Local||slot==Slot::Attitude;
      if(!sampled&&(kind==1||kind==2||kind==6))continue;
      SafetyReceiveNode n;
      for(unsigned tick=0;tick<10;++tick)n.receive_cycle();
      n.resting=true;
      for(unsigned tick=0;tick<45;++tick)n.receive_cycle();
      assert(n.node.runtime_->attitude_core().rc_recovery_diagnostics().window_active);
      n.timing_slot=slot;n.timing_case=kind;n.receive_cycle();
      const auto current=n.node.snapshot(n.now);
      const auto d=n.node.runtime_->attitude_core().rc_recovery_diagnostics();
      // The continuous source clock has a valid, nonregressing healthy tail.
      // A long expiry gap may additionally retain stale RC authority facts.
      assert(n.passes>1&&current.attitude.source_clock_valid&&!current.time_regressed);
      assert(d.event_blocked&&!d.window_active&&!d.ready);
      const bool rc_expiry=kind==4&&n.node.source_timeout(slot)>=2.5;
      assert(d.block_reason==(rc_expiry?"RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_FRESH_SAFE_RC_REQUIRED":
        "RC_RECOVERY_QUEUED_EVENT_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE"));
      const char* reasons[]={"", "MISSING_SAMPLE", "SAMPLE_AFTER_ENVELOPE", "MISSING_ENVELOPE",
        "ENVELOPE_EXPIRED", "ENVELOPE_TOO_FAR_FUTURE", "SAMPLE_EXPIRED"};
      assert(n.timing_source_seen==fc01::source_slot_name(slot)&&n.timing_reason_seen==reasons[kind]);
      assert(n.node.receive_timing_block_source_==Slot::Count&&
        std::string(n.node.receive_timing_block_reason_).empty());
      n.timing_slot=Slot::Count;n.timing_case=0;
      if(kind==5){n.now+=.2;n.stamp+=200000;}
      n.receive_cycle();
      if(rc_expiry)n.receive_cycle(); // Restore both required advancing RC observations.
      assert(!n.node.runtime_->attitude_core().rc_recovery_diagnostics().event_blocked);
      assert(n.node.runtime_->attitude_core().rc_recovery_diagnostics().window_active);
      assert(n.node.runtime_->rc_attempts_completed()==0);
    }
  }
  // A callback delivered before begin_cycle must survive until it is scanned;
  // a later bad source must not overwrite the first diagnostic cause.
  {
  SafetyReceiveNode n;
  for(unsigned tick=0;tick<10;++tick)n.receive_cycle();
  n.resting=true;for(unsigned tick=0;tick<45;++tick)n.receive_cycle();
  n.now+=.05;n.stamp+=50000;recovery_test_steady=n.now;recovery_test_wall_ns=std::int64_t(n.stamp)*1000;
  n.timing_slot=Slot::Local;n.timing_case=1;n.timing_head();
  n.timing_slot=Slot::Attitude;n.timing_case=2;n.timing_head();
  n.timing_slot=Slot::Count;n.timing_case=0;n.receive_cycle();
  assert(n.timing_source_seen=="local"&&n.timing_reason_seen=="MISSING_SAMPLE");
  assert(n.node.snapshot(n.now).attitude.source_clock_valid&&!n.node.snapshot(n.now).time_regressed);
  assert(n.node.runtime_->attitude_core().rc_recovery_diagnostics().event_blocked);
  }
  {
    SafetyReceiveNode invalid_receipt;
    for(unsigned tick=0;tick<10;++tick)invalid_receipt.receive_cycle();
    invalid_receipt.resting=true;
    for(unsigned tick=0;tick<45;++tick)invalid_receipt.receive_cycle();
    auto value=invalid_receipt.node.local_;value.timestamp+=1000;value.timestamp_sample=value.timestamp;
    recovery_test_steady=std::numeric_limits<double>::quiet_NaN();
    invalid_receipt.node.local_sub_->callback(std::make_shared<Local>(value),rclcpp::MessageInfo{});
    invalid_receipt.receive_cycle();
    assert(invalid_receipt.timing_source_seen=="local"&&invalid_receipt.timing_reason_seen=="MISSING_RECEIPT");
    assert(invalid_receipt.node.snapshot(invalid_receipt.now).attitude.source_clock_valid);
    assert(invalid_receipt.node.runtime_->attitude_core().rc_recovery_diagnostics().event_blocked);
  }
  std::cout<<"SAFETY_CATCHUP_UNSAFE_TIMING_AND_FIRST_CAUSE_PASS\n";
}

static void current_resting_ground_can_finish_a_new_independent_recovery(){
  for(int payload:{1,2}){
    SafetyReceiveNode n;n.payload_case=payload;
    for(unsigned tick=0;tick<10;++tick)n.receive_cycle();
    n.resting=true;const auto returned=n.now;
    unsigned ticks=0;
    while(n.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::RcWaitArm&&ticks++<180){
      n.receive_cycle();
      if(n.now-returned<4.)assert(n.node.runtime_->rc_attempts_completed()==0);
    }
    assert(n.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcWaitArm);
    assert(n.node.runtime_->rc_attempts_completed()==1);
    assert(n.node.runtime_->last_rc_attempt_reason()=="OPERATOR_TAKEOVER_LATCHED");
    assert(!n.node.runtime_->attitude_core().flight_started()&&n.node.last_attitude_output_.publish);
  }
  std::cout<<"SAFETY_CATCHUP_FRESH_GROUND_RECOVERY_PASS\n";
}

static void catching_up_envelopes_cannot_refresh_frozen_or_regressed_samples(){
  SafetyReceiveNode n;n.payload_case=1;
  for(unsigned tick=0;tick<10;++tick)n.receive_cycle();
  n.raw_override=n.node.local_sample_.value;
  const auto raw_seen=n.node.local_sample_.received;
  for(unsigned tick=0;tick<20;++tick)n.receive_cycle();
  auto snapshot=n.node.snapshot(n.now);
  assert(n.node.local_stamp_.fresh(n.now,.5)&&n.node.local_sample_.received==raw_seen);
  assert(!snapshot.local_fresh&&!snapshot.attitude.local_sample_fresh&&!snapshot.attitude.source_clock_valid);
  assert(!snapshot.attitude.attitude_sample_fresh&&n.node.runtime_->rc_attempts_completed()==0);
  --n.raw_override;n.receive_cycle();
  assert(n.node.local_sample_.regressed&&n.node.attitude_sample_.regressed);
  assert(n.node.snapshot(n.now).time_regressed);
  n.raw_override=0;n.receive_cycle();
  assert(n.node.snapshot(n.now).time_regressed&&!n.node.last_attitude_output_.publish);
  std::cout<<"SAFETY_CATCHUP_FROZEN_AND_REGRESSED_SAMPLE_GATES_PASS\n";
}

static void catchup_never_changes_active_or_integrity_gates(){
  SafetyReceiveNode n;assert(!n.node.stop_receive_on_events());
  n.node.last_attitude_output_.publish=true;assert(n.node.stop_receive_on_events());
  n.node.last_attitude_output_.publish=false;
  n.node.config_.profile="sim";assert(n.node.stop_receive_on_events());
  n.node.config_.profile="attitude_altitude";
  Flight prepared(production_no_vision_rc_config());prepared.warm();
  n.node.runtime_=std::make_unique<fc01::Runtime>(std::move(prepared.r));
  assert(n.node.stop_receive_on_events());
  n.node.runtime_->abort_attitude("INTEGRITY_TEST");assert(!n.node.stop_receive_on_events());
  n.receive_cycle();
  for(unsigned i=0;i<257;++i)n.node.latch_event(n.node.local_events_,n.node.local_);
  n.receive_cycle();
  assert(n.node.receive_event_overflow_&&n.node.snapshot(n.now).time_regressed);
  assert(n.node.runtime_->rc_attempts_completed()==0&&!n.node.last_attitude_output_.publish);
  std::cout<<"SAFETY_CATCHUP_ACTIVE_AND_OVERFLOW_GATES_PASS\n";
}

static void preparation_catches_up_with_unavailable_horizontal_estimates(){
  for(int payload:{0,2}){
    SafetyReceiveNode n;n.payload_case=payload;n.resting=true;
    n.node.runtime_=std::make_unique<fc01::Runtime>(production_no_vision_rc_config());
    for(unsigned i=0;i<180;++i)n.receive_cycle();
    assert(n.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcWaitArm);
    assert(n.local_queue.empty()&&n.node.snapshot(n.now).local_fresh);
    // A real scheduling gap fills the synthetic depth-ten source queues.
    // Recovery must work without changing the unavailable XY payload.
    n.now+=.5;n.stamp+=500000;
    for(unsigned i=0;i<9;++i)n.enqueue(n.stamp-450000+i*50000);
    for(unsigned i=0;i<180;++i)n.receive_cycle();
    assert(n.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcWaitArm);
    assert(n.local_queue.empty()&&n.node.snapshot(n.now).local_fresh);
    assert(n.node.runtime_->rc_attempts_completed()==0);
  }
  std::cout<<"PREPARATION_UNAVAILABLE_XY_BACKLOG_RECOVERY_PASS\n";
}

static void horizontal_consumers_keep_their_nonfinite_events(){
  for(int consumer:{0,1,2,3}){
    SafetyReceiveNode n;
    if(consumer==1)n.node.config_.profile="ground";
    if(consumer==2)n.node.config_.attitude.visual.enabled=true;
    if(consumer==3)n.node.config_.attitude.px4_land_fallback_confirmed=true;
    Local value{};value.z=2;value.z_valid=value.v_z_valid=true;
    value.x=value.y=value.vx=value.vy=value.heading=std::numeric_limits<float>::quiet_NaN();
    std::uint64_t before{};
    for(unsigned i=0;i<2;++i){
      n.now+=.05;n.stamp+=50000;recovery_test_steady=n.now;recovery_test_wall_ns=std::int64_t(n.stamp)*1000;
      value.timestamp=value.timestamp_sample=n.stamp;
      before=n.node.receive_events();
      n.node.local_sub_->callback(std::make_shared<Local>(value),rclcpp::MessageInfo{});
    }
    assert(n.node.receive_events()-before==unsigned(consumer!=0));
  }
  std::cout<<"HORIZONTAL_CONSUMERS_KEEP_INVALID_FEEDBACK_EVENTS_PASS\n";
}

static void finite_height_excursions_restart_both_ground_proof_windows(){
  for(bool safety:{false,true}){
    SafetyReceiveNode n;n.resting=true;
    if(!safety)n.node.runtime_=std::make_unique<fc01::Runtime>(production_no_vision_rc_config());
    bool window=false;
    for(unsigned i=0;i<100&&!window;++i){
      n.receive_cycle();const auto& core=n.node.runtime_->attitude_core();
      window=safety?(core.rc_recovery_diagnostics().window_active&&core.rc_recovery_diagnostics().elapsed_s>=1.):
        (core.phase()==fc01::attitude::Phase::RcPrestream&&core.rc_ground_reference_seconds()>=1.);
    }
    assert(window);
    // Causally ordered source stamps: previous tail, 25ms excursion, 25ms
    // healthy return. Both are before receipt and below the old .45m jump
    // threshold, but .20m exceeds the ground proof's .15m tolerance.
    n.height_excursion=true;n.receive_cycle();
    if(safety){const auto d=n.node.runtime_->attitude_core().rc_recovery_diagnostics();
      assert(d.event_blocked&&!d.window_active&&!d.ready);
      assert(d.block_reason=="RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_HEIGHT_DISCONTINUITY");
    }else assert(!n.node.runtime_->attitude_core().rc_ground_reference_qualified());
    for(unsigned i=0;i<100;++i){
      n.receive_cycle();
      assert(n.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::RcWaitArm);
      assert(n.node.runtime_->rc_attempts_completed()==0);
      assert(!n.node.snapshot(n.now).time_regressed);
    }
    n.height_excursion=false;const auto stable_since=n.now;
    for(unsigned i=0;i<180&&n.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::RcWaitArm;++i){
      n.receive_cycle();
      if(n.now-stable_since<2.)assert(n.node.runtime_->attitude_core().phase()!=fc01::attitude::Phase::RcWaitArm);
    }
    assert(n.node.runtime_->attitude_core().phase()==fc01::attitude::Phase::RcWaitArm);
    assert(n.node.runtime_->attitude_core().rc_ground_reference_qualified());
    assert(n.node.runtime_->rc_attempts_completed()==unsigned(safety));
  }
  std::cout<<"GROUND_PROOF_RETAINED_HEIGHT_EXCURSION_AND_STABLE_RETRY_PASS\n";
}

int main(){
  backlog_reproduces_old_stall_and_catches_up_only_in_stopped_safety();
  every_unsafe_payload_still_vetoes_recovery_behind_a_healthy_tail();
  unsafe_timing_cannot_borrow_a_healthy_tails_clock();
  current_resting_ground_can_finish_a_new_independent_recovery();
  catching_up_envelopes_cannot_refresh_frozen_or_regressed_samples();
  catchup_never_changes_active_or_integrity_gates();
  preparation_catches_up_with_unavailable_horizontal_estimates();
  horizontal_consumers_keep_their_nonfinite_events();
  finite_height_excursions_restart_both_ground_proof_windows();
}
'''


def write_fixture(root):
    recovery.write_fixture(root)
    path=root/'rclcpp/rclcpp.hpp'
    ros=path.read_text().replace('#include <functional>', '#include <functional>\n#include <type_traits>')
    old='template<class T>struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;};'
    new=('template<class T>struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;'
         'std::function<void(typename T::ConstSharedPtr,const MessageInfo&)> callback;};')
    if ros.count(old)!=1:
        raise AssertionError('Update callback capture for the ROS API fixture')
    ros=ros.replace(old,new,1)
    old=('template<class T,class Qos,class Callback>typename Subscription<T>::SharedPtr '
         'create_subscription(const std::string&,Qos,Callback){return std::make_shared<Subscription<T>>();}')
    new=('template<class T,class Qos,class Callback>typename Subscription<T>::SharedPtr '
         'create_subscription(const std::string&,Qos,Callback callback){'
         'auto sub=std::make_shared<Subscription<T>>();'
         'sub->callback=[callback](typename T::ConstSharedPtr msg,const MessageInfo& info){'
         'if constexpr(std::is_invocable_v<Callback,typename T::ConstSharedPtr,const MessageInfo&>)'
         'callback(msg,info);else callback(msg);};return sub;}')
    if ros.count(old)!=1:
        raise AssertionError('Update subscription construction for the ROS API fixture')
    path.write_text(ros.replace(old,new,1))
    (root/'test.cpp').write_text(HARNESS)


class SafetyReceiveAdapterTest(unittest.TestCase):
    def test_output_silent_safety_catches_up_without_losing_recovery_vetoes(self):
        with tempfile.TemporaryDirectory(prefix='fc01_safety_receive_') as directory:
            root=Path(directory);write_fixture(root)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                '-I',str(root),'-I',str(recovery.fixture.ROOT/'src/jikl_fc01_system/include'),
                '-I',str(recovery.fixture.ROOT/'tests'),str(root/'test.cpp'),'-o',str(root/'test')],check=True)
            env=os.environ.copy();env['FC01_SOCKET']=str(root/'control.sock')
            result=subprocess.run([str(root/'test')],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr+'\n'+result.stdout[-12000:])
            self.assertEqual(result.stdout.count('SAFETY_BACKLOG_PASS'),6)
            for marker in ('SAFETY_CATCHUP_UNSAFE_PAYLOAD_VETO_PASS',
                           'SAFETY_CATCHUP_UNSAFE_TIMING_AND_FIRST_CAUSE_PASS',
                           'SAFETY_CATCHUP_FRESH_GROUND_RECOVERY_PASS',
                           'SAFETY_CATCHUP_FROZEN_AND_REGRESSED_SAMPLE_GATES_PASS',
                           'SAFETY_CATCHUP_ACTIVE_AND_OVERFLOW_GATES_PASS',
                           'PREPARATION_UNAVAILABLE_XY_BACKLOG_RECOVERY_PASS',
                           'HORIZONTAL_CONSUMERS_KEEP_INVALID_FEEDBACK_EVENTS_PASS',
                           'GROUND_PROOF_RETAINED_HEIGHT_EXCURSION_AND_STABLE_RETRY_PASS'):
                self.assertIn(marker,result.stdout)
            states=[json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
            states=[row for row in states if row.get('kind')=='state']
            self.assertTrue(states)
            for row in states:
                for field in ('local_payload_z_valid','local_payload_vz_valid','px4_user_takeover_reported'):
                    self.assertIsInstance(row[field],bool)
                for field in ('receive_timing_block_source','receive_timing_block_reason'):
                    self.assertIsInstance(row[field],str)
            expired=[row for row in states if not row['local_fresh'] and row['local_payload_z_valid']]
            self.assertTrue(expired)
            self.assertTrue(all(not row['z_valid'] and not row['vz_valid'] and row['local_payload_vz_valid']
                                for row in expired))
            self.assertTrue(any(row['px4_user_takeover_reported'] for row in states))


if __name__=='__main__':
    unittest.main()
