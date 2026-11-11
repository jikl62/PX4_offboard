#pragma once
// 地面链路单次请求验证：只在未解锁、接地静止条件下测试 Stabilized 模式请求。
// Status/Land/Ack 来自 PX4 消息；now 为自测试开始计的单调秒，源 timestamp 单位 µs。
// 本 Core 不发解锁、推力或起飞命令；tick 返回 true 仅表示应发送一次 Command 176。
#include <cstddef>
#include <cstdint>
#include <string>

namespace jikl_fc01_ground_link {
// 保存源身份、模式和地面前提；system/component 必须指向预期飞控 1/1。
struct Status {
  std::uint64_t timestamp{};
  bool disarmed{}, failsafe{}, calibration{}, hil{}, stabilized_selectable{};
  int system{}, component{}, nav_state{};
};
struct Land {
  std::uint64_t timestamp{};
  bool landed{}, at_rest{};
};
struct Traffic {
  std::size_t status_publishers{}, land_publishers{}, ack_publishers{};
  std::size_t command_subscribers{}, command_publishers{};
};
struct Ack {
  std::uint64_t timestamp{};
  std::uint32_t command{};
  int result{}, system{}, component{};
  bool external{};
};
// Waiting 等地面稳定，Sent 等 ACK/新状态；其它值都是本轮终态，不能再次发请求。
enum class State { Waiting, Sent, Pass, Rejected, ModeUnconfirmed, AckTimeout, Blocked };
inline const char* name(State s) {
  switch (s) {
  case State::Waiting:
    return "WAITING";
  case State::Sent:
    return "SENT";
  case State::Pass:
    return "PASS";
  case State::Rejected:
    return "REJECTED";
  case State::ModeUnconfirmed:
    return "MODE_UNCONFIRMED";
  case State::AckTimeout:
    return "ACK_TIMEOUT";
  case State::Blocked:
    return "BLOCKED";
  }
  return "BLOCKED";
}
class Core {
public:
  // 这是地面链路验证Core，不是姿态高度Core；方法接收值快照并更新自己的记账，不保存外部引用。
  // 收到已解锁或倒退时间戳立即 Block；只有发送之后的新状态才能证明模式变化。
  void status(const Status& s, double now) {
    if (finished())
      return;
    if (!s.disarmed) {
      block("NOT_DISARMED");
      return;
    }
    if (s.timestamp != 0 && s.timestamp < status_.timestamp) {
      block("STATUS_TIMESTAMP_REGRESSION");
      return;
    }
    if (s.timestamp > status_.timestamp) {
      status_at_ = now;
      status_seen_ = true;
      if (sent_ && s.timestamp > before_send_timestamp_ && now >= sent_at_ && now - sent_at_ < 5)
        mode_confirmed_ = s.nav_state == 15;
    }
    status_ = s;
    if (s.timestamp == 0)
      status_seen_ = false;
  }
  // 着陆检测独立更新其到达寿命；时间戳为零撤销 seen，不能拿旧 landed 状态续期。
  void land(const Land& l, double now) {
    if (finished())
      return;
    if (l.timestamp != 0 && l.timestamp < land_.timestamp) {
      block("LAND_TIMESTAMP_REGRESSION");
      return;
    }
    if (l.timestamp > land_.timestamp) {
      land_at_ = now;
      land_seen_ = true;
    }
    land_ = l;
    if (l.timestamp == 0)
      land_seen_ = false;
  }
  // ACK 必须属于命令 176、请求者身份和当前水位；from_external 只诊断，不能误当 PX4 必然置真的字段。
  bool ack(const Ack& a, double now) {
    // PX4 v1.16应答目标为请求者source system/component，from_external仍可能是默认0。
    // 因此按1/191请求者身份及时间关联，而非要求external=true；ACK本身不证明模式已改变。
    if (a.command != 176 || a.system != 1 || a.component != 191)
      return false;
    if (state_ != State::Sent) {
      // 非等待期也推进ACK水位，防止先来的旧应答被稍后新请求认领；false表示未归属本请求。
      if (a.timestamp > ack_watermark_)
        ack_watermark_ = a.timestamp;
      return false;
    }
    if (now < sent_at_ || a.timestamp <= before_send_timestamp_ || a.timestamp <= ack_watermark_)
      return false;
    ack_watermark_ = a.timestamp;
    if (now - sent_at_ >= 5) {
      state_ = accepted_ ? State::ModeUnconfirmed : State::AckTimeout;
      reason_ = accepted_ ? "ACK_ACCEPTED_BUT_MODE_NOT_CONFIRMED" : "NO_TERMINAL_ACK";
      return false;
    }
    ack_seen_ = true;
    ack_result_ = a.result;
    if (a.result == 0)
      // 0为ACCEPTED，5为IN_PROGRESS；进展应答不直接终结，还需独立模式反馈。
      accepted_ = true;
    else if (a.result != 5) {
      state_ = State::Rejected;
      reason_ = "PX4_REJECTED_REQUEST";
    }
    return true;
  }
  // 先查来源/命令独占和当前地面条件；连续 2 s 稳定后只返回一次发送许可。
  bool tick(double now, const Traffic& t) {
    // true仅为一次地面176计划，本对象没有publisher；先写发送记账再返回，失败处理仍属调用者。
    if (finished())
      return false;
    if (t.status_publishers > 1 || t.land_publishers > 1 || t.ack_publishers > 1 ||
        t.command_subscribers > 1 || t.command_publishers > 1) {
      block("SOURCE_OR_COMMAND_CONFLICT");
      return false;
    }
    const std::string why = guard(now, t);
    if (state_ == State::Waiting) {
      reason_ = why;
      if (!why.empty())
        stable_at_ = -1;
      else {
        if (stable_at_ < 0)
          stable_at_ = now;
        if (now - stable_at_ >= 2) {
          state_ = State::Sent;
          sent_ = true;
          sent_at_ = now;
          before_send_timestamp_ = status_.timestamp;
          reason_ = "WAIT_FOR_ACK_AND_NEW_MODE_STATUS";
          return true;
        }
        reason_ = "GROUND_CONDITIONS_STABILIZING";
      }
      // 准备阶段最多 15 s；发送后最多 5 s，同时需要接受 ACK 和发送后的新 Stabilized 状态。
      if (now >= 15)
        block(why.empty() ? "GROUND_STABLE_WINDOW_TIMEOUT" : why);
    } else {
      if (!why.empty()) {
        block(why);
        return false;
      }
      if (accepted_ && mode_confirmed_) {
        state_ = State::Pass;
        reason_ = "ACK_ACCEPTED_AND_NEW_STABILIZED_STATUS";
      } else if (now - sent_at_ >= 5) {
        state_ = accepted_ ? State::ModeUnconfirmed : State::AckTimeout;
        reason_ = accepted_ ? "ACK_ACCEPTED_BUT_MODE_NOT_CONFIRMED" : "NO_TERMINAL_ACK";
      }
    }
    return false;
  }
  State state() const {
    return state_;
  }
  void abort(const std::string& reason) {
    if (!finished())
      block(reason);
  }
  bool finished() const {
    return state_ != State::Waiting && state_ != State::Sent;
  }
  bool sent() const {
    return sent_;
  }
  bool ack_seen() const {
    return ack_seen_;
  }
  int ack_result() const {
    return ack_result_;
  }
  const std::string& reason() const {
    return reason_;
  }

private:
  void block(const std::string& reason) {
    state_ = State::Blocked;
    reason_ = reason;
  }
  // 地面验证缺任何一个来源或新鲜状态都拒绝，不把“无消息”当作安全。
  std::string guard(double now, const Traffic& t) const {
    // 只读返回首个缺条件的协议字符串，空字符串为本层通过；不修改源时钟或补齐缺失反馈。
    if (t.status_publishers != 1 || t.land_publishers != 1 || t.ack_publishers != 1 ||
        t.command_subscribers != 1 || t.command_publishers != 1)
      return "SOURCE_MISSING";
    if (!status_seen_ || now - status_at_ >= 2.5)
      return "STATUS_STALE";
    if (!land_seen_ || now - land_at_ >= 2.5)
      return "LAND_STALE";
    if (status_.system != 1 || status_.component != 1)
      return "TARGET_ID_NOT_1_1";
    if (!status_.disarmed)
      return "NOT_DISARMED";
    if (status_.failsafe || status_.calibration || status_.hil)
      return "STATUS_NOT_GROUND_TEST_READY";
    if (!land_.landed || !land_.at_rest)
      return "NOT_LANDED_AND_AT_REST";
    if (!status_.stabilized_selectable)
      return "STABILIZED_NOT_SELECTABLE";
    return {};
  }
  Status status_{};
  Land land_{};
  State state_{State::Waiting};
  double status_at_{}, land_at_{}, stable_at_{-1}, sent_at_{};
  std::uint64_t before_send_timestamp_{}, ack_watermark_{};
  bool status_seen_{}, land_seen_{}, sent_{}, accepted_{}, ack_seen_{}, mode_confirmed_{};
  int ack_result_{-1};
  std::string reason_{"WAIT_FOR_TELEMETRY"};
};
} // namespace jikl_fc01_ground_link
