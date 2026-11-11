#pragma once
// PX4 源时钟与样本寿命：同时核对同步时钟域、消息外壳和原始测量进展。
// source/envelope/raw/host 均为微秒；received/now/timeout 为本机单调秒。两类寿命分别判断。
// Agent 同步域尚未成立、原始采样停滞或回退时均拒绝，不能因收到新 envelope 给旧测量续期。
//
// 先分清两个问题：
//   源时间检查：测量生成得是否足够近、是否落在已同步的host域里？看source/sample与host的微秒差。
//   接收进展检查：本机最近是否真的见到一个更晚的原始测量？看received与now的单调秒差。
// 只过前者可能在重复读旧测量；只过后者可能收到刚到达的历史积压。控制链需要两边一起成立。
// envelope/message.timestamp是PX4消息外壳戳，sample/timestamp_sample是原始测量戳；
// 新外壳可能仍装旧测量，所以两个戳不能合并为“收到消息的时间”。
#include <cmath>
#include <cstdint>
#include <limits>
namespace fc01 {
// 允许源时间最多领先本机 100 ms；这是既有同步偏差预算，不是额外 TTL。
constexpr std::uint64_t source_clock_skew_budget_us = 100000;
// 命令后因果水位加同步容差并饱和到 uint64 上限，防止溢出绕回导致旧 ACK 归属新事务。
inline std::uint64_t source_causal_floor(std::uint64_t wire) {
  // uint64没有负数，直接溢出会绕回小数；先用max-budget比较，再决定是否能安全相加。
  // 这个下界只帮助剔除不满足因果要求的旧ACK/状态，不是“超过它就一定属于该命令”的证明。
  return wire > std::numeric_limits<std::uint64_t>::max() - source_clock_skew_budget_us
             ? std::numeric_limits<std::uint64_t>::max()
             : wire + source_clock_skew_budget_us;
}
// PX4 v1.16 uXRCE序列化会加上Agent同步偏移；正式适配层先核验该同步域，
// 才能将PX4源戳与ROS命令戳比较。启动相对戳不能直接冒充已同步的host戳。
// 源 stamp 为 0/本机未取时/超时配置无效都失败；未来时间只在 100 ms 容差内接受。
inline bool source_stamp_in_clock(std::uint64_t source, std::uint64_t host, double timeout) {
  if (!source || !host || !std::isfinite(timeout) || timeout <= 0)
    return false;
  // 分两条路相减，保证uint64始终用较大数减较小数，避免“负年龄”绕成巨大正数。
  if (source > host)
    return source - host <= source_clock_skew_budget_us;
  // timeout单位秒，乘1e6变微秒；先在整数域做差再转double，不先转换两个大epoch相减。
  return double(host - source) <= timeout * 1e6;
}
// 原始测量不能晚于外壳，两者分别与 host 比较，外壳新鲜不能遮住 raw 过期。
inline bool source_sample_in_clock(std::uint64_t envelope, std::uint64_t raw, std::uint64_t host,
                                   double timeout) {
  // &&从左到右短路：0代表缺测，raw晚于外壳也拒绝；之后分别检查外壳和测量的源年龄。
  return raw && raw <= envelope && source_stamp_in_clock(envelope, host, timeout) &&
         source_stamp_in_clock(raw, host, timeout);
}
// 新外壳不能给未推进的旧测量续期；当前无效载荷也不能借用上次有效载荷的新鲜度。
// 缓存 raw 最大源戳、最后推进的单调接收时刻和回退锁存；不是每收到消息就续期。
struct SourceSampleProgress {
  // value保存已见过的最大合法raw源戳；received只在这个水位真正上升时更新。
  // -1是尚未见过进展的单调时间哨兵；regressed一旦置真，本对象内不靠后来好帧清除。
  std::uint64_t value{};
  double received{-1};
  bool regressed{};
  // 先标记当前载荷有效性；一旦观测 raw 回退，后续健康包也不能在本对象内清锁存。
  void observe(std::uint64_t envelope, std::uint64_t sample, double now) {
    // 此处先检查载荷关系和本机时刻，不宣称已经验证同步域；fresh随后还会查host年龄。
    valid_ = sample && sample <= envelope && std::isfinite(now) && now >= 0;
    if (!valid_)
      return;
    if (value && sample < value)
      regressed = true;
    if (regressed) {
      valid_ = false;
      return;
    }
    // sample==value时不改received：例如100ms内重收十次同一测量，也只算第一次取得进展。
    if (sample > value) {
      value = sample;
      received = now;
    }
  }
  // 接收进展 TTL 与同步源 TTL 同时通过；合法重复包保持原 received，不延长寿命。
  bool fresh(std::uint64_t host, double now, double timeout) const {
    // 本机进展年龄使用严格<timeout；source_stamp_in_clock的源年龄边界沿原规则使用<=。
    // 这是两个独立合同，不能为了“统一写法”改动边界或把now/received转成PX4微秒。
    return valid_ && !regressed && received >= 0 && std::isfinite(now) && now >= received &&
           std::isfinite(timeout) && timeout > 0 && now - received < timeout &&
           source_stamp_in_clock(value, host, timeout);
  }

private:
  // valid_描述最后一次被观察的载荷；value/received描述最后真实进展，寿命不同。
  bool valid_{};
};
template <class Message, class Envelope>
// 先让外壳跟踪器接受；接受后保留真实载荷并独立记录 raw 状态，供下游解释当前无效样本。
bool receive_sampled_source(const Message& message, Message& destination, Envelope& envelope,
                            SourceSampleProgress& raw, double now) {
  if (!envelope.accept(message.timestamp, now))
    return false;
  // 接受外壳后先保留真实载荷，再观察raw；raw无效也不能悄悄用上一帧健康载荷替换它。
  // 返回true只表示这一外壳进入缓存，不能当作完整数据健康或飞行动作许可。
  destination = message;
  raw.observe(message.timestamp, message.timestamp_sample, now);
  return true;
}
template <class Command, class Clock, class Sink>
// 在真正 sink 前由调用者时钟重新填命令 stamp，避免排队或编码时长让时间戳过旧。
void publish_command_with_stamp(Command& command, Clock&& clock, Sink&& sink) {
  // Command&会改调用者这条命令；Clock/Sink由模板接收可调用对象（生产里常是lambda）。
  // 先从真实出口时钟取戳，紧接着调用sink；sink抛错会原样向上传播，这里不伪造成功。
  command.timestamp = clock();
  sink(command);
}
} // namespace fc01
