#pragma once
// 姿态只读诊断：将实际/请求四元数转换为角度，供日志和接收事件判断。
// 输入 Hamilton 四元数顺序 w,x,y,z，表示机体 FRD 到世界 NED；输出均为度。
// 此处不改变请求姿态、推力或安全阈值；无效输入返回 valid=false。
#include "jikl_fc01_system/attitude_reference.hpp"
#include <algorithm>
#include <cmath>

namespace fc01::attitude {
// roll/pitch/yaw 是实际欧拉角；tilt 是机体下轴相对世界下轴夹角；error 是两姿态最短旋转角。
struct AttitudeDiagnostics {
  bool valid{};
  double roll_deg{}, pitch_deg{}, yaw_deg{}, tilt_deg{}, command_tilt_deg{}, error_deg{};
};
// Observation only: NO influence on command formation or safety thresholds.
inline AttitudeDiagnostics attitude_diagnostics(const Quaternion& measured,
                                                const Quaternion& requested) {
  // 两端先经过同一四元数有效性检查，拒绝非有限或模长明显偏离 1 的输入。
  auto m = normalized(measured), r = normalized(requested);
  if (!m || !r)
    return {};
  constexpr double k = 180. / 3.141592653589793238462643383279502884;
  const double w = (*m)[0], x = (*m)[1], y = (*m)[2], z = (*m)[3];
  const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // asin 输入限于 [-1,1]，避免浮点舍入将几何上合法的俯仰角变成 NaN。
  const double pitch = std::asin(std::clamp(2 * (w * y - z * x), -1., 1.));
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const double cosine = std::clamp(1 - 2 * (x * x + y * y), -1., 1.);
  const double command_cosine =
      std::clamp(1 - 2 * ((*r)[1] * (*r)[1] + (*r)[2] * (*r)[2]), -1., 1.);
  // q 与 -q 表示同一姿态，点积取绝对值后求最小旋转角。
  const double dot =
      std::clamp(std::abs(w * (*r)[0] + x * (*r)[1] + y * (*r)[2] + z * (*r)[3]), 0., 1.);
  return {true,
          roll * k,
          pitch * k,
          yaw * k,
          std::acos(cosine) * k,
          std::acos(command_cosine) * k,
          2 * std::acos(dot) * k};
}
} // namespace fc01::attitude
