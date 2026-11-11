// 地面授权失败诊断回归：当前健康值与最近失败触发样本必须分别保留。
// stationary_seconds单位s，Vz是NED下正m/s；0.75 s静止不足和0.12 m/s速度超限共同形成位图24。
#include "authority_test_support.hpp"
#include <iostream>
using namespace authority_test;
int main() {
  auto c = config();
  f::Runtime r(c);
  auto s = sample();
  r.request("INTERACTIVE", {}, s, 0);
  s.stationary_seconds = .75;
  s.attitude.vz = .12;
  assert(r.request("AUTHORITY_RC", {}, s, 0).message == "AUTHORITY_SAFE_GROUND_REQUIRED");
  // 拒绝时JSON必须显示原静止时长、门限和失败位；数据恢复健康只改变当前位，不抹掉历史触发。
  const auto failed = r.authority_json(s);
  assert(failed.find("\"authority_ground_failed_mask\":24") != std::string::npos);
  assert(failed.find("\"stationary_seconds\":0.75") != std::string::npos);
  assert(failed.find("\"required_ground_seconds\":2") != std::string::npos);
  assert(failed.find("\"ground_vz_tolerance\":0.1") != std::string::npos);
  s.stationary_seconds = 3;
  s.attitude.vz = 0;
  const auto recovered = r.authority_json(s);
  assert(recovered.find("\"authority_ground_failed_mask\":0") != std::string::npos);
  assert(recovered.find("\"last_authority_ground_failed_mask\":24") != std::string::npos);
  // 新的合法授权选择成功后才更新最近选择证据，避免把历史错误显示成当前仍阻塞。
  assert(r.request("AUTHORITY_RC", {}, s, 0).accepted);
  assert(r.authority_json(s).find("\"last_authority_ground_failed_mask\":0") != std::string::npos);
  std::cout << "REVIEW_EXACT_FAILED_GROUND_SAMPLE_AND_CURRENT_DIAGNOSTICS PASS\n";
}
