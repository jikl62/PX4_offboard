// 控制权状态 JSON 样例：展示未选择、已选择、已锁定及故障后的诊断字段。
// 输出供 Python 工具/测试检查显示合同；合成状态不是当前真实飞机遥测。
#include "authority_test_support.hpp"
#include <iostream>
int main(){
  // 先输出默认 UNSET，再建立交互会话与自主权限选择，展示选定不等于锁定准备。
  auto c=authority_test::config();fc01::Runtime r(c);auto s=authority_test::sample();
  std::cout<<"{"<<r.authority_json(s)<<"}\n";
  r.request("INTERACTIVE",{},s,0);r.request("AUTHORITY_AUTONOMOUS",{},s,0);
  std::cout<<"{"<<r.authority_json(s)<<"}\n";
  c.attitude.autonomous_failsafe_confirmed=true;fc01::Runtime live(c);double t=0;
  authority_test::ready(live,s,t,"AUTHORITY_AUTONOMOUS");
  std::cout<<"{"<<live.authority_json(s)<<"}\n";
  // 注入 Kill 后观察即时诊断，再经控制步骤和显式发布故障确认历史原因不会被擦除。
  s.attitude.kill=true;std::cout<<"{"<<live.authority_json(s)<<"}\n";
  authority_test::tick(live,s,t);s.attitude.kill=false;live.abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");
  std::cout<<"{"<<live.authority_json(s)<<"}\n";
}
