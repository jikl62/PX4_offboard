#pragma once
// 监督进程寿命绑定：父进程消失时请求与正常 SIGTERM 相同的有序停止。
// expected_parent 必须在 ROS 初始化前捕获；prctl 设置之后再次检查父 PID，覆盖设置前后父进程退出竞态。
// 返回 false 让上层进入原关闭流程，绝不以 SIGKILL 杀死已解锁控制器或抹掉飞行状态。
#include <stdexcept>
#include <sys/prctl.h>
#include <unistd.h>

namespace fc01 {
// 输入expected_parent是早先观察到的父PID，signal_number是请求有序停止的信号；
// 返回true只表示设置后仍是同一个有效父进程，不证明父进程未来不会退出。
// 副作用是设置本进程内核父退出信号；prctl失败抛runtime_error，调用者不得悄悄忽略。
inline bool arm_parent_loss_signal(pid_t expected_parent, int signal_number) {
  // 无法建立内核父进程退出信号时抛错，不能在“以为受监督”的状态继续运行。
  if (prctl(PR_SET_PDEATHSIG, signal_number) != 0)
    throw std::runtime_error("cannot establish supervisor lifetime signal");
  return expected_parent > 1 && getppid() == expected_parent;
}
} // namespace fc01
