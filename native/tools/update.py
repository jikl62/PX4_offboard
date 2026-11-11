#!/usr/bin/env python3
# 显式版本激活工具：仅在服务停止、控制 socket 不存在且持有独占锁时切换 current。
# 不能清理或抢占运行期 socket，也不启动新版本；激活与飞行启动仍是独立动作。
"""Explicit activation only, while quiesced; never cleans runtime sockets."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import uuid
from socket_safety import SocketLease,SocketSafetyError,require_absent_socket

# 只接受 systemctl 明确返回 inactive 的代码 3；查询失败/超时属于未知状态，必须拒绝更新。
def require_inactive_service():
    try:result=subprocess.run(['systemctl','--user','is-active','--quiet','fc01-system.service'],check=False,timeout=3)
    except (OSError,subprocess.TimeoutExpired) as error:raise SocketSafetyError('SERVICE_STATE_UNKNOWN: '+str(error)) from error
    if result.returncode==0:raise SocketSafetyError('SERVICE_ACTIVE: explicitly quiesce on confirmed safe ground before updating')
    if result.returncode!=3:raise SocketSafetyError('SERVICE_STATE_UNKNOWN: systemctl exit='+str(result.returncode))

# 验证目标为 releases 下已构建目录；用与原生节点相同的 socket 租约阻止并发激活。
def activate(base,target,socket_path):
    base=Path(base).resolve();target=Path(target).resolve();socket_path=Path(socket_path)
    if not target.is_relative_to(base/'releases') or not target.is_dir():raise SocketSafetyError('TARGET_OUTSIDE_RELEASES')
    node=target/'workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
    if not node.is_file() or not os.access(node,os.X_OK):raise SocketSafetyError('TARGET_NOT_BUILT')
    # First sample, then take the same lock held by the 0.4.1 controller.
    require_inactive_service()
    socket_path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
    # 锁内再次检查 socket；采样与最终替换之间也复验服务、锁 inode 和 socket 身份。
    with SocketLease(socket_path) as lease:
        require_absent_socket(socket_path)
        # UUID path avoids deleting/overwriting another updater's current.next.
        # 每次操作用独立 UUID 临时链接，不能误删另一个更新进程的 staging 路径。
        pending=base/('current.next.'+uuid.uuid4().hex)
        pending.symlink_to(target)
        try:
            require_inactive_service()
            lease.verify_identity()
            require_absent_socket(socket_path)
            # 原子替换 current，只改变下一次启动使用的版本；不向已运行进程注入新代码。
            os.replace(pending,base/'current')
        finally:
            # Only this activation's freshly created symlink, never socket state.
            if pending.is_symlink() and pending.readlink()==target:pending.unlink()

# 向操作者明确输出拒绝原因，发生文件/租约错误时以非零退出，不尝试自动重启。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base',type=Path);parser.add_argument('release',type=Path);parser.add_argument('socket',type=Path)
    args=parser.parse_args()
    try:activate(args.base,args.release,args.socket);return 0
    except (OSError,SocketSafetyError) as error:print('[UPDATE_REFUSED] '+str(error),file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
