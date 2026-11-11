#!/usr/bin/env python3
# Unix 控制 socket 的只读诊断与启动租约：路径存在即活性未知，不能由 ECONNREFUSED 推断可删除。
# 锁文件永不删除，协作启动器和原生 UnixServer 必须共享同一 inode，避免双控制器。
"""Diagnostic-only socket checks and a lifetime lease for cooperating starters.

Never connects, deletes a socket/lock, kills a process, or infers liveness from
ECONNREFUSED. Existing socket => liveness UNKNOWN => activation refused.
"""
import argparse
import fcntl
import os
from pathlib import Path
import shlex
import stat
import sys

# 把路径身份/租约错误与普通 CLI 错误区分，调用方必须拒绝激活。
class SocketSafetyError(RuntimeError):
    pass

# 父目录须为当前用户所有且群组/其他人不可写的真实目录，拒绝符号链接和非私有目录。
def checked_parent(path):
    parent=Path(path).parent
    try:info=parent.lstat()
    except OSError as error:raise SocketSafetyError('SOCKET_DIRECTORY_UNKNOWN: '+str(error)) from error
    if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o022:
        raise SocketSafetyError('SOCKET_DIRECTORY_NOT_PRIVATE_OWNED_DIRECTORY: '+str(parent))
    return info

# 只用 lstat 描述对象类型/所有者/inode；不连接、不删除、不终止占用者。
def socket_diagnosis(path):
    path=Path(path)
    try:info=path.lstat()
    except FileNotFoundError:return 'SOCKET_ABSENT: '+str(path)
    except OSError as error:return 'SOCKET_PATH_UNKNOWN: '+str(error)
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid!=os.getuid():
        return 'SOCKET_PATH_NOT_OWNED_UNIX_SOCKET: '+str(path)+'; refused; object preserved'
    quoted=shlex.quote(str(path))
    return ('SOCKET_PRESENT_LIVENESS_UNKNOWN: '+str(path)+
            '; inode='+str(info.st_ino)+' device='+str(info.st_dev)+
            '; may be live, starting, stopping, or stale; object preserved. '+
            'ECONNREFUSED alone never permits deletion. Quiesce every starter '+
            'and service, verify no listener and recheck owner/type/inode. '+
            'Only after independent human confirmation of stale state: rm -- '+quoted+
            '. Never remove the .lock file; do not run this command while a process can start.')

# 在私有父目录下，仅 FileNotFoundError 可证明路径缺席，其它错误仍属未知。
def require_absent_socket(path):
    checked_parent(path)
    try:Path(path).lstat()
    except FileNotFoundError:return
    except OSError as error:raise SocketSafetyError('SOCKET_PATH_UNKNOWN: '+str(error)) from error
    raise SocketSafetyError(socket_diagnosis(path))

# 生命周期持有 flock；进入时检查类型与权限，退出仅关闭描述符，不 unlink 锁路径。
class SocketLease:
    """Advisory lock shared with UnixServer; lock pathname is never removed."""
    # socket 与 .lock 是不同路径；fd=None 表示本实例尚未获得有效租约。
    def __init__(self,path):
        self.path=Path(path);self.lock=Path(str(path)+'.lock');self.fd=None
    # 以 NOFOLLOW/CLOEXEC 打开锁文件，再非阻塞独占加锁；失败关闭自己打开的 fd。
    def __enter__(self):
        checked_parent(self.path)
        try:
            self.fd=os.open(self.lock,os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW|os.O_NONBLOCK,0o600)
            # 记录锁真实设备号与 inode，后续复验拒绝路径被替换后的虚假独占。
            self.identity=os.fstat(self.fd)
            if not stat.S_ISREG(self.identity.st_mode) or self.identity.st_uid!=os.getuid() or self.identity.st_mode&0o022:
                raise SocketSafetyError('SOCKET_LOCK_NOT_PRIVATE_OWNED_REGULAR_FILE')
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.verify_identity()
        except (OSError,SocketSafetyError) as error:
            if self.fd is not None:os.close(self.fd);self.fd=None
            if isinstance(error,SocketSafetyError):raise
            raise SocketSafetyError('SOCKET_LOCK_BUSY_OR_UNKNOWN: '+str(error)) from error
        return self
    # 最终动作前同时复验父目录、锁类型/权限及 inode，防止检查后被更换。
    def verify_identity(self):
        checked_parent(self.path)
        try:current=self.lock.lstat()
        except OSError as error:raise SocketSafetyError('SOCKET_LOCK_IDENTITY_UNKNOWN') from error
        if (not stat.S_ISREG(current.st_mode) or current.st_uid!=os.getuid() or current.st_mode&0o022 or
                (current.st_dev,current.st_ino)!=(self.identity.st_dev,self.identity.st_ino)):
            raise SocketSafetyError('SOCKET_LOCK_PATH_CHANGED: activation refused')
    # 关闭本实例描述符释放 advisory lock；不删除共享锁文件，保留后续启动器的协调身份。
    def __exit__(self,*_args):
        if self.fd is not None:os.close(self.fd);self.fd=None

# 只输出诊断，不尝试自动修复；错误返回 2，让调用脚本停止。
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('path',type=Path)
    args=parser.parse_args()
    try:checked_parent(args.path);print(socket_diagnosis(args.path));return 0
    except SocketSafetyError as error:print(str(error),file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
