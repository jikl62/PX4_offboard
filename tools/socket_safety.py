#!/usr/bin/env python3
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

class SocketSafetyError(RuntimeError):
    pass

def checked_parent(path):
    parent=Path(path).parent
    try:info=parent.lstat()
    except OSError as error:raise SocketSafetyError('SOCKET_DIRECTORY_UNKNOWN: '+str(error)) from error
    if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o022:
        raise SocketSafetyError('SOCKET_DIRECTORY_NOT_PRIVATE_OWNED_DIRECTORY: '+str(parent))
    return info

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

def require_absent_socket(path):
    checked_parent(path)
    try:Path(path).lstat()
    except FileNotFoundError:return
    except OSError as error:raise SocketSafetyError('SOCKET_PATH_UNKNOWN: '+str(error)) from error
    raise SocketSafetyError(socket_diagnosis(path))

class SocketLease:
    """Advisory lock shared with UnixServer; lock pathname is never removed."""
    def __init__(self,path):
        self.path=Path(path);self.lock=Path(str(path)+'.lock');self.fd=None
    def __enter__(self):
        checked_parent(self.path)
        try:
            self.fd=os.open(self.lock,os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW|os.O_NONBLOCK,0o600)
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
    def verify_identity(self):
        checked_parent(self.path)
        try:current=self.lock.lstat()
        except OSError as error:raise SocketSafetyError('SOCKET_LOCK_IDENTITY_UNKNOWN') from error
        if (not stat.S_ISREG(current.st_mode) or current.st_uid!=os.getuid() or current.st_mode&0o022 or
                (current.st_dev,current.st_ino)!=(self.identity.st_dev,self.identity.st_ino)):
            raise SocketSafetyError('SOCKET_LOCK_PATH_CHANGED: activation refused')
    def __exit__(self,*_args):
        if self.fd is not None:os.close(self.fd);self.fd=None

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('path',type=Path)
    args=parser.parse_args()
    try:checked_parent(args.path);print(socket_diagnosis(args.path));return 0
    except SocketSafetyError as error:print(str(error),file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
