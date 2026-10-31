#!/usr/bin/env python3
"""Explicit activation only, while quiesced; never cleans runtime sockets."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import uuid
from socket_safety import SocketLease,SocketSafetyError,require_absent_socket

def require_inactive_service():
    try:result=subprocess.run(['systemctl','--user','is-active','--quiet','fc01-system.service'],check=False,timeout=3)
    except (OSError,subprocess.TimeoutExpired) as error:raise SocketSafetyError('SERVICE_STATE_UNKNOWN: '+str(error)) from error
    if result.returncode==0:raise SocketSafetyError('SERVICE_ACTIVE: explicitly quiesce on confirmed safe ground before updating')
    if result.returncode!=3:raise SocketSafetyError('SERVICE_STATE_UNKNOWN: systemctl exit='+str(result.returncode))

def activate(base,target,socket_path):
    base=Path(base).resolve();target=Path(target).resolve();socket_path=Path(socket_path)
    if not target.is_relative_to(base/'releases') or not target.is_dir():raise SocketSafetyError('TARGET_OUTSIDE_RELEASES')
    node=target/'workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
    if not node.is_file() or not os.access(node,os.X_OK):raise SocketSafetyError('TARGET_NOT_BUILT')
    # First sample, then take the same lock held by the 0.4.1 controller.
    require_inactive_service()
    socket_path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
    with SocketLease(socket_path) as lease:
        require_absent_socket(socket_path)
        # UUID path avoids deleting/overwriting another updater's current.next.
        pending=base/('current.next.'+uuid.uuid4().hex)
        pending.symlink_to(target)
        try:
            require_inactive_service()
            lease.verify_identity()
            require_absent_socket(socket_path)
            os.replace(pending,base/'current')
        finally:
            # Only this activation's freshly created symlink, never socket state.
            if pending.is_symlink() and pending.readlink()==target:pending.unlink()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base',type=Path);parser.add_argument('release',type=Path);parser.add_argument('socket',type=Path)
    args=parser.parse_args()
    try:activate(args.base,args.release,args.socket);return 0
    except (OSError,SocketSafetyError) as error:print('[UPDATE_REFUSED] '+str(error),file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
