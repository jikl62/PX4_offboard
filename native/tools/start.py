#!/usr/bin/env python3
"""Supervise agent/node; terminate the node before closing its agent."""
import argparse
from pathlib import Path
import os
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--external-agent',action='store_true')
    args=parser.parse_args()
    base=Path(os.environ.get('FC01_HOME',str(Path.home()/'fc01_system')))
    logs=base/'logs';logs.mkdir(parents=True,exist_ok=True)
    stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
    stopped=False
    def stop(_sig,_frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    children=[];node=None;code=2
    with (logs/('node_'+stamp+'.log')).open('a') as node_log,(logs/('agent_'+stamp+'.log')).open('a') as agent_log:
        try:
            if not args.external_agent:
                agent=subprocess.Popen(['bash',str(ROOT/'scripts/agent.sh')],stdout=agent_log,stderr=subprocess.STDOUT,start_new_session=True);children.append(agent)
                time.sleep(.3)
                if agent.poll() is not None:
                    print('Agent 启动失败，见 '+str(agent_log.name),file=sys.stderr);return 2
            node=subprocess.Popen(['bash',str(ROOT/'scripts/run.sh')],stdout=node_log,stderr=subprocess.STDOUT,start_new_session=True);children.append(node)
            print('控制进程已启动（默认不解锁）。日志：'+str(node_log.name),flush=True)
            while not stopped:
                if node.poll() is not None:
                    code=node.returncode;break
                if not args.external_agent and agent.poll() is not None:
                    print('Agent 退出；节点 watchdog/PX4 丢失策略将处理控制状态。',file=sys.stderr);code=2;break
                time.sleep(.1)
            if stopped:code=0
        finally:
            # Allow the controller to request/confirm LAND while the Agent is still alive.
            if node and node.poll() is None:
                os.killpg(node.pid,signal.SIGTERM)
                waiting_since=time.monotonic();last_warning=waiting_since
                while node.poll() is None:
                    try:node.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        now=time.monotonic()
                        if now-waiting_since>=65 and now-last_warning>=5:
                            last_warning=now
                            print('控制节点仍在恢复/等待交权；保留节点与 Agent，需操作者确认飞控状态。',file=sys.stderr,flush=True)
            for child in children:
                if child is not node and child.poll() is None:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    return code
if __name__=='__main__':sys.exit(main())
