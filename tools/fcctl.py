#!/usr/bin/env python3
"""Operator console. No ROS dependency, no flight commands on connection."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import shlex
import socket
import sys
import time

def parse_command(text):
    parts=shlex.split(text)
    if not parts:
        raise ValueError('请输入命令')
    op=parts[0].lower()
    if op=='authority':
        routes={'status':'AUTHORITY_STATUS','rc':'AUTHORITY_RC','autonomous':'AUTHORITY_AUTONOMOUS',
                'reset':'AUTHORITY_RESET','takeover':'AUTHORITY_TAKEOVER'}
        if len(parts)!=2 or parts[1].lower() not in routes:
            raise ValueError('用法: authority status|rc|autonomous|reset|takeover')
        return routes[parts[1].lower()],[],0.
    names={'status':'STATUS','mode':'MODE','attitude-ready':'ATTITUDE_READY','takeoff':'TAKEOFF','vel':'VEL','hold':'HOLD','land':'LAND','update-check':'UPDATE_CHECK','stop':'STOP','quit':'QUIT','help':'HELP'}
    if op not in names:
        raise ValueError('未知命令；输入 help')
    expected={'takeoff':1,'vel':5}.get(op,0)
    if len(parts)!=expected+1:
        raise ValueError('参数数量不正确')
    values=[float(x) for x in parts[1:]]
    if not all(math.isfinite(x) for x in values):
        raise ValueError('参数必须是有限数字')
    if op=='takeoff' and values[0]<=0:
        raise ValueError('高度必须大于 0')
    duration=0.
    if op=='vel':
        duration=values.pop()
        if not 0<duration<=2:
            raise ValueError('单次移动持续时间必须在 (0, 2] 秒内')
    return names[op],values,duration

def socket_path():
    return os.environ.get('FC01_SOCKET',str(Path.home()/'.cache/fc01/control.sock'))

def show(obj,raw):
    if raw:
        print(json.dumps(obj,ensure_ascii=False),flush=True)
    elif obj.get('kind')=='state' or (obj.get('kind')=='reply' and obj.get('message') in {'STATUS','AUTHORITY_STATUS'} and 'phase' in obj):
        print('[{phase}] mode={nav_state} armed={armed} valid={estimate_valid} ground={ground_test} reason={reason}'.format(**obj),flush=True)
        if 'authority_mode' in obj:
            print('authority={} locked={} ready={} reason={} takeover={}'.format(
                obj['authority_mode'],obj.get('authority_locked'),obj.get('authority_ready'),
                obj.get('authority_reason'),obj.get('takeover_reason')),flush=True)
    else:
        print('[{}] {}'.format('OK' if obj.get('accepted') else 'BLOCKED',obj.get('message',obj)),flush=True)

def run(args):
    initial=parse_command(args.command) if args.command else None
    if initial and initial[0] in {'TAKEOFF','VEL','HOLD','LAND','AUTHORITY_RC','AUTHORITY_AUTONOMOUS','AUTHORITY_RESET','AUTHORITY_TAKEOVER'}:
        raise ValueError('Flight/motion commands require the live interactive console')
    if initial and initial[0] in {'HELP','QUIT'}:
        raise ValueError('Use the interactive console for help/quit')
    if not initial and not sys.stdin.isatty():
        raise ValueError('Live interactive console requires a TTY; use --command for status/mode/attitude-ready')
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect(args.socket)
    sock.setblocking(False)
    seq=0; pending={};buf=b'';velocity=None;velocity_until=0.;last_ping=last_vel=0.;target_seq=None
    def send(op,values=()):
        nonlocal seq
        seq+=1
        packet=(str(seq)+' '+op+''.join(' '+format(x,'.17g') for x in values)+'\n').encode()
        # Requests are short, but use select/send loops so backpressure cannot lose a command.
        end=time.monotonic()+.3
        while packet:
            if time.monotonic()>end:
                raise TimeoutError('命令连接拥塞')
            _,w,_=select.select([],[sock],[],.05)
            if w:
                sent=sock.send(packet)
                if not sent:
                    raise ConnectionError('命令连接关闭')
                packet=packet[sent:]
        if op!='PING':
            pending[seq]=op
        return seq
    with sock:
        if initial:
            target_seq=send(initial[0],initial[1])
        else:
            print('命令: status | authority status/rc/autonomous/reset/takeover | mode | attitude-ready | takeoff 高度 | vel 前 左 上 左旋 时长 | hold | land | quit',flush=True)
            send('INTERACTIVE')
            send('STATUS')
        deadline=time.monotonic()+args.timeout if initial else math.inf
        mode_wait=False;ready_wait=False
        while time.monotonic()<deadline:
            now=time.monotonic()
            if not initial and now-last_ping>=.2:
                send('PING');last_ping=now
            if velocity is not None:
                if now>=velocity_until:
                    send('HOLD');velocity=None
                elif now-last_vel>=.1:
                    send('VEL',velocity);last_vel=now
            readers=[sock]
            if not initial:
                readers.append(sys.stdin)
            readable,_,_=select.select(readers,[],[],.05)
            if sock in readable:
                chunk=sock.recv(8192)
                if not chunk:
                    raise ConnectionError('控制节点已断开；检查机载日志')
                buf+=chunk
                if len(buf)>131072:
                    raise ValueError('返回数据过长')
                while b'\n' in buf:
                    line,buf=buf.split(b'\n',1)
                    obj=json.loads(line)
                    if obj.get('kind')=='error':
                        show(obj,args.json);return 2
                    op=pending.pop(obj.get('seq'),None)
                    if obj.get('kind')=='state':
                        if not initial or mode_wait or ready_wait:
                            show(obj,args.json)
                        if mode_wait and obj.get('ground_test') not in {'WAITING','SENT','NOT_RUN'}:
                            return 0 if obj.get('ground_test')=='PASS' else 2
                        if ready_wait and obj.get('phase') in {'OFFBOARD_READY','SAFETY','IDLE','SAFE_DISARMED'}:
                            return 0 if obj.get('phase')=='OFFBOARD_READY' and obj.get('armed') is False else 2
                    elif op:
                        show(obj,args.json)
                        if initial and obj.get('seq')==target_seq:
                            if not obj.get('accepted'):
                                return 2
                            if initial[0]=='MODE':
                                mode_wait=True
                            elif initial[0]=='ATTITUDE_READY':
                                ready_wait=True
                            else:
                                return 0
            if not initial and sys.stdin in readable:
                text=sys.stdin.readline()
                if not text:
                    return 0
                try:
                    op,values,duration=parse_command(text)
                    if op=='QUIT':
                        return 0
                    if op=='HELP':
                        print('速度单位 m/s；左旋 rad/s；时长 <=2 s。mode / attitude-ready 需要 ACK+新模式状态；attitude-ready 不解锁。quit/断线清除旧速度并请求降落。',flush=True)
                        print('authority rc/autonomous 只选择策略，不确认失效保护、不改 PX4 参数；准备后锁定。takeover 释放本次控制；reset 仅在安全地面、输出停止且事务结束后回到 UNSET。',flush=True)
                        continue
                    # Any new operator command cancels the previous timed movement.
                    if velocity is not None:
                        send('HOLD');velocity=None
                    send(op,values)
                    if op=='VEL':
                        velocity=values;velocity_until=time.monotonic()+duration;last_vel=time.monotonic()
                except (ValueError,OverflowError) as error:
                    print(str(error),file=sys.stderr,flush=True)
        raise TimeoutError('等待执行结果超时')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',default=socket_path())
    parser.add_argument('--command',help='single read-only/ground command; flight uses interactive console')
    parser.add_argument('--timeout',type=float,default=25.)
    parser.add_argument('--json',action='store_true')
    args=parser.parse_args()
    try:
        if not math.isfinite(args.timeout) or args.timeout<=0:
            raise ValueError('timeout must be positive and finite')
        return run(args)
    except (OSError,ValueError,TimeoutError,ConnectionError) as error:
        print(str(error),file=sys.stderr);return 2
    except KeyboardInterrupt:
        return 130

if __name__=='__main__':
    sys.exit(main())
