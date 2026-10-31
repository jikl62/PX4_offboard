#!/usr/bin/env python3
"""Passive, bounded deployment observation. No flight publishers or commands."""
from __future__ import annotations
import json
import os
from pathlib import Path
import subprocess
import time

from launcher import ControlError, choose_agent, process_list, uart_owners

class GroundEvidence:
    def __init__(self):
        self.rows={};self.counts={'status':0,'land':0};self.armed_seen=False
        self.reason='等待新的未解锁、接地、静止消息'
    def observe(self,kind,msg,wall,mono):
        if kind=='status' and msg.get('arming_state')==2:
            self.armed_seen=True;self.reason='观察到飞机已解锁；保留原服务'
        stamp=msg.get('timestamp',0)/1e6
        good=-.1<=wall-stamp<=2.5
        if kind=='status':good=good and msg.get('system_id')==1 and msg.get('component_id')==1 and msg.get('arming_state')==1
        else:good=good and msg.get('landed') is True and msg.get('at_rest') is True
        if not good:
            self.rows.pop(kind,None);self.counts[kind]=0;return
        old=self.rows.get(kind)
        if old and stamp<=old['stamp']:
            if stamp<old['stamp']:self.rows.pop(kind,None);self.counts[kind]=0
            return
        self.rows[kind]={'stamp':stamp,'receipt':mono};self.counts[kind]+=1
    def ready(self,wall,mono,status_publishers,land_publishers):
        if self.armed_seen:return False
        if status_publishers!=1 or land_publishers!=1:
            self.reason='等待每个反馈话题恰好一个发布源';return False
        for kind in ('status','land'):
            r=self.rows.get(kind)
            if not r or self.counts[kind]<2 or not 0<=mono-r['receipt']<=2.5 or not -.1<=wall-r['stamp']<=2.5:
                self.reason='等待同步时钟下连续的新鲜未解锁/接地消息';return False
        self.reason='已观察到新鲜、单一来源的未解锁接地静止反馈';return True

def probe(root,agent_binary,evidence_path=None):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus,VehicleLandDetected
    evidence_path=evidence_path or Path.home()/'FC01_navigation/probe.json'
    evidence_path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
    evidence=GroundEvidence();agent_pid=None;created=False;node=None;ok=False;verified_at=0.
    try:
        match=choose_agent(process_list(),uart_owners('/dev/ttyS2'),'/dev/ttyS2',os.getuid())
        if match:agent_pid=match.pid
        else:
            child=subprocess.Popen([str(agent_binary),'serial','-D','/dev/ttyS2','-b','115200','-v','0'],
                stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
            agent_pid=child.pid;created=True
        rclpy.init();node=rclpy.create_node('fc01_install_ground_observer')
        def receive(kind,msg):
            fields=('timestamp','system_id','component_id','arming_state') if kind=='status' else ('timestamp','landed','at_rest')
            evidence.observe(kind,{k:getattr(msg,k) for k in fields},time.time(),time.monotonic())
        subs=[node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',lambda m:receive('status',m),qos_profile_sensor_data),
              node.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',lambda m:receive('land',m),qos_profile_sensor_data)]
        deadline=time.monotonic()+18
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if evidence.armed_seen:break
            ok=evidence.ready(time.time(),time.monotonic(),node.count_publishers('/fmu/out/vehicle_status_v1'),node.count_publishers('/fmu/out/vehicle_land_detected'))
            if ok:verified_at=time.monotonic();break
        if not ok:raise ControlError(evidence.reason+'；未变更服务')
        print(evidence.reason,flush=True);return 0
    finally:
        evidence_path.write_text(json.dumps(dict(ok=ok,reason=evidence.reason,armed_seen=evidence.armed_seen,
            samples=evidence.counts,agent_pid=agent_pid,agent_created=created,agent_retained=True,
            updated_unix=time.time(),updated_monotonic=verified_at,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()),ensure_ascii=False,indent=2)+'\n')
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
