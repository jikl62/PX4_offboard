#!/usr/bin/env python3
"""Opt-in real ROS adapter check. Creates only /fc01_demo/** data endpoints."""
from pathlib import Path
import math
import signal
import subprocess
import sys
import tempfile
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from jikl_fc01_system.msg import VisionHealth, VisionState

    root=Path(__file__).resolve().parents[1]
    params=dict(input_topic='/fc01_demo/odom',health_topic='/fc01_demo/health',
                output_topic='/fc01_demo/state',source_id='synthetic-ros-smoke',
                input_frame_id='demo_enu',input_body_frame_id='demo_flu',
                world_convention='ENU',body_convention='FLU',
                extrinsics_id='synthetic-identity',alignment_id='synthetic-identity',
                body_from_sensor_wxyz='[1.0,0.0,0.0,0.0]',sensor_position_body_frd='[0.0,0.0,0.0]',
                world_alignment_wxyz='[1.0,0.0,0.0,0.0]',world_translation_ned='[0.0,0.0,0.0]')
    argv=[sys.executable,str(root/'native/tools/vision_odometry_adapter.py'),'--ros-args']
    for name,value in params.items():argv+=['-p',name+':='+value]
    with tempfile.TemporaryFile(mode='w+') as logs:
        process=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=logs,stderr=subprocess.STDOUT)
        node=None;rclpy.init()
        try:
            node=Node('fc01_vision_smoke_source')
            sensor=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
            reliable=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
            odom_pub=node.create_publisher(Odometry,params['input_topic'],sensor)
            health_pub=node.create_publisher(VisionHealth,params['health_topic'],reliable)
            received=[]
            subscription=node.create_subscription(VisionState,params['output_topic'],received.append,sensor)
            sequence=0

            def publish():
                nonlocal sequence
                sequence+=1;stamp=node.get_clock().now().to_msg()
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='demo_enu';odom.child_frame_id='demo_flu'
                odom.pose.pose.position.x=1.;odom.pose.pose.position.y=2.;odom.pose.pose.position.z=3.
                odom.pose.pose.orientation.w=1.
                odom.twist.twist.linear.x=.4;odom.twist.twist.linear.y=.5;odom.twist.twist.linear.z=.6
                odom.twist.twist.angular.x=.1;odom.twist.twist.angular.y=.2;odom.twist.twist.angular.z=.3
                health=VisionHealth();health.stamp=stamp;health.sequence=sequence;health.source_id=params['source_id']
                health.frame_id='demo_enu';health.body_frame_id='demo_flu'
                for flag in ('reset_valid','initialized','tracking','calibration_valid','frame_alignment_valid',
                             'time_sync_valid','extrinsics_valid','metric_scale_valid','position_valid',
                             'velocity_valid','attitude_valid','angular_velocity_valid','quality_valid'):
                    setattr(health,flag,True)
                health.angular_velocity_source='synthetic-imu';health.quality=1.
                health.extrinsics_id=health.alignment_id='synthetic-identity'
                health_pub.publish(health);odom_pub.publish(odom)

            def wait_for(predicate):
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    if process.poll() is not None:raise RuntimeError('adapter exited before data check')
                    publish();rclpy.spin_once(node,timeout_sec=.05)
                    if predicate():return
                raise RuntimeError('ROS adapter data check timed out')

            wait_for(lambda:any((x.valid_fields&71)==71 for x in received))
            valid=next(x for x in reversed(received) if x.valid_fields&71==71)
            for actual,want in zip(valid.position,(2.,1.,-3.)):
                if abs(actual-want)>1e-8:raise RuntimeError('ENU position conversion mismatch')
            for actual,want in zip(valid.velocity,(.5,.4,-.6)):
                if abs(actual-want)>1e-8:raise RuntimeError('child-frame velocity conversion mismatch')
            if abs(sum(x*x for x in valid.attitude)-1)>1e-8:raise RuntimeError('invalid output quaternion')
            if valid.valid_fields&48 or not all(math.isnan(x) for x in valid.pose_covariance):
                raise RuntimeError('untransformed covariance was marked valid')
            last_sequence=valid.sequence
            duplicate=node.create_publisher(Odometry,params['input_topic'],sensor)
            wait_for(lambda:any(x.sequence>last_sequence and x.valid_fields==0 for x in received))
            node.destroy_publisher(duplicate)
            endpoints=node.get_publisher_names_and_types_by_node('fc01_vision_odometry_adapter','/')
            if any(name.startswith('/fmu/in/') for name,_ in endpoints):raise RuntimeError('unexpected hardware endpoint')
            if subscription is None:raise RuntimeError('subscription was not retained')
            print('REAL_ROS_VISION_ADAPTER_PASS valid_conversion=1 raw_conflict_revoked=1 fmu_publishers=0')
        except Exception:
            logs.seek(0);print(logs.read()[-5000:],file=sys.stderr);raise
        finally:
            if node is not None:node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
    return 0


if __name__=='__main__':raise SystemExit(main())
