#!/usr/bin/env python3
"""VM-only ground telemetry: never publishes Arm/mode/setpoint, never claims hardware."""
import rclpy
import json,time
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from px4_msgs.msg import (BatteryStatus,FailsafeFlags,ManualControlSetpoint,ManualControlSwitches,
 VehicleAttitude,VehicleLandDetected,VehicleLocalPosition,VehicleStatus,VehicleCommandAck,VehicleAttitudeSetpoint,VehicleCommand,OffboardControlMode)
rclpy.init(args=[])
n=Node('fc01_vm_synthetic_disarmed_ground')
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
classes={'vehicle_status_v1':VehicleStatus,'vehicle_land_detected':VehicleLandDetected,
'vehicle_local_position':VehicleLocalPosition,'vehicle_attitude':VehicleAttitude,
'failsafe_flags':FailsafeFlags,'battery_status':BatteryStatus,'vehicle_command_ack':VehicleCommandAck,
'manual_control_setpoint':ManualControlSetpoint,'manual_control_switches':ManualControlSwitches}
pubs={name:n.create_publisher(cls,'/fmu/out/'+name,q) for name,cls in classes.items()}
def publish():
 t=n.get_clock().now().nanoseconds//1000
 values={
 'vehicle_status_v1':VehicleStatus(timestamp=t,system_id=1,component_id=1,arming_state=1,nav_state=15,
  pre_flight_checks_pass=True,power_input_valid=True,can_set_nav_states_mask=(1<<15)),
 'vehicle_land_detected':VehicleLandDetected(timestamp=t,landed=True,at_rest=True,ground_contact=True,maybe_landed=True),
 'vehicle_local_position':VehicleLocalPosition(timestamp=t,timestamp_sample=t-2000,x=0.,y=0.,z=2.,vx=0.,vy=0.,vz=0.,z_deriv=0.,
  xy_valid=True,v_xy_valid=True,z_valid=True,v_z_valid=True,heading_good_for_control=True,
  dist_bottom_valid=True,dist_bottom=.1,dist_bottom_var=.0025,dist_bottom_sensor_bitfield=VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE),
 'vehicle_attitude':VehicleAttitude(timestamp=t,timestamp_sample=t-2000,q=[1.,0.,0.,0.]),
 'failsafe_flags':FailsafeFlags(timestamp=t),
 'battery_status':BatteryStatus(timestamp=t,id=1,connected=True,voltage_v=7.6,remaining=1.),
 'manual_control_setpoint':ManualControlSetpoint(timestamp=t,timestamp_sample=t-2000,valid=True,data_source=ManualControlSetpoint.SOURCE_RC,throttle=-1.),
 'manual_control_switches':ManualControlSwitches(timestamp=t,timestamp_sample=t-2000,mode_slot=6,arm_switch=3,kill_switch=3,return_switch=3,loiter_switch=3,offboard_switch=3)}
 for key,value in values.items():pubs[key].publish(value)
output=open('/home/orangepi/vm-output.jsonl','a',buffering=1)
def observe(kind,msg):
 value={'kind':kind,'monotonic':time.monotonic(),'timestamp':msg.timestamp}
 if kind=='setpoint':
  value['thrust_body']=[float(x) for x in msg.thrust_body]
  if any(msg.thrust_body):raise RuntimeError('nonzero thrust during always-disarmed VM install')
 if kind=='command':raise RuntimeError('unexpected VehicleCommand during always-disarmed VM install')
 output.write(json.dumps(value)+'\n')
reliable=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
subs=[n.create_subscription(VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint',lambda m:observe('setpoint',m),reliable),
 n.create_subscription(VehicleCommand,'/fmu/in/vehicle_command',lambda m:observe('command',m),reliable),
 n.create_subscription(OffboardControlMode,'/fmu/in/offboard_control_mode',lambda m:observe('heartbeat',m),reliable)]
n.create_timer(.025,publish)
rclpy.spin(n)
