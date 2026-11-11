#!/usr/bin/env python3
# 真实Humble下的只读年龄探针冒烟：发布合成反馈，核对包络时间与原始采样时间。
# 仅使用localhost的Domain 189；不启动控制器/Agent，不能证明实际飞控数据健康。
"""Isolated real-Humble probe check: synthetic feedback only, no controller/Agent."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


# 创建隔离ROS节点与临时状态文件，运行探针子进程，核对回调统计和只读边界。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    os.environ.update(
        ROS_DOMAIN_ID='189', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp'
    )
    # 清除外部发现/XML配置，避免本机环境将合成话题接入其他DDS域。
    for key in (
        'ROS_DISCOVERY_SERVER',
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
        'ROS_STATIC_PEERS',
    ):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

    rclpy.init(args=[])
    node = Node('fc01_probe_synthetic_source', enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    publications = {
        'attitude': node.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos),
        'local': node.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos
        ),
    }
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='fc01-probe-smoke-') as directory:
            directory = Path(directory)
            status, output = directory / 'status.json', directory / 'probe.jsonl'
            stamp = time.time_ns() // 1000
            status.write_text(
                json.dumps(
                    dict(
                        updated_unix=time.time(),
                        feedback_current=False,
                        controller_pid=None,
                        state=dict(
                            source_timing={
                                key: dict(
                                    envelope_us=stamp,
                                    sample_us=stamp - 600_000,
                                    envelope_age_s=0.0,
                                    sample_age_s=0.6,
                                )
                                for key in publications
                            }
                        ),
                    )
                )
            )
            original_status = status.read_bytes()
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(root / 'feedback_age_probe.py'),
                    '--duration',
                    '5',
                    '--status',
                    str(status),
                    '--output',
                    str(output),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            stop = time.monotonic() + 9.0
            next_publish = time.monotonic()
            observed_probe_publishers = set()
            while process.poll() is None and time.monotonic() < stop:
                now = time.monotonic()
                if now >= next_publish:
                    # 包络采用当前时钟，timestamp_sample故意落后600ms，用来防止将新包络误当成新测量。
                    stamp = time.time_ns() // 1000
                    publications['attitude'].publish(
                        VehicleAttitude(
                            timestamp=stamp,
                            timestamp_sample=stamp - 600_000,
                            q=[1.0, 0.0, 0.0, 0.0],
                        )
                    )
                    publications['local'].publish(
                        VehicleLocalPosition(timestamp=stamp, timestamp_sample=stamp - 600_000)
                    )
                    next_publish += 0.025
                rclpy.spin_once(node, timeout_sec=0.002)
                if 'fc01_feedback_age_probe' in node.get_node_names():
                    for topic, _ in node.get_topic_names_and_types():
                        for info in node.get_publishers_info_by_topic(topic):
                            if info.node_name == 'fc01_feedback_age_probe':
                                observed_probe_publishers.add(topic)
            stdout, stderr = process.communicate(timeout=2)
            if process.returncode:
                raise RuntimeError(f'probe exit {process.returncode}: {stderr}\n{stdout}')
            records = [json.loads(line) for line in output.read_text().splitlines()]
            assert [json.loads(line) for line in stdout.splitlines()] == records
            start, final = records[0], records[-1]
            assert start['ros_domain_id'] == 189, start
            assert start['rmw_implementation'] == 'rmw_fastrtps_cpp', start
            assert final['kind'] == 'feedback_age_probe_final', final
            assert final['controller_status']['available'], final
            # 探针必须保持输入状态文件逐字节不变，且不得发布任何/fmu/in控制话题。
            assert original_status == status.read_bytes()
            assert observed_probe_publishers <= {'/parameter_events'}, observed_probe_publishers
            assert not any(
                topic.startswith('/fmu/in/') for topic, _ in node.get_topic_names_and_types()
            )
            for row in final['topics'].values():
                # 同时要求足够回调、新包络、旧样本与可用RMW接收时间，避免只看收包数量。
                assert row['callbacks_total'] >= 140, row
                assert row['last']['callback_envelope_age_s'] < 0.15, row
                assert row['last']['callback_sample_age_s'] > 0.5, row
                assert row['last']['rmw_received_available'], row
                assert row['last']['rmw_to_callback_s'] is not None, row
                assert row['envelope_progress']['regressed'] == 0, row
            report = dict(
                passed=True,
                isolation='localhost Domain 189, synthetic /fmu/out only',
                probe_publishers_observed=sorted(observed_probe_publishers),
                records=records,
            )
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2) + '\n')
            print(
                json.dumps(
                    dict(
                        passed=True,
                        probe_publishers_observed=sorted(observed_probe_publishers),
                        topics={
                            key: dict(callbacks=row['callbacks_total'], last=row['last'])
                            for key, row in final['topics'].items()
                        },
                    )
                ),
                flush=True,
            )
    # 即使断言失败，也要终止探针并销毁ROS资源，防止遗留测试节点。
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.communicate(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
