#!/usr/bin/env python3
# 视觉候选配置导出器：只读现有基础 YAML/导航与视觉设置，写入明确指定的新 YAML。
# 它不运行 ROS、检查硬件或激活服务；--enable-vision 仅作用于导出的候选，不回写原设置。
"""Export a candidate YAML without ROS, hardware checks, or service operations."""
import argparse
import json
from pathlib import Path
import sys

import yaml
from launcher import ControlError
from fc01_control import DEFAULT_TAKEOFF_HEIGHT_M
from navigation_service import navigation_profile
from vision_settings import apply_settings

ROOT = Path(__file__).resolve().parent


# 解析文件路径，先合成原导航参数，再叠加已校验视觉参数；返回 0 表示导出成功，2 表示配置拒绝。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=ROOT / 'native/config/attitude_altitude.yaml')
    parser.add_argument('--settings', type=Path, default=ROOT / 'vision_settings.json')
    parser.add_argument(
        '--navigation-settings', type=Path, default=ROOT / 'navigation_settings.json'
    )
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--enable-vision',
        action='store_true',
        help='enable only in the exported candidate, retaining the source settings',
    )
    args = parser.parse_args(argv)
    try:
        base = yaml.safe_load(args.base.read_text())
        navigation = json.loads(args.navigation_settings.read_text())
        selected = json.loads(args.settings.read_text())
        # 只复制当前内存中的视觉设置，避免导出操作修改磁盘上的默认关闭策略。
        if args.enable_vision:
            selected = {**selected, 'enabled': True}
        # 高度与导航档位来自导航合同，视觉设置不得绕过原飞行包络校验。
        config = navigation_profile(
            base,
            navigation.get('target_height_m', DEFAULT_TAKEOFF_HEIGHT_M),
            navigation.get('navigation_mode_slot', 6),
        )
        apply_settings(config, selected)
        args.output.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
        print('候选 YAML 已生成：' + str(args.output))
        print(
            'visual_navigation_enabled='
            + str(config['fc01_system']['ros__parameters']['visual_navigation_enabled'])
        )
        return 0
    # 文件、类型或合同错误统一拒绝；没有部分启动或回退到不受控默认值。
    except (OSError, ValueError, TypeError, KeyError, ControlError, yaml.YAMLError) as exc:
        print('配置生成失败：' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
