#!/usr/bin/env python3
"""Export a candidate YAML without ROS, hardware checks, or service operations."""
import argparse
import json
from pathlib import Path
import sys

import yaml
from launcher import ControlError
from navigation_service import navigation_profile
from vision_settings import apply_settings

ROOT=Path(__file__).resolve().parent


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=ROOT/'native/config/attitude_altitude.yaml')
    parser.add_argument('--settings',type=Path,default=ROOT/'vision_settings.json')
    parser.add_argument('--navigation-settings',type=Path,default=ROOT/'navigation_settings.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--enable-vision',action='store_true',help='enable only in the exported candidate, retaining the source settings')
    args=parser.parse_args(argv)
    try:
        base=yaml.safe_load(args.base.read_text())
        navigation=json.loads(args.navigation_settings.read_text())
        selected=json.loads(args.settings.read_text())
        if args.enable_vision:selected={**selected,'enabled':True}
        config=navigation_profile(base,navigation.get('target_height_m',.5),navigation.get('navigation_mode_slot',6))
        apply_settings(config,selected)
        args.output.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
        print('候选 YAML 已生成：'+str(args.output))
        print('visual_navigation_enabled='+str(config['fc01_system']['ros__parameters']['visual_navigation_enabled']))
        return 0
    except (OSError,ValueError,TypeError,KeyError,ControlError,yaml.YAMLError) as exc:
        print('配置生成失败：'+str(exc),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
