#!/usr/bin/env python3
# 原生源码包导出/传输工具：校验 Ubuntu/Humble 目标与固定 px4_msgs 提交，打包自研源码和真实消息。
# copy 子命令确实会通过 SSH 上传并执行安装；本轮阅读/验证不调用该外部写入路径。
"""Build a source-only portable bundle or copy it to an SSH target."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import tarfile
import zipfile

# 固定消息源码身份是协议合同，目录名称相同不表示与当前 PX4 固件消息兼容。
PIN='392e831c1f659429ca83902e66820d7094591410'
VERSION=(Path(__file__).resolve().parents[1]/'VERSION').read_text().strip()
ROOT=Path(__file__).resolve().parents[1]

# 只允许已声明的系统版本与原生架构；不自动跨平台转译或下载依赖。
def validate_platform(os_data,arch):
    if os_data.get('ID')!='ubuntu' or os_data.get('VERSION_ID')!='22.04':
        raise ValueError('本交付固定 Ubuntu 22.04 + ROS Humble；请先提供当前系统信息')
    if arch not in {'aarch64','x86_64'}:
        raise ValueError('只支持原生 aarch64 或 x86_64 编译')

# 排除版本控制、构建安装、日志及 Python 缓存，避免把运行态和凭据装入源码包。
def include_in_bundle(relative):
    return not any(x in {'.git','.internal','.superpowers','__pycache__','build','install','log','logs'} for x in relative.parts) and relative.suffix!='.pyc' and relative.name!='core'

# 按相对路径计算文件真实字节摘要；SOURCE.json 自身排除以避免自引用。
def hashes(root):
    return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file() and p.name!='SOURCE.json' and '.git' not in p.relative_to(root).parts}

# 为无 Git 元数据的 vendor 副本写入固定提交与逐文件摘要合同。
def write_vendor_manifest(root):
    (root/'SOURCE.json').write_text(json.dumps({'commit':PIN,'files':hashes(root)},indent=2)+'\n')

# vendor 的提交标识和完整文件树必须同时匹配，新增/缺失/改字节均拒绝。
def verify_vendor(root):
    manifest=json.loads((root/'SOURCE.json').read_text())
    if manifest.get('commit')!=PIN or manifest.get('files')!=hashes(root):
        raise ValueError('消息源码校验失败；没有使用该目录')

# 有 SOURCE.json 时按归档摘要检查；否则要求 Git 精确提交且没有本地修改。
def verify_messages(root):
    if (root/'SOURCE.json').is_file():
        verify_vendor(root);return
    result=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    changes=subprocess.check_output(['git','-C',str(root),'status','--porcelain'],text=True).strip()
    if result!=PIN or changes:
        raise ValueError('px4_msgs 必须是已验证提交且没有本地修改：'+PIN)

# 构造源码 ZIP 并返回路径；拒绝输出到工程内部，避免递归把新包再次打包。
def export_bundle(messages,destination):
    verify_messages(messages)
    # git archive exports the checked commit, without remote config or credentials.
    if (messages/'SOURCE.json').is_file():
        vendor={p.relative_to(messages).as_posix():p.read_bytes() for p in messages.rglob('*') if p.is_file()}
    else:
        data=subprocess.check_output(['git','-C',str(messages),'archive',PIN])
        vendor={}
        with tarfile.open(fileobj=io.BytesIO(data)) as archive:
            for entry in archive.getmembers():
                if entry.isfile():vendor[entry.name]=archive.extractfile(entry).read()
        vendor['SOURCE.json']=(json.dumps({'commit':PIN,'files':{name:hashlib.sha256(data).hexdigest() for name,data in vendor.items()}},indent=2)+'\n').encode()
    if destination.resolve().is_relative_to(ROOT):
        raise ValueError('输出 zip 请放到工程目录以外，避免递归打包')
    with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as archive:
        prefix='fc01_system_v'+VERSION+'/'
        for path in sorted(ROOT.rglob('*')):
            relative=path.relative_to(ROOT)
            if path.is_file() and include_in_bundle(relative) and relative.parts[0]!='vendor':
                archive.write(path,prefix+relative.as_posix())
        for name,data in vendor.items():archive.writestr(prefix+'vendor/px4_msgs/'+name,data)
    return destination

# 只接受单个安全 ZIP 文件名，再整体转义远端脚本；不允许文件名注入 shell 命令。
def remote_install_command(filename):
    if not re.fullmatch(r'[A-Za-z0-9_.-]+\.zip',filename):
        raise ValueError('远端包文件名只能包含字母、数字、下划线、点和短横线')
    script='set -e\nmkdir -p "$HOME/fc01_uploads/unpacked"\n'
    script+='unzip -o "$HOME/fc01_uploads/'+filename+'" -d "$HOME/fc01_uploads/unpacked"\n'
    script+='bash "$HOME/fc01_uploads/unpacked/fc01_system_v'+VERSION+'/scripts/install.sh"\n'
    return 'bash -c '+shlex.quote(script)

# 分派本地检查/导出或显式 SSH 安装，异常返回非零；没有隐式激活或飞行命令。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    subs=parser.add_subparsers(dest='op',required=True)
    check=subs.add_parser('check-platform')
    check.add_argument('--os-release',default='/etc/os-release');check.add_argument('--arch',default=platform.machine())
    msgs=subs.add_parser('verify-messages');msgs.add_argument('path',type=Path)
    export=subs.add_parser('export');export.add_argument('messages',type=Path);export.add_argument('output',type=Path)
    copy=subs.add_parser('copy');copy.add_argument('host');copy.add_argument('bundle',type=Path)
    args=parser.parse_args()
    try:
        if args.op=='check-platform':
            data={}
            for line in Path(args.os_release).read_text().splitlines():
                if '=' in line:
                    key,val=line.split('=',1);data[key]=val.strip('"')
            validate_platform(data,args.arch);print('Platform OK:',args.arch)
        elif args.op=='verify-messages':verify_messages(args.path);print('Message source OK:',PIN)
        elif args.op=='export':print(export_bundle(args.messages,args.output))
        else:
            if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:\[\]-]*',args.host):raise ValueError('SSH host invalid')
            bundle=args.bundle.resolve();remote=remote_install_command(bundle.name)
            subprocess.run(['ssh','--',args.host,'mkdir -p "$HOME/fc01_uploads"'],check=True)
            subprocess.run(['scp','--',str(bundle),args.host+':fc01_uploads/'+bundle.name],check=True)
            subprocess.run(['ssh','--',args.host,remote],check=True)
    except (ValueError,OSError,subprocess.CalledProcessError,json.JSONDecodeError) as error:
        print(str(error),file=sys.stderr);return 2
    return 0
if __name__=='__main__':sys.exit(main())
