#!/usr/bin/env python3
"""扫描自主源码候选并结合人工记录生成中文可读性报告，不以汉字存在判定完成。

输入：Git 管理及未忽略的文件，和显式指定的逐文件人工审查 JSON。
输出：标准输出 JSON；只读扫描，不改源码、配置、飞行参数或 Git 状态。
人工结论绑定 SHA256，文件变化即失效。复杂度指标仅提示复核，不能证明算法正确。
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess


# 枚举业务源码和部署格式；后缀相同的生成安装包仍须由排除规则识别。
LANGUAGES = {
    '.cpp': 'cpp',
    '.hpp': 'cpp',
    '.h': 'cpp',
    '.cc': 'cpp',
    '.cxx': 'cpp',
    '.py': 'python',
    '.sh': 'shell',
    '.msg': 'ros_message',
    '.cmake': 'cmake',
    '.yaml': 'yaml',
    '.yml': 'yaml',
    '.service': 'systemd',
    '.inc': 'cpp',
}
# 只纳入本工程的这三份元数据，不把任意第三方 XML/TOML 或嵌套工具配置算成自主源码。
# package.xml 位于唯一的正式 ROS 包根；两份排版配置只认仓库根路径。
SOURCE_METADATA = {
    'native/src/jikl_fc01_system/package.xml': 'package_xml',
    '.clang-format': 'formatter_config',
    'pyproject.toml': 'python_tool_config',
}
CHINESE = re.compile(r'[\u3400-\u9fff]')


def git(root, *arguments):
    """在 root 执行只读 Git 查询；失败直接退出，避免用残缺清单报告覆盖率。"""
    return subprocess.run(
        ['git', '-C', str(root), *arguments],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ).stdout


def language(path):
    """识别业务源码及三份明确登记的工程元数据，数据 JSON/说明文档仍单列。"""
    if path.as_posix() in SOURCE_METADATA:
        return SOURCE_METADATA[path.as_posix()]
    return 'cmake' if path.name == 'CMakeLists.txt' else LANGUAGES.get(path.suffix)


def exclusion(path):
    """按来源与用途分类排除；历史漏洞反例保持字节不变，当前正向测试不排除。"""
    parts = path.parts
    if parts[0] == 'dist':
        return 'generated_release_bundle', '生成的自解压发布包；应修改打包输入，不能直接编辑载荷'
    if parts[:2] == ('docs', 'evidence'):
        return 'historical_evidence', '历史证据快照，保留原始可复现字节'
    if 'c_independent_repro' in parts or 'c_s10_independent' in parts:
        return 'historical_reproduction', '独立历史审查反例；单列保留，不能当成当前正向回归'
    if parts[:2] == ('native', 'tests') and any(
        part.startswith('historical_') for part in parts[2:]
    ):
        return 'historical_reproduction', '归档的历史测试载荷；保留原字节，不计入当前源码完成数'
    return None


def ignored_entry(name):
    """逐项分类 Git 忽略文件，已确认的回放生成映射与未知来源明确区分。"""
    parts = Path(name).parts
    if parts[:2] == ('.superpowers', 'vision_replay'):
        category, reason = (
            'generated_replay_cache',
            'vision_replay.py 按源码哈希生成的回放二进制/参数映射缓存',
        )
    elif parts[0] in {'build', '.build', 'log', 'dist'}:
        category, reason = (
            'generated_build_or_release',
            '构建树、构建日志或生成发布产物，不直接修改',
        )
    elif '__pycache__' in parts or '.pytest_cache' in parts:
        category, reason = 'interpreter_test_cache', 'Python 字节码或测试工具缓存，来源是受审源码'
    else:
        category, reason = (
            'git_ignored_requires_origin_review',
            'Git 忽略文件，来源仍需人工确认，不计作完成',
        )
    return {'path': name, 'category': category, 'reason': reason}


def group_ignored(names):
    """同类原因只保存一次，每个具体路径仍有唯一归属，避免报告被重复说明撑大。"""
    groups = {}
    for name in sorted(set(names)):
        entry = ignored_entry(name)
        key = (entry['category'], entry['reason'])
        groups.setdefault(key, []).append(name)
    return [
        {'category': category, 'reason': reason, 'paths': paths}
        for (category, reason), paths in sorted(groups.items())
    ]


def inspect(root, name, reviews, initially_modified):
    """逐文件计算统计并核对人工摘要；SHA 不匹配的旧完成状态按未完成处理。"""
    path = root / name
    kind = language(Path(name))
    omitted = exclusion(Path(name))
    if path.is_symlink():
        return {
            'path': name,
            'excluded': True,
            'category': 'symlink',
            'reason': '不跟随链接扫描外部来源，需显式核对链接目标',
        }
    if not path.is_file():
        return {
            'path': name,
            'excluded': True,
            'category': 'missing',
            'reason': 'Git 清单存在但工作树无文件，不能认定已审查',
        }
    if not kind or omitted:
        category, reason = omitted or ('non_source', '非本次可注释源码格式；数据、文档或元数据单列')
        return {'path': name, 'excluded': True, 'category': category, 'reason': reason}
    data = path.read_bytes()
    try:
        source = data.decode('utf-8')
    except UnicodeDecodeError:
        return {
            'path': name,
            'excluded': True,
            'category': 'non_utf8',
            'reason': '非 UTF-8 文本，需先人工确认来源和编码',
        }
    lines = source.splitlines()
    digest = hashlib.sha256(data).hexdigest()
    review = reviews.get(name, {})
    current_review = review if review.get('sha256') == digest else {}
    # 注释、排版和结构拆分是不同事实。旧记录没有新排版字段时，只继承同 SHA 的
    # readability_complete；旧 SHA 的所有完成标记都失效，不能靠文件中有汉字补回。
    annotation_complete = current_review.get('annotation_complete', False) is True
    readability_complete = current_review.get('readability_complete', False) is True
    layout_finished = current_review.get('layout_finished', readability_complete) is True
    structural_pending = current_review.get('structural_refactor_pending', [])
    if not isinstance(structural_pending, list) or not all(
        isinstance(item, str) for item in structural_pending
    ):
        raise ValueError(name + ': structural_refactor_pending must be a list of explicit items')
    # 这些指标故意保守：字符串/注释中的分号也会触发，仅作为人工复核线索。
    dense = [
        i
        for i, line in enumerate(lines, 1)
        if len(line) > 120 or (kind == 'cpp' and line.count(';') >= 2)
    ]
    return {
        'path': name,
        'language': kind,
        'excluded': False,
        'sha256': digest,
        'lines': len(lines),
        'chinese_lines': sum(bool(CHINESE.search(line)) for line in lines),
        'density_review_lines': dense,
        'reviewed': bool(current_review),
        'annotation_complete': annotation_complete,
        'readability_complete': readability_complete,
        'layout_finished': layout_finished,
        'structural_refactor_pending': structural_pending,
        'review_notes': current_review.get('notes', '尚未完成当前文件逐项人工复核'),
        'outstanding': current_review.get(
            'outstanding', ['逐项核对职责、输入输出、物理量、判断、状态和格式']
        ),
        'stale_review': bool(review and not current_review),
        'preexisting_uncommitted_change': name in initially_modified,
    }


def report(root, ledger):
    """合并真实扫描与当前字节对应的人工结论，所有未完成文件保留完整路径。"""
    names = sorted(
        set(
            git(root, 'ls-files', '--cached', '--others', '--exclude-standard', '-z')
            .decode()
            .rstrip('\0')
            .split('\0')
        )
        - {''}
    )
    files = [
        inspect(root, name, ledger.get('reviews', {}), set(ledger.get('initially_modified', [])))
        for name in names
    ]
    sources = [entry for entry in files if not entry['excluded']]
    groups = {}
    for kind in sorted(set(LANGUAGES.values()) | set(SOURCE_METADATA.values())):
        group = [entry for entry in sources if entry['language'] == kind]
        groups[kind] = {
            'total': len(group),
            'reviewed': sum(entry['reviewed'] for entry in group),
            'annotation_complete': sum(entry['annotation_complete'] for entry in group),
            'readability_complete': sum(entry['readability_complete'] for entry in group),
            'layout_finished': sum(entry['layout_finished'] for entry in group),
            'structural_refactor_pending': sum(
                bool(entry['structural_refactor_pending']) for entry in group
            ),
        }
    # 忽略目录通常含构建树、缓存和二进制。分类理由只写一次，具体路径一个不省略；
    # 报告压缩的是重复说明，不放宽安装包的文件数、总字节或单文件大小保护。
    ignored = git(root, 'ls-files', '--others', '--ignored', '--exclude-standard', '-z')
    ignored_names = sorted(set(ignored.decode().rstrip('\0').split('\0')) - {''})
    excluded = [entry for entry in files if entry['excluded']]
    ignored_groups = group_ignored(ignored_names)
    changed_during_scan = [
        entry['path']
        for entry in sources
        if not (root / entry['path']).is_file()
        or hashlib.sha256((root / entry['path']).read_bytes()).hexdigest() != entry['sha256']
    ]
    changed = set(git(root, 'diff', ledger['baseline_commit'], '--name-only').decode().splitlines())
    changed.update(git(root, 'ls-files', '--others', '--exclude-standard').decode().splitlines())
    return {
        'schema_version': 2,
        'baseline_commit': ledger.get('baseline_commit'),
        'current_commit': git(root, 'rev-parse', 'HEAD').decode().strip(),
        'changed_during_scan': changed_during_scan,
        'quality_basis': '逐文件人工结论绑定当前 SHA256；汉字/行长计数不表示充分解释',
        'source_total': len(sources),
        'reviewed': sum(entry['reviewed'] for entry in sources),
        'modified_since_baseline': sum(entry['path'] in changed for entry in sources),
        'all_source_criteria_met': all(
            entry['annotation_complete'] and entry['readability_complete'] for entry in sources
        ),
        'annotation_complete': sum(entry['annotation_complete'] for entry in sources),
        'annotation_unfinished': [
            entry['path'] for entry in sources if not entry['annotation_complete']
        ],
        'readability_complete': sum(entry['readability_complete'] for entry in sources),
        'layout_finished': sum(entry['layout_finished'] for entry in sources),
        'structural_refactor_pending': [
            {'path': entry['path'], 'remaining': entry['structural_refactor_pending']}
            for entry in sources
            if entry['structural_refactor_pending']
        ],
        'unfinished': [
            entry['path']
            for entry in sources
            if not (entry['annotation_complete'] and entry['readability_complete'])
        ],
        'by_language': groups,
        'density_review_files': [
            entry['path'] for entry in sources if entry['density_review_lines']
        ],
        'exclusion_counts': dict(Counter(entry['category'] for entry in excluded)),
        'excluded': excluded,
        'ignored_file_count': len(ignored_names),
        'ignored_group_count': len(ignored_groups),
        'ignored_files': ignored_groups,
        'ignored_source_origin_review_required': [
            name
            for group in ignored_groups
            if group['category'] == 'git_ignored_requires_origin_review'
            for name in group['paths']
            if language(Path(name))
        ],
        'files': sources,
    }


def main():
    """命令行只接受仓库根和人工记录路径；JSON 输出可由调用者重定向到证据文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--ledger', type=Path, required=True)
    arguments = parser.parse_args()
    ledger = json.loads(arguments.ledger.read_text(encoding='utf-8'))
    print(json.dumps(report(arguments.root.resolve(), ledger), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
