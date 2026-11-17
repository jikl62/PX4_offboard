#!/usr/bin/env python3
"""只读比较注释/排版前后的源码；输出 JSON，任一受保护项变化即返回非零。

示例：python3 tests/readability_equivalence.py --baseline-git HEAD~1 --output /tmp/equivalence.json
也可将 --baseline-git 换成 --baseline-dir /path/to/snapshot；--inventory 可限定已审查清单。
这是源码结构保真检查，不能代替编译、回归或实飞验证；不会执行被检查的脚本。
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


# 历史反例、证据及发行载荷必须原字节保留；常规源码才允许按对应语言忽略注释/排版。
PROTECTED_PREFIXES = (
    'dist/',
    'docs/evidence/',
    'docs/data/',
    'native/tests/historical_',
    'native/tests/c_independent_repro/',
    'native/tests/c_s10_independent/',
)
CPP_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx'}


class CheckError(ValueError):
    """无法可靠解释输入时明确失败，不能把解析失败当成两份源码相同。"""


@dataclass(frozen=True)
class BaselineFile:
    data: bytes
    # Git 基线保留 executable 位；普通目录快照可能未保存权限，因此不猜测其权限。
    executable: bool | None = None
    symlink: bool = False


def git(root, *args, input=None):
    result = subprocess.run(
        ['git', '-C', str(root), *args], input=input, capture_output=True, timeout=60
    )
    if result.returncode:
        raise CheckError(result.stderr.decode(errors='replace').strip())
    return result.stdout


def git_baseline(root, revision):
    """按不可变 blob ID 批量读取 Git 内容，不 checkout、不碰工作区。"""
    commit = git(root, 'rev-parse', '--verify', revision + '^{commit}').decode().strip()
    rows = []
    for entry in git(root, 'ls-tree', '-r', '-z', commit).split(b'\0'):
        if not entry:
            continue
        metadata, raw_name = entry.split(b'\t', 1)
        mode, kind, object_id = metadata.decode().split()
        if kind != 'blob':
            raise CheckError('unsupported non-blob baseline entry: ' + os.fsdecode(raw_name))
        rows.append((os.fsdecode(raw_name), mode, object_id))
    stream = git(root, 'cat-file', '--batch', input=(''.join(x[2] + '\n' for x in rows)).encode())
    files, cursor = {}, 0
    for name, mode, object_id in rows:
        end = stream.index(b'\n', cursor)
        header = stream[cursor:end].decode().split()
        if len(header) != 3 or header[:2] != [object_id, 'blob']:
            raise CheckError('unexpected git cat-file header for ' + name)
        size = int(header[2])
        cursor = end + 1
        files[name] = BaselineFile(
            stream[cursor : cursor + size], mode == '100755', mode == '120000'
        )
        cursor += size
        if stream[cursor : cursor + 1] != b'\n':
            raise CheckError('truncated git blob for ' + name)
        cursor += 1
    return files, dict(kind='git', revision=revision, commit=commit)


def directory_baseline(folder):
    """目录模式适合已有快照；不跟随符号链接，不读取 .git 内部文件。"""
    folder = folder.resolve()
    if not folder.is_dir():
        raise CheckError('baseline directory does not exist: ' + str(folder))
    files = {}
    for parent, directories, names in os.walk(folder, followlinks=False):
        directories[:] = sorted(x for x in directories if x not in ('.git', '__pycache__'))
        for name in sorted(names):
            path = Path(parent) / name
            relative = path.relative_to(folder).as_posix()
            link = path.is_symlink()
            files[relative] = BaselineFile(
                os.fsencode(os.readlink(path)) if link else path.read_bytes(), symlink=link
            )
    return files, dict(kind='directory', path=str(folder))


def text(data):
    # 保留换行，不使用正则直接剥除字符串中的注释符；每种语言由下方自己的解析器处理。
    return data.decode('utf-8')


def cpp_structure(data, clang):
    """由 Clang 原始词法器识别完整 token，另外保留预处理指令的逻辑行边界。

    不加载 include，也不依赖 ROS。字面量按源码字节保留，不能把字符串中的 //、/* 或 # 当注释。
    临时文件名随机，解析 Clang 输出时只认该实际文件位置尾标，不按字面量内容猜 token 结尾。
    """
    with tempfile.TemporaryDirectory(prefix='fc01-raw-tokens-') as directory:
        path = Path(directory) / 'source.cpp'
        path.write_bytes(data)
        result = subprocess.run(
            [
                clang,
                '-x',
                'c++',
                '-std=c++17',
                '-fsyntax-only',
                '-Xclang',
                '-dump-raw-tokens',
                str(path),
            ],
            capture_output=True,
            timeout=30,
        )
        if result.returncode:
            raise CheckError(
                'Clang raw lexer failed: ' + result.stderr[:1000].decode(errors='replace')
            )
        dump = result.stderr
        # 位置中的列按字节计算；原始 token 跨行时，直到下一 token 的起始位置才结束。
        marker = re.compile(rb'\tLoc=<' + re.escape(os.fsencode(path)) + rb':(\d+):(\d+)>\r?\n')
        offsets = [0]
        for match in re.finditer(b'\n', data):
            offsets.append(match.end())
        raw, cursor = [], 0
        for match in marker.finditer(dump):
            record = dump[cursor : match.start()]
            kind_match = re.match(rb"([a-zA-Z_]+) '", record)
            if kind_match is None:
                raise CheckError('unrecognized Clang raw token output')
            line, column = int(match[1]), int(match[2])
            if line > len(offsets):
                raise CheckError('invalid Clang token location')
            raw.append((kind_match[1].decode(), offsets[line - 1] + column - 1, line))
            cursor = match.end()
        if dump[cursor:].strip():
            raise CheckError('unparsed Clang raw token output')
        if data and not raw:
            raise CheckError('Clang returned no raw tokens')

    tokens, directive, line_start = [], None, True
    for index, (kind, start, line) in enumerate(raw):
        end = raw[index + 1][1] if index + 1 < len(raw) else len(data)
        spelling = data[start:end]
        # 翻译阶段的反斜线换行不终止预处理行；字符串原始 spelling 仍完整保留。
        logical = re.sub(rb'\\\r?\n', b'', spelling)
        if kind == 'comment':
            continue
        if kind == 'unknown':
            if logical.strip():
                raise CheckError('non-whitespace unknown C++ token: ' + repr(spelling[:80]))
            if b'\n' in logical:
                if directive is not None:
                    tokens.append(('pp_end', b''))
                    directive = None
                line_start = True
            continue
        if kind == 'hash' and line_start:
            tokens.append(('pp_begin', b''))
            directive = []
        token_spelling = spelling if 'literal' in kind else logical
        tokens.append((kind, token_spelling))
        if directive is not None:
            # #define F(x) 与 #define F (x) 不等价。只保留决定函数式宏身份的邻接，
            # 宏其余空白可排版；指令结束通过 pp_end 独立保护，不能吞入下一语句。
            if len(directive) == 3 and directive[1][1] == b'define' and kind == 'l_paren':
                previous_end = directive[-1][2]
                tokens.append(('function_macro_adjacent', start == previous_end))
            directive.append((kind, logical, end))
        if kind == 'raw_identifier' and logical == b'__LINE__':
            tokens.append(('line_sensitive_builtin', line))
        line_start = False
    if directive is not None:
        tokens.append(('pp_end', b''))
    return tokens


def shell_structure(source):
    """只忽略确定处于 shell 语法外层的整行注释/空行，执行行保持原字节。

    shell 不做格式重写。引号、命令替换、参数展开的嵌套分别保存，避免把
    $(printf "多行文字") 内部的 # 文本当注释；heredoc 的正文和终止行完整保留。
    """
    output, continuation = [], False
    contexts = [('root', 0)]
    pending, active = [], None
    for line in source.splitlines(keepends=True):
        if active is not None:
            output.append(line)
            delimiter, strip_tabs = active
            comparison = line.lstrip('\t') if strip_tabs else line
            if comparison.rstrip('\r\n') == delimiter:
                active = pending.pop(0) if pending else None
            continue
        if (
            contexts == [('root', 0)]
            and not continuation
            and (not line.strip() or (line.lstrip().startswith('#') and not line.startswith('#!')))
        ):
            continue
        output.append(line)
        index, comment = 0, False
        while index < len(line):
            mode, depth = contexts[-1]
            char = line[index]
            if mode == 'single':
                if char == "'":
                    contexts.pop()
                index += 1
                continue
            if char == '\\':
                index += 2
                continue
            if mode in ('double', 'backtick', 'ansi'):
                closing = {'double': '"', 'backtick': '`', 'ansi': "'"}[mode]
                if char == closing:
                    contexts.pop()
                    index += 1
                    continue
                if mode in ('backtick', 'ansi'):
                    index += 1
                    continue
            # 命令替换/参数展开可嵌套在双引号中；进入独立上下文后内部引号不关闭外层。
            if line.startswith('$((', index):
                contexts.append(('arithmetic', 2))
                index += 3
                continue
            if line.startswith('$(', index):
                contexts.append(('command', 1))
                index += 2
                continue
            if line.startswith('${', index):
                contexts.append(('parameter', 1))
                index += 2
                continue
            if mode == 'double':
                if char == '`':
                    contexts.append(('backtick', 0))
                index += 1
                continue
            if mode in ('command', 'arithmetic') and char in '()':
                depth += 1 if char == '(' else -1
                if depth:
                    contexts[-1] = (mode, depth)
                else:
                    contexts.pop()
                index += 1
                continue
            if mode == 'parameter' and char in '{}':
                depth += 1 if char == '{' else -1
                if depth:
                    contexts[-1] = (mode, depth)
                else:
                    contexts.pop()
                index += 1
                continue
            if line.startswith("$'", index):
                contexts.append(('ansi', 0))
                index += 2
                continue
            if char in ('"', "'", '`'):
                contexts.append(({'"': 'double', "'": 'single', '`': 'backtick'}[char], 0))
                index += 1
                continue
            if mode in ('parameter', 'arithmetic'):
                index += 1
                continue
            if char == '#' and (
                index == 0 or line[index - 1].isspace() or line[index - 1] in ';&|()'
            ):
                comment = True
                break
            if line.startswith('((', index):
                contexts.append(('arithmetic', 2))
                index += 2
                continue
            if line.startswith('<<<', index):
                index += 3
                continue
            if line.startswith('<<', index):
                index += 2
                strip_tabs = line[index : index + 1] == '-'
                index += int(strip_tabs)
                while index < len(line) and line[index] in ' \t':
                    index += 1
                delimiter, delimiter_quote = [], None
                while index < len(line):
                    char = line[index]
                    if delimiter_quote:
                        if char == delimiter_quote:
                            delimiter_quote = None
                        else:
                            delimiter.append(char)
                    elif char in ('"', "'"):
                        delimiter_quote = char
                    elif char == '\\' and index + 1 < len(line):
                        index += 1
                        delimiter.append(line[index])
                    elif char.isspace() or char in ';&|()<>':
                        break
                    else:
                        delimiter.append(char)
                    index += 1
                if delimiter_quote or not delimiter:
                    raise CheckError('unsupported or incomplete heredoc delimiter')
                pending.append((''.join(delimiter), strip_tabs))
                continue
            index += 1
        stripped = line.rstrip('\r\n')
        trailing_slashes = len(stripped) - len(stripped.rstrip('\\'))
        continuation = bool(trailing_slashes % 2) and not comment and contexts[-1][0] != 'single'
        if pending and not continuation:
            active = pending.pop(0)
    if active is not None or pending:
        raise CheckError('unterminated heredoc')
    if contexts != [('root', 0)]:
        raise CheckError('unsupported or unterminated multiline shell context')
    return output


def cmake_structure(source):
    """按 CMake 的行注释、括号注释、引号参数和 bracket argument 分词，绝不执行 CMake。"""
    tokens, index = [], 0
    while index < len(source):
        if source[index].isspace():
            index += 1
            continue
        comment = source[index] == '#'
        opening = index + int(comment)
        bracket = re.match(r'\[(=*)\[', source[opening:])
        if bracket:
            close = ']' + bracket[1] + ']'
            end = source.find(close, opening + len(bracket[0]))
            if end < 0:
                raise CheckError('unterminated CMake bracket argument/comment')
            end += len(close)
            if not comment:
                tokens.append(('bracket', source[index:end]))
            index = end
        elif comment:
            end = source.find('\n', index)
            index = len(source) if end < 0 else end + 1
        elif source[index] in '()':
            tokens.append(('paren', source[index]))
            index += 1
        else:
            start, quoted = index, False
            while index < len(source):
                char = source[index]
                if char == '\\':
                    index += 2
                    continue
                if char == '"':
                    quoted = not quoted
                elif not quoted and (char.isspace() or char in '()#'):
                    break
                index += 1
            if quoted:
                raise CheckError('unterminated CMake quoted argument')
            tokens.append(('argument', source[start:index]))
    return tokens


def type_value(value):
    """让 YAML 的 true、1、1.0 仍是不同值，避免 Python 宽松相等掩盖参数类型变化。"""
    if isinstance(value, dict):
        return (
            'mapping',
            sorted([(type_value(k), type_value(v)) for k, v in value.items()], key=repr),
        )
    if isinstance(value, (tuple, list)):
        return (type(value).__name__, [type_value(x) for x in value])
    return (type(value).__name__, repr(value))


def message_structure(source):
    # ROS .msg 声明按行处理；常量/默认字符串中的引号与 # 原样保留。
    result = []
    for line in source.splitlines():
        quote, escaped, content = None, False, []
        for char in line:
            if escaped:
                content.append(char)
                escaped = False
            elif char == '\\':
                content.append(char)
                escaped = True
            elif quote:
                content.append(char)
                if char == quote:
                    quote = None
            elif char in ('"', "'"):
                content.append(char)
                quote = char
            elif char == '#':
                break
            else:
                content.append(char)
        declaration = ''.join(content).strip()
        if declaration:
            # 保守保留声明内部文本；只允许首尾排版，不重写字段类型/默认值。
            result.append(declaration)
    return result


def xml_structure(source):
    def element(node):
        return (
            node.tag,
            sorted(node.attrib.items()),
            (node.text or '').strip(),
            [(element(child), (child.tail or '').strip()) for child in node],
        )

    return element(ET.fromstring(source))


def comparison_kind(name, historical=False):
    path = PurePosixPath(name)
    if (
        historical
        or name.startswith(PROTECTED_PREFIXES)
        or path.suffix in ('.bak', '.original')
        or '.before_' in name
    ):
        return 'protected_bytes'
    if path.suffix in CPP_SUFFIXES:
        return 'cpp_raw_tokens'
    if path.suffix == '.py':
        return 'python_ast'
    if path.suffix == '.sh':
        return 'shell_execution_and_heredoc'
    if path.suffix in ('.yaml', '.yml'):
        return 'yaml_typed_values'
    if path.suffix == '.msg':
        return 'ros_message_declarations'
    if path.name == 'CMakeLists.txt' or path.suffix == '.cmake':
        return 'cmake_execution_tokens'
    if path.name == 'package.xml':
        return 'package_xml_semantics'
    if path.suffix == '.service':
        return 'service_execution_lines'
    if path.suffix == '.md':
        return 'documentation_only'
    return 'protected_bytes'


def structure(kind, data, clang):
    if kind in ('protected_bytes', 'documentation_only'):
        return data
    if kind == 'cpp_raw_tokens':
        return cpp_structure(data, clang)
    if kind == 'python_ast':
        return ast.dump(ast.parse(data), include_attributes=False)
    source = text(data)
    if kind == 'shell_execution_and_heredoc':
        parsed = subprocess.run(['bash', '-n'], input=data, capture_output=True, timeout=10)
        if parsed.returncode:
            raise CheckError('bash -n failed: ' + parsed.stderr.decode(errors='replace')[:1000])
        return shell_structure(source)
    if kind == 'yaml_typed_values':
        import yaml

        return type_value(yaml.safe_load(source))
    if kind == 'ros_message_declarations':
        return message_structure(source)
    if kind == 'cmake_execution_tokens':
        return cmake_structure(source)
    if kind == 'package_xml_semantics':
        return xml_structure(source)
    if kind == 'service_execution_lines':
        return [
            line
            for line in source.splitlines()
            if line.strip() and not line.lstrip().startswith(('#', ';'))
        ]
    raise CheckError('unsupported comparison kind ' + kind)


def inventory_rows(path):
    if path is None:
        return None
    rows = json.loads(path.read_text())
    if not isinstance(rows, list):
        raise CheckError('inventory must be a list of file records')
    result = {}
    for row in rows:
        name = row['path']
        if (
            not isinstance(name, str)
            or PurePosixPath(name).is_absolute()
            or '..' in PurePosixPath(name).parts
            or name in result
        ):
            raise CheckError('invalid or duplicate inventory path')
        result[name] = row
    return result


def compare(root, baseline, *, clang, inventory=None):
    """每个文件独立捕获失败，完整报告所有差异；新增文件单列，不能声称已有等价证据。"""
    entries, errors = [], []
    names = sorted(inventory if inventory is not None else baseline)
    if not names:
        raise CheckError('no baseline files selected; refusing an empty equivalence result')
    for name in names:
        row = dict(
            path=name,
            kind=comparison_kind(name, bool(inventory and inventory[name].get('historical'))),
        )
        try:
            original = baseline[name]
            if inventory and inventory[name].get('sha256'):
                if hashlib.sha256(original.data).hexdigest() != inventory[name]['sha256']:
                    raise CheckError('baseline SHA256 disagrees with inventory')
            path = root / name
            if not path.exists() and not path.is_symlink():
                raise CheckError('baseline file is missing from current tree')
            symlink = path.is_symlink()
            current = os.fsencode(os.readlink(path)) if symlink else path.read_bytes()
            if symlink != original.symlink:
                raise CheckError('symlink/file kind changed')
            if not symlink and original.executable is not None:
                if bool(path.stat().st_mode & stat.S_IXUSR) != original.executable:
                    raise CheckError('executable file mode changed')
            row.update(
                baseline_sha256=hashlib.sha256(original.data).hexdigest(),
                current_sha256=hashlib.sha256(current).hexdigest(),
                bytes_equal=current == original.data,
            )
            if symlink:
                equal = current == original.data
            elif row['kind'] == 'documentation_only':
                row['status'] = 'documentation_unchecked'
                entries.append(row)
                continue
            else:
                before = structure(row['kind'], original.data, clang)
                after = structure(row['kind'], current, clang)
                equal = before == after
            if not equal:
                raise CheckError('protected structure differs')
            row['status'] = 'pass'
        except Exception as error:
            row.update(status='fail', error=type(error).__name__ + ': ' + str(error))
            errors.append(dict(path=name, error=row['error']))
        entries.append(row)
    # 若 root 是 Git 工作区，包含尚未提交的新增源码；无需递归扫描 build/.git 或忽略目录。
    try:
        current_names = [
            os.fsdecode(x)
            for x in git(
                root, 'ls-files', '--cached', '--others', '--exclude-standard', '-z'
            ).split(b'\0')
            if x
        ]
        added = sorted(set(current_names) - set(baseline))
    except CheckError:
        added = []
    return dict(
        passed=not errors,
        checked_files=len(entries),
        counts=dict(Counter(row['kind'] for row in entries)),
        errors=errors,
        added_files_not_equivalence_checked=added,
        files=entries,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    baseline_group = parser.add_mutually_exclusive_group(required=True)
    baseline_group.add_argument('--baseline-git')
    baseline_group.add_argument('--baseline-dir', type=Path)
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--clang', default=shutil.which('clang++') or shutil.which('clang'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        if not args.clang:
            raise CheckError('Clang is required for reliable C++ raw-token checking')
        root = args.root.resolve()
        baseline, provenance = (
            git_baseline(root, args.baseline_git)
            if args.baseline_git
            else directory_baseline(args.baseline_dir)
        )
        report = compare(root, baseline, clang=args.clang, inventory=inventory_rows(args.inventory))
        report.update(
            schema_version=1,
            baseline=provenance,
            root=str(root),
            qualification='SOURCE_STRUCTURE_ONLY_NOT_RUNTIME_OR_FLIGHT_PROOF',
            limitations=[
                'Only baseline files receive equivalence checks; added files are listed separately.',
                'Markdown outside protected evidence directories is documentation, not behavior checked.',
                'C++ token equality does not prove compiler-generated source-location/assertion text unchanged.',
            ],
        )
    except Exception as error:
        report = dict(
            schema_version=1,
            passed=False,
            errors=[dict(error=type(error).__name__ + ': ' + str(error))],
        )
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered)
        except OSError as error:
            report['passed'] = False
            report['errors'].append(dict(error='report write failed: ' + str(error)))
            rendered = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    sys.stdout.write(rendered)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
