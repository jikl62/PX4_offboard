#!/usr/bin/env python3
"""显式检查隔离 Humble 安装中的五种消息；只做类型加载和本机 CDR 往返。

输入：--prefix 指向本轮 install/jikl_fc01_system，--report 指向开发 cache 的 JSON。
输出：逐消息的身份、字段/常量/默认值及 CDR 证据；任一检查失败均返回非零。
不调用 rclpy.init，不创建 Context、Node、Publisher，也不访问飞控或启动服务。
普通 import 只定义函数，不导入 ROS；本入口不由 unittest discover 自动执行。
"""

import argparse
import ast
import hashlib
import importlib
import json
from pathlib import Path
import re
import sys
import traceback


PACKAGE = 'jikl_fc01_system'
MESSAGE_NAMES = (
    'VisionState',
    'NavigationCommand',
    'VelocityCommand',
    'VisionHealth',
    'AttitudeThrustPreview',
)
SOURCE_PACKAGE = Path(__file__).resolve().parents[1] / 'native/src' / PACKAGE
SUPPORTED_TYPE = re.compile(
    r'(?:uint(?:8|16|32|64)|bool|float64(?:\[(?:3|4|36)\])?|builtin_interfaces/Time|string)\Z'
)


def require(condition, message):
    # 不依赖可被 python -O 删除的 assert；失败原因进入报告和 stderr。
    if not condition:
        raise RuntimeError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def literal_value(type_name, text):
    # 只解析当前五份 MSG 实际使用的简单声明；不充当完整 ROS MSG 语法解析器。
    if text is not None:
        if type_name == 'bool':
            require(text in ('true', 'false', 'True', 'False'), 'unsupported bool literal: ' + text)
            return text.lower() == 'true'
        if type_name.startswith('uint'):
            return int(text, 0)
        if type_name == 'float64':
            return float(text)
        if type_name == 'string':
            value = ast.literal_eval(text)
            require(type(value) is str, 'string literal required')
            return value
        raise ValueError('explicit default unsupported for ' + type_name)
    if type_name == 'builtin_interfaces/Time':
        return {'sec': 0, 'nanosec': 0}
    if '[' in type_name:
        return [0.0] * int(type_name.split('[')[1][:-1])
    return {'bool': False, 'string': '', 'float64': 0.0}.get(type_name, 0)


def parse_declarations(data):
    """提取当前简单 MSG 的声明；这能发现生成物漂移，不能证明协议设计正确。"""
    fields, constants = [], {}
    names = set()
    for line_number, raw in enumerate(data.decode('utf-8').splitlines(), 1):
        # 当前声明的字符串字面量不含 #；遇到超出支持范围的语法必须失败，不能静默跳过。
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        match = re.fullmatch(r'(\S+)\s+([A-Za-z][A-Za-z0-9_]*)(.*)', line)
        require(match is not None, f'unsupported MSG declaration at line {line_number}: {line}')
        type_name, name, tail = match.groups()
        require(SUPPORTED_TYPE.fullmatch(type_name) is not None, 'unsupported type: ' + type_name)
        require(name not in names, 'duplicate declaration: ' + name)
        names.add(name)
        tail = tail.strip()
        if tail.startswith('='):
            require(name.isupper(), 'constant must use its generated uppercase name: ' + name)
            constants[name] = literal_value(type_name, tail[1:].strip())
        else:
            fields.append(
                {
                    'name': name,
                    'type': type_name,
                    'default': literal_value(type_name, tail or None),
                }
            )
    require(bool(fields), 'MSG has no fields')
    return fields, constants


def generated_type_name(type_name):
    # get_fields_and_field_types 使用 IDL 名称：bool→boolean、float64→double。
    return type_name.replace('float64', 'double').replace('bool', 'boolean')


def inside_prefix(path, prefix):
    resolved = Path(path).resolve(strict=True)
    require(resolved.is_relative_to(prefix), f'identity outside isolated prefix: {resolved}')
    return resolved


def module_identity(module, prefix):
    # 同名包可以被旧 overlay 遮蔽；记录并校验实际加载文件，不能只看请求的类型名称。
    path = inside_prefix(module.__file__, prefix)
    return {'path': str(path), 'sha256': digest(path.read_bytes())}


def field_value(message, field):
    value = getattr(message, field['name'])
    if field['type'] == 'builtin_interfaces/Time':
        return {'sec': value.sec, 'nanosec': value.nanosec}
    if '[' in field['type']:
        return [float(item) for item in value]
    return value


def fill_fixture(message, fields):
    # 这里只验证类型编码，不声称这些合成数值满足飞行接收/物理有效性合同。
    # uint64 超过 2**53，避免错误地经浮点中转后仍在小整数样本上蒙混通过。
    for index, field in enumerate(fields, 1):
        name, type_name = field['name'], field['type']
        if name == 'schema_version':
            value = field['default']
        elif type_name == 'builtin_interfaces/Time':
            getattr(message, name).sec = 123
            getattr(message, name).nanosec = 456789123
            continue
        elif type_name == 'bool':
            value = True
        elif type_name == 'string':
            value = 'fixture_' + name + '_相机'
        elif type_name == 'uint64':
            value = (1 << 60) + index
        elif type_name.startswith('uint'):
            value = index + 7
        elif '[' in type_name:
            length = int(type_name.split('[')[1][:-1])
            value = [(-1.0 if item % 2 else 1.0) * (index + item + 1) / 8 for item in range(length)]
        else:
            value = -index / 8.0
        setattr(message, name, value)


def cdr_padding_ranges(fields, values, wire):
    """标出本检查器支持的 CDR1 对齐填充；不把填充字节当成协议字段。

    四字节封装头之后重新计算对齐；字符串包括长度前缀和 UTF-8 末尾零字节。
    未知封装或布局长度不符直接失败，不能随意忽略不属于已确认填充的差异。
    """
    require(
        len(wire) >= 4 and wire[:2] in (b'\x00\x00', b'\x00\x01'),
        'checker supports CDR1 encapsulation only',
    )
    offset, padding = 4, []
    for field in fields:
        name, type_name = field['name'], field['type']
        if type_name in ('string', 'builtin_interfaces/Time', 'uint32'):
            alignment = 4
        elif type_name.startswith('float64') or type_name == 'uint64':
            alignment = 8
        else:
            alignment = 2 if type_name == 'uint16' else 1
        gap = (-(offset - 4)) % alignment
        if gap:
            padding.append({'before_field': name, 'offset': offset, 'bytes': gap})
            offset += gap
        if type_name == 'string':
            offset += 4 + len(values[name].encode('utf-8')) + 1
        elif type_name == 'builtin_interfaces/Time':
            offset += 8
        elif '[' in type_name:
            offset += 8 * len(values[name])
        else:
            offset += alignment
    require(offset == len(wire), 'CDR1 layout size differs from serialized bytes')
    return padding


def check_installation(prefix, report):
    """加载已安装类型并检查本机序列化；调用方先 source 正确 overlay，本函数不初始化 ROS。"""
    # ROS 依赖只在显式执行时加载；导入脚本或阅读/测试解析函数不需要 ROS 环境。
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_generator_py import import_type_support
    from rosidl_runtime_py.utilities import get_message

    indexed = Path(get_package_prefix(PACKAGE)).resolve(strict=True)
    require(indexed == prefix, f'ament selected {indexed}, expected {prefix}')
    share = inside_prefix(get_package_share_directory(PACKAGE), prefix)
    require(share == prefix / 'share' / PACKAGE, 'unexpected package share layout')
    report['ament'] = {'prefix': str(indexed), 'share': str(share)}
    report['python_package'] = module_identity(importlib.import_module(PACKAGE), prefix)
    report['python_msg_package'] = module_identity(
        importlib.import_module(PACKAGE + '.msg'), prefix
    )
    report['type_support_module'] = module_identity(import_type_support(PACKAGE), prefix)

    for name in MESSAGE_NAMES:
        row = {'name': name, 'type': PACKAGE + '/msg/' + name, 'passed': False}
        report['messages'].append(row)
        source_path = SOURCE_PACKAGE / 'msg' / (name + '.msg')
        installed_path = inside_prefix(share / 'msg' / (name + '.msg'), prefix)
        source, installed = source_path.read_bytes(), installed_path.read_bytes()
        row['source_msg_sha256'] = digest(source)
        row['installed_msg'] = {'path': str(installed_path), 'sha256': digest(installed)}
        require(source == installed, name + ': installed MSG differs from current source bytes')
        fields, constants = parse_declarations(source)
        message_class = get_message(row['type'])
        package_module = importlib.import_module(PACKAGE + '.msg')
        require(
            getattr(package_module, name) is message_class, name + ': get_message class mismatch'
        )
        row['python_module'] = module_identity(
            importlib.import_module(message_class.__module__), prefix
        )
        expected_fields = [(field['name'], generated_type_name(field['type'])) for field in fields]
        actual_fields = list(message_class.get_fields_and_field_types().items())
        require(actual_fields == expected_fields, name + ': field order/type metadata differs')
        # Humble 类命名空间还含大写 SLOT_TYPES：它是字段类型描述元组，不是 MSG 常量。
        # 只排除这个已核实的生成器元数据；其它未知大写项仍参加比较，不能漏掉额外协议常量。
        actual_constants = {
            key: getattr(message_class, key)
            for key in vars(message_class)
            if re.fullmatch(r'[A-Z][A-Z0-9_]*', key)
            and not key.endswith('__DEFAULT')
            and key != 'SLOT_TYPES'
        }
        require(actual_constants == constants, name + ': generated constants differ')
        defaults = message_class()
        expected_defaults = {field['name']: field['default'] for field in fields}
        actual_defaults = {field['name']: field_value(defaults, field) for field in fields}
        require(actual_defaults == expected_defaults, name + ': generated field defaults differ')
        require(defaults.schema_version == 1, name + ': expected schema V1 default')
        require(
            message_class.SCHEMA_VERSION__DEFAULT == 1, name + ': schema default metadata differs'
        )
        message_class.__class__.__import_type_support__()
        require(
            message_class.__class__._TYPE_SUPPORT is not None, name + ': TYPE_SUPPORT unavailable'
        )

        fixture = message_class()
        fill_fixture(fixture, fields)
        expected = {field['name']: field_value(fixture, field) for field in fields}
        changed = [key for key in expected if expected[key] != actual_defaults[key]]
        require(bool(changed), name + ': fixture accidentally contains only defaults')
        cdr = serialize_message(fixture)
        restored = deserialize_message(cdr, message_class)
        actual = {field['name']: field_value(restored, field) for field in fields}
        require(actual == expected, name + ': CDR round trip changed a field')
        reencoded = serialize_message(restored)
        padding = cdr_padding_ranges(fields, expected, cdr)
        require(len(reencoded) == len(cdr), name + ': CDR reserialization changed size')
        differences = [index for index, (a, b) in enumerate(zip(cdr, reencoded)) if a != b]
        padding_offsets = {
            index
            for item in padding
            for index in range(item['offset'], item['offset'] + item['bytes'])
        }
        # Fast CDR 不保证对齐填充内容；已受控复现仅填充随分配器旧内容变化。
        # 字段字节仍必须相同，再次解码后所有整数、浮点、字符串和数组也必须精确相同。
        require(set(differences) <= padding_offsets, name + ': nonpadding CDR bytes changed')
        restored_again = deserialize_message(reencoded, message_class)
        require(
            {field['name']: field_value(restored_again, field) for field in fields} == expected,
            name + ': reencoded CDR changed a field',
        )
        row.update(
            passed=True,
            fields=actual_fields,
            constants=constants,
            defaults=actual_defaults,
            fixture=expected,
            nondefault_fields=changed,
            cdr_bytes=len(cdr),
            cdr_sha256=digest(cdr),
            cdr_padding=padding,
            cdr_reencoded_sha256=digest(reencoded),
            cdr_reencoded_bytes_equal=not differences,
            cdr_reencoded_differing_offsets=differences,
            cdr_twice_decoded_fields_exact=True,
            type_support_loaded=True,
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--prefix', required=True, type=Path, help='本轮 isolated install/jikl_fc01_system'
    )
    parser.add_argument('--report', required=True, type=Path, help='开发 cache 中的 JSON 报告路径')
    args = parser.parse_args(argv)
    report = {
        'passed': False,
        'requested_prefix': str(args.prefix),
        'script_sha256': digest(Path(__file__).read_bytes()),
        'ros_init_called': False,
        'nodes_or_publishers_created': False,
        'scope': 'Generated identity and local CDR only; not a ROS graph, protocol design or flight test.',
        'messages': [],
    }
    try:
        require(args.prefix.is_absolute(), '--prefix must be an absolute isolated package prefix')
        prefix = args.prefix.resolve(strict=True)
        require(prefix.is_dir(), '--prefix must be an installed package directory')
        check_installation(prefix, report)
        report['passed'] = True
    except Exception as error:
        report['error'] = {
            'type': type(error).__name__,
            'message': str(error),
            'traceback': traceback.format_exc(),
        }
        print(report['error']['traceback'], file=sys.stderr, end='')
    try:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
        )
    except OSError:
        # 取证写盘失败也必须非零；此前检查错误已原样输出，不被写盘失败遮住。
        traceback.print_exc()
        return 1
    print(
        json.dumps(
            {
                'passed': report['passed'],
                'messages': len(report['messages']),
                'report': str(args.report),
            },
            ensure_ascii=False,
        )
    )
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
