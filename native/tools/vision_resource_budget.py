"""视觉身份的应用层资源合同：256 UTF-8 字节，不修改 ROS 消息类型。

先按字符数快速拒绝，再编码最多 256 个字符，因此检查不会为超大输入分配等长副本。
这只限制工程内的缓存、复制和日志；ROS/DDS 反序列化已经发生的分配不在此保证内。
"""

IDENTITY_MAX_BYTES = 256
RESOURCE_ERROR = 'VISION_IDENTITY_RESOURCE_LIMIT'


def identity_within_budget(value):
    """检查类型、UTF-8 字节数和 NUL；空身份仍由调用方的原协议条件判断。"""
    if not isinstance(value, str) or len(value) > IDENTITY_MAX_BYTES or '\0' in value:
        return False
    try:
        return len(value.encode('utf-8')) <= IDENTITY_MAX_BYTES
    except UnicodeEncodeError:
        return False


def valid_identity(value, *, allow_empty=False):
    """在资源检查后沿用必填非空白规则；不截断身份，避免把两个来源混成一个。"""
    return identity_within_budget(value) and ((allow_empty and not value) or bool(value.strip()))


def bounded_text(value):
    """仅供日志取证：最多保留 256 UTF-8 字节，不能将此返回值用于来源资格判断。"""
    if not isinstance(value, str):
        return None
    return (
        value[:IDENTITY_MAX_BYTES]
        .encode('utf-8', errors='replace')[:IDENTITY_MAX_BYTES]
        .decode('utf-8', errors='ignore')
    )


def _identity_fields_error(message, names):
    """只检查实际存在的字段；缺失字段交给原协议验证，不把巨大字符串拼进原因。"""
    for name in names:
        value = getattr(message, name, None)
        if value is not None and not identity_within_budget(value):
            return RESOURCE_ERROR
    return ''


def odometry_identity_error(message):
    """在原始 Odometry 进入配对缓存前检查世界和机体坐标名；没有副作用。"""
    return _identity_fields_error(getattr(message, 'header', None), ('frame_id',)) or (
        _identity_fields_error(message, ('child_frame_id',))
    )


def health_identity_error(message):
    """在健康消息进入缓存或诊断之前检查所有来源/坐标/标定身份。"""
    return _identity_fields_error(
        message,
        (
            'source_id',
            'frame_id',
            'body_frame_id',
            'angular_velocity_source',
            'extrinsics_id',
            'alignment_id',
        ),
    )
