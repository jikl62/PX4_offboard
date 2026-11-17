# 原生 stdout 日志按字节分块到达，LogPump 必须还原完整记录并正确处理 UTF-8 边界。
# 测试注入 os.read 块与内存 handler，既不读取真实 FD，也不写生产日志。
"""Native JSON records survive arbitrary pipe chunk and UTF-8 boundaries."""
import json
import logging
import types
import unittest
from unittest.mock import patch

import standby


# 收集 Logger 实际交给 handler 的最终文本，用它验证记录边界而非内部缓存实现。
class Records(logging.Handler):
    def __init__(self):
        super().__init__()
        self.messages = []

    def emit(self, record):
        self.messages.append(record.getMessage())


class NativeLogRecordsTests(unittest.TestCase):
    # 用指定块序列模拟管道读取，并追加 EOF；max_bytes 用来缩小超长行边界。
    def pump(self, chunks, *, max_bytes=4 * 1024 * 1024):
        handler = Records()
        handler.maxBytes = max_bytes
        log = logging.Logger('native-test')
        log.addHandler(handler)
        pump = object.__new__(standby.LogPump)
        pump.stream = types.SimpleNamespace(fileno=lambda: 7, close=lambda: None)
        pump.log = log
        with patch.object(standby.os, 'read', side_effect=[*chunks, b'']):
            pump.run()
        return handler.messages

    # 16 KB 状态跨多个 4096 字节块后仍是一条 JSON，紧邻 startup 也不能被粘合。
    def test_large_state_and_adjacent_startup_remain_whole_json_records(self):
        state = dict(kind='state', reason='等待反馈', payload='x' * 16000)
        startup = dict(kind='startup', controller_build='current')
        raw = (json.dumps(state, ensure_ascii=False) + '\n' + json.dumps(startup) + '\n').encode()
        messages = self.pump([raw[i : i + 4096] for i in range(0, len(raw), 4096)])
        self.assertEqual([json.loads(message) for message in messages], [state, startup])

    # 中文多字节字符跨块、末尾没有换行时，也必须在 EOF 输出完整文字。
    def test_split_utf8_and_unterminated_final_record_are_preserved(self):
        raw = '中文尾部'.encode()
        messages = self.pump([raw[:1], raw[1:5], raw[5:]])
        self.assertEqual(messages, ['中文尾部'])

    # 无界单行只能分成有明确片段标记的有限输出，最后正常记录不可丢失。
    def test_unbounded_line_is_drained_in_marked_bounded_fragments(self):
        raw = b'x' * 10000 + b'LAST_RECORD\n'
        messages = self.pump([raw[i : i + 4096] for i in range(0, len(raw), 4096)], max_bytes=4096)
        self.assertGreater(len(messages), 4)
        self.assertTrue(all(len(message) < 2100 for message in messages))
        self.assertTrue(messages[0].startswith('NATIVE_LOG_RECORD_FRAGMENT '))
        self.assertTrue(messages[-1].endswith('LAST_RECORD'))


if __name__ == '__main__':
    unittest.main()
