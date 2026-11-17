"""仅用临时文件系统验证 IPC 路径分配；不建 socket、不启动进程或飞控。"""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import launcher


class SocketDirectoryOwnershipTests(unittest.TestCase):
    """路径长度检查失败时，仅清理由本次 mkdtemp 创建的空目录。"""

    def test_repeated_overlong_paths_remove_new_directory_and_preserve_previous(self):
        with tempfile.TemporaryDirectory(prefix='fc01-s4-ipc-') as temporary:
            root = Path(temporary) / ('r' * 80)
            launcher.private_dir(root)
            previous = root / 'previous-session'
            previous.mkdir()
            sentinel = previous / 'control.sock'
            sentinel.write_text('previous evidence, not a real socket')
            for _ in range(10):
                with self.assertRaisesRegex(launcher.ControlError, 'Unix socket 路径过长'):
                    launcher.make_socket_path(root)
                self.assertEqual(list(root.iterdir()), [previous])
                self.assertEqual(sentinel.read_text(), 'previous evidence, not a real socket')

    def test_valid_path_transfers_directory_cleanup_to_caller(self):
        with tempfile.TemporaryDirectory(prefix='fc01-s4-ipc-') as temporary:
            root = Path(temporary) / 'ipc'
            path = launcher.make_socket_path(root)
            self.assertEqual(path.name, 'control.sock')
            self.assertTrue(path.parent.is_dir())
            self.assertFalse(path.exists())
            self.assertEqual(path.parent.stat().st_mode & 0o777, 0o700)
            self.assertEqual(list(root.iterdir()), [path.parent])

    def test_cleanup_error_preserves_path_rejection_and_exposes_original_cleanup_error(self):
        with tempfile.TemporaryDirectory(prefix='fc01-s4-ipc-') as temporary:
            root = Path(temporary) / ('r' * 80)
            failure = OSError('injected rmdir failure')
            with patch.object(Path, 'rmdir', side_effect=failure) as remove:
                with self.assertRaises(launcher.ControlError) as caught:
                    launcher.make_socket_path(root)
            self.assertIn('Unix socket 路径过长', str(caught.exception))
            self.assertIs(caught.exception.__cause__, failure)
            remove.assert_called_once_with()
            # 无法删除时保留失败目录，不能递归删共享根或伪称已清理。
            self.assertEqual(len(list(root.iterdir())), 1)


if __name__ == '__main__':
    unittest.main()
