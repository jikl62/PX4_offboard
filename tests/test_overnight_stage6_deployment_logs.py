"""部署日志的文件归属和错误传播；不执行部署命令、子进程、ROS 或 systemd。"""

from contextlib import redirect_stdout
import errno
import io
from pathlib import Path
import tempfile
import traceback
import unittest
from unittest.mock import patch

import navigation_deploy as deploy


class TrackedOutput:
    """模拟磁盘满/关闭失败，明确记录是否尝试释放本次打开的文件。"""

    def __init__(self, write_error=None, close_error=None):
        self.write_error = write_error
        self.close_error = close_error
        self.lines = []
        self.close_calls = 0

    def write(self, text):
        if self.write_error is not None:
            raise self.write_error
        self.lines.append(text)

    def close(self):
        self.close_calls += 1
        if self.close_error is not None:
            raise self.close_error


class Summary:
    """只替代摘要文件入口，正文记录仍走真实 DeploymentLog.record。"""

    def __init__(self, error=None):
        self.error = error
        self.content = io.StringIO()

    def open(self, mode):
        if self.error is not None:
            raise self.error
        return self.content


def log_fixture(output, summary):
    # 跳过构造仅为精准注入finish阶段的文件故障；另有真实构造/重复运行测试。
    log = object.__new__(deploy.DeploymentLog)
    log.path = Path('/fixture/owned.log')
    log.stage = 'fixture-complete'
    log.output = output
    log.summary = summary
    return log


class DeploymentLogOwnershipTests(unittest.TestCase):
    def test_constructor_begin_write_failure_closes_owned_output(self):
        error = OSError(errno.ENOSPC, 'fixture BEGIN disk full')
        output = TrackedOutput(write_error=error)
        with (
            tempfile.TemporaryDirectory() as folder,
            patch.object(Path, 'open', return_value=output),
        ):
            with self.assertRaises(OSError) as caught:
                deploy.DeploymentLog(Path(folder) / 'entry', Path(folder), 'offline')
        self.assertIs(caught.exception, error)
        self.assertEqual(output.close_calls, 1)

    def test_constructor_double_failure_retains_primary_cause_and_cleanup(self):
        cause = ValueError('fixture original BEGIN cause')
        error = OSError(errno.ENOSPC, 'fixture BEGIN error')
        error.__cause__ = cause
        cleanup = OSError(errno.EIO, 'fixture constructor close error')
        output = TrackedOutput(write_error=error, close_error=cleanup)
        with (
            tempfile.TemporaryDirectory() as folder,
            patch.object(Path, 'open', return_value=output),
        ):
            with self.assertRaises(OSError) as caught:
                deploy.DeploymentLog(Path(folder) / 'entry', Path(folder), 'offline')
        self.assertIs(caught.exception, error)
        self.assertIs(error.__cause__, cause)
        self.assertEqual(output.close_calls, 1)
        self.assertIn(str(cleanup), ''.join(traceback.format_exception(error)))

    def test_finish_record_failure_closes_output_without_success_print(self):
        error = OSError(errno.ENOSPC, 'fixture final record disk full')
        output = TrackedOutput(write_error=error)
        log = log_fixture(output, Summary())
        printed = io.StringIO()
        with redirect_stdout(printed), self.assertRaises(OSError) as caught:
            log.finish('FC01_DEPLOY_COMPLETE')
        self.assertIs(caught.exception, error)
        self.assertEqual(output.close_calls, 1)
        self.assertEqual(printed.getvalue(), '')

    def test_finish_summary_open_failure_still_closes_output(self):
        error = OSError(errno.EACCES, 'fixture summary unavailable')
        output = TrackedOutput()
        log = log_fixture(output, Summary(error))
        with self.assertRaises(OSError) as caught:
            log.finish('FC01_DEPLOY_FAILED', RuntimeError('fixture command error'))
        self.assertIs(caught.exception, error)
        self.assertEqual(output.close_calls, 1)
        self.assertIn('error=fixture command error', output.lines[0])

    def test_finish_double_failure_retains_primary_and_existing_context(self):
        context = ValueError('fixture earlier context')
        error = OSError(errno.ENOSPC, 'fixture finish write error')
        error.__context__ = context
        cleanup = OSError(errno.EIO, 'fixture finish close error')
        output = TrackedOutput(write_error=error, close_error=cleanup)
        log = log_fixture(output, Summary())
        with self.assertRaises(OSError) as caught:
            log.finish('FC01_DEPLOY_COMPLETE')
        self.assertIs(caught.exception, error)
        self.assertIs(error.__context__, context)
        rendered = ''.join(traceback.format_exception(error))
        self.assertIn(str(context), rendered)
        self.assertIn(str(cleanup), rendered)
        self.assertEqual(output.close_calls, 1)

    def test_stop_exception_during_record_still_closes_output(self):
        for error in (KeyboardInterrupt('fixture interrupt'), SystemExit(17)):
            with self.subTest(type=type(error).__name__):
                output = TrackedOutput(write_error=error)
                with self.assertRaises(type(error)) as caught:
                    log_fixture(output, Summary()).finish('FC01_DEPLOY_COMPLETE')
                self.assertIs(caught.exception, error)
                self.assertEqual(output.close_calls, 1)

    def test_close_only_error_is_not_ignored(self):
        error = OSError(errno.EIO, 'fixture close alone')
        output = TrackedOutput(close_error=error)
        with self.assertRaises(OSError) as caught:
            log_fixture(output, Summary()).finish('FC01_DEPLOY_COMPLETE')
        self.assertIs(caught.exception, error)
        self.assertEqual(output.close_calls, 1)

    def test_repeated_real_create_finish_releases_each_owned_log(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            home = Path(folder)
            previous = home / 'FC01_navigation/deploy-logs/previous.log'
            previous.parent.mkdir(parents=True)
            previous.write_text('unrelated evidence retained')
            for _ in range(10):
                log = deploy.DeploymentLog(home / 'entry', home, 'offline')
                log.finish('FC01_DEPLOY_COMPLETE')
                self.assertTrue(log.output.closed)
                self.assertIn('BEGIN offline', log.path.read_text())
                self.assertIn('FC01_DEPLOY_COMPLETE', log.path.read_text())
            self.assertEqual(previous.read_text(), 'unrelated evidence retained')
            self.assertEqual(
                (home / 'FC01_navigation/install.log').read_text().count('COMPLETE'), 10
            )


if __name__ == '__main__':
    unittest.main()
