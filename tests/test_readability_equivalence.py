#!/usr/bin/env python3
"""证明等价检查会放行注释/排版，并能拒绝藏在字面量、宏、配置与载荷里的行为改动。"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import readability_equivalence as check


class SourceStructureTests(unittest.TestCase):
    def test_python_keeps_docstrings_and_embedded_programs(self):
        # AST 忽略位置与注释，但 __doc__ 和字符串内嵌程序仍是运行内容。
        old = b'"module"\nvalue = "# actual data"\n'
        same = b'# comment\n"module"\nvalue=("# actual data")\n'
        changed = b'"new module"\nvalue = "# actual data"\n'
        self.assertEqual(
            check.structure('python_ast', old, None), check.structure('python_ast', same, None)
        )
        self.assertNotEqual(
            check.structure('python_ast', old, None), check.structure('python_ast', changed, None)
        )

    def test_shell_multiline_strings_are_not_comments(self):
        old = 'echo "first\n# literal payload\nlast"\n'
        same = '# outer comment\n' + old + '\n# closing comment\n'
        changed = old.replace('# literal payload', '# changed payload')
        self.assertEqual(check.shell_structure(old), check.shell_structure(same))
        self.assertNotEqual(check.shell_structure(old), check.shell_structure(changed))

    def test_shell_heredoc_payload_and_delimiter_remain_exact(self):
        old = "cat <<'PY'\n# Python payload comment\nprint('x')\nPY\n"
        same = '# shell comment\n' + old
        changed = old.replace('# Python payload comment', '# changed Python payload')
        self.assertEqual(check.shell_structure(old), check.shell_structure(same))
        self.assertNotEqual(check.shell_structure(old), check.shell_structure(changed))

    def test_multiple_and_tab_stripped_heredocs(self):
        old = "cat <<ONE <<-'TWO'\n# first payload\nONE\n\t# second payload\n\tTWO\n"
        self.assertEqual(check.shell_structure('# before\n' + old), check.shell_structure(old))
        self.assertNotEqual(
            check.shell_structure(old), check.shell_structure(old.replace('second', 'other'))
        )

    def test_comment_after_shell_continuation_is_not_ignored(self):
        old = 'echo first \\\n second\n'
        changed = 'echo first \\\n# inserted inside command\n second\n'
        self.assertNotEqual(check.shell_structure(old), check.shell_structure(changed))

    def test_shell_quote_characters_in_comment_do_not_change_state(self):
        source = "echo x # 'not a quote\n# outer explanation\necho y\n"
        self.assertEqual(check.shell_structure(source), ['echo x # \'not a quote\n', 'echo y\n'])

    def test_nested_shell_contexts_keep_literal_comment_lines(self):
        # 外层双引号中的命令替换/参数展开有独立引号上下文，不能只计算引号奇偶。
        samples = (
            'x="$(printf \'%s\\n\' "a\n# payload\nb")"\n',
            'x="${a:-"a\n# payload\nb"}"\n',
        )
        for source in samples:
            with self.subTest(source=source):
                self.assertNotEqual(
                    check.shell_structure(source),
                    check.shell_structure(source.replace('# payload', '# changed')),
                )

    def test_shell_shift_and_here_string_are_not_heredocs(self):
        for source in ('echo $((2<<1))\n', '((x=2<<1))\n', 'cat <<<"text"\n'):
            self.assertEqual(
                check.shell_structure(source), check.shell_structure('# comment\n' + source)
            )

    def test_shell_shebang_is_execution_contract(self):
        self.assertNotEqual(
            check.shell_structure('#!/bin/bash\necho x\n'),
            check.shell_structure('#!/bin/sh\necho x\n'),
        )

    def test_unterminated_heredoc_fails_closed(self):
        with self.assertRaises(check.CheckError):
            check.shell_structure("cat <<END\n# unterminated\n")

    def test_cmake_preserves_quoted_and_bracket_payloads(self):
        old = 'set(X "a # actual text")\nset(Y [=[# payload\n)]=])\n'
        same = '# explanation\nset( X "a # actual text" )\n#[==[ comment ) ]==]\nset(Y [=[# payload\n)]=])\n'
        self.assertEqual(check.cmake_structure(old), check.cmake_structure(same))
        self.assertNotEqual(
            check.cmake_structure(old), check.cmake_structure(old.replace('# payload', '# changed'))
        )

    def test_cmake_escaped_quote_is_one_argument(self):
        old = 'set(X "a \\" # actual text")\n'
        self.assertEqual(len(check.cmake_structure(old)), 5)
        with self.assertRaises(check.CheckError):
            check.cmake_structure('set(X [=[unterminated)')

    def test_yaml_types_cannot_compare_equal_by_python_coercion(self):
        import yaml

        self.assertNotEqual(
            check.type_value(yaml.safe_load('x: true')), check.type_value(yaml.safe_load('x: 1'))
        )
        self.assertNotEqual(
            check.type_value(yaml.safe_load('x: 1')), check.type_value(yaml.safe_load('x: 1.0'))
        )
        self.assertEqual(
            check.type_value(yaml.safe_load('x: 1 # explanation')),
            check.type_value(yaml.safe_load('x: 1')),
        )

    def test_message_fields_and_string_defaults_are_preserved(self):
        old = 'float32 vx # velocity\nstring LABEL="body#frd"\n'
        self.assertEqual(
            check.message_structure(old), check.message_structure('# 中文说明\n' + old)
        )
        self.assertNotEqual(
            check.message_structure(old),
            check.message_structure(old.replace('body#frd', 'body#flu')),
        )

    def test_package_xml_semantics_preserve_dependency_and_attributes(self):
        old = '<package format="3"><depend>rclcpp</depend></package>'
        same = '<package format="3">\n<!-- 依赖说明 -->\n<depend>rclcpp</depend>\n</package>'
        self.assertEqual(check.xml_structure(old), check.xml_structure(same))
        self.assertNotEqual(
            check.xml_structure(old), check.xml_structure(old.replace('rclcpp', 'rclpy'))
        )

    def test_history_kind_wins_over_source_suffix(self):
        for name in (
            'dist/release.sh',
            'docs/evidence/case.cpp',
            'native/tests/c_s10_independent/case.py',
            'native/tests/historical_r6/case.cpp.original',
            'native/config/example.yaml.bak',
        ):
            self.assertEqual(check.comparison_kind(name), 'protected_bytes')
        self.assertEqual(
            check.comparison_kind('native/config/historical/candidate.yaml'), 'yaml_typed_values'
        )


@unittest.skipUnless(
    shutil.which('clang++') or shutil.which('clang'), 'Clang raw lexer is required'
)
class CppStructureTests(unittest.TestCase):
    clang = shutil.which('clang++') or shutil.which('clang')

    def tokens(self, source):
        return check.cpp_structure(source.encode(), self.clang)

    def test_real_comments_and_whitespace_are_ignored(self):
        old = 'int f(int x){return x+1;}\n'
        same = '// 中文说明\nint f ( int x ) { /* explanation */ return x + 1; }\n'
        self.assertEqual(self.tokens(old), self.tokens(same))
        self.assertNotEqual(self.tokens(old), self.tokens(old.replace('+1', '+2')))

    def test_normal_and_raw_literals_keep_comment_symbols_and_newlines(self):
        old = 'auto s="// actual /* data */"; auto r=R"tag(line\n# // data\n)tag";\n'
        self.assertEqual(self.tokens(old), self.tokens('// comment\n' + old))
        for changed in (
            old.replace('// actual', '// modified'),
            old.replace('# // data', '# // other'),
        ):
            self.assertNotEqual(self.tokens(old), self.tokens(changed))

    def test_macro_end_cannot_swallow_following_statement(self):
        old = '#define X 1\nint y=X;\n'
        changed = '#define X 1 int y=X;\n'
        self.assertNotEqual(self.tokens(old), self.tokens(changed))
        self.assertEqual(self.tokens(old), self.tokens('#define X   1\n// comment\nint y = X;\n'))

    def test_function_macro_adjacency_is_semantic(self):
        old = '#define F(x) x\n'
        changed = '#define F (x) x\n'
        self.assertNotEqual(self.tokens(old), self.tokens(changed))

    def test_macro_continuation_can_be_reformatted_but_not_removed(self):
        old = '#define X 1\\\n+2\nint x=X;\n'
        same = '#define X 1 + 2\nint x = X;\n'
        changed = '#define X 1\n+2\nint x=X;\n'
        self.assertEqual(self.tokens(old), self.tokens(same))
        self.assertNotEqual(self.tokens(old), self.tokens(changed))

    def test_line_sensitive_builtin_is_checked(self):
        self.assertNotEqual(
            self.tokens('int x=__LINE__;\n'), self.tokens('// added line\nint x=__LINE__;\n')
        )


class BaselineAndReportTests(unittest.TestCase):
    def test_missing_or_empty_baseline_cannot_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with self.assertRaises(check.CheckError):
                check.directory_baseline(folder / 'missing')
            with self.assertRaises(check.CheckError):
                check.compare(folder, {}, clang=None)

    def test_directory_report_detects_changes_missing_and_inventory_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            root, baseline = folder / 'root', folder / 'baseline'
            root.mkdir()
            baseline.mkdir()
            (baseline / 'example.py').write_text('x=1\n')
            (root / 'example.py').write_text('# 中文说明\nx = 1\n')
            files, _ = check.directory_baseline(baseline)
            report = check.compare(root, files, clang=None)
            self.assertTrue(report['passed'])
            (root / 'example.py').write_text('x=2\n')
            self.assertFalse(check.compare(root, files, clang=None)['passed'])
            (root / 'example.py').unlink()
            self.assertFalse(check.compare(root, files, clang=None)['passed'])
            bad_inventory = {'example.py': {'sha256': '0' * 64}}
            report = check.compare(root, files, clang=None, inventory=bad_inventory)
            self.assertFalse(report['passed'])
            self.assertIn('baseline SHA256', report['errors'][0]['error'])

    def test_git_baseline_preserves_original_blob_and_executable_mode(self):
        # 仅在临时仓库提交夹具；不提交、不修改真实工程 Git 状态。
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*args):
                return subprocess.run(
                    ['git', '-C', str(root), *args], check=True, capture_output=True
                )

            git('init', '-q')
            (root / 'example.py').write_text('x=1\n')
            git('add', 'example.py')
            git(
                '-c',
                'user.name=Test',
                '-c',
                'user.email=test@example.invalid',
                'commit',
                '-qm',
                'fixture',
            )
            files, provenance = check.git_baseline(root, 'HEAD')
            self.assertEqual(files['example.py'].data, b'x=1\n')
            self.assertEqual(provenance['kind'], 'git')
            (root / 'example.py').write_text('# comment\nx = 1\n')
            self.assertTrue(check.compare(root, files, clang=None)['passed'])
            (root / 'new.py').write_text('x=3\n')
            self.assertIn(
                'new.py',
                check.compare(root, files, clang=None)['added_files_not_equivalence_checked'],
            )
            (root / 'example.py').chmod(0o755)
            report = check.compare(root, files, clang=None)
            self.assertFalse(report['passed'])
            self.assertIn('executable file mode', report['errors'][0]['error'])

    def test_cli_failure_is_json_and_nonzero(self):
        import json

        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            root, baseline = folder / 'root', folder / 'baseline'
            root.mkdir()
            baseline.mkdir()
            (root / 'example.py').write_text('x=2\n')
            (baseline / 'example.py').write_text('x=1\n')
            output = folder / 'report.json'
            command = [
                sys.executable,
                str(Path(check.__file__)),
                '--root',
                str(root),
                '--baseline-dir',
                str(baseline),
                '--output',
                str(output),
                '--clang',
                '/unused',
            ]
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 1)
            self.assertEqual(json.loads(result.stdout), json.loads(output.read_text()))
            self.assertFalse(json.loads(result.stdout)['passed'])


if __name__ == '__main__':
    unittest.main()
