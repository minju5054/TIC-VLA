"""Preparation/identity guard tests only; no simulator or model execution."""
import importlib.util
import builtins
import json
from pathlib import Path
import symtable
import tempfile
import unittest
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[1]/'tools/h05_api_smoke/runner.py'
spec = importlib.util.spec_from_file_location('h05_api_smoke_test_module', PATH)
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)


class PreparationGuards(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.binding = patch.object(smoke, 'ROOT', self.root)
        self.binding.start()
        self.inventory = patch.object(smoke, 'source_files', side_effect=lambda: [smoke.ROOT/'research/example.py'])
        self.inventory.start()
        self.session = self.root/'outputs/new-setup'
        self.session.mkdir(parents=True)
        self.code = self.root/'research/example.py'
        self.code.parent.mkdir()
        self.code.write_text('value = 1\n')
        smoke.write(self.session/'config.json', {'value': 1})
        self.receipt = {'schema': 1, 'scope': smoke.SCOPE,
            'source_sha256': {'research/example.py': smoke.digest(self.code)},
            'reference_sha256': {}, 'external_sha256': {},
            'config_sha256': smoke.digest(self.session/'config.json'),
            'launch': {'headless': True}, 'run_id': 'new', 'run_relative': 'outputs/new'}

    def tearDown(self):
        self.inventory.stop()
        self.binding.stop()
        self.tmp.cleanup()

    def save(self):
        smoke.write(self.session/'receipt.json', self.receipt)

    def test_changed_source_rejected_before_configuration_or_runtime(self):
        self.save()
        self.code.write_text('value = 2\n')
        with patch.object(smoke, 'current_config') as current:
            with self.assertRaisesRegex(ValueError, 'Prepared file changed'):
                smoke.verify(self.session)
            current.assert_not_called()

    def test_tampered_config_rejected(self):
        self.save()
        (self.session/'config.json').write_text('{"value": 2}')
        with self.assertRaisesRegex(ValueError, 'configuration changed'):
            smoke.verify(self.session)

    def test_new_source_file_rejected(self):
        self.save()
        with patch.object(smoke, 'source_files', return_value=[self.code, self.root/'research/new.py']):
            with self.assertRaisesRegex(ValueError, 'source inventory changed'):
                smoke.verify(self.session)

    def test_relative_manifest_survives_repository_move(self):
        self.save()
        relocated = self.root/'relocated'
        self.code.parent.rename(relocated)
        # Construct a moved copy using the same bytes and relative locations.
        (relocated/'research').mkdir()
        (relocated/'example.py').rename(relocated/'research/example.py')
        (relocated/'outputs').mkdir()
        moved_session = relocated/'outputs/new-setup'
        self.session.rename(moved_session)
        with patch.object(smoke, 'ROOT', relocated), patch.object(smoke, 'current_config',
                return_value=({'value': 1}, {'headless': True}, {})):
            _, _, destination = smoke.verify(moved_session)
            self.assertEqual(destination, relocated/'outputs/new')

    def test_existing_run_rejected_without_overwrite(self):
        run = self.root/'outputs/new'
        run.mkdir()
        marker = run/'raw.txt'
        marker.write_text('keep')
        with self.assertRaises(FileExistsError):
            smoke.prepare(self.root/'outputs/other-setup', 'new')
        self.assertEqual(marker.read_text(), 'keep')

    def test_existing_session_rejected(self):
        with self.assertRaises(FileExistsError):
            smoke.prepare(self.session, 'unused')

    def test_path_escape_and_absolute_paths_rejected(self):
        for path in ('../outside', '/tmp/outside'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                smoke.local_path(path)

    def test_symlink_escape_rejected(self):
        (self.root/'escape').symlink_to('/tmp')
        with self.assertRaises(ValueError):
            smoke.local_path('escape/a')

    def test_failed_run_does_not_proceed_to_analysis(self):
        destination = self.root/'outputs/new'
        with patch.object(smoke, 'verify', return_value=(self.receipt, {}, destination)):
            with self.assertRaisesRegex(ValueError, 'No completed runtime PASS'):
                smoke.analyze(self.session)
        self.assertFalse(destination.exists())

    def test_semantic_changes_are_not_normalized_away(self):
        before = {'runner_sha256': 'old', 'episode': {'goal': [1, 2], 'start_yaw': 90}}
        new_hash = {'runner_sha256': 'new', 'episode': {'goal': [1, 2], 'start_yaw': 90}}
        new_goal = {'runner_sha256': 'new', 'episode': {'goal': [2, 2], 'start_yaw': 90}}
        self.assertEqual(smoke.clean_identity(before), smoke.clean_identity(new_hash))
        self.assertNotEqual(smoke.clean_identity(before), smoke.clean_identity(new_goal))

    def test_runtime_functions_have_no_unbound_global_names(self):
        table = symtable.symtable(PATH.read_text(), str(PATH), 'exec')
        available = set(vars(smoke)) | set(vars(builtins))
        missing = []
        def check(scope):
            if scope.get_type() == 'function':
                missing.extend((scope.get_name(), symbol.get_name())
                               for symbol in scope.get_symbols()
                               if symbol.is_global() and symbol.is_referenced()
                               and symbol.get_name() not in available)
            for child in scope.get_children():
                check(child)
        check(table)
        self.assertEqual(missing, [], 'Runtime-only global names must resolve before simulator startup')


if __name__ == '__main__':
    unittest.main()
