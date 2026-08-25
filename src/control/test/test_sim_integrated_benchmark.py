import json
from pathlib import Path
import tempfile
import unittest

from tools.sim_integrated_benchmark import cache_matches


class CacheTests(unittest.TestCase):
    def test_cache_requires_matching_commit_and_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'result.json'
            path.write_text(json.dumps({
                'commit': 'new-commit',
                'config_sha256': 'new-config',
            }))

            self.assertTrue(cache_matches(
                path, 'new-commit', 'new-config'
            ))
            self.assertFalse(cache_matches(
                path, 'old-commit', 'new-config'
            ))
            self.assertFalse(cache_matches(
                path, 'new-commit', 'old-config'
            ))
            path.write_text('{broken')
            self.assertFalse(cache_matches(path, 'new-commit', 'new-config'))
