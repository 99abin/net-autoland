import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from capture_diagnostics import log_inventory, preserve_native_logs

class NativeLogCaptureTests(unittest.TestCase):
    def test_archives_changed_logs_with_hash_without_touching_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            logs=root/'logs';logs.mkdir()
            original=logs/'1.BIN';original.write_bytes(b'old log')
            with patch('capture_diagnostics.LOGS',logs):
                before=log_inventory()
                new=logs/'2.BIN';new.write_bytes(b'new flight')
                run=root/'run';run.mkdir()
                records=preserve_native_logs(run,before)
                self.assertEqual(len(records),1)
                self.assertEqual((run/records[0]['file']).read_bytes(),b'new flight')
                self.assertEqual(new.read_bytes(),b'new flight')
                self.assertEqual(len(records[0]['sha256']),64)
                self.assertEqual(len(json.loads((run/'diagnostics/native_manifest.json').read_text())),1)
