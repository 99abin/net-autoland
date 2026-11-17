"""Contract evidence must match this exact managed session and code."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import simulation_contract as contract


class ContractEvidenceTests(unittest.TestCase):
    def test_missing_failed_changed_and_other_session_evidence_is_rejected(self):
        current = dict(session_pid=12, sitl_pid=13, files={"binary": "new"})
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report.json"
            with patch.object(contract, "REPORT", report), patch.object(
                    contract, "identity", return_value=current):
                with self.assertRaises(RuntimeError):
                    contract.require_contract({})
                for evidence in (
                    dict(passed=False, identity=current),
                    dict(passed=True, identity={**current, "session_pid": 11}),
                    dict(passed=True, identity={**current, "files": {"binary": "old"}}),
                ):
                    report.write_text(json.dumps(evidence))
                    with self.assertRaises(RuntimeError):
                        contract.require_contract({})
                report.write_text(json.dumps(dict(passed=True, identity=current)))
                self.assertTrue(contract.require_contract({})["passed"])
