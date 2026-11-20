"""Diagnostic correlation and noninterference; no aircraft transport involved."""
import inspect
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import test_nodes as helpers
from autoland.timing_diagnostics import TimingRecorder, command_key
from analyse_command_timing import analyse


class TimingTests(unittest.TestCase):
    def test_humble_message_only_subscription_callback_is_supported(self):
        case = helpers.NodeTests()
        case.setUp()
        try:
            node = case.interface(True)
            callback = next(cb for topic, cb in node.subscriptions
                            if topic == '/autoland/control_setpoint')
            message = case.target(node)
            callback(message)
            self.assertIs(node.command, message)
        finally:
            case.tearDown()

    def test_disabled_keeps_original_callback_and_returns_same_result(self):
        node = helpers.ros.Node('test')
        trace = TimingRecorder(node, directory='')
        def callback(message, info):
            return message + info
        self.assertIs(trace.wrap('receive', callback), callback)
        self.assertEqual(callback(2, 3), 5)
        trace.close()

    def test_enabled_preserves_messageinfo_signature_and_exception_and_flushes(self):
        node = helpers.ros.Node('test')
        with tempfile.TemporaryDirectory() as directory:
            trace = TimingRecorder(node, directory)
            def callback(message, info):
                raise ValueError(message + info)
            wrapped = trace.wrap('receive', callback)
            self.assertEqual(inspect.signature(wrapped), inspect.signature(callback))
            with self.assertRaisesRegex(ValueError, 'ab'):
                wrapped('a', 'b')
            trace.close()
            rows = [json.loads(line) for line in trace.path.read_text().splitlines()]
            self.assertTrue(any(r['event'] == 'session' for r in rows))
            exit_record = next(r for r in rows if r['event'] == 'callback_exit')
            self.assertGreaterEqual(exit_record['duration_ns'], exit_record['callback_cpu_ns'])
            self.assertEqual(exit_record['callback'], 'receive')
            self.assertEqual(trace.dropped, 0)
            self.assertFalse(trace.thread.is_alive())

    def test_wire_fingerprint_matches_copy_and_distinguishes_changed_command(self):
        import copy
        command = helpers.ros.ControlSetpoint(plan_id='plan', valid=True, thrust=.4)
        received = copy.deepcopy(command)
        self.assertEqual(command_key(command), command_key(received))
        received.thrust = .41
        self.assertNotEqual(command_key(command), command_key(received))

    def test_join_separates_delivery_delay_from_wall_receipt_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'timing').mkdir()
            session = dict(event='session',host='same',boot_id='boot',monotonic_ns=0)
            publisher = [session, dict(event='command_publish_begin',command_key='x',monotonic_ns=1_000_000_000),
                         dict(event='command_publish_end',command_key='x',monotonic_ns=1_001_000_000)]
            receiver = [session, dict(event='command_received',command_key='x',monotonic_ns=1_005_000_000,
                        system_ns=11_005_000_000,dds_source_system_ns=11_000_000_000,
                        dds_received_system_ns=11_002_000_000),
                        dict(event='ownership_preview_rejected',command_key='x',monotonic_ns=1_305_000_000,
                             receipt_age_s=.3,stamp_age_s=.054)]
            for name, rows in (('publisher',publisher),('receiver',receiver)):
                (root/'timing'/f'{name}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            result = analyse(root)
            self.assertTrue(result['shared_monotonic_clock'])
            decision = result['decisions'][0]
            self.assertAlmostEqual(decision['publish_begin_to_receive_s'],.005)
            self.assertAlmostEqual(decision['rmw_received_to_callback_s'],.003)
            self.assertEqual(decision['decision']['receipt_age_s'],.3)


if __name__ == '__main__':
    unittest.main()
