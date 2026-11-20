import struct
import unittest
from simulation_autotune import rc_override_packet
from autoland.mission_partial import crc

class AutotunePacketTests(unittest.TestCase):
    def test_rc_command_uses_configured_gcs_identity_and_correct_channel_order(self):
        channels=[1900,1500,1700,1500,65535,65535,65535,65535]
        m=rc_override_packet(channels,255,17)
        raw=b''.join(v.to_bytes(8,'little') for v in m['payload64'])[:m['len']]
        self.assertEqual(struct.unpack('<8HBB',raw),(*channels,1,1))
        self.assertEqual(m['sysid'],255)
        header=bytes([18,0,0,17,255,193,70,0,0])
        self.assertEqual(m['checksum'],crc(header+raw+bytes([124])))

    def test_release_and_identity_pwm_validation(self):
        m=rc_override_packet([0]*8,255,0)
        self.assertEqual(m['len'],18)
        for channels,sysid in (([1500]*8,0),([1500]*7,255),([3000]*8,255)):
            with self.assertRaises(ValueError):rc_override_packet(channels,sysid,0)
