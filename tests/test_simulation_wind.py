import math
import unittest
from simulation_wind import state_with_wind

class WindRelayTests(unittest.TestCase):
    def test_headwind_and_tailwind_preserve_sensor_fields_and_timestamp(self):
        state=dict(timestamp=42.,velocity=[-11.,0.,0.],quaternion=[0.,0.,0.,1.],imu={'gyro':[1,2,3]})
        result=state_with_wind(state,[0.,4.,0.])
        self.assertEqual(result['airspeed'],15.)
        self.assertEqual(result['velocity_wind'],[4.,0.,0.])
        self.assertEqual(result['timestamp'],42.)
        self.assertEqual(result['imu'],state['imu'])
        self.assertNotIn('airspeed',state)
        state.update(velocity=[19.,0.,0.],quaternion=[1.,0.,0.,0.])
        self.assertEqual(state_with_wind(state,[0.,4.,0.])['airspeed'],15.)
    def test_east_heading_and_invalid_orientation(self):
        state=dict(velocity=[0.,11.,0.],quaternion=[math.sqrt(.5),0.,0.,math.sqrt(.5)])
        self.assertAlmostEqual(state_with_wind(state,[-4.,0.,0.])['airspeed'],15.)
        state['quaternion']=[0.,0.,0.,0.]
        with self.assertRaises(ValueError):state_with_wind(state,[0.,4.,0.])
