"""Regression checks for generated simulation assets, without launching ROS/Gazebo."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
spec = importlib.util.spec_from_file_location("simulation_setup", ROOT / "tools/simulation.py")
simulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulation)
sys.path.insert(0, str(ROOT / "src/autoland"))
from autoland.mission_builder import Item, LandingConfig, extend_mission, landing_pair
from autoland.geometry import leg_position


class SimulationSetupTests(unittest.TestCase):
    def test_startup_refuses_stale_mavros_publishers_or_nodes(self):
        for count, names, blocked in ((0, [], False), (1, [], True),
                                      (0, [("mavros", "/")], True)):
            with self.subTest(count=count, names=names):
                ros = MagicMock()
                node = ros.create_node.return_value
                node.count_publishers.return_value = count
                node.get_node_names_and_namespaces.return_value = names
                with patch.dict(sys.modules, {"rclpy": ros}), patch.object(
                    simulation.time, "monotonic", side_effect=[0, 3]
                ):
                    if blocked:
                        with self.assertRaisesRegex(RuntimeError, "MAVROS already exists"):
                            simulation.ensure_no_existing_mavros({"ros_domain_id": 37})
                    else:
                        simulation.ensure_no_existing_mavros({"ros_domain_id": 37})
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()

    def test_generated_world_preserves_spawn_and_original_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            scene = root / "scene.sdf"
            mesh = root / "triangle.obj"
            mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
            scene.write_text('''<sdf version="1.9"><world name="original">
              <physics><real_time_factor>-1</real_time_factor></physics>
              <include><uri>model://zephyr_with_ardupilot</uri>
                <pose degrees="true">0 0 0.422 -90 0 180</pose></include>
              <spherical_coordinates><latitude_deg>-35.363262</latitude_deg>
                <longitude_deg>149.165237</longitude_deg><elevation>584</elevation>
                <heading_deg>0</heading_deg></spherical_coordinates>
              <model name="among_us_landing_panel"><pose>0 -50 0 0 0 0</pose>
                <link><visual><geometry><mesh><uri>triangle.obj</uri></mesh>
                </geometry></visual></link></model></world></sdf>''')
            aircraft = root / "aircraft"
            aircraft.mkdir()
            (aircraft / "merlinv2_world.sdf").write_text('''<sdf><world>
              <include><uri>model://merlinv2</uri><pose>0 0 0.095 0 0 0</pose>
              </include></world></sdf>''')
            original_world, original_mesh = scene.read_bytes(), mesh.read_bytes()
            output = root / "generated"
            with patch.object(simulation, "STATE", output):
                geometry = simulation.prepare(dict(aircraft_directory=str(aircraft),
                                                   source_world=str(scene)))
            world = ET.parse(output / "merlin_landing.sdf").find("world")
            self.assertEqual(world.findtext("include/uri"), "model://merlinv2")
            self.assertEqual(world.findtext("include/pose"), "0 0 0.095 0 0 0")
            self.assertEqual(world.find("include/pose").attrib, {})
            self.assertEqual(world.findtext("physics/real_time_factor"), "1")
            self.assertTrue(any(p.get("name") == "gz::sim::systems::Sensors"
                                for p in world.findall("plugin")))
            self.assertEqual(geometry["net_lat"], -35.3637111)
            self.assertEqual(geometry["net_lon"], 149.165237)
            self.assertEqual(scene.read_bytes(), original_world)
            self.assertEqual(mesh.read_bytes(), original_mesh)
            self.assertIn("vn 0.0 0.0 1.0", (output / "meshes/triangle.obj").read_text())
            params = yaml.safe_load((output / "perception.yaml").read_text())["net_pos"]["ros__parameters"]
            self.assertEqual(params["camera_frame"], "merlinv2/body/nose_camera")
            self.assertEqual(params["camera_translation_body_m"], [0.451, 0., 0.008])
            self.assertFalse(geometry["launch_assist"])
            self.assertFalse(any(p.get("name") == "autoland::LaunchAssist"
                                 for p in world.findall("plugin")))
            with patch.object(simulation, "STATE", output):
                geometry = simulation.prepare(dict(aircraft_directory=str(aircraft),
                                                   source_world=str(scene)), launch_assist=True)
            assisted = ET.parse(output / "merlin_landing.sdf").find("world")
            self.assertTrue(geometry["launch_assist"])
            self.assertEqual(assisted.findtext("include/pose"), "0 0 0.095 0 0 0")
            plugin = assisted.find("plugin[@name='autoland::LaunchAssist']")
            self.assertEqual(plugin.findtext("duration_s"), "0.75")
            self.assertEqual(plugin.findtext("forward_force_n"), "120.0")

    def test_configured_southbound_500m_landing_extends_existing_mission(self):
        params = yaml.safe_load((ROOT / "src/start/config/mission.yaml").read_text())["mission_manager"]["ros__parameters"]
        config = LandingConfig(**{k: params[k] for k in LandingConfig.__dataclass_fields__})
        approach, land = landing_pair(config)
        self.assertEqual(config.approach_heading_deg, 180.)
        self.assertEqual(config.approach_distance_m, 500.)
        self.assertEqual(approach.z_alt, 50.)
        self.assertGreater(approach.x_lat, land.x_lat)
        _, cross_track, _, distance = leg_position(
            approach.x_lat, approach.y_long, land.x_lat, land.y_long,
            land.x_lat, land.y_long)
        self.assertAlmostEqual(distance, 500., places=5)
        self.assertEqual(cross_track, 0.)
        original = [Item(x_lat=-35.36, y_long=149.16) for _ in range(16)]
        extended, already = extend_mission(original, config)
        self.assertFalse(already)
        self.assertEqual(len(original), 16)
        self.assertEqual(len(extended), 18)
        self.assertEqual(extended[:16], original)
        self.assertEqual([item.command for item in extended[-2:]], [16, 21])
        again, already = extend_mission(extended, config)
        self.assertTrue(already)
        self.assertEqual(again, extended)
        self.assertFalse(params["upload_enabled"])


if __name__ == "__main__":
    unittest.main()
