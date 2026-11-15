import ast
import sys
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ("autoland", "perception"):
    sys.path.insert(0, str(ROOT / "src" / package))


class PackagingTests(unittest.TestCase):
    def test_python_syntax_and_manifest_dependencies(self):
        for p in (ROOT / "src").rglob("*.py"):
            ast.parse(p.read_text(), filename=str(p))
        for p in (ROOT / "src").glob("*/package.xml"):
            tree = ET.parse(p)
            self.assertEqual(tree.findtext("name"), p.parent.name)
        cmake = (ROOT / "src/autoland_interfaces/CMakeLists.txt").read_text()
        for p in (ROOT / "src/autoland_interfaces/msg").glob("*.msg"):
            self.assertIn(p.name, cmake)

    def test_expected_executables_exist(self):
        expected = {
            "autoland": [
                "parameter_manager",
                "mission_manager",
                "landing_supervisor",
                "vision_controller",
                "fcu_interface",
            ],
            "perception": ["net_det", "net_pos"],
        }
        for package, names in expected.items():
            setup = (ROOT / "src" / package / "setup.py").read_text()
            for name in names:
                self.assertIn(name, setup)
                tree = ast.parse((ROOT / "src" / package / package / f"{name}.py").read_text())
                self.assertIn("main", [n.name for n in tree.body if isinstance(n, ast.FunctionDef)])

    def test_yaml_parameters_match_nodes(self):
        import ros_stubs as ros
        from autoland.parameter_manager import ParameterManager
        from autoland.mission_manager import MissionManager
        from autoland.landing_supervisor import LandingSupervisor
        from autoland.vision_controller import VisionController
        from autoland.fcu_interface import FCUInterface

        classes = {
            "parameter_manager": ParameterManager,
            "mission_manager": MissionManager,
            "landing_supervisor": LandingSupervisor,
            "vision_controller": VisionController,
            "fcu_interface": FCUInterface,
        }
        try:
            from perception.net_det import NetDetector
            from perception.net_pos import NetPosition

            classes.update(net_det=NetDetector, net_pos=NetPosition)
        except ImportError:
            pass
        documents = {}
        for p in (ROOT / "src/start/config").glob("*.yaml"):
            documents.update(yaml.safe_load(p.read_text()))
        for name, cls in classes.items():
            values = documents[name]["ros__parameters"]
            ros.Node.overrides = {name: values}
            node = cls()
            self.assertFalse(
                set(values) - set(node.parameters), f"{name}: undeclared YAML parameters"
            )
        ros.Node.overrides = {}


if __name__ == "__main__":
    unittest.main()
