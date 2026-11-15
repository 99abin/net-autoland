"""Minimal ROS transport doubles for contract tests; NOT a DDS/ROS integration test."""

import copy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

clock = NS(value=1000.0)


class Msg:
    defaults = {}

    def __init__(self, **kwargs):
        for key, make in self.defaults.items():
            setattr(self, key, make())
        for key, value in kwargs.items():
            if key not in self.defaults:
                raise TypeError(f"Unknown {type(self).__name__} field {key}")
            setattr(self, key, value)


def message(name, fields):
    return type(name, (Msg,), {"defaults": fields})


Stamp = message("Stamp", {"sec": lambda: 0, "nanosec": lambda: 0})
Header = message("Header", {"stamp": Stamp, "frame_id": str})
Vector3 = message("Vector3", {k: float for k in ("x", "y", "z")})
Quaternion = message("Quaternion", {k: float for k in ("x", "y", "z", "w")})
Pose = message("Pose", {"position": Vector3, "orientation": Quaternion})
PoseStamped = message("PoseStamped", {"header": Header, "pose": Pose})
Twist = message("Twist", {"linear": Vector3, "angular": Vector3})
TwistStamped = message("TwistStamped", {"header": Header, "twist": Twist})
String = message("String", {"data": str})
Float64 = message("Float64", {"data": float})
State = message("State", {"header": Header, "connected": bool, "armed": bool, "mode": str})
NavSatFix = message(
    "NavSatFix",
    {
        "header": Header,
        "latitude": float,
        "longitude": float,
        "altitude": float,
        "status": lambda: NS(status=0),
    },
)
VFR_HUD = message("VFR_HUD", {"header": Header, "airspeed": float, "groundspeed": float})
Waypoint = message(
    "Waypoint",
    {
        **{k: float for k in ("param1", "param2", "param3", "param4", "x_lat", "y_long", "z_alt")},
        "frame": int,
        "command": int,
        "is_current": bool,
        "autocontinue": bool,
    },
)
WaypointList = message("WaypointList", {"current_seq": int, "waypoints": list})
AttitudeTarget = message(
    "AttitudeTarget",
    {
        "header": Header,
        "type_mask": int,
        "orientation": Quaternion,
        "thrust": float,
        "body_rate": Vector3,
    },
)
CameraInfo = message(
    "CameraInfo",
    {
        "header": Header,
        "k": lambda: [0.0] * 9,
        "d": list,
        "width": int,
        "height": int,
        "distortion_model": str,
    },
)
Image = message("Image", {"header": Header, "data": lambda: None})


def interface(name):
    defaults = {}
    text = (
        Path(__file__).resolve().parents[1] / "src/autoland_interfaces/msg" / f"{name}.msg"
    ).read_text()
    for line in text.splitlines():
        if not line or line.startswith("#"):
            continue
        typ, key = line.split()
        if "[" in typ:
            base, size = typ.rstrip("]").split("[")
            n = int(size)
            make = lambda n=n: [0.0] * n
        else:
            make = {
                "std_msgs/Header": Header,
                "geometry_msgs/Pose": Pose,
                "bool": bool,
                "string": str,
                "uint32": int,
                "uint8": int,
                "float64": float,
            }[typ]
        defaults[key] = make
    return message(name, defaults)


class Future:
    def __init__(self):
        self.complete = False
        self.value = None

    def done(self):
        return self.complete

    def result(self):
        return self.value

    def finish(self, **kw):
        self.value = NS(**kw)
        self.complete = True


class Client:
    def __init__(self):
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        f = Future()
        self.calls.append((request, f))
        return f


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, m):
        self.messages.append(copy.deepcopy(m))

    def get_subscription_count(self):
        return 0


class Node:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.logs = []
        self.services = {}
        self.subscriptions = []

    def declare_parameter(self, k, v):
        self.parameters[k] = self.overrides.get(self.name, {}).get(k, v)

    def get_parameter(self, k):
        return NS(value=self.parameters[k])

    def create_client(self, *a):
        return Client()

    def create_publisher(self, *a):
        return Publisher()

    def create_subscription(self, typ, topic, cb, qos):
        self.subscriptions.append((topic, cb))
        return NS()

    def create_service(self, typ, name, cb):
        self.services[name] = cb
        return NS()

    def create_timer(self, period, cb):
        return NS(period=period, cb=cb)

    def get_clock(self):
        def now():
            nano = round(clock.value * 1e9)
            return NS(
                nanoseconds=nano, to_msg=lambda: Stamp(sec=nano // 10**9, nanosec=nano % 10**9)
            )

        return NS(now=now)

    def get_logger(self):
        return NS(info=self.logs.append, error=self.logs.append, warning=self.logs.append)

    def destroy_node(self):
        pass


def service(name, request):
    return type(name, (), {"Request": message(name + "Request", request)})


def install():
    global _installed
    if globals().get("_installed", False):
        return
    _installed = True
    modules = [
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "sensor_msgs",
        "sensor_msgs.msg",
        "geometry_msgs",
        "geometry_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
        "std_srvs",
        "std_srvs.srv",
        "mavros_msgs",
        "mavros_msgs.msg",
        "mavros_msgs.srv",
        "autoland_interfaces",
        "autoland_interfaces.msg",
        "autoland_interfaces.srv",
        "rcl_interfaces",
        "rcl_interfaces.msg",
        "rcl_interfaces.srv",
        "cv_bridge",
    ]
    for name in modules:
        sys.modules[name] = ModuleType(name)
    sys.modules["rclpy.node"].Node = Node
    sys.modules["rclpy.qos"].QoSProfile = lambda **kw: NS(**kw)
    sys.modules["rclpy.qos"].DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
    sys.modules["rclpy.qos"].ReliabilityPolicy = NS(RELIABLE=1)
    sys.modules["rclpy.qos"].qos_profile_sensor_data = object()
    maps = {
        "sensor_msgs.msg": ["NavSatFix", "CameraInfo", "Image"],
        "geometry_msgs.msg": ["PoseStamped", "TwistStamped"],
        "std_msgs.msg": ["String", "Float64"],
        "mavros_msgs.msg": ["State", "Waypoint", "WaypointList", "VFR_HUD", "AttitudeTarget"],
    }
    for module, names in maps.items():
        for name in names:
            setattr(sys.modules[module], name, globals()[name])
    for name in (
        "LandingPlan",
        "LandingStatus",
        "NetEstimate",
        "ControlSetpoint",
        "NetDetection",
        "ParameterProfileStatus",
    ):
        typ = interface(name)
        globals()[name] = typ
        setattr(sys.modules["autoland_interfaces.msg"], name, typ)
    sys.modules["std_srvs.srv"].Trigger = service("Trigger", {})
    sys.modules["mavros_msgs.srv"].WaypointPull = service("WaypointPull", {})
    value = message("ParameterValue", {"type": int, "integer_value": int, "double_value": float})
    parameter = message("Parameter", {"name": str, "value": value})
    event = message(
        "ParameterEvent",
        {
            "node": str,
            "new_parameters": list,
            "changed_parameters": list,
            "deleted_parameters": list,
        },
    )
    sys.modules["rcl_interfaces.msg"].ParameterValue = value
    sys.modules["rcl_interfaces.msg"].Parameter = parameter
    sys.modules["rcl_interfaces.msg"].ParameterEvent = event
    sys.modules["rcl_interfaces.srv"].GetParameters = service("GetParameters", {"names": list})
    sys.modules["rcl_interfaces.srv"].ListParameters = service(
        "ListParameters", {"prefixes": list, "depth": int}
    )
    sys.modules["rcl_interfaces.srv"].SetParameters = service("SetParameters", {"parameters": list})
    sys.modules["mavros_msgs.srv"].ParamPull = service("ParamPull", {"force_pull": bool})

    sys.modules["mavros_msgs.srv"].WaypointPush = service(
        "WaypointPush", {"start_index": int, "waypoints": list}
    )
    sys.modules["mavros_msgs.srv"].SetMode = service("SetMode", {"custom_mode": str})
    sys.modules["autoland_interfaces.srv"].SetControlOwnership = service(
        "SetControlOwnership", {"owner": str, "plan_id": str}
    )
    sys.modules["cv_bridge"].CvBridge = type(
        "Bridge", (), {"imgmsg_to_cv2": lambda self, m, desired_encoding: m.data}
    )


install()


def verified_profile(node, values=None, verified=True):
    import json
    from autoland.parameter_profile import profile_id
    from autoland_interfaces.msg import ParameterProfileStatus

    values = values or {"LAND_TYPE": 0, "LAND_FLARE_ALT": 3.0, "LAND_FLARE_SEC": 2.0}
    msg = ParameterProfileStatus(
        verified=verified,
        state="READY",
        profile_id=profile_id(values),
        parameters_json=json.dumps(values),
    )
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg
