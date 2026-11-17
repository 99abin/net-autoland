"""Independent local-SITL recovery, usable even after another ROS context closes."""
import time
from simulation_flight import FlightObserver, managed_session


def recover(config):
    managed_session(config)
    from mavros_msgs.srv import SetMode
    observer = FlightObserver(full_autoland=True)
    try:
        observer.spin(2)
        def state():
            value = observer.messages.get("state")
            if value is None or not value.connected or time.monotonic() - observer.received["state"] > 3:
                raise RuntimeError("No fresh local SITL state; recovery cannot be confirmed")
            return value
        current = state()
        if not current.armed or current.mode not in ("AUTO", "GUIDED", "RTL"):
            print("Disarmed/pilot-selected mode preserved:", current.mode)
            return
        if current.mode != "RTL":
            client = observer.node.create_client(SetMode, "/mavros/set_mode")
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError("No local mode service")
            request = SetMode.Request()
            request.custom_mode = "RTL"
            future = client.call_async(request)
            observer.ros.spin_until_future_complete(observer.node, future, timeout_sec=3)
        end = time.monotonic() + 8
        while time.monotonic() < end:
            observer.spin(.1)
            managed_session(config)
            if state().mode == "RTL":
                print("RTL confirmed by managed local SITL telemetry")
                return
        raise RuntimeError("RTL recovery not confirmed")
    finally:
        observer.close()
