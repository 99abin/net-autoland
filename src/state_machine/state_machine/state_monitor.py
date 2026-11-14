import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from calculation import calc_distance, calc_bearing_error, calc_xtrack_error, calc_wp_proportion
from state_condition import should_enter_approach, should_enter_preflare, should_enter_flare
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import VFR_HUD

class state_machine(Node):
    def __init__(self):
        super().__init__('fsm')

        self.get_logger().info('state machine initialized')

        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.position_cb, qos_profile_sensor_data)
        self.vel_sub = self.create_subscription(TwistStamped, '/velocity', self.velocity_cb, 10)
        self.vfr_sub = self.create_subscription(VFR_HUD, '/mavros/vfr_hud', self.speed_cb, qos_profile_sensor_data)


        self.timer = self.create_timer(0.05, self.fsm_loop)

        self.declare_parameter('START_LAT', 0.0)
        self.declare_parameter('START_LON', 0.0)
        self.declare_parameter('START_ALT', 0.0)
        self.declare_parameter('LAND_LAT', 0.0)
        self.declare_parameter('LAND_LON', 0.0)
        self.declare_parameter('LAND_ALT', 0.0)
        self.declare_parameter('LAND_PF_ALT', 0.0)
        self.declare_parameter('LAND_PF_SEC', 0.0)
        self.declare_parameter('LAND_PF_ARSPD', 0.0)
        self.declare_parameter('LAND_FLARE_ALT', 0.0)
        self.declare_parameter('LAND_FLARE_SEC', 0.0)

        self.start_lat = self.get_parameter('START_LAT').value
        self.start_lon = self.get_parameter('START_LON').value
        self.start_alt = self.get_parameter('START_ALT').value
        self.land_lat = self.get_parameter('LAND_LAT').value
        self.land_lon = self.get_parameter('LAND_LON').value
        self.land_alt = self.get_parameter('LAND_ALT').value
        self.pf_alt = self.get_parameter('LAND_PF_ALT').value
        self.pf_sec = self.get_parameter('LAND_PF_SEC').value
        self.pf_arspd = self.get_parameter('LAND_PF_ARSPD').value
        self.f_alt = self.get_parameter('LAND_FLARE_ALT').value
        self.f_sec = self.get_parameter('LAND_FLARE_SEC').value

        self.total_distance = calc_distance(self.start_lat, self.start_lon, self.land_lat, self.land_lon)

        self.curr_lat = None
        self.curr_lon = None
        self.curr_alt = None
        self.vx = None
        self.vy = None
        self.vz = None
        self.airspeed = None
        self.groundspeed = None
        self.sink_rate = None

        self.wp_proportion = 0
        self.bearing_error = 0
        self.xtrack_error = 0

        self.curr_state = 'NORMAL'

    def position_cb(self, msg):
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude
        self.curr_alt = msg.altitude

    def velocity_cb(self, msg):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

        raw_sink_rate = -self.vz

        if self.sink_rate is None:
            self.sink_rate = raw_sink_rate

        else:
            self.sink_rate = (
                0.8 * self.sink_rate + 0.2 * raw_sink_rate
            )

    def speed_cb(self, msg):
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed

    def data_ready(self):

        required_data = [
            self.curr_lat,
            self.curr_lon,
            self.curr_alt,
            self.vx,
            self.vy,
            self.vz,
            self.sink_rate
        ]

        return all(
            value is not None
            for value in required_data
        )
        
    def fsm_loop(self):
        if not self.data_ready():
            return

        self.xtrack_error = calc_xtrack_error(self.curr_lat, self.curr_lon, self.start_lat, self.start_lon, self.land_lat, self.land_lon)
        self.bearing_error = calc_bearing_error(self.curr_lat, self.curr_lon, self.land_lat, self.land_lon, self.vx, self.vy)
        self.wp_proportion = calc_wp_proportion(self.start_lat, self.start_lon, self.curr_lat, self.curr_lon, self.land_lat, self.land_lon)

        if self.curr_state == 'NORMAL':
            enter_approach = should_enter_approach(self.bearing_error, self.xtrack_error, self.wp_proportion, self.curr_alt, self.start_alt)

            if enter_approach:
                self.curr_state = 'APPROACH'
                self.get_logger().info('NORMAL -> APPROACH')

        elif self.curr_state == 'APPROACH':
            enter_preflare = should_enter_preflare(self.pf_arspd, self.curr_alt, self.land_alt, self.sink_rate, self.pf_alt, self.pf_sec)

            if enter_preflare:
                self.curr_state = 'PREFLARE'
                self.get_logger().info('APPROACH -> PREFLARE')

        elif self.curr_state in ('APPROACH', 'PREFLARE'):
            if should_enter_flare(
                self.curr_alt, self.f_alt, self.f_sec,
                self.land_alt, self.sink_rate, self.wp_proportion
            ):
                previous_state = self.curr_state
                self.curr_state = 'FLARE'
                self.get_logger().info(f'{previous_state} -> FLARE')

            elif self.curr_state == 'APPROACH':
                if should_enter_preflare(
                    self.pf_arspd, self.curr_alt, self.land_alt,
                    self.sink_rate, self.pf_alt, self.pf_sec
                ):
                    self.curr_state = 'PREFLARE'
                    self.get_logger().info('APPROACH -> PREFLARE')


def main(args=None):
    rclpy.init(args=args)
    node = state_machine()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()