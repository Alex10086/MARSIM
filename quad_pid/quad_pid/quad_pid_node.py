import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from quad_pid.mixer import allocate
from quad_pid.geometry import accel_to_attitude, yaw_from_quaternion, shortest_angle
from quad_pid.goal import goal_is_active, sanitize_goal


class QuadPIDNode(Node):
    def __init__(self):
        super().__init__('quad_pid_node')
        self._declare_params()
        self._init_state()
        self._create_subscriptions()
        self._create_publisher()
        # Publish hover RPM immediately to eliminate the zero-thrust window
        # between node startup and the first control timer tick.
        self._publish_rpm(np.full(4, self._hover_rpm()))
        self._create_timer()
        self.get_logger().info(
            f'quad_pid_node started (hover RPM={self._hover_rpm():.1f})')

    def _hover_rpm(self):
        """RPM per motor that exactly balances gravity (F = m*g)."""
        mass = self.get_parameter('mass').value
        k_F = self.get_parameter('k_F').value
        return math.sqrt(mass * 9.81 / (4.0 * k_F))

    def _declare_params(self):
        """Declare all ROS parameters with defaults from YAML."""
        self.declare_parameter('KP_XY', 0.5)
        self.declare_parameter('KD_XY', 1.0)
        self.declare_parameter('KP_Z', 1.0)
        self.declare_parameter('KD_Z', 1.4)
        self.declare_parameter('KI_Z', 0.0)
        self.declare_parameter('KP_ATT', 0.2)
        self.declare_parameter('KD_ATT', 0.04)
        self.declare_parameter('KP_YAW', 0.2)
        self.declare_parameter('KD_YAW', 0.05)
        self.declare_parameter('mass', 1.9)
        self.declare_parameter('arm_length', 0.22)
        self.declare_parameter('k_F', 2.694396e-8)
        self.declare_parameter('k_T', 3.508e-10)
        self.declare_parameter('max_tilt_deg', 20.0)
        self.declare_parameter('max_horiz_acc', 3.0)
        self.declare_parameter('max_vert_acc_up', 4.0)
        self.declare_parameter('max_vert_acc_down', -4.0)
        self.declare_parameter('max_torque_xy', 1.0)
        self.declare_parameter('max_torque_z', 0.3)
        self.declare_parameter('max_rpm', 35000)
        self.declare_parameter('min_rpm', 0)
        self.declare_parameter('goal_timeout', 1.0)
        # RViz 2D Goal Pose sends the goal once per click (one-shot). Set
        # goal_latched=true to keep tracking it indefinitely. Set false for a
        # streaming planner that republishes every cycle.
        self.declare_parameter('goal_latched', True)
        # Reject goals farther than this (horizontally) from the vehicle. A
        # click near the horizon in RViz projects onto the ground plane
        # kilometres away; such a goal must not be tracked.
        self.declare_parameter('max_goal_dist', 50.0)
        self.declare_parameter('max_horiz_dist', 5.0)
        self.declare_parameter('max_horiz_speed', 2.0)
        self.declare_parameter('max_descend_speed', 1.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('verbose', False)

    def _init_state(self):
        """Initialize internal state."""
        self.goal = None
        self.goal_stamp = 0.0
        now = time.time()
        self.state = {
            'pos': np.zeros(3),
            'vel': np.zeros(3),
            'quat': np.array([0.0, 0.0, 0.0, 1.0]),  # x,y,z,w
            'gyro': np.zeros(3),
            'odom_stamp': now,   # avoid freshness timeout before first odom
            'imu_stamp': 0.0,
        }
        self.last_rpm = np.zeros(4)
        self.last_rpm_valid = False
        self.integral_z = 0.0
        self._last_log_time = 0.0
        self._has_odom = False  # track whether we've ever received odom
        self._debug_tick = 0
        self._bad_goal_warned = False  # warn once per rejected goal

    def _create_subscriptions(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/cmd', self._goal_callback, qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, qos)

    def _create_publisher(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/cmd_RPM', qos)

    def _create_timer(self):
        rate = self.get_parameter('control_rate').value
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._control_tick)

    def _read_params(self):
        """Re-read all parameters from param server (supports hot reload)."""
        p = lambda name: self.get_parameter(name).value
        return {
            'KP_XY': p('KP_XY'), 'KD_XY': p('KD_XY'),
            'KP_Z': p('KP_Z'), 'KD_Z': p('KD_Z'), 'KI_Z': p('KI_Z'),
            'KP_ATT': p('KP_ATT'), 'KD_ATT': p('KD_ATT'),
            'KP_YAW': p('KP_YAW'), 'KD_YAW': p('KD_YAW'),
            'mass': p('mass'), 'arm_length': p('arm_length'),
            'k_F': p('k_F'), 'k_T': p('k_T'),
            'max_tilt_rad': math.radians(p('max_tilt_deg')),
            'max_horiz_acc': p('max_horiz_acc'),
            'max_vert_acc_up': p('max_vert_acc_up'),
            'max_vert_acc_down': p('max_vert_acc_down'),
            'max_torque_xy': p('max_torque_xy'),
            'max_torque_z': p('max_torque_z'),
            'max_rpm': p('max_rpm'), 'min_rpm': p('min_rpm'),
            'goal_timeout': p('goal_timeout'),
            'goal_latched': p('goal_latched'),
            'max_goal_dist': p('max_goal_dist'),
            'max_horiz_dist': p('max_horiz_dist'),
            'max_horiz_speed': p('max_horiz_speed'),
            'max_descend_speed': p('max_descend_speed'),
            'use_imu': p('use_imu'), 'verbose': p('verbose'),
        }

    def _goal_callback(self, msg: PoseStamped):
        self.goal = msg
        self.goal_stamp = time.time()

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self.state['pos'] = np.array([p.x, p.y, p.z])
        self.state['vel'] = np.array([v.x, v.y, v.z])
        self.state['quat'] = np.array([q.x, q.y, q.z, q.w])
        self.state['odom_stamp'] = time.time()
        self._has_odom = True

    def _imu_callback(self, msg: Imu):
        g = msg.angular_velocity
        self.state['gyro'] = np.array([g.x, g.y, g.z])
        self.state['imu_stamp'] = time.time()

    def _quat_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to ZYX Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _compute_current_tilt(self, qx, qy, qz, qw):
        """Compute angle between body z-axis and world z-axis."""
        # body z in world = rotate [0,0,1] by quaternion
        z_body_world_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        z_body_world_z = max(-1.0, min(1.0, z_body_world_z))
        return math.acos(z_body_world_z)

    def _control_tick(self):
        """Main control loop: L1→L2→L3→L4 → publish RPM."""
        now = time.time()
        cfg = self._read_params()

        # ── Freshness checks ──
        if not self._has_odom:
            # No odom ever received → publish known hover RPM
            self._publish_rpm(np.full(4, self._hover_rpm()))
            return

        if now - self.state['odom_stamp'] > 0.5:
            return  # Stale odom → don't publish

        if not goal_is_active(has_goal=self.goal is not None,
                              goal_age=now - self.goal_stamp,
                              goal_timeout=cfg['goal_timeout'],
                              latched=cfg['goal_latched']):
            # No goal (yet) or streaming goal went stale → hold current pose.
            pos_des = self.state['pos'].copy()
            yaw_des = yaw_from_quaternion(*self.state['quat'])
        else:
            g = self.goal.pose.position
            cur = self.state['pos']
            gx, gy, gz, ok = sanitize_goal(g.x, g.y, g.z,
                                           cur[0], cur[1], cur[2],
                                           cfg['max_goal_dist'])
            q = self.goal.pose.orientation
            if not ok:
                # Unusable goal (horizon click kilometres away, or NaN):
                # ignore it and hold the current pose.
                if not self._bad_goal_warned:
                    self._bad_goal_warned = True
                    self.get_logger().warn(
                        f'Ignoring goal ({g.x:.1f},{g.y:.1f},{g.z:.1f}): '
                        f'farther than {cfg["max_goal_dist"]:.0f}m from vehicle '
                        f'or invalid. Holding position.')
                pos_des = self.state['pos'].copy()
                yaw_des = yaw_from_quaternion(*self.state['quat'])
            else:
                pos_des = np.array([gx, gy, gz])
                # If orientation is identity, keep current yaw
                if abs(q.x) < 1e-9 and abs(q.y) < 1e-9 and abs(q.z) < 1e-9 and abs(q.w - 1.0) < 1e-9:
                    yaw_des = yaw_from_quaternion(*self.state['quat'])
                else:
                    yaw_des = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Safety S2: distance limiting
        delta = pos_des - self.state['pos']
        dist = np.linalg.norm(delta)
        if dist > cfg['max_horiz_dist']:
            delta = delta * (cfg['max_horiz_dist'] / dist)
            pos_des = self.state['pos'] + delta

        # ── Layer 1: Position error → desired acceleration (PD) ──
        e_pos = pos_des - self.state['pos']
        e_vel = np.zeros(3) - self.state['vel']  # vel_des = [0,0,0]

        a_des = np.zeros(3)
        a_des[0] = cfg['KP_XY'] * e_pos[0] + cfg['KD_XY'] * e_vel[0]
        a_des[1] = cfg['KP_XY'] * e_pos[1] + cfg['KD_XY'] * e_vel[1]
        a_des[2] = cfg['KP_Z']  * e_pos[2] + cfg['KD_Z']  * e_vel[2] + 9.81

        # Integrator (Z only, default KI_Z=0)
        if cfg['KI_Z'] > 1e-9:
            self.integral_z += e_pos[2] * (1.0 / self.get_parameter('control_rate').value)
            self.integral_z = max(-2.0, min(2.0, self.integral_z))
            a_des[2] += cfg['KI_Z'] * self.integral_z

        # Clamp accelerations (PD output only, before gravity compensation)
        a_des[0] = max(-cfg['max_horiz_acc'], min(cfg['max_horiz_acc'], a_des[0]))
        a_des[1] = max(-cfg['max_horiz_acc'], min(cfg['max_horiz_acc'], a_des[1]))
        # Z: clamp PD component, then add gravity
        pd_z = a_des[2] - 9.81
        pd_z = max(cfg['max_vert_acc_down'], min(cfg['max_vert_acc_up'], pd_z))
        a_des[2] = pd_z + 9.81

        # Safety S3: descend speed limit
        if e_pos[2] < -3.0:
            vel_z = self.state['vel'][2]
            if vel_z < -cfg['max_descend_speed']:
                a_des_z_corr = cfg['KP_Z'] * e_pos[2] + cfg['KD_Z'] * (-cfg['max_descend_speed'] - vel_z)
                a_des[2] = min(a_des[2], max(cfg['max_vert_acc_down'], a_des_z_corr))

        # ── Layer 2: acceleration → attitude + thrust ──
        roll_des, pitch_des = accel_to_attitude(a_des[0], a_des[1], a_des[2])

        # Clamp attitude
        mt = cfg['max_tilt_rad']
        roll_des = max(-mt, min(mt, roll_des))
        pitch_des = max(-mt, min(mt, pitch_des))

        # Total thrust: F = m·|a_des|, where a_des is the full desired world
        # acceleration (already includes gravity compensation).
        # Do NOT divide by cos(tilt): |a_des| and its direction already encode
        # the required thrust magnitude; dividing inflates thrust whenever the
        # drone is tilted and causes runaway climb.
        acc_mag = math.sqrt(a_des[0]**2 + a_des[1]**2 + a_des[2]**2)
        F_total = cfg['mass'] * acc_mag
        F_total = max(0.0, min(4.0 * cfg['mass'] * 9.81, F_total))

        # ── Layer 3: attitude error → torque ──
        q = self.state['quat']
        roll_cur, pitch_cur, yaw_cur = self._quat_to_euler(q[0], q[1], q[2], q[3])

        e_roll = shortest_angle(roll_des, roll_cur)
        e_pitch = shortest_angle(pitch_des, pitch_cur)
        e_yaw = shortest_angle(yaw_des, yaw_cur)

        # Use IMU gyro for D term if available and fresh
        use_gyro = cfg['use_imu'] and (now - self.state['imu_stamp']) < 0.2
        gyro_damp = self.state['gyro'] if use_gyro else np.zeros(3)

        tau = np.zeros(3)
        tau[0] = cfg['KP_ATT'] * e_roll  + cfg['KD_ATT'] * (-gyro_damp[0])
        tau[1] = cfg['KP_ATT'] * e_pitch + cfg['KD_ATT'] * (-gyro_damp[1])
        tau[2] = cfg['KP_YAW'] * e_yaw   + cfg['KD_YAW'] * (-gyro_damp[2])

        # Clamp torques
        tau[0] = max(-cfg['max_torque_xy'], min(cfg['max_torque_xy'], tau[0]))
        tau[1] = max(-cfg['max_torque_xy'], min(cfg['max_torque_xy'], tau[1]))
        tau[2] = max(-cfg['max_torque_z'],  min(cfg['max_torque_z'],  tau[2]))

        # ── Layer 4: Mixer → RPM ──
        rpm = allocate(F_total, tau[0], tau[1], tau[2],
                       cfg['k_F'], cfg['k_T'], cfg['arm_length'])

        # Clamp RPM
        rpm = np.clip(rpm, cfg['min_rpm'], cfg['max_rpm'])

        # ── NaN guard: hold last valid RPM if any output is NaN ──
        if np.any(np.isnan(rpm)):
            if self.last_rpm_valid:
                rpm = self.last_rpm
            else:
                return
        else:
            self.last_rpm = rpm.copy()
            self.last_rpm_valid = True

        # ── Publish ──
        self._publish_rpm(rpm)

        # Verbose logging
        if cfg['verbose'] and (now - self._last_log_time) > 1.0:
            self._last_log_time = now
            self.get_logger().info(
                f'pos=({self.state["pos"][0]:.1f},{self.state["pos"][1]:.1f},{self.state["pos"][2]:.1f}) '
                f'RPM=[{rpm[0]:.0f},{rpm[1]:.0f},{rpm[2]:.0f},{rpm[3]:.0f}] '
                f'tilt=({math.degrees(roll_cur):.1f},{math.degrees(pitch_cur):.1f})'
            )

    def _publish_rpm(self, rpm: np.ndarray):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension(label='motor', size=4, stride=1)]
        msg.data = rpm.tolist()
        self.rpm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuadPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()