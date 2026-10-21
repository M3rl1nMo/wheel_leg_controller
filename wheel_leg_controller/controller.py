import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Twist
from rm_msgs.msg import WheelLegCmd
from wheel_leg_controller.lqr_update import lqr_controller
from wheel_leg_controller.vmc_cal import five_links_param
from wheel_leg_controller.climb_traj import VelocityRampFilter
from wheel_leg_controller.roll_leg_control import RollLegController
from wheel_leg_controller.leg_control import PIDController, control_left_leg, control_right_leg

def clamp(x, limit):
    return max(-limit ,min(limit, x))
def deadzone(x, dz=0.1):
    return 0.0 if abs(x) < dz else x
class WheelLegController(Node):
        

    def __init__(self):
        super().__init__('wheel_leg_controller')
        self.get_logger().info('启动控制器')
        self.imu = None
        self.create_subscription(Imu, "imu", self.imu_cb, 10)
        self.joints = None
        self.create_subscription(JointState, "joint_states", self.joint_cb, 10)
        self.v_set = 0.0
        self.w_set = 0.0
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.cmd_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.timer = self.create_timer(0.005, self.control_loop)
        """此处直接照抄了"""
                # 基本参数
        self.dt = 0.002
        self.wheel_radius = 0.06
        self.target_length = 0.1
        self.ground_detection_enabled = False
        self.joy_v = 0.0
        self.joy_w = 0.0
        self.last_joy_time = None
        self.last_buttons = []
        self.joy_timeout = 0.5
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        # 运行中会变化的状态
        self.current_v = 0.0
        self.robot_is_airborne = False

        # 算法对象（左右腿各一套）
        self.five_links_right = five_links_param(dt=self.dt)
        self.five_links_left = five_links_param(dt=self.dt)
        self.lqr_right = lqr_controller(dt=self.dt)
        self.lqr_left = lqr_controller(dt=self.dt)

        self.left_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0, feedforward=0.0)
        self.right_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0, feedforward=0.0)
        self.left_L0_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0, feedforward=0.0)
        self.right_L0_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0, feedforward=0.0)

        self.v_filter = VelocityRampFilter(dt=self.dt, accel_rate=0.8, decel_rate=1.4)
        self.roll_balancer = RollLegController(kp=1.0, ki=0.0, kd=0.0, max_out=0.14,
                                               min_leg=0.10, max_leg=0.14)


    def imu_cb(self, msg):
        self.imu = msg
        q = msg.orientation
        #用一个算法将这个存储角度和角速度的数组拿出更直观
    def joint_cb(self, msg):
        self.joints = {}
        for i, name in enumerate(msg.name):
            self.joints[name] = (msg.position[i], msg.velocity[i])
    def cmd_vel_cb(self, msg):
        self.v_set = msg.linear.x
        self.w_set = msg.angular.z
        self.get_logger().info(
            f'目标速度 v={self.v_set:.2f} w={self.w_set:.2f}'
        )

    def joy_cb(self, msg):
        if len(msg.axes) < 4 or len(msg.buttons) < 6:
            return
        self.last_joy_time = self.get_clock().now()

        self.joy_v = deadzone(msg.axes[1]) * 0.6
        self.joy_w = -deadzone(msg.axes[3]) * 2.0

        for idx, step in ((5, 0.005), (4, -0.005)):
            pressed = msg.buttons[idx] == 1
            was_pressed = len(self.last_buttons) > idx and self.last_buttons[idx] == 1
            if pressed and not was_pressed:
                new_len = round(self.target_length + step, 3)
                self.target_length = max(0.10, min(0.14, new_len))
                self.get_logger().info(f'目标腿长 = {self.target_length:.3f}, m')
            self.last_buttons = list(msg.buttons)

    def select_cmd(self):
        if self.last_joy_time is not None:
            age = (self.get_clock().now() - self.last_joy_time).nanoseconds *1e-9
            if age < self.joy_timeout:
                return self.joy_v, self.joy_w
        return self.v_set, self.w_set


    def build_info(self):
        info = {}
        info['pos'] = {name:pv[0] for name, pv in self.joints.items()}
        info['vel'] = {name:pv[1] for name, pv in self.joints.items()}

        q = self.imu.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z - z * x)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        info['euler'] =  {'roll': roll, 'pitch': pitch, 'yaw': yaw}

        g = self.imu.angular_velocity
        info['gyro_yaw'] = g.x
        info['gyro_pitch'] = g.y
        info['gyro_roll'] = g.y
        return info

    
    def control_loop(self):
        if self.imu is None or self.joints is None:
            return

    
        # 1. 读取传感器(改变了字典，不用d中的参数)
        info = self.build_info()
        pitch = -info['euler']['pitch']
        roll = -info['euler']['roll']
        gyro_pitch = -info['gyro_pitch']
        gyro_yaw = info['gyro_yaw']

        # 2. Roll 平衡：根据车身左右倾斜，算出左右腿目标长度
        target_length_left, target_length_right = self.roll_balancer.compute_leg_lengths(
            current_roll=roll, target_roll=0.0, base_length=self.target_length)

        # 3. 共模提取：用左右轮算出整车的位移和速度
        common_motor_speed = (info['vel']['leftwheel'] - info['vel']['rightwheel']) / 2.0
        common_motor_pos = (info['pos']['leftwheel'] - info['pos']['rightwheel']) / 2.0

        # 4. 速度与位置设定
        target_v, target_w = self.select_cmd()

        if self.ground_detection_enabled and self.robot_is_airborne:
            self.lqr_left.x_b_set = common_motor_pos * self.wheel_radius
            self.lqr_right.x_b_set = common_motor_pos * self.wheel_radius
            self.current_v = 0.0
            target_w = 0.0
        else:
            self.current_v = self.v_filter.update(current_v=self.current_v, target_v=target_v)
            if not (self.current_v == 0.0 and target_v == 0.0):
                self.lqr_left.x_b_set = common_motor_pos * self.wheel_radius
                self.lqr_right.x_b_set = common_motor_pos * self.wheel_radius
        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v

        # 5. 左右腿控制：LQR + VMC，算出每条腿的 2 个关节力矩和轮子力矩
        ctrl_left_0, ctrl_left_1, left_wheel_T, left_flight = control_left_leg(
            info, self.five_links_left, self.lqr_left, self.left_yaw_pid, self.left_L0_pid,
            pitch, gyro_pitch, gyro_yaw, target_w, target_length_left,
            common_motor_pos, common_motor_speed, self.wheel_radius,
            is_airborne_last_step=self.robot_is_airborne,
            ground_detection_enabled=self.ground_detection_enabled,
            leg_tp_comp=0.0)

        ctrl_right_0, ctrl_right_1, right_wheel_T, right_flight = control_right_leg(
            info, self.five_links_right, self.lqr_right, self.right_yaw_pid, self.right_L0_pid,
            pitch, gyro_pitch, gyro_yaw, target_w, target_length_right,
            common_motor_pos, common_motor_speed, self.wheel_radius,
            is_airborne_last_step=self.robot_is_airborne,
            ground_detection_enabled=self.ground_detection_enabled,
            leg_tp_comp=0.0)

        # 6. 离地状态汇总
        if self.ground_detection_enabled:
            self.robot_is_airborne = (left_flight == 1) or (right_flight == 1)
        else:
            self.robot_is_airborne = False

        # 7. 输出（原：d.ctrl[0..5] = ...）
        cmd = WheelLegCmd()
        cmd.right_phi1 = clamp(float(ctrl_right_0), 6.0)
        cmd.right_phi4 = clamp(float(ctrl_right_1), 6.0)
        cmd.left_phi1 = clamp(float(ctrl_left_0), 6.0)
        cmd.left_phi4 = clamp(float(ctrl_left_1), 6.0)
        cmd.right_wheel = clamp(float(right_wheel_T), 2.0)
        cmd.left_wheel = clamp(float(left_wheel_T), 2.0)
        self.cmd_pub.publish(cmd)


def main(args = None):  
    rclpy.init(args=args)
    node = WheelLegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()