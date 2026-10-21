import math
import numpy as np
from wheel_leg_controller.ground_detection import detect_leg_ground_status


"""此处为复制的主文件相关控制参数，没有复制import mujoco相关的部分"""
class PIDController:
    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp; self.ki = ki; self.kd = kd; self.max_out = max_out; self.max_i = max_i; self.feedforward = feedforward
        self.integral = 0.0; self.prev_error = 0.0
    def compute(self, current, target):
        error = target - current; self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = (error - self.prev_error); self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative + self.feedforward
        return np.clip(output, -self.max_out, self.max_out)


# ==================== 封装的轮腿控制函数 ====================

def control_left_leg(info, five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw, 
                     target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                     is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
    """
    左腿 LQR + VMC 核心控制计算与条件离地保护机制
    :param ground_detection_enabled: 是否开启离地检测（即 teleop.control_enabled）
    """
    motor1_angle = info['pos']['left1'] 
    motor2_angle = info['pos']['left2']


    # 1. 运动学与状态更新
    five_links.forward_kinematics_cal(1, motor1_angle, motor2_angle, pitch, gyro_pitch)
    lqr.states_update(1, five_links, pitch, gyro_pitch, common_motor_pos, common_motor_speed, wheel_radius)
    lqr.calc_k_matrix_from_poly(five_links.L_0)

    # 2. VMC 虚拟力基本计算
    F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
    F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
    F_0 = np.clip(F_0, -100.0, 100.0)

    # 预计算常规状况下的 Tp
    T_p = (lqr.K[1] * (lqr.theta - 0.0)
           + lqr.K[3] * (lqr.d_theta - 0.0)
           + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
           + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
           + lqr.K[9] * (lqr.phi - 0.0)
           + lqr.K[11] * (lqr.d_phi - 0.0))

    # 3. 核心条件判定：只有在 Enter 激活后，才执行离地保护状态切换
    if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
        # 离地保护状态：轮子力矩清零，髋关节切换为空中阻尼姿态维持
        left_wheel_T = 0.0
        T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
        T_p += leg_tp_comp
        flight_flag = 1
    else:
        # 正常控制状态（未开启 Enter 或者是处于着地状态）：正常的自平衡控制
        yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
        wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                   + lqr.K[2] * (lqr.d_theta - 0.0)
                   + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                   + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                   + lqr.K[8] * (lqr.phi - 0.0)
                   + lqr.K[10] * (lqr.d_phi - 0.0))
        
        left_wheel_T = -wheel_T - yaw_ctrl
        left_wheel_T = np.clip(left_wheel_T, -1.5, 1.5)
        flight_flag = 0

    # 4. 最终 VMC 力矩映射
    torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)
    
    return -torque_set_0, -torque_set_1, left_wheel_T, flight_flag


def control_right_leg(info, five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw, 
                      target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                      is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
    """
    右腿 LQR + VMC 核心控制计算与条件离地保护机制
    """
    motor1_angle = info['pos']['right1']
    motor2_angle = info['pos']['right2']
    
    # 1. 运动学与状态更新
    five_links.forward_kinematics_cal(0, motor1_angle, motor2_angle, pitch, gyro_pitch)
    lqr.states_update(0, five_links, pitch, gyro_pitch, -common_motor_pos, -common_motor_speed, wheel_radius)
    lqr.calc_k_matrix_from_poly(five_links.L_0)

    # 2. VMC 虚拟力基本计算
    F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
    F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
    F_0 = np.clip(F_0, -100.0, 100.0)

    # 预计算常规状况下的 Tp
    T_p = (lqr.K[1] * (lqr.theta - 0.0)
           + lqr.K[3] * (lqr.d_theta - 0.0)
           + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
           + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
           + lqr.K[9] * (lqr.phi - 0.0)
           + lqr.K[11] * (lqr.d_phi - 0.0))
    
    # 3. 核心条件判定
    if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
        right_wheel_T = 0.0
        T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
        T_p += leg_tp_comp
        flight_flag = 1
    else:
        yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
        wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                   + lqr.K[2] * (lqr.d_theta - 0.0)
                   + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                   + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                   + lqr.K[8] * (lqr.phi - 0.0)
                   + lqr.K[10] * (lqr.d_phi - 0.0))
                   
        right_wheel_T = wheel_T - yaw_ctrl
        right_wheel_T = np.clip(right_wheel_T, -1.5, 1.5)
        flight_flag = 0

    torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)
    
    return torque_set_0, torque_set_1, right_wheel_T, flight_flag