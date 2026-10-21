"""从 ros2 bag 中读取 /imu 和 /joint_states，画出 pitch 曲线和位移曲线。

用法（先 conda activate rm_ros2 并 source 工作空间）：
    python3 plot_bag.py balance_test
输出：balance_test_plot.png，并在终端打印统计结果。
"""
import sys
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib
matplotlib.use('Agg')          # 不弹窗口，直接保存图片
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.06            # 轮子半径 (m)，与控制器一致

bag_path = sys.argv[1] if len(sys.argv) > 1 else 'balance_test'

# 1. 打开 bag
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr'))
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=['/imu', '/joint_states']))

# 2. 逐条读取消息
t0 = None
imu_t, pitch_deg = [], []
js_t, x_m = [], []

while reader.has_next():
    topic, data, stamp_ns = reader.read_next()
    msg = deserialize_message(data, get_message(topic_types[topic]))
    if t0 is None:
        t0 = stamp_ns
    t = (stamp_ns - t0) * 1e-9          # 换算成从 0 开始的秒数

    if topic == '/imu':
        q = msg.orientation
        s = 2.0 * (q.w * q.y - q.z * q.x)
        s = max(-1.0, min(1.0, s))
        imu_t.append(t)
        pitch_deg.append(math.degrees(math.asin(s)))

    elif topic == '/joint_states':
        pos = dict(zip(msg.name, msg.position))
        common_pos = (pos['leftwheel'] - pos['rightwheel']) / 2.0
        js_t.append(t)
        x_m.append(common_pos * WHEEL_RADIUS)

# 位移以录制开始时为 0
x0 = x_m[0]
x_m = [x - x0 for x in x_m]

# 3. 打印统计
max_pitch = max(abs(p) for p in pitch_deg)
mean_pitch = sum(pitch_deg) / len(pitch_deg)
print(f'录制时长      : {imu_t[-1]:.1f} s')
print(f'pitch 平均值  : {mean_pitch:.2f} deg')
print(f'pitch 最大偏差: {max_pitch:.2f} deg (验收要求 < 10 deg)')
print(f'最终位移      : {x_m[-1]:.2f} m')
print(f'最大位移      : {max(abs(x) for x in x_m):.2f} m')

# 4. 画图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

ax1.plot(imu_t, pitch_deg, linewidth=0.8)
ax1.axhline(10, color='red', linestyle='--', linewidth=0.8, label='±10° limit')
ax1.axhline(-10, color='red', linestyle='--', linewidth=0.8)
ax1.set_ylabel('pitch (deg)')
ax1.set_title('Body pitch')
ax1.legend(loc='upper right')
ax1.grid(alpha=0.3)

ax2.plot(js_t, x_m, linewidth=0.8, color='tab:green')
ax2.set_xlabel('time (s)')
ax2.set_ylabel('displacement (m)')
ax2.set_title('Forward displacement (from wheel odometry)')
ax2.grid(alpha=0.3)

plt.tight_layout()
out = f'{bag_path.rstrip("/")}_plot.png'
plt.savefig(out, dpi=150)
print(f'图已保存到    : {out}')