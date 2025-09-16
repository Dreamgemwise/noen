# # 以下是增加滤波代码

# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# lowpass_plot.py  ——  传感器数据低通滤波 + 画图
# -----------------------------------------------------------
# • 自动读取 output_data.csv
# • Butterworth 4th-order 低通（默认 20 Hz）
# • 输出原始 vs. 滤波后曲线
# """

# import pandas as pd
# import numpy as np
# from scipy import signal
# import matplotlib.pyplot as plt
# from pathlib import Path

# # -------------------------------------------------
# # 1. 读取数据
# # -------------------------------------------------
# file_path = Path(__file__).with_name('output_data.csv')  # 同目录
# df = pd.read_csv(file_path)                              # 若文件不在同目录请改绝对路径

# l1 = 0.324
# l2 = 0.184
# time = df['Time']

# # 轨迹
# q1 = df['q_0']
# q2 = df['q_4']
# x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
# y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
# figure = plt.figure(figsize=(6, 6))
# plt.plot(x, y, label='trajectory')
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# plt.legend()
# # 绘制圆
# circle = plt.Circle((0.25, 0.25), 0.08, color='r', fill=False)
# plt.gca().add_patch(circle)
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# plt.legend()

# # 速度
# J_00 = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
# J_01 = -l2 * np.sin(q1 + q2)
# J_10 = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
# J_11 = l2 * np.cos(q1 + q2)
# vx = J_00 * df['dq_0'] + J_01 * df['dq_4']
# vy = J_10 * df['dq_0'] + J_11 * df['dq_4']
# v_norm = np.sqrt(vx**2 + vy**2)
# plt.figure(figsize=(6, 3))
# plt.plot(time, v_norm, label='v')
# plt.axhline(y=0.05, color='r', linestyle='--', label='reference')
# plt.xlabel('time (s)')
# plt.ylabel('(m/s)')
# plt.legend()

# # 误差
# e_0 = df['e_0']
# e_1 = df['e_1']
# plt.figure(figsize=(6, 3))
# plt.plot(time, e_0, label='e_0')
# plt.plot(time, e_1, label='e_1')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.legend()

# # 滑膜面
# s_0 = df['s_0']
# s_1 = df['s_1']
# plt.figure(figsize=(6, 3))
# plt.plot(time, s_0, label='s_0')
# plt.plot(time, s_1, label='s_1')
# plt.xlabel('Time (s)')
# plt.ylabel('s')
# plt.legend()

# # 控制量
# tau_0 = df['tau_0']
# tau_4 = df['tau_4']
# plt.figure(figsize=(6, 3))
# plt.plot(time, tau_0, label='tau_0')
# plt.plot(time, tau_4, label='tau_4')
# plt.xlabel('Time (s)')
# plt.ylabel('tau')
# plt.legend()

# # k1
# k1_0 = df['k1_0']
# k1_1 = df['k1_1']
# plt.figure(figsize=(6, 6))
# plt.plot(time, k1_0, label='k1_0')
# plt.plot(time, k1_1, label='k1_1')
# plt.xlabel('Time (s)')
# plt.ylabel('k1')
# plt.legend()

# # 交互力
# Fext_0 = df['Fext_0']
# Fext_1 = df['Fext_1']
# Fext_norm = np.sqrt(Fext_0**2 + Fext_1**2)
# plt.figure(figsize=(6, 6))
# plt.plot(time, Fext_norm, label='Fext')
# plt.xlabel('Time (s)')
# plt.ylabel('Fext')
# plt.legend()

# # -------------------------------------------------
# # 2. 低通滤波函数
# # -------------------------------------------------
# def butter_lowpass_filter(data, fs, cutoff=20.0, order=4):
#     """零相移 4 阶 Butterworth 低通"""
#     nyq = 0.5 * fs
#     b, a = signal.butter(order, cutoff / nyq, btype='low')
#     return signal.filtfilt(b, a, data)

# # -------------------------------------------------
# # 3. 采样频率
# # -------------------------------------------------
# dt = np.diff(df['Time']).mean()        # s
# fs = 1.0 / dt                          # Hz
# print(f"采样频率 ≈ {fs:.1f} Hz (Δt≈{dt*1e3:.2f} ms)")

# # -------------------------------------------------
# # 4. 需要滤波的列 & 参数
# # -------------------------------------------------
# sensor_cols = ['Fext_0', 'Fext_1', 'tau_0', 'tau_4','e_0', 'e_1']     # 想加别的列直接写进列表
# cutoff = 20.0                                            # Hz

# # -------------------------------------------------
# # 5. 执行滤波
# # -------------------------------------------------
# for col in sensor_cols:
#     df[f'{col}_lp'] = butter_lowpass_filter(df[col], fs, cutoff=cutoff)

# # -------------------------------------------------
# # 6. 画图
# # -------------------------------------------------
# time = df['Time']
# nrows = len(sensor_cols)
# fig, axes = plt.subplots(nrows, 1, figsize=(8, 2.6*nrows), sharex=True)

# for ax, col in zip(axes, sensor_cols):
#     ax.plot(time, df[col],  label=f'{col} (raw)', alpha=0.4)
#     ax.plot(time, df[f'{col}_lp'], label=f'{col} LP ({cutoff} Hz)', lw=1.2)
#     ax.set_ylabel(col)
#     ax.grid(True)
#     ax.legend(loc='upper right')

# axes[-1].set_xlabel('Time [s]')
# fig.suptitle('Low-pass filtering of sensor data', y=1.02, fontsize=14)
# plt.tight_layout()
# plt.show()

# # -------------------------------------------------
# # 7. 如需保存过滤后的数据：
# # df.to_csv('output_data_filtered.csv', index=False)

# 以下是未滤波画图代码

#!/usr/bin/env python3control
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
script_dir = Path(__file__).parent

# file_path = script_dir.joinpath('EXP-PONITSMC', 'output_data.csv')
file_path = Path(__file__).with_name('output_data.csv')
data = pd.read_csv(file_path)

# 对比算法
file_path1 = Path(__file__).with_name('output_data1.csv')
data1 = pd.read_csv(file_path1)


# Time,e_0,e_1,s_0,s_1,ds_0,ds_1,k1_0,k1_1,q_0,q_4,dq_0,dq_4,dqr_0,dqr_1,dqd_0,dqd_1,ddqd_0,ddqd_1,tau_0,tau_4,Fext_0,Fext_1

l1 = 0.324
l2 = 0.184
time = data['Time']

# 轨迹
q1 = data['q_0']
q2 = data['q_4']
x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
figure = plt.figure(figsize=(6, 6))
plt.plot(x, y, label='trajectory', zorder=1)
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
# 绘制圆
circle = plt.Circle((0.25, 0.25), 0.08, color='r', linestyle='--', fill=False, label='reference', zorder=2)
plt.gca().add_patch(circle)
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()

# 速度
J_00 = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
J_01 = -l2 * np.sin(q1 + q2)
J_10 = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
J_11 = l2 * np.cos(q1 + q2)
vx = J_00 * data['dq_0'] + J_01 * data['dq_4']
vy = J_10 * data['dq_0'] + J_11 * data['dq_4']
v_norm = np.sqrt(vx**2 + vy**2)
plt.figure(figsize=(6, 3))
plt.plot(time, v_norm, label='v')
plt.axhline(y=0.05, color='r', linestyle='--', label='reference')
plt.xlabel('time (s)')
plt.ylabel('(m/s)')
plt.legend()

# 误差
# e_0 = data['e_0']
# e_1 = data['e_1']
dqd_0 = data['dqd_0']
dqd_1 = data['dqd_1']
dq_0 = data['dq_0']
dq_1 = data['dq_4']
e_0 = dqd_0 - dq_0
e_1 = dqd_1 - dq_1
plt.figure(figsize=(6, 3))
plt.plot(time, e_0, label='e_0')
plt.plot(time, e_1, label='e_1')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.legend()

# 滑膜面
s_0 = data['s_0']
s_1 = data['s_1']
plt.figure(figsize=(6, 3))
plt.plot(time, s_0, label='s_0')
plt.plot(time, s_1, label='s_1')
plt.xlabel('Time (s)')
plt.ylabel('s')
plt.legend()

# 控制量
tau_0 = data['tau_0']
tau_4 = data['tau_4']
tau1=data1['tau_0']
tau2=data1['tau_4']
plt.figure(figsize=(6, 3))
plt.plot(time, tau_0, label='tau1')
plt.plot(time, tau_4, label='tau2')
# plt.plot(time, tau1, label='proposed_tau1')
# plt.plot(time, tau2, label='proposed_tau2')
plt.xlabel('Time (s)')
plt.ylabel('tau')
plt.legend()

# k1
k1_0 = data['k1_0']
k1_1 = data['k1_1']
plt.figure(figsize=(6, 6))
plt.plot(time, k1_0, label='k1_0')
plt.plot(time, k1_1, label='k1_1')
plt.xlabel('Time (s)')
plt.ylabel('k1')
plt.legend()

# 交互力
Fext_0 = data['Fext_0']
Fext_1 = data['Fext_1']
Fext_x=data1['Fext_0']
Fext_y=data1['Fext_1']
Fext_norm = np.sqrt(Fext_0**2 + Fext_1**2)
plt.figure(figsize=(6, 6))
plt.plot(time, Fext_norm, label='Fext')
plt.xlabel('Time (s)')
plt.ylabel('||Fext||')
plt.legend()

plt.figure(figsize=(6, 6))
plt.plot(time, Fext_0, label='Fext_0')
plt.plot(time, Fext_1, label='Fext_1')
# plt.plot(time, Fext_x, label='proposed_Fext_0')
# plt.plot(time, Fext_y, label='proposed_Fext_1')
plt.xlabel('Time (s)')
plt.ylabel('Fext')
plt.legend()

plt.tight_layout()
plt.show()
