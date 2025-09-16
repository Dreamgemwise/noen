#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lowpass_all_to_file.py  ——  给 output_data.csv 全列做低通滤波并写新文件

  1. 读取 output_data.csv
  2. 自动估算采样频率 fs
  3. 对除 'Time' 之外的所有列做 Butterworth 低通 (4th, 20 Hz)
  4. 把滤波后的数据写 output_data_filtered.csv（列名保持不变）
"""

import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
from pathlib import Path
import sys

# ----------------------------------------------------------------------
# 参数 —— 如需修改可直接改这里
# ----------------------------------------------------------------------
input_path  = Path(__file__).with_name("output_data.csv")          # 源文件
output_path = Path(__file__).with_name("output_data_filtered.csv") # 目标文件
cutoff      = 30.0   # 低通过零截止频率 (Hz)
order       = 4      # Butterworth 阶数

# ----------------------------------------------------------------------
# Butterworth 低通函数
# ----------------------------------------------------------------------
def butter_lowpass(data, fs, fc, n=4):
    nyq = 0.5 * fs
    b, a = butter(n, fc / nyq, btype="low")
    return filtfilt(b, a, data)

# ----------------------------------------------------------------------
# 1. 读取数据
# ----------------------------------------------------------------------
try:
    df = pd.read_csv(input_path)
except FileNotFoundError:
    sys.exit(f"[Error] 找不到文件: {input_path}")

if "Time" not in df.columns:
    sys.exit("[Error] 数据必须包含 'Time' 列以计算采样频率")

# ----------------------------------------------------------------------
# 2. 采样频率
# ----------------------------------------------------------------------
dt = np.diff(df["Time"]).mean()
fs = 1.0 / dt
print(f"[Info] 采样频率 ≈ {fs:.2f} Hz (Δt≈{dt*1e3:.2f} ms)")

# ----------------------------------------------------------------------
# 3. 对所有数值列做滤波（Time 列除外）
# ----------------------------------------------------------------------
numeric_cols = df.select_dtypes(include=[np.number]).columns.tolist()
cols_to_filter = [c for c in numeric_cols if c != "Time"]

for col in cols_to_filter:
    df[col] = butter_lowpass(df[col].values, fs, cutoff, order)
    print(f"[Info] 列 {col} 已完成低通滤波")

# ----------------------------------------------------------------------
# 4. 写出结果
# ----------------------------------------------------------------------
df.to_csv(output_path, index=False)
print(f"[Done] 过滤后数据已写入 {output_path.resolve()}")




# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# lowpass_to_file.py  ——  将指定列做 Butterworth 低通滤波并保存结果

# 步骤:
#   1. 读取 output_data.csv
#   2. 自动估算采样频率
#   3. 4th-order Butterworth 低通 (默认 20 Hz)
#   4. 生成 *_lp 结尾的新列
#   5. 保存到 output_data_filtered.csv
# """

# import pandas as pd
# import numpy as np
# from scipy import signal
# from pathlib import Path

# # ----------------------------------------------------------------------
# # 用户可调参数
# # ----------------------------------------------------------------------
# input_path  = Path(__file__).with_name("output_data.csv")        # 原始数据
# output_path = Path(__file__).with_name("output_data_filtered.csv")  # 结果文件
# sensor_cols = ["Fext_0", "Fext_1", "tau_0", "tau_4"]            # 要滤波的列
# cutoff      = 0.50   # Hz   —— 截止频率
# order       = 4      # 阶数 —— 1~6 常用

# # ----------------------------------------------------------------------
# # 低通滤波函数
# # ----------------------------------------------------------------------
# def butter_lowpass(data, fs, fc, n=4):
#     """零相移 Butterworth 低通"""
#     nyq = 0.5 * fs
#     b, a = signal.butter(n, fc / nyq, btype="low")
#     return signal.filtfilt(b, a, data)

# # ----------------------------------------------------------------------
# # 1. 读取 & 采样频率
# # ----------------------------------------------------------------------
# df = pd.read_csv(input_path)
# if "Time" not in df.columns:
#     raise KeyError("缺少 'Time' 列，无法计算采样频率")

# dt = np.diff(df["Time"]).mean()
# fs = 1.0 / dt
# print(f"[Info] 采样频率 ≈ {fs:.2f} Hz（Δt≈{dt*1e3:.2f} ms）")

# # ----------------------------------------------------------------------
# # 2. 滤波指定列
# # ----------------------------------------------------------------------
# for col in sensor_cols:
#     if col not in df.columns:
#         print(f"[Warn] 列 {col} 不存在，跳过")
#         continue
#     df[f"{col}_lp"] = butter_lowpass(df[col].values, fs, cutoff, order)
#     print(f"[Info] 列 {col} 已滤波，生成 {col}_lp")

# # ----------------------------------------------------------------------
# # 3. 保存结果
# # ----------------------------------------------------------------------
# df.to_csv(output_path, index=False)
# print(f"[Done] 过滤后数据已保存至 {output_path.resolve()}")
