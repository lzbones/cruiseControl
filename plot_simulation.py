#!/Users/qingxu/.ai-env/bin/python3
# -*- coding: utf-8 -*-

import os
import csv
import subprocess
import matplotlib
matplotlib.use('Agg')  # 显式使用 Agg 无 GUI 后端，防止在后台运行或无显示器终端中发生阻塞
import matplotlib.pyplot as plt

def main():
    print("========= Cruise Control 仿真结果可视化工具 =========")
    
    # 1. 确保 C++ 仿真数据最新，自动运行编译好的仿真文件
    cpp_executable = "./bin/test_integration"
    if os.path.exists(cpp_executable):
        print("正在运行 C++ 联合物理仿真以生成最新数据...")
        try:
            # 运行 C++ 仿真，静默输出
            result = subprocess.run([cpp_executable], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if result.returncode == 0:
                print("[INFO] C++ 仿真数据生成成功。")
            else:
                print(f"[WARNING] C++ 仿真程序异常退出，错误码: {result.returncode}")
        except Exception as e:
            print(f"[ERROR] 运行 C++ 仿真程序失败: {e}")
    else:
        print("[WARNING] 未找到编译好的 bin/test_integration 程序，将直接尝试读取已有 CSV 文件。")

    csv_filename = "output/simulation_results.csv"
    if not os.path.exists(csv_filename):
        print(f"[ERROR] 未找到仿真结果数据文件 {csv_filename}！请先编译并运行仿真程序。")
        return

    # 2. 读取 CSV 数据
    times = []
    speeds = []
    target_speeds = []
    virtual_targets = []
    step_indices = []
    torques = []
    slopes = []
    states = []

    with open(csv_filename, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['Time']))
            speeds.append(float(row['Speed']))
            target_speeds.append(float(row['TargetSpeed']))
            virtual_targets.append(float(row['VirtualTargetSpeed']))
            step_indices.append(int(row['StepIndex']))
            torques.append(float(row['Torque']))
            slopes.append(float(row['Slope']))
            states.append(int(row['State']))

    print(f"[INFO] 成功读取了 {len(times)} 个时间步的数据点。")

    # 3. 设置美观的绘图风格
    plt.rcParams['font.sans-serif'] = ['Heiti TC', 'PingFang SC', 'SimHei', 'sans-serif']  # 支持中文字体
    plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

    # 创建双子图布局
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("定速巡航（CC）系统联合物理仿真曲线 (SPPVT 算法)", fontsize=14, fontweight='bold', color='#2c3e50')

    # --- 子图 1: 车速控制曲线 ---
    ax1.plot(times, speeds, label='实际车速 $v(t)$', color='#3498db', linewidth=2.5)
    ax1.plot(times, target_speeds, label='设定目标 $V_t$', color='#e74c3c', linestyle='--', linewidth=1.5)
    ax1.plot(times, virtual_targets, label='SPPVT 虚拟目标 $V_i$', color='#2ecc71', drawstyle='steps-post', linewidth=1.8, alpha=0.9)
    
    # 标注重要事件时间轴
    ax1.axvline(x=1.0, color='#9b59b6', linestyle=':', label='1.0s: 巡航激活 (Resume)', alpha=0.8)
    ax1.axvline(x=12.0, color='#d35400', linestyle=':', label='12.0s: 引入 5° 陡坡', alpha=0.8)
    
    ax1.set_ylabel("车速 (m/s)", fontsize=11, fontweight='bold', color='#2c3e50')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower right', framealpha=0.9, fontsize=9)
    ax1.set_title("车速收敛与自适应虚拟目标阶跃调整", fontsize=11, fontweight='bold', pad=8)
    
    # 填充车速误差区间
    ax1.fill_between(times, speeds, target_speeds, where=[(t >= 1.0) for t in times], color='#3498db', alpha=0.08, label='车速偏差')

    # --- 子图 2: 执行机构输出与道路阻力 ---
    # 左轴: 输出扭矩
    ax2.plot(times, torques, label='控制输出扭矩 $T_i(t)$', color='#f39c12', linewidth=2.0)
    ax2.set_ylabel("电机输出扭矩 (Nm)", fontsize=11, fontweight='bold', color='#f39c12')
    ax2.tick_params(axis='y', labelcolor='#f39c12')
    ax2.set_ylim(-10.0, 160.0)
    
    # 右轴: 坡度
    ax2_right = ax2.twinx()
    ax2_right.fill_between(times, 0, slopes, color='#7f8c8d', alpha=0.2, label='坡度')
    ax2_right.plot(times, slopes, color='#7f8c8d', linestyle='-', linewidth=1.0)
    ax2_right.set_ylabel("道路坡度 (度)", fontsize=11, fontweight='bold', color='#7f8c8d')
    ax2_right.tick_params(axis='y', labelcolor='#7f8c8d')
    ax2_right.set_ylim(-1.0, 10.0)

    # 组合双轴图例
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_right.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left', framealpha=0.9, fontsize=9)
    
    ax2.set_xlabel("时间 (秒)", fontsize=11, fontweight='bold', color='#2c3e50')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.set_title("执行机构输出扭矩与坡度阻力扰动", fontsize=11, fontweight='bold', pad=8)

    plt.tight_layout()
    
    # 保存结果到本地
    plot_filename = "output/simulation_plot.png"
    plt.savefig(plot_filename, dpi=300)
    print(f"[INFO] 成功保存可视化曲线图至: {plot_filename}")

    # 4. 尝试显示窗口 (如果当前有 GUI 显示环境)
    try:
        plt.show()
    except Exception as e:
        print(f"[INFO] 在当前无 GUI 终端环境下跳过窗口显示。")

if __name__ == '__main__':
    main()
