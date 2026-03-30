#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PLF编队MPC数据分析与可视化工具
功能：
1. 读取CSV数据并解析为数组
2. 绘制关键指标图表（间距、速度、加速度等）
3. 导出为.mat格式供MATLAB使用
"""

import os
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat
from datetime import datetime

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class PLFDataAnalyzer:
    """PLF编队数据分析器"""

    def __init__(self, csv_file):
        """
        初始化分析器

        Args:
            csv_file: CSV文件路径
        """
        self.csv_file = csv_file
        self.data = {}
        self.arrays = {}

    def load_data(self):
        """读取CSV文件并解析为字典和数组格式"""
        with open(self.csv_file, 'r') as f:
            reader = csv.DictReader(f)

            # 初始化数据列表
            for key in reader.fieldnames:
                self.data[key] = []

            # 读取每一行
            for row in reader:
                for key in reader.fieldnames:
                    try:
                        self.data[key].append(float(row[key]))
                    except ValueError:
                        self.data[key].append(row[key])

        # 转换为numpy数组
        for key, values in self.data.items():
            try:
                self.arrays[key] = np.array(values, dtype=float)
            except ValueError:
                self.arrays[key] = np.array(values)

        print(f"数据加载完成: {len(self.arrays['t_s'])} 条记录")
        print(f"时间范围: {self.arrays['t_s'][0]:.2f}s - {self.arrays['t_s'][-1]:.2f}s")

        return self.arrays

    def get_array(self, field_name):
        """获取指定字段的数组"""
        return self.arrays.get(field_name, None)

    def print_summary(self):
        """打印数据摘要统计"""
        print("\n" + "="*60)
        print("数据摘要统计")
        print("="*60)

        summary_fields = [
            ('ego_v', '本车速度 (m/s)'),
            ('target_a', '目标加速度 (m/s²)'),
            ('gap_front', '前车净间距 (m)'),
            ('gap_leader', '领航车净间距 (m)'),
            ('distance_front', '前车中心距 (m)'),
            ('distance_leader', '领航车中心距 (m)'),
            ('ref_v', '参考速度 (m/s)'),
            ('front_v', '前车速度 (m/s)'),
            ('leader_v', '领航车速度 (m/s)'),
        ]

        for field, name in summary_fields:
            if field in self.arrays:
                arr = self.arrays[field]
                print(f"{name:20s}: min={arr.min():7.2f}, max={arr.max():7.2f}, "
                      f"mean={arr.mean():7.2f}, std={arr.std():7.2f}")

        # MPC状态统计
        if 'mpc_status' in self.arrays:
            status = self.arrays['mpc_status']
            print(f"\nMPC状态分布:")
            print(f"  状态1 (正常): {np.sum(status == 1)} 次 ({100*np.mean(status == 1):.1f}%)")
            print(f"  状态2 (回退): {np.sum(status == 2)} 次 ({100*np.mean(status == 2):.1f}%)")
            print(f"  状态3 (安全): {np.sum(status == 3)} 次 ({100*np.mean(status == 3):.1f}%)")

    def plot_all(self, save_dir=None):
        """
        绘制所有关键指标图表

        Args:
            save_dir: 图片保存目录，None则不保存
        """
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        t = self.arrays['t_s']

        # 图1: 速度对比
        self._plot_velocity(t, save_dir)

        # 图2: 间距变化
        self._plot_gap(t, save_dir)

        # 图3: 加速度与执行器
        self._plot_acceleration_actuator(t, save_dir)

        # 图4: MPC状态
        self._plot_mpc_status(t, save_dir)

        # 图5: 综合仪表板
        self._plot_dashboard(t, save_dir)

        plt.show()

    def _plot_velocity(self, t, save_dir):
        """绘制速度对比图"""
        fig, ax = plt.subplots(figsize=(12, 6))

        ax.plot(t, self.arrays['ego_v'], 'b-', linewidth=2, label='本车速度 ego_v')
        ax.plot(t, self.arrays['ref_v'], 'g--', linewidth=1.5, label='参考速度 ref_v')
        ax.plot(t, self.arrays['front_v'], 'r-', linewidth=1.5, label='前车速度 front_v')
        ax.plot(t, self.arrays['leader_v'], 'm:', linewidth=1.5, label='领航车速度 leader_v')

        ax.set_xlabel('时间 (s)', fontsize=12)
        ax.set_ylabel('速度 (m/s)', fontsize=12)
        ax.set_title('PLF编队速度跟踪', fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_xlim([t[0], t[-1]])

        if save_dir:
            fig.savefig(os.path.join(save_dir, 'velocity.png'), dpi=150, bbox_inches='tight')
            print(f"已保存: velocity.png")

    def _plot_gap(self, t, save_dir):
        """绘制间距变化图"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # 净间距
        ax1.plot(t, self.arrays['gap_front'], 'b-', linewidth=2, label='前车净间距 gap_front')
        ax1.plot(t, self.arrays['gap_leader'], 'r--', linewidth=1.5, label='领航车净间距 gap_leader')
        ax1.axhline(y=5.0, color='g', linestyle=':', linewidth=1, label='期望净间距 (5m)')
        ax1.axhline(y=1.3, color='orange', linestyle='--', linewidth=1, label='最小安全间距 (1.3m)')

        ax1.set_ylabel('净间距 (m)', fontsize=12)
        ax1.set_title('PLF编队间距控制', fontsize=14)
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)

        # 中心距
        ax2.plot(t, self.arrays['distance_front'], 'b-', linewidth=2, label='前车中心距 distance_front')
        ax2.plot(t, self.arrays['distance_leader'], 'r--', linewidth=1.5, label='领航车中心距 distance_leader')
        ax2.axhline(y=9.4, color='g', linestyle=':', linewidth=1, label='期望中心距 (9.4m = 5m + 4.4m)')

        ax2.set_xlabel('时间 (s)', fontsize=12)
        ax2.set_ylabel('中心距 (m)', fontsize=12)
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim([t[0], t[-1]])

        plt.tight_layout()

        if save_dir:
            fig.savefig(os.path.join(save_dir, 'gap.png'), dpi=150, bbox_inches='tight')
            print(f"已保存: gap.png")

    def _plot_acceleration_actuator(self, t, save_dir):
        """绘制加速度与执行器输出图"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # 加速度
        ax1.plot(t, self.arrays['target_a'], 'b-', linewidth=2, label='目标加速度')
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax1.axhline(y=2.0, color='g', linestyle=':', linewidth=1, label='加速度上限 (2.0)')
        ax1.axhline(y=-4.0, color='r', linestyle=':', linewidth=1, label='加速度下限 (-4.0)')

        ax1.set_ylabel('加速度 (m/s²)', fontsize=12)
        ax1.set_title('MPC控制输出', fontsize=14)
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)

        # 执行器
        ax2.plot(t, self.arrays['thr'], 'g-', linewidth=2, label='油门 (%)')
        ax2.plot(t, self.arrays['brk'], 'r-', linewidth=2, label='刹车 (%)')
        ax2.fill_between(t, 0, self.arrays['thr'], alpha=0.3, color='green')
        ax2.fill_between(t, 0, self.arrays['brk'], alpha=0.3, color='red')

        ax2.set_xlabel('时间 (s)', fontsize=12)
        ax2.set_ylabel('执行器输出 (%)', fontsize=12)
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim([t[0], t[-1]])
        ax2.set_ylim([0, 110])

        plt.tight_layout()

        if save_dir:
            fig.savefig(os.path.join(save_dir, 'acceleration_actuator.png'), dpi=150, bbox_inches='tight')
            print(f"已保存: acceleration_actuator.png")

    def _plot_mpc_status(self, t, save_dir):
        """绘制MPC状态图"""
        fig, ax = plt.subplots(figsize=(12, 4))

        status = self.arrays['mpc_status']

        # 用不同颜色标记状态
        colors = {1: 'green', 2: 'yellow', 3: 'red'}
        labels = {1: '正常求解', 2: '回退控制', 3: '安全模式'}

        for s in [1, 2, 3]:
            mask = status == s
            if np.any(mask):
                ax.scatter(t[mask], status[mask], c=colors[s], label=labels[s], s=20, alpha=0.7)

        ax.set_xlabel('时间 (s)', fontsize=12)
        ax.set_ylabel('MPC状态', fontsize=12)
        ax.set_title('MPC求解状态', fontsize=14)
        ax.set_yticks([1, 2, 3])
        ax.set_yticklabels(['正常', '回退', '安全'])
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_xlim([t[0], t[-1]])

        if save_dir:
            fig.savefig(os.path.join(save_dir, 'mpc_status.png'), dpi=150, bbox_inches='tight')
            print(f"已保存: mpc_status.png")

    def _plot_dashboard(self, t, save_dir):
        """绘制综合仪表板"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 左上: 速度
        ax1 = axes[0, 0]
        ax1.plot(t, self.arrays['ego_v'], 'b-', linewidth=2, label='本车')
        ax1.plot(t, self.arrays['ref_v'], 'g--', linewidth=1.5, label='参考')
        ax1.plot(t, self.arrays['leader_v'], 'm:', linewidth=1.5, label='领航车')
        ax1.set_ylabel('速度 (m/s)')
        ax1.set_title('速度跟踪')
        ax1.legend(loc='best', fontsize=8)
        ax1.grid(True, alpha=0.3)

        # 右上: 间距
        ax2 = axes[0, 1]
        ax2.plot(t, self.arrays['gap_front'], 'b-', linewidth=2, label='前车净间距')
        ax2.axhline(y=5.0, color='g', linestyle=':', label='期望 (5m)')
        ax2.axhline(y=1.3, color='r', linestyle='--', label='最小 (1.3m)')
        ax2.set_ylabel('净间距 (m)')
        ax2.set_title('间距控制')
        ax2.legend(loc='best', fontsize=8)
        ax2.grid(True, alpha=0.3)

        # 左下: 加速度
        ax3 = axes[1, 0]
        ax3.plot(t, self.arrays['target_a'], 'b-', linewidth=2)
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('加速度 (m/s²)')
        ax3.set_title('目标加速度')
        ax3.grid(True, alpha=0.3)

        # 右下: 执行器
        ax4 = axes[1, 1]
        ax4.plot(t, self.arrays['thr'], 'g-', linewidth=2, label='油门')
        ax4.plot(t, self.arrays['brk'], 'r-', linewidth=2, label='刹车')
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('执行器 (%)')
        ax4.set_title('执行器输出')
        ax4.legend(loc='best', fontsize=8)
        ax4.grid(True, alpha=0.3)

        for ax in axes.flat:
            ax.set_xlim([t[0], t[-1]])

        plt.suptitle('PLF编队MPC控制综合仪表板', fontsize=14, fontweight='bold')
        plt.tight_layout()

        if save_dir:
            fig.savefig(os.path.join(save_dir, 'dashboard.png'), dpi=150, bbox_inches='tight')
            print(f"已保存: dashboard.png")

    def export_to_mat(self, mat_file=None):
        """
        导出数据为.mat格式

        Args:
            mat_file: 输出文件路径，None则自动生成
        """
        if mat_file is None:
            base_name = os.path.splitext(self.csv_file)[0]
            mat_file = base_name + '.mat'

        # 准备数据字典 (MATLAB变量名不能以数字开头)
        mat_data = {
            # 时间
            't_s': self.arrays['t_s'],

            # 车辆ID和模式
            'car_id': self.arrays['id'],
            'platoon_mode': self.arrays['mode'],

            # 本车状态
            'ego_arc': self.arrays['ego_arc'],
            'ego_v': self.arrays['ego_v'],
            'ego_lat': self.arrays['lat'],
            'ego_lon': self.arrays['lon'],

            # 控制输出
            'target_a': self.arrays['target_a'],
            'throttle': self.arrays['thr'],
            'brake': self.arrays['brk'],

            # 间距 (净间距)
            'gap_front': self.arrays['gap_front'],
            'gap_leader': self.arrays['gap_leader'],

            # 距离 (中心距)
            'distance_front': self.arrays['distance_front'],
            'distance_leader': self.arrays['distance_leader'],

            # 速度
            'ref_v': self.arrays['ref_v'],
            'front_v': self.arrays['front_v'],
            'leader_v': self.arrays['leader_v'],

            # MPC状态
            'mpc_status': self.arrays['mpc_status'],

            # PLF参数
            'alpha': self.arrays['alpha'],
            'beta': self.arrays['beta'],
        }

        # 添加元数据
        mat_data['info'] = {
            'source_file': self.csv_file,
            'export_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'num_samples': len(self.arrays['t_s']),
            'duration_s': self.arrays['t_s'][-1] - self.arrays['t_s'][0],
        }

        savemat(mat_file, mat_data)
        print(f"\n数据已导出为MAT格式: {mat_file}")
        print(f"包含变量: {list(mat_data.keys())}")

        return mat_file


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="PLF编队MPC数据分析工具")
    parser.add_argument("csv_file", help="待分析的 CSV 文件路径")
    parser.add_argument(
        "--save-dir",
        default="figures",
        help="图表输出目录，默认保存到 ./figures",
    )
    args = parser.parse_args()

    # ========== 运行分析 ==========
    print("="*60)
    print("PLF编队MPC数据分析工具")
    print("="*60)

    # 创建分析器
    analyzer = PLFDataAnalyzer(args.csv_file)

    # 加载数据
    arrays = analyzer.load_data()

    # 打印摘要
    analyzer.print_summary()

    # 导出MAT文件
    # analyzer.export_to_mat()

    # 绘制图表
    print("\n正在生成图表...")
    analyzer.plot_all(save_dir=args.save_dir)

    # ========== 示例：如何使用数组 ==========
    print("\n" + "="*60)
    print("数组使用示例")
    print("="*60)
    print(f"时间数组 t_s: shape={arrays['t_s'].shape}, dtype={arrays['t_s'].dtype}")
    print(f"速度数组 ego_v: shape={arrays['ego_v'].shape}")
    print(f"前5个速度值: {arrays['ego_v'][:5]}")
    print(f"\n可用字段: {list(arrays.keys())}")


if __name__ == '__main__':
    main()
