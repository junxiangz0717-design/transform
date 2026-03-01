#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import numpy as np # 引入numpy以防万一

# 固定读取这个文件
FILE_PATH = '/home/robotlab/universal_debug.csv' 

def auto_render():
    print(f"=== [AutoPlot] 正在处理: {FILE_PATH} ===")
    
    if not os.path.exists(FILE_PATH):
        print("错误：文件不存在，无法绘图。")
        return

    try:
        # 读取数据 (自动去除首尾空格)
        df = pd.read_csv(FILE_PATH, skipinitialspace=True)
        
        # [清洗] 确保列名没问题
        df.columns = [c.strip() for c in df.columns]

        # 获取所有唯一的“图表分组”
        if 'plot_group' not in df.columns:
            print("数据格式错误：找不到 plot_group 列")
            return
            
        groups = df['plot_group'].unique()
        n_groups = len(groups)
        
        if n_groups == 0:
            print("警告：数据为空，未检测到任何绘图组。")
            return

        print(f">> 检测到 {n_groups} 个图表分组: {groups}")

        # 动态创建画布：每个组一张子图
        # 如果组太多，增加画布高度
        fig, axes = plt.subplots(n_groups, 1, figsize=(12, 4 * n_groups), sharex=True)
        
        # 处理只有1个组的情况（matplotlib行为差异，确保axes是列表）
        if n_groups == 1:
            axes = [axes]

        # 遍历每个分组进行绘图
        for i, group_name in enumerate(groups):
            ax = axes[i]
            
            # 筛选出当前组的数据
            group_data = df[df['plot_group'] == group_name]
            
            # 在这个组里，有哪些线条？
            series_list = group_data['series_name'].unique()
            
            for series in series_list:
                # 筛选线条数据
                line_data = group_data[group_data['series_name'] == series]
                # 排序（按时间）
                line_data = line_data.sort_values('timestamp')
                
                # [关键修复] 添加 .values 以解决 Multi-dimensional indexing 报错
                ax.plot(line_data['timestamp'].values, line_data['value'].values, label=series, linewidth=1.5, alpha=0.8)
            
            ax.set_title(f"Group: {group_name}", fontsize=12, fontweight='bold', pad=10)
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.legend(loc='upper right')
            
            if i == n_groups - 1:
                ax.set_xlabel("Time (s)")

        plt.tight_layout()
        output_file = FILE_PATH.replace('.csv', '_report.png')
        plt.savefig(output_file, dpi=150)
        print(f"\n✅ 全自动绘图完成！图片已保存至:\n{output_file}")
        
        # 可选：如果是在本地桌面环境，可以取消注释下面这行直接弹窗
        # plt.show()

    except Exception as e:
        print(f"❌ 绘图失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    auto_render()