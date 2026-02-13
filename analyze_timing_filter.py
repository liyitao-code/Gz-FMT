#!/usr/bin/env python3
# coding: utf-8
"""
自动筛选 timing 导致的假阳性实验

原理：
  对于 force_additivity 测试，如果误差是由 wall-clock timing 导致的，
  Test B 的位移应该是 Test A 位移的均匀缩放（所有轴按相同比例放大/缩小）。
  如果某个轴的行为异常（比如 z 轴突然下沉而其他轴正常），则可能是真实 bug。

  对于 motion / time_scaling / mass_scaling 测试，也有类似的判断逻辑。

分类规则：
  1. TIMING_LIKELY  - 误差模式符合 timing 问题（均匀缩放）
  2. SUSPICIOUS     - 误差模式不符合 timing，可能是真实 bug
  3. UNCLASSIFIED   - 数据不足以判断

用法：
    python3 analyze_timing_filter.py /path/to/test_dir
    python3 analyze_timing_filter.py /path/to/test_dir --verbose
    python3 analyze_timing_filter.py /path/to/test_dir --only-suspicious
"""

import os
import sys
import re
import math
import argparse
from collections import defaultdict


def parse_result_file(path):
    """解析 metamorphic_test_result.txt"""
    info = {}
    if not os.path.exists(path):
        return info
    with open(path) as f:
        text = f.read()

    info['raw'] = text

    m = re.search(r'^Test Type:\s*(.+)$', text, re.MULTILINE)
    if m:
        info['test_type'] = m.group(1).strip().lower().replace(' ', '_')

    m = re.search(r'^Result:\s*(\w+)', text, re.MULTILINE)
    if m:
        info['result'] = m.group(1)

    m = re.search(r'^Model:\s*(.+)$', text, re.MULTILINE)
    if m:
        info['model'] = m.group(1).strip()

    # Initial Position
    m = re.search(r'Initial Position:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['initial_pos'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    # Force Additivity
    m = re.search(r'Position with F1\+F2.*:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['pos_f1f2'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    m = re.search(r'Position with F_total.*:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['pos_ftotal'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    # Motion Test
    m = re.search(r'Position \(Run A\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['pos_run_a'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    m = re.search(r'Position \(Run B\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['pos_run_b'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    # Error magnitude and relative error
    m = re.search(r'Error magnitude:\s*([-\d.e+]+)', text)
    if m:
        info['error_magnitude'] = float(m.group(1))
    m = re.search(r'Relative error:\s*([-\d.e+]+)%', text)
    if m:
        info['relative_error'] = float(m.group(1))
    m = re.search(r'Max displacement:\s*([-\d.e+]+)', text)
    if m:
        info['max_displacement'] = float(m.group(1))

    # Test Duration
    m = re.search(r'Test Duration:\s*([-\d.e+]+)', text)
    if m:
        info['duration'] = float(m.group(1))

    # Forces
    m = re.search(r'F1:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['f1'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    m = re.search(r'F2:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['f2'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    m = re.search(r'F_total:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', text)
    if m:
        info['ftotal'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    return info


def check_has_special_plugins(sdf_path):
    """检查 SDF 中是否包含特殊插件（buoyancy, lift-drag, wheel-slip, hydrodynamics 等）"""
    if not os.path.exists(sdf_path):
        return []
    with open(sdf_path) as f:
        content = f.read()

    plugins_found = []
    plugin_patterns = [
        ('buoyancy', r'gz-sim-buoyancy-system'),
        ('lift_drag', r'gz-sim-lift-drag-system'),
        ('wheel_slip', r'gz-sim-wheel-slip-system'),
        ('hydrodynamics', r'gz-sim-hydrodynamics-system'),
        ('thruster', r'gz-sim-thruster-system'),
        ('wind_effects', r'gz-sim-wind-effects-system'),
    ]
    for name, pattern in plugin_patterns:
        if re.search(pattern, content):
            plugins_found.append(name)

    return plugins_found


def classify_force_additivity(info, sdf_path):
    """
    对 force_additivity 测试进行分类。

    timing 特征：
      - Test B 位移 = Test A 位移 × k (所有轴同比例缩放)
      - 因为 d ∝ t²，k = (t_B/t_A)²

    非 timing 特征（可能是真实 bug）：
      - 各轴缩放比例不一致
      - 某轴方向反转（如本来正变成负）
      - z 轴异常变化（可能是 buoyancy/ground 问题）
    """
    result = {
        'classification': 'UNCLASSIFIED',
        'reason': '',
        'details': {}
    }

    pos_a = info.get('pos_f1f2')
    pos_b = info.get('pos_ftotal')
    initial = info.get('initial_pos', (0, 0, 0))

    if not pos_a or not pos_b:
        result['reason'] = '缺少位置数据'
        return result

    # 计算位移（相对于初始位置）
    disp_a = [pos_a[i] - initial[i] for i in range(3)]
    disp_b = [pos_b[i] - initial[i] for i in range(3)]

    result['details']['disp_a'] = disp_a
    result['details']['disp_b'] = disp_b

    # 过滤掉位移过小的轴（噪声主导）
    MIN_DISP = 0.5  # 位移小于 0.5m 的轴不参与分析
    axis_names = ['x', 'y', 'z']
    valid_ratios = {}

    for i, ax in enumerate(axis_names):
        if abs(disp_a[i]) > MIN_DISP and abs(disp_b[i]) > MIN_DISP:
            ratio = disp_b[i] / disp_a[i]
            valid_ratios[ax] = ratio

    result['details']['ratios'] = valid_ratios

    if len(valid_ratios) < 2:
        result['reason'] = f'有效轴不足（{len(valid_ratios)} 个有位移 > {MIN_DISP}m）'
        return result

    # 检查方向反转
    for ax, ratio in valid_ratios.items():
        if ratio < 0:
            result['classification'] = 'SUSPICIOUS'
            result['reason'] = f'{ax}轴方向反转 (ratio={ratio:.3f})，可能是真实 bug'
            return result

    # 检查比例一致性
    ratio_values = list(valid_ratios.values())
    mean_ratio = sum(ratio_values) / len(ratio_values)
    max_dev = max(abs(r - mean_ratio) / mean_ratio for r in ratio_values) if mean_ratio != 0 else 999

    result['details']['mean_ratio'] = mean_ratio
    result['details']['max_ratio_deviation'] = max_dev

    # 检查特殊插件
    plugins = check_has_special_plugins(sdf_path)
    result['details']['special_plugins'] = plugins

    # 判断逻辑
    RATIO_THRESHOLD = 0.15  # 各轴比例偏差超过 15% 视为可疑

    if max_dev < RATIO_THRESHOLD:
        # 所有轴按近似相同比例缩放 → timing 问题
        result['classification'] = 'TIMING_LIKELY'
        result['reason'] = (f'各轴缩放一致 (mean={mean_ratio:.3f}, max_dev={max_dev:.1%})，'
                           f'符合 timing 特征')
    else:
        # 缩放比例不一致
        if plugins:
            result['classification'] = 'SUSPICIOUS'
            result['reason'] = (f'各轴缩放不一致 (max_dev={max_dev:.1%})，'
                               f'且含特殊插件: {", ".join(plugins)}')
        else:
            # 无特殊插件但比例不一致 —— 可能是地面接触、摩擦等非线性效应 + timing
            if max_dev < 0.4:
                result['classification'] = 'TIMING_LIKELY'
                result['reason'] = (f'各轴缩放略有差异 (max_dev={max_dev:.1%})，'
                                   f'无特殊插件，可能是地面接触 + timing')
            else:
                result['classification'] = 'SUSPICIOUS'
                result['reason'] = (f'各轴缩放差异较大 (max_dev={max_dev:.1%})，'
                                   f'即使无特殊插件也值得调查')

    return result


def classify_motion_test(info, sdf_path):
    """
    对 motion 测试进行分类。
    Motion test: 两次相同操作应得到相同结果。
    如果 Run A 和 Run B 的位移方向一致但大小不同 → timing
    """
    result = {
        'classification': 'UNCLASSIFIED',
        'reason': '',
        'details': {}
    }

    pos_a = info.get('pos_run_a')
    pos_b = info.get('pos_run_b')
    initial = info.get('initial_pos', (0, 0, 0))

    if not pos_a or not pos_b:
        result['reason'] = '缺少位置数据'
        return result

    disp_a = [pos_a[i] - initial[i] for i in range(3)]
    disp_b = [pos_b[i] - initial[i] for i in range(3)]

    result['details']['disp_a'] = disp_a
    result['details']['disp_b'] = disp_b

    MIN_DISP = 0.5
    axis_names = ['x', 'y', 'z']
    valid_ratios = {}
    for i, ax in enumerate(axis_names):
        if abs(disp_a[i]) > MIN_DISP and abs(disp_b[i]) > MIN_DISP:
            ratio = disp_b[i] / disp_a[i]
            valid_ratios[ax] = ratio

    result['details']['ratios'] = valid_ratios
    plugins = check_has_special_plugins(sdf_path)
    result['details']['special_plugins'] = plugins

    if len(valid_ratios) < 1:
        result['reason'] = '位移过小，无法判断'
        return result

    # 方向反转检查
    for ax, ratio in valid_ratios.items():
        if ratio < 0:
            result['classification'] = 'SUSPICIOUS'
            result['reason'] = f'{ax}轴方向反转 (ratio={ratio:.3f})'
            return result

    # Motion test 期望 ratio ≈ 1.0（两次应该一样）
    # timing 问题会导致 ratio != 1.0 但各轴一致
    ratio_values = list(valid_ratios.values())
    mean_ratio = sum(ratio_values) / len(ratio_values)

    if len(valid_ratios) >= 2:
        max_dev = max(abs(r - mean_ratio) / mean_ratio for r in ratio_values) if mean_ratio != 0 else 999
    else:
        max_dev = 0  # 只有一个轴，无法判断一致性

    result['details']['mean_ratio'] = mean_ratio
    result['details']['max_ratio_deviation'] = max_dev

    if max_dev < 0.15 or len(valid_ratios) < 2:
        result['classification'] = 'TIMING_LIKELY'
        result['reason'] = (f'各轴缩放一致 (mean={mean_ratio:.3f}, max_dev={max_dev:.1%})，'
                           f'符合 timing 特征')
    else:
        if plugins:
            result['classification'] = 'SUSPICIOUS'
            result['reason'] = (f'各轴缩放不一致 (max_dev={max_dev:.1%})，'
                               f'含特殊插件: {", ".join(plugins)}')
        elif max_dev < 0.4:
            result['classification'] = 'TIMING_LIKELY'
            result['reason'] = (f'各轴缩放略有差异 (max_dev={max_dev:.1%})，'
                               f'无特殊插件，可能是非线性效应 + timing')
        else:
            result['classification'] = 'SUSPICIOUS'
            result['reason'] = f'各轴缩放差异较大 (max_dev={max_dev:.1%})'

    return result


def classify_experiment(exp_dir):
    """分类单个实验"""
    result_path = os.path.join(exp_dir, 'metamorphic_test_result.txt')
    sdf_path = os.path.join(exp_dir, 'a.sdf')

    info = parse_result_file(result_path)
    if not info:
        return None

    if info.get('result') != 'FAILED':
        return None

    test_type = info.get('test_type', '')

    if 'force_additivity' in test_type:
        classification = classify_force_additivity(info, sdf_path)
    elif 'motion' in test_type:
        classification = classify_motion_test(info, sdf_path)
    elif 'time_scaling' in test_type or 'mass_scaling' in test_type:
        # time_scaling 和 mass_scaling 也受 timing 影响，使用通用判断
        # 简单处理：有特殊插件 → SUSPICIOUS，否则 → TIMING_LIKELY
        plugins = check_has_special_plugins(sdf_path)
        if plugins:
            classification = {
                'classification': 'SUSPICIOUS',
                'reason': f'含特殊插件: {", ".join(plugins)}，需要进一步验证',
                'details': {'special_plugins': plugins}
            }
        else:
            classification = {
                'classification': 'TIMING_LIKELY',
                'reason': f'无特殊插件，误差可能由 timing 导致',
                'details': {'special_plugins': []}
            }
    else:
        classification = {
            'classification': 'UNCLASSIFIED',
            'reason': f'未知测试类型: {test_type}',
            'details': {}
        }

    classification['test_type'] = test_type
    classification['model'] = info.get('model', 'N/A')
    classification['error_magnitude'] = info.get('error_magnitude')
    classification['relative_error'] = info.get('relative_error')
    classification['max_displacement'] = info.get('max_displacement')

    return classification


def main():
    parser = argparse.ArgumentParser(description='自动筛选 timing 导致的假阳性实验')
    parser.add_argument('test_dir', help='测试目录路径')
    parser.add_argument('--verbose', '-v', action='store_true', help='显示详细信息')
    parser.add_argument('--only-suspicious', action='store_true', help='只显示可疑（非 timing）实验')
    parser.add_argument('--only-timing', action='store_true', help='只显示 timing 导致的实验')
    parser.add_argument('--output', '-o', default=None, help='输出报告文件路径')
    args = parser.parse_args()

    test_dir = os.path.abspath(args.test_dir)

    # 收集所有实验目录
    exp_dirs = []
    for name in sorted(os.listdir(test_dir),
                       key=lambda x: int(x[1:]) if x.startswith('_') and x[1:].isdigit() else 0):
        if not name.startswith('_'):
            continue
        d = os.path.join(test_dir, name)
        if os.path.isdir(d) and os.path.exists(os.path.join(d, 'metamorphic_test_result.txt')):
            exp_dirs.append((name, d))

    print(f"扫描目录: {test_dir}")
    print(f"共 {len(exp_dirs)} 个实验目录\n")

    # 分类
    stats = defaultdict(list)
    all_results = []

    for name, d in exp_dirs:
        cls = classify_experiment(d)
        if cls is None:
            continue  # PASSED 或无数据
        cls['name'] = name
        cls['path'] = d
        all_results.append(cls)
        stats[cls['classification']].append(cls)

    # 汇总
    total_failed = len(all_results)
    n_timing = len(stats['TIMING_LIKELY'])
    n_suspicious = len(stats['SUSPICIOUS'])
    n_unclassified = len(stats['UNCLASSIFIED'])

    lines = []
    lines.append("=" * 75)
    lines.append("              实验结果分类报告 — Timing 筛选")
    lines.append("=" * 75)
    lines.append(f"  总实验数:      {len(exp_dirs)}")
    lines.append(f"  失败实验数:    {total_failed}")
    lines.append(f"")
    lines.append(f"  ┌─ 分类结果 ──────────────────────────────┐")
    lines.append(f"  │ TIMING_LIKELY  (timing 导致): {n_timing:4d}     │")
    lines.append(f"  │ SUSPICIOUS     (可能真实bug): {n_suspicious:4d}     │")
    lines.append(f"  │ UNCLASSIFIED   (无法判断):    {n_unclassified:4d}     │")
    lines.append(f"  └──────────────────────────────────────────┘")
    lines.append(f"")

    if n_timing > 0:
        lines.append(f"  如果将 TIMING_LIKELY 视为假阳性：")
        lines.append(f"    原始失败率: {total_failed}/{len(exp_dirs)} = "
                     f"{total_failed/len(exp_dirs)*100:.1f}%")
        adjusted = total_failed - n_timing
        lines.append(f"    调整后失败率: {adjusted}/{len(exp_dirs)} = "
                     f"{adjusted/len(exp_dirs)*100:.1f}%")
        lines.append(f"    减少假阳性: {n_timing} 个 ({n_timing/total_failed*100:.1f}% of failures)")
    lines.append("")

    # 按测试类型统计
    type_stats = defaultdict(lambda: defaultdict(int))
    for r in all_results:
        tt = r.get('test_type', 'unknown')
        type_stats[tt][r['classification']] += 1
        type_stats[tt]['total'] += 1

    lines.append("  ┌─ 按测试类型统计 ──────────────────────────────────────┐")
    for tt in sorted(type_stats.keys()):
        ts = type_stats[tt]
        lines.append(f"  │ {tt:25s}  total={ts['total']:3d}  "
                     f"timing={ts.get('TIMING_LIKELY',0):3d}  "
                     f"suspicious={ts.get('SUSPICIOUS',0):3d}  "
                     f"unclassified={ts.get('UNCLASSIFIED',0):3d}")
    lines.append("  └──────────────────────────────────────────────────────┘")
    lines.append("")

    # SUSPICIOUS 详细列表
    if stats['SUSPICIOUS']:
        lines.append("=" * 75)
        lines.append(f"  可疑实验详情 (SUSPICIOUS) — 共 {n_suspicious} 个")
        lines.append("=" * 75)
        for r in stats['SUSPICIOUS']:
            lines.append(f"")
            lines.append(f"  [{r['name']}]  test={r.get('test_type','?')}  "
                        f"model={r.get('model','?')}")
            lines.append(f"    误差: {r.get('error_magnitude', '?')} m  "
                        f"相对误差: {r.get('relative_error', '?')}%")
            lines.append(f"    原因: {r['reason']}")
            if args.verbose:
                details = r.get('details', {})
                if 'ratios' in details:
                    lines.append(f"    各轴比例: {details['ratios']}")
                if 'special_plugins' in details and details['special_plugins']:
                    lines.append(f"    特殊插件: {details['special_plugins']}")

    # TIMING_LIKELY 列表（简略）
    if not args.only_suspicious and stats['TIMING_LIKELY']:
        lines.append("")
        lines.append("=" * 75)
        lines.append(f"  Timing 导致的实验 (TIMING_LIKELY) — 共 {n_timing} 个")
        lines.append("=" * 75)
        if args.verbose:
            for r in stats['TIMING_LIKELY']:
                lines.append(f"  [{r['name']}]  test={r.get('test_type','?')}  "
                            f"err={r.get('relative_error', '?')}%  "
                            f"reason={r['reason'][:60]}")
        else:
            # 只列名字
            names = [r['name'] for r in stats['TIMING_LIKELY']]
            for i in range(0, len(names), 10):
                chunk = names[i:i+10]
                lines.append(f"  {', '.join(chunk)}")

    # UNCLASSIFIED 列表
    if stats['UNCLASSIFIED']:
        lines.append("")
        lines.append("=" * 75)
        lines.append(f"  无法分类的实验 (UNCLASSIFIED) — 共 {n_unclassified} 个")
        lines.append("=" * 75)
        for r in stats['UNCLASSIFIED']:
            lines.append(f"  [{r['name']}]  test={r.get('test_type','?')}  "
                        f"reason={r['reason']}")

    report = "\n".join(lines)
    print(report)

    # 保存报告
    output_dir = args.output or os.path.join(test_dir, 'analysis_results')
    os.makedirs(output_dir, exist_ok=True)

    # 保存文本报告
    report_path = os.path.join(output_dir, 'timing_filter_report.txt')
    with open(report_path, 'w') as f:
        f.write(report)
    print(f"\n文本报告已保存: {report_path}")

    # --- 提取 world_name 的辅助函数 ---
    def get_world_name(exp_path):
        log_path = os.path.join(exp_path, 'experiment_log.json')
        if not os.path.exists(log_path):
            return 'unknown'
        try:
            import json as _json
            with open(log_path) as f:
                log_data = _json.load(f)
            for entry in log_data:
                if entry.get('type') == 'command':
                    cmd = entry.get('command', '')
                    m2 = re.search(r'/world/([^/]+)/', cmd)
                    if m2:
                        return m2.group(1)
        except:
            pass
        return 'unknown'

    # --- 保存 CSV：所有失败实验（含分类） ---
    all_csv_path = os.path.join(output_dir, 'failed_experiments_classified.csv')
    with open(all_csv_path, 'w') as f:
        f.write('实验目录,测试类型,timing分类,模型名,世界名,误差量级,相对误差%,特殊插件,分类原因,实验路径\n')
        for r in all_results:
            world = get_world_name(r['path'])
            plugins = ','.join(r.get('details', {}).get('special_plugins', []))
            reason = r['reason'].replace(',', '，')  # 避免逗号干扰 CSV
            f.write(f"{r['name']},{r.get('test_type','')},{r['classification']},"
                    f"{r.get('model','')},{world},"
                    f"{r.get('error_magnitude','')},{r.get('relative_error','')},"
                    f"{plugins},{reason},{r['path']}\n")
    print(f"全量 CSV 已保存: {all_csv_path}")

    # --- 保存 CSV：timing 假阳性 ---
    timing_csv_path = os.path.join(output_dir, 'timing_false_positives.csv')
    with open(timing_csv_path, 'w') as f:
        f.write('实验目录,测试类型,模型名,世界名,误差量级,相对误差%,分类原因,实验路径\n')
        for r in stats['TIMING_LIKELY']:
            world = get_world_name(r['path'])
            reason = r['reason'].replace(',', '，')
            f.write(f"{r['name']},{r.get('test_type','')},{r.get('model','')},{world},"
                    f"{r.get('error_magnitude','')},{r.get('relative_error','')},"
                    f"{reason},{r['path']}\n")
    print(f"Timing 假阳性 CSV 已保存: {timing_csv_path}")

    # --- 保存 CSV：可疑实验（可能真实 bug） ---
    suspicious_csv_path = os.path.join(output_dir, 'suspicious_possible_bugs.csv')
    with open(suspicious_csv_path, 'w') as f:
        f.write('实验目录,测试类型,模型名,世界名,误差量级,相对误差%,特殊插件,分类原因,实验路径\n')
        for r in stats['SUSPICIOUS']:
            world = get_world_name(r['path'])
            plugins = ','.join(r.get('details', {}).get('special_plugins', []))
            reason = r['reason'].replace(',', '，')
            f.write(f"{r['name']},{r.get('test_type','')},{r.get('model','')},{world},"
                    f"{r.get('error_magnitude','')},{r.get('relative_error','')},"
                    f"{plugins},{reason},{r['path']}\n")
    print(f"可疑实验 CSV 已保存: {suspicious_csv_path}")


if __name__ == '__main__':
    main()
