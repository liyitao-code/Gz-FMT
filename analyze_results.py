#!/usr/bin/env python3
"""
蜕变测试实验数据批量分析工具

功能：
1. 统计测试通过、测试不通过（检测到错误）、测试执行失败的占比
2. 按蜕变测试方法分类统计
3. 筛选出"真正看起来是错误"的实验，提取关键信息

用法:
    python3 analyze_results.py /path/to/test_results/
    python3 analyze_results.py /path/to/test_results/ --export-errors /path/to/output_errors.csv
"""

import os
import sys
import json
import re
from collections import defaultdict
from pathlib import Path


def parse_result_file(result_path):
    """解析 metamorphic_test_result.txt 文件，提取关键信息"""
    info = {
        'test_type': None,
        'result': None,        # 'PASSED', 'FAILED', 'EXECUTION_FAILED'
        'model': None,
        'error_detail': {},
        'raw_content': '',
    }
    
    if not os.path.exists(result_path):
        return None
    
    with open(result_path, 'r') as f:
        content = f.read().strip()
    
    info['raw_content'] = content
    
    # 提取 Test Type
    m = re.search(r'^Test Type:\s*(.+)$', content, re.MULTILINE)
    if m:
        info['test_type'] = m.group(1).strip()
    
    # 判断结果类型
    if 'returned None' in content or 'failed to execute' in content:
        info['result'] = 'EXECUTION_FAILED'
    elif 'Result: PASSED' in content:
        info['result'] = 'PASSED'
    elif 'Result: FAILED' in content:
        info['result'] = 'FAILED'
    else:
        info['result'] = 'UNKNOWN'
    
    # 提取 Model
    m = re.search(r'^Model:\s*(.+)$', content, re.MULTILINE)
    if m:
        info['model'] = m.group(1).strip()
    
    # 提取错误详情（针对不同测试类型）
    # Force Additivity
    m = re.search(r'Position difference:\s*x=([\d.]+),\s*y=([\d.]+),\s*z=([\d.]+)', content)
    if m:
        info['error_detail']['pos_diff_x'] = float(m.group(1))
        info['error_detail']['pos_diff_y'] = float(m.group(2))
        info['error_detail']['pos_diff_z'] = float(m.group(3))
        info['error_detail']['total_pos_diff'] = (
            info['error_detail']['pos_diff_x']**2 + 
            info['error_detail']['pos_diff_y']**2 + 
            info['error_detail']['pos_diff_z']**2
        )**0.5
    
    # Time Scaling
    m = re.search(r'Real Time Factor 1:\s*([\d.]+)', content)
    if m:
        info['error_detail']['rtf1'] = float(m.group(1))
    m = re.search(r'Real Time Factor 2:\s*([\d.]+)', content)
    if m:
        info['error_detail']['rtf2'] = float(m.group(1))
    
    # Mass Scaling
    m = re.search(r'Mass Scale Factor.*?:\s*([\d.]+)', content)
    if m:
        info['error_detail']['mass_scale_factor'] = float(m.group(1))
    m = re.search(r'Relative Error:\s*([\d.]+)%', content)
    if m:
        info['error_detail']['relative_error'] = float(m.group(1))
    
    # Rewind Test
    m = re.search(r'Differences found:\s*(\d+)', content)
    if m:
        info['error_detail']['diff_count'] = int(m.group(1))
    
    # Motion Test - Error
    m = re.search(r'^Error:\s*x=([\d.]+),\s*y=([\d.]+),\s*z=([\d.]+)', content, re.MULTILINE)
    if m:
        info['error_detail']['error_x'] = float(m.group(1))
        info['error_detail']['error_y'] = float(m.group(2))
        info['error_detail']['error_z'] = float(m.group(3))
    
    # Force values
    m = re.search(r'Force F1:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', content)
    if m:
        info['error_detail']['force_f1'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    
    # Test Duration
    m = re.search(r'Test Duration.*?:\s*([\d.]+)\s*s', content)
    if m:
        info['error_detail']['test_duration'] = float(m.group(1))
    
    # 提取位置信息（各种格式）
    for label, key in [
        ('Initial Position', 'initial_pos'),
        ('Final Position', 'final_pos'),
        ('Expected Position', 'expected_pos'),
        (r'Position with F1\+F2', 'pos_f1f2'),
        ('Position with F_total', 'pos_ftotal'),
        ('Position with rtf=', 'pos_rtf'),
    ]:
        m = re.search(rf'{label}.*?\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', content)
        if m:
            info['error_detail'][key] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    
    return info


def normalize_test_type(test_type):
    """统一测试类型名称"""
    if test_type is None:
        return 'unknown'
    t = test_type.lower().strip()
    if 'motion' in t:
        return 'motion'
    elif 'force' in t or 'additivity' in t:
        return 'force_additivity'
    elif 'time' in t or 'scaling' in t and 'mass' not in t:
        return 'time_scaling'
    elif 'mass' in t:
        return 'mass_scaling'
    elif 'rewind' in t:
        return 'rewind'
    else:
        return test_type


def analyze_experiment_dir(exp_dir):
    """分析单个实验目录"""
    exp_info = {
        'dir': exp_dir,
        'dir_name': os.path.basename(exp_dir),
        'has_passed_tag': os.path.exists(os.path.join(exp_dir, 'METAMORPHIC_TEST_PASSED')),
        'has_failed_tag': os.path.exists(os.path.join(exp_dir, 'METAMORPHIC_TEST_FAILED')),
        'has_sdf': os.path.exists(os.path.join(exp_dir, 'a.sdf')),
        'has_log': os.path.exists(os.path.join(exp_dir, 'experiment_log.json')),
        'result_info': None,
        'sdf_world': None,
    }
    
    # 解析结果文件
    result_path = os.path.join(exp_dir, 'metamorphic_test_result.txt')
    if os.path.exists(result_path):
        exp_info['result_info'] = parse_result_file(result_path)
    
    # 提取 SDF 世界名
    sdf_path = os.path.join(exp_dir, 'a.sdf')
    if os.path.exists(sdf_path):
        try:
            with open(sdf_path, 'r') as f:
                sdf_content = f.read(2000)  # 只读前面部分
            m = re.search(r'<world\s+name="([^"]*)"', sdf_content)
            if m:
                exp_info['sdf_world'] = m.group(1)
        except:
            pass
    
    return exp_info


def classify_result(exp_info):
    """
    将实验分为三类：
    - PASSED: 蜕变测试通过
    - DETECTED_ERROR: 蜕变测试检测到错误（Result: FAILED）
    - EXECUTION_FAILED: 测试执行失败（returned None / 没有结果）
    """
    ri = exp_info.get('result_info')
    if ri is None:
        return 'EXECUTION_FAILED'
    
    result = ri.get('result')
    if result == 'PASSED':
        return 'PASSED'
    elif result == 'FAILED':
        return 'DETECTED_ERROR'
    elif result == 'EXECUTION_FAILED':
        return 'EXECUTION_FAILED'
    else:
        return 'EXECUTION_FAILED'


def is_interesting_error(exp_info):
    """
    判断一个 DETECTED_ERROR 是否是"看起来真的像 bug"的错误。
    
    筛选逻辑：
    - Force Additivity: 两种方式施加相同合力，位置差异 > 阈值 → 可能是物理引擎 bug
    - Time Scaling: RTF 不同但物理时间相同，位置应一致却不一致 → 可能是 RTF 相关 bug
    - Rewind: 回溯后状态与记录不符 → 可能是状态恢复 bug
    - Motion: 施力后位移与预期不符 → 这个测试本身的物理模型有简化，需谨慎
    - Mass Scaling: 位移比不符合质量反比 → 可能是惯性计算 bug
    """
    ri = exp_info.get('result_info')
    if ri is None:
        return False, "无结果信息"
    
    test_type = normalize_test_type(ri.get('test_type'))
    detail = ri.get('error_detail', {})
    
    if test_type == 'force_additivity':
        # Force additivity 错误几乎都是真 bug（物理引擎在同时/分别施力时结果不同）
        total_diff = detail.get('total_pos_diff', 0)
        if total_diff > 0.5:
            return True, f"力可加性违反: 位置差 {total_diff:.3f}m"
        return False, f"位置差太小: {total_diff:.3f}m"
    
    elif test_type == 'time_scaling':
        # Time scaling: RTF 不应影响物理结果
        total_diff = detail.get('total_pos_diff', 0)
        if total_diff > 0.5:
            rtf2 = detail.get('rtf2', 'N/A')
            return True, f"时间缩放不一致: 位置差 {total_diff:.3f}m, RTF2={rtf2}"
        return False, f"位置差太小: {total_diff:.3f}m"
    
    elif test_type == 'rewind':
        # Rewind: 状态回溯后不一致
        diff_count = detail.get('diff_count', 0)
        if diff_count > 0:
            return True, f"回溯后 {diff_count} 个模型状态不一致"
        return False, "无差异"
    
    elif test_type == 'motion':
        # Motion test 由于物理模型简化（力≠匀速），需要谨慎判断
        # 超大误差可能反映了真实问题
        err_x = detail.get('error_x', 0)
        err_y = detail.get('error_y', 0)
        err_z = detail.get('error_z', 0)
        total_err = (err_x**2 + err_y**2 + err_z**2)**0.5
        if total_err > 10.0:
            return True, f"运动测试大偏差: 总误差 {total_err:.3f}m"
        return False, f"运动偏差可能由物理模型简化导致: {total_err:.3f}m"
    
    elif test_type == 'mass_scaling':
        rel_err = detail.get('relative_error', 0)
        if rel_err > 20:
            return True, f"质量缩放比例违反: 相对误差 {rel_err:.1f}%"
        return False, f"相对误差在容忍范围内: {rel_err:.1f}%"
    
    else:
        return True, f"未知测试类型的错误: {test_type}"


def main():
    if len(sys.argv) < 2:
        print(f"用法: python3 {sys.argv[0]} <实验数据目录> [--export-errors <输出CSV路径>]")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    export_csv = None
    if '--export-errors' in sys.argv:
        idx = sys.argv.index('--export-errors')
        if idx + 1 < len(sys.argv):
            export_csv = sys.argv[idx + 1]
    
    if not os.path.isdir(data_dir):
        print(f"错误: {data_dir} 不是有效目录")
        sys.exit(1)
    
    # ==================== 1. 扫描所有实验目录 ====================
    print(f"正在扫描 {data_dir} ...")
    exp_dirs = sorted([
        os.path.join(data_dir, d) for d in os.listdir(data_dir) 
        if os.path.isdir(os.path.join(data_dir, d)) and d.startswith('_')
    ], key=lambda x: int(os.path.basename(x).lstrip('_')) if os.path.basename(x).lstrip('_').isdigit() else 0)
    
    total = len(exp_dirs)
    print(f"共找到 {total} 个实验目录\n")
    
    # ==================== 2. 分析每个实验 ====================
    all_exps = []
    for exp_dir in exp_dirs:
        exp_info = analyze_experiment_dir(exp_dir)
        all_exps.append(exp_info)
    
    # ==================== 3. 分类统计 ====================
    categories = defaultdict(list)
    for exp in all_exps:
        cat = classify_result(exp)
        categories[cat].append(exp)
    
    passed = categories['PASSED']
    detected = categories['DETECTED_ERROR']
    exec_failed = categories['EXECUTION_FAILED']
    
    print("=" * 70)
    print("                       总体统计")
    print("=" * 70)
    print(f"  总实验数:                         {total}")
    print(f"  ✅ 测试通过 (PASSED):             {len(passed):>6}  ({len(passed)/total*100:.1f}%)")
    print(f"  ❌ 检测到错误 (DETECTED_ERROR):   {len(detected):>6}  ({len(detected)/total*100:.1f}%)")
    print(f"  ⚠️  测试执行失败 (EXECUTION_FAILED): {len(exec_failed):>6}  ({len(exec_failed)/total*100:.1f}%)")
    print()
    
    # ==================== 4. 按测试方法分类 ====================
    print("=" * 70)
    print("                  按蜕变测试方法分类统计")
    print("=" * 70)
    
    # 汇总每种测试类型的通过/错误/失败
    type_stats = defaultdict(lambda: {'passed': 0, 'detected': 0, 'exec_failed': 0, 'total': 0})
    
    for exp in all_exps:
        ri = exp.get('result_info')
        if ri:
            tt = normalize_test_type(ri.get('test_type'))
        else:
            tt = 'unknown'
        cat = classify_result(exp)
        type_stats[tt]['total'] += 1
        if cat == 'PASSED':
            type_stats[tt]['passed'] += 1
        elif cat == 'DETECTED_ERROR':
            type_stats[tt]['detected'] += 1
        else:
            type_stats[tt]['exec_failed'] += 1
    
    header = f"{'测试类型':<20} {'总数':>6} {'通过':>6} {'检测到错误':>10} {'执行失败':>8} {'错误率':>8}"
    print(header)
    print("-" * len(header) + "---")
    for tt in sorted(type_stats.keys()):
        s = type_stats[tt]
        valid = s['passed'] + s['detected']
        err_rate = f"{s['detected']/valid*100:.1f}%" if valid > 0 else "N/A"
        print(f"  {tt:<18} {s['total']:>6} {s['passed']:>6} {s['detected']:>10} {s['exec_failed']:>8} {err_rate:>8}")
    print()
    
    # ==================== 5. 筛选真正的错误 ====================
    print("=" * 70)
    print("            筛选「真正看起来是 Bug」的实验")
    print("=" * 70)
    
    interesting_errors = []
    not_interesting = []
    
    for exp in detected:
        is_interesting, reason = is_interesting_error(exp)
        if is_interesting:
            interesting_errors.append((exp, reason))
        else:
            not_interesting.append((exp, reason))
    
    print(f"\n  检测到错误的实验总数: {len(detected)}")
    print(f"  🔴 疑似真实 Bug:      {len(interesting_errors)}")
    print(f"  ⚪ 不确定/误报:       {len(not_interesting)}")
    print()
    
    # 按测试类型分类统计有效错误
    interesting_by_type = defaultdict(list)
    for exp, reason in interesting_errors:
        ri = exp.get('result_info')
        tt = normalize_test_type(ri.get('test_type')) if ri else 'unknown'
        interesting_by_type[tt].append((exp, reason))
    
    print("  疑似 Bug 按测试类型分布:")
    for tt in sorted(interesting_by_type.keys()):
        items = interesting_by_type[tt]
        print(f"    {tt:<20} {len(items):>5} 个")
    print()
    
    # ==================== 6. 按测试类型展示典型错误 ====================
    print("=" * 70)
    print("                  各类型典型错误案例")
    print("=" * 70)
    
    for tt in sorted(interesting_by_type.keys()):
        items = interesting_by_type[tt]
        print(f"\n--- {tt} ({len(items)} 个疑似 Bug) ---")
        
        # 按照错误大小排序，展示前5个
        def sort_key(item):
            exp, reason = item
            ri = exp.get('result_info', {})
            detail = ri.get('error_detail', {}) if ri else {}
            return detail.get('total_pos_diff', 0) + detail.get('diff_count', 0)
        
        items_sorted = sorted(items, key=sort_key, reverse=True)
        
        for i, (exp, reason) in enumerate(items_sorted[:5]):
            ri = exp['result_info']
            print(f"\n  [{i+1}] 目录: {exp['dir_name']}")
            print(f"      SDF世界: {exp.get('sdf_world', 'N/A')}")
            print(f"      模型: {ri.get('model', 'N/A')}")
            print(f"      原因: {reason}")
            
            detail = ri.get('error_detail', {})
            if 'test_duration' in detail:
                print(f"      持续时间: {detail['test_duration']:.2f}s")
            if 'force_f1' in detail:
                f = detail['force_f1']
                print(f"      力F1: ({f[0]:.2f}, {f[1]:.2f}, {f[2]:.2f}) N")
        
        if len(items) > 5:
            print(f"\n  ... 还有 {len(items) - 5} 个类似错误")
    
    # ==================== 7. 按 SDF 世界/模型 聚合错误 ====================
    print()
    print("=" * 70)
    print("          按 SDF 世界聚合错误（辅助去重）")
    print("=" * 70)
    
    errors_by_world = defaultdict(list)
    for exp, reason in interesting_errors:
        world = exp.get('sdf_world', 'unknown')
        errors_by_world[world].append((exp, reason))
    
    print(f"\n  共涉及 {len(errors_by_world)} 个不同的 SDF 世界\n")
    
    # 按错误数量排序
    for world, items in sorted(errors_by_world.items(), key=lambda x: -len(x[1])):
        # 统计该世界下各类型的错误
        type_count = defaultdict(int)
        models_set = set()
        for exp, reason in items:
            ri = exp.get('result_info')
            tt = normalize_test_type(ri.get('test_type')) if ri else 'unknown'
            type_count[tt] += 1
            if ri and ri.get('model'):
                models_set.add(ri['model'])
        
        types_str = ", ".join(f"{t}:{c}" for t, c in sorted(type_count.items()))
        models_str = ", ".join(sorted(models_set)[:5])
        if len(models_set) > 5:
            models_str += f" 等{len(models_set)}个"
        world_display = world if world is not None else "(无世界名)"
        print(f"  {world_display:<35} 错误:{len(items):>4}  类型: {types_str}")
        print(f"  {'':35} 模型: {models_str}")
    
    # ==================== 8. 导出错误列表 ====================
    if export_csv:
        print(f"\n正在导出错误列表到 {export_csv} ...")
        import csv
        with open(export_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                '实验目录', 'SDF世界', '测试类型', '模型', '错误原因', 
                '位置差(m)', '持续时间(s)', '力F1', '实验路径'
            ])
            for exp, reason in interesting_errors:
                ri = exp.get('result_info', {})
                detail = ri.get('error_detail', {}) if ri else {}
                force_str = ""
                if 'force_f1' in detail:
                    f1 = detail['force_f1']
                    force_str = f"({f1[0]:.2f},{f1[1]:.2f},{f1[2]:.2f})"
                writer.writerow([
                    exp['dir_name'],
                    exp.get('sdf_world', ''),
                    normalize_test_type(ri.get('test_type')) if ri else '',
                    ri.get('model', '') if ri else '',
                    reason,
                    f"{detail.get('total_pos_diff', ''):.3f}" if 'total_pos_diff' in detail else '',
                    f"{detail.get('test_duration', ''):.2f}" if 'test_duration' in detail else '',
                    force_str,
                    exp['dir'],
                ])
        print(f"已导出 {len(interesting_errors)} 条错误记录到 {export_csv}")
    
    # ==================== 9. 总结建议 ====================
    print()
    print("=" * 70)
    print("                         分析总结")
    print("=" * 70)
    print(f"""
  📊 数据概况:
     - 共 {total} 次实验
     - {len(passed)} 次通过 ({len(passed)/total*100:.1f}%)
     - {len(detected)} 次检测到蜕变关系违反 ({len(detected)/total*100:.1f}%)
     - {len(exec_failed)} 次执行失败 ({len(exec_failed)/total*100:.1f}%)
  
  🔍 筛选结果:
     - {len(interesting_errors)} 个疑似真实 Bug
     - 涉及 {len(errors_by_world)} 个不同的 SDF 世界
  
  💡 建议下一步:
     1. 优先关注 force_additivity 类型的错误 — 物理引擎中力的叠加性是基本物理定律
     2. time_scaling 错误表明 RTF 参数影响了物理结果 — 这是确定性的 bug
     3. rewind 错误中，大的位置偏差值得关注，极小偏差可能是浮点精度问题
     4. motion 测试由于物理模型简化，误报率较高，需人工确认
     5. 对于同一个 SDF 世界出现多次相同类型错误的情况，可能是同一个 bug
""")


if __name__ == '__main__':
    main()

