#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蜕变测试实验数据批量分析脚本

功能：
1. 统计测试通过、测试不通过（蜕变关系违反）、测试失败（执行异常）的数量和占比
2. 按蜕变测试类型分类统计
3. 筛选出真正的错误（蜕变关系违反），提取可复现的实验设置

使用方法:
    python analyze_experiments.py <test_directory>
    python analyze_experiments.py /home/liyitao/workspace/meta/test_15

可选参数:
    --output <dir>      输出分析结果的目录（默认在test_directory下创建analysis_results）
    --verbose           显示详细信息
"""

import os
import sys
import json
import re
import argparse
import shutil
from collections import defaultdict
from datetime import datetime


# ============================================================
# 数据模型
# ============================================================

class ExperimentResult:
    """单个实验的结果"""
    def __init__(self, dir_path):
        self.dir_path = dir_path
        self.dir_name = os.path.basename(dir_path)
        self.idx = self._parse_idx()

        # 原始内容
        self.result_text = ""
        self.experiment_log = None
        self.sdf_content = ""

        # 解析出的字段
        self.test_type = None            # 蜕变测试类型（统一后的名称）
        self.test_type_raw = None        # 原始 Test Type 字符串
        self.result_status = None        # "PASSED" / "FAILED" / "ERROR" (执行异常)
        self.model_name = None           # 测试模型名
        self.world_name = None           # SDF 中的世界名
        self.sdf_source = None           # SDF 来源文件名

        # 数值结果
        self.error_info = ""             # 详细错误信息
        self.position_error = None       # 位置误差 (x, y, z)
        self.error_magnitude = None      # 误差总量

        # 分类标签
        self.error_category = None       # 错误分类标签
        self.is_real_bug = False          # 是否看起来像真正的 bug

    def _parse_idx(self):
        """从目录名提取索引号"""
        m = re.match(r'_(\d+)', self.dir_name)
        return int(m.group(1)) if m else -1


# ============================================================
# 文件解析
# ============================================================

def parse_result_file(filepath):
    """解析 metamorphic_test_result.txt"""
    if not os.path.exists(filepath):
        return None
    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        return f.read()


def parse_experiment_log(filepath):
    """解析 experiment_log.json"""
    if not os.path.exists(filepath):
        return None
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (json.JSONDecodeError, Exception):
        return None


def extract_world_name_from_sdf(sdf_path):
    """从 SDF 文件提取世界名"""
    if not os.path.exists(sdf_path):
        return None
    try:
        with open(sdf_path, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read(4096)  # 只读前 4K
        m = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']', content)
        return m.group(1) if m else None
    except Exception:
        return None


def extract_sdf_source(log_data):
    """从 experiment_log.json 中提取 SDF 来源文件名"""
    if not log_data:
        return None
    for entry in log_data:
        if entry.get("type") == "command" and entry.get("command_type") == "launch":
            cmd = entry.get("command", "")
            # 从 gz sim <sdf_path> 中提取
            m = re.match(r'gz sim\s+(\S+)', cmd)
            if m:
                return os.path.basename(m.group(1))
    return None


# ============================================================
# 测试类型统一化
# ============================================================

TYPE_NORMALIZATION = {
    "motion":               "motion",
    "Motion Test":          "motion",
    "rewind":               "rewind",
    "Rewind Test":          "rewind",
    "force_additivity":     "force_additivity",
    "Force Additivity Test": "force_additivity",
    "time_scaling":         "time_scaling",
    "Time Scaling Test":    "time_scaling",
    "mass_scaling":         "mass_scaling",
    "Mass Scaling Test":    "mass_scaling",
}

TYPE_DISPLAY_NAME = {
    "motion":           "匀速运动测试 (Motion)",
    "rewind":           "状态回溯测试 (Rewind)",
    "force_additivity": "力叠加测试 (Force Additivity)",
    "time_scaling":     "时间系数测试 (Time Scaling)",
    "mass_scaling":     "质量系数测试 (Mass Scaling)",
}


def normalize_test_type(raw_type):
    """将原始测试类型字符串统一为标准名称"""
    if raw_type is None:
        return "unknown"
    return TYPE_NORMALIZATION.get(raw_type.strip(), raw_type.strip())


# ============================================================
# 结果解析
# ============================================================

def parse_experiment(dir_path):
    """
    解析一个实验目录，返回 ExperimentResult
    """
    exp = ExperimentResult(dir_path)

    # 1. 读取结果文件
    result_path = os.path.join(dir_path, "metamorphic_test_result.txt")
    exp.result_text = parse_result_file(result_path) or ""

    # 2. 读取实验日志
    log_path = os.path.join(dir_path, "experiment_log.json")
    exp.experiment_log = parse_experiment_log(log_path)

    # 3. 读取 SDF 信息
    sdf_path = os.path.join(dir_path, "a.sdf")
    exp.world_name = extract_world_name_from_sdf(sdf_path)
    exp.sdf_source = extract_sdf_source(exp.experiment_log)

    # 4. 解析 Test Type
    m = re.search(r'^Test Type:\s*(.+)$', exp.result_text, re.MULTILINE)
    if m:
        exp.test_type_raw = m.group(1).strip()
        exp.test_type = normalize_test_type(exp.test_type_raw)
    else:
        # 尝试从 experiment_log 获取
        if exp.experiment_log:
            for entry in exp.experiment_log:
                if entry.get("type") == "test_info":
                    exp.test_type_raw = entry.get("test_type", "")
                    exp.test_type = normalize_test_type(exp.test_type_raw)
                    break

    # 5. 解析 Result
    m = re.search(r'^Result:\s*(\w+)', exp.result_text, re.MULTILINE)
    if m:
        exp.result_status = m.group(1)  # PASSED or FAILED
    elif "returned None" in exp.result_text or "failed to execute" in exp.result_text:
        exp.result_status = "ERROR"
    elif exp.result_text.strip() == "":
        exp.result_status = "NO_DATA"
    else:
        exp.result_status = "ERROR"

    # 6. 解析 Model
    m = re.search(r'^Model:\s*(.+)$', exp.result_text, re.MULTILINE)
    if m:
        exp.model_name = m.group(1).strip()

    # 7. 解析位置误差
    m = re.search(r'Position difference:\s*x=([\d.e+-]+),?\s*y=([\d.e+-]+),?\s*z=([\d.e+-]+)', exp.result_text)
    if not m:
        m = re.search(r'^Error:\s*x=([\d.e+-]+),?\s*y=([\d.e+-]+),?\s*z=([\d.e+-]+)', exp.result_text, re.MULTILINE)
    if m:
        try:
            exp.position_error = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            exp.error_magnitude = (exp.position_error[0]**2 + exp.position_error[1]**2 + exp.position_error[2]**2)**0.5
        except (ValueError, OverflowError):
            exp.position_error = None
            exp.error_magnitude = float('inf')

    # 8. 提取 Error Info 块
    idx = exp.result_text.find("Error Info:")
    if idx >= 0:
        exp.error_info = exp.result_text[idx:]
    elif "Differences found:" in exp.result_text:
        idx2 = exp.result_text.find("Differences found:")
        exp.error_info = exp.result_text[idx2:]

    # 9. 对 FAILED 结果进行错误分类
    if exp.result_status == "FAILED":
        exp.error_category = classify_error(exp)
        exp.is_real_bug = (exp.error_category not in [
            "model_not_moving",       # 模型没动，通常是被约束
            "model_exploded",         # 模型爆炸（数值不稳定）
            "trivial_error",          # 误差非常小，可能是浮点精度
        ])

    return exp


# ============================================================
# 错误分类器
# ============================================================

def classify_error(exp):
    """
    对 FAILED 的实验结果进行错误分类

    分类标签:
    - model_exploded:         模型位置爆炸到极大值（数值不稳定）
    - model_not_moving:       模型几乎没有移动
    - trivial_error:          误差很小（接近阈值边界）
    - significant_deviation:  显著偏差（蜕变关系明显违反）
    - rewind_state_mismatch:  回溯后状态不匹配
    - ratio_mismatch:         比例关系不符合预期
    """
    text = exp.result_text

    # 1. 数值爆炸检测（位置远超正常范围，如 > 1e10）
    if exp.error_magnitude is not None:
        try:
            if exp.error_magnitude > 1e10:
                return "model_exploded"
        except OverflowError:
            return "model_exploded"

    # 从文本中检查极端大数字
    all_numbers = re.findall(r'[\d.]+e\+(\d+)', text)
    for n in all_numbers:
        if int(n) > 10:
            return "model_exploded"

    # 2. Rewind 类型特殊处理
    if exp.test_type == "rewind":
        if "Differences found:" in text:
            # 检查差异大小
            diffs = re.findall(r'diff=\(([\d.e+-]+(?:,\s*[\d.e+-]+)*)\)', text)
            max_diff = 0
            for diff_str in diffs:
                vals = [abs(float(v.strip())) for v in diff_str.split(",")]
                max_diff = max(max_diff, max(vals))
            if max_diff < 1e-4:
                return "trivial_error"
            return "rewind_state_mismatch"
        return "rewind_state_mismatch"

    # 3. 模型没动检测
    if exp.test_type == "motion":
        # 检查 Final Position 是否与 Initial Position 几乎相同
        init_m = re.search(r'Initial Position:\s*\(([\d.e+-]+),\s*([\d.e+-]+),\s*([\d.e+-]+)\)', text)
        final_m = re.search(r'Final Position:\s*\(([\d.e+-]+),\s*([\d.e+-]+),\s*([\d.e+-]+)\)', text)
        if init_m and final_m:
            try:
                diff = sum((float(final_m.group(i)) - float(init_m.group(i)))**2 for i in range(1, 4))**0.5
                if diff < 0.01:
                    return "model_not_moving"
            except (ValueError, OverflowError):
                pass

    if exp.test_type in ("force_additivity", "time_scaling"):
        # 如果两组位置都几乎没动
        if exp.position_error and exp.error_magnitude is not None:
            if exp.error_magnitude < 0.001:
                return "model_not_moving"

    # 4. 比例关系检测（time_scaling, mass_scaling）
    if exp.test_type == "time_scaling":
        ratio_m = re.search(r'Observed.*?Ratio.*?:\s*([\d.e+-]+)', exp.error_info)
        expected_m = re.search(r'Expected.*?Ratio.*?:\s*([\d.e+-]+)', exp.error_info)
        if ratio_m and expected_m:
            return "ratio_mismatch"

    if exp.test_type == "mass_scaling":
        ratio_m = re.search(r'Observed.*?Ratio.*?:\s*([\d.e+-]+)', exp.error_info)
        expected_m = re.search(r'Expected.*?Ratio.*?:\s*([\d.e+-]+)', exp.error_info)
        if ratio_m and expected_m:
            return "ratio_mismatch"

    # 5. 误差大小评估
    if exp.error_magnitude is not None:
        if exp.error_magnitude < 0.01:
            return "trivial_error"
        return "significant_deviation"

    return "significant_deviation"


# ============================================================
# 统计报告
# ============================================================

def generate_report(experiments, output_dir, verbose=False):
    """
    生成完整的分析报告
    """
    os.makedirs(output_dir, exist_ok=True)

    total = len(experiments)
    if total == 0:
        print("没有找到任何实验数据")
        return

    # ---- 按状态分类 ----
    passed = [e for e in experiments if e.result_status == "PASSED"]
    failed = [e for e in experiments if e.result_status == "FAILED"]
    error  = [e for e in experiments if e.result_status == "ERROR"]
    no_data = [e for e in experiments if e.result_status == "NO_DATA"]

    # ---- 报告内容 ----
    lines = []
    lines.append("=" * 80)
    lines.append("蜕变测试实验数据分析报告")
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"数据目录: {experiments[0].dir_path.rsplit('/_', 1)[0] if experiments else 'N/A'}")
    lines.append("=" * 80)

    # ===== 第一部分：总体统计 =====
    lines.append("")
    lines.append("一、总体统计")
    lines.append("-" * 60)
    lines.append(f"  实验总数:              {total}")
    lines.append(f"  测试通过 (PASSED):     {len(passed):>6}  ({len(passed)/total*100:.1f}%)")
    lines.append(f"  蜕变违反 (FAILED):     {len(failed):>6}  ({len(failed)/total*100:.1f}%)")
    lines.append(f"  执行异常 (ERROR):      {len(error):>6}  ({len(error)/total*100:.1f}%)")
    if no_data:
        lines.append(f"  无数据 (NO_DATA):      {len(no_data):>6}  ({len(no_data)/total*100:.1f}%)")
    lines.append("")

    # 说明
    lines.append("  说明:")
    lines.append("    PASSED = 蜕变关系满足，测试通过（仿真器行为正确）")
    lines.append("    FAILED = 蜕变关系违反，检测到潜在错误")
    lines.append("    ERROR  = 测试执行中断，返回 None（环境问题/模型无法操作）")
    lines.append("")

    # ===== 第二部分：按蜕变测试类型统计 =====
    lines.append("二、按蜕变测试类型分类统计")
    lines.append("-" * 60)

    type_stats = defaultdict(lambda: {"PASSED": 0, "FAILED": 0, "ERROR": 0, "NO_DATA": 0, "total": 0})
    for e in experiments:
        t = e.test_type or "unknown"
        type_stats[t][e.result_status] += 1
        type_stats[t]["total"] += 1

    # 表头
    lines.append(f"  {'测试类型':<32} {'总计':>6} {'通过':>6} {'违反':>6} {'异常':>6} {'通过率':>8} {'违反率':>8}")
    lines.append("  " + "-" * 88)

    for t_key in sorted(type_stats.keys()):
        s = type_stats[t_key]
        display = TYPE_DISPLAY_NAME.get(t_key, t_key)
        pass_rate = s["PASSED"] / s["total"] * 100 if s["total"] > 0 else 0
        fail_rate = s["FAILED"] / s["total"] * 100 if s["total"] > 0 else 0
        lines.append(f"  {display:<32} {s['total']:>6} {s['PASSED']:>6} {s['FAILED']:>6} {s['ERROR']:>6} {pass_rate:>7.1f}% {fail_rate:>7.1f}%")
    lines.append("")

    # ===== 第三部分：FAILED 结果错误分类 =====
    lines.append("三、蜕变关系违反(FAILED)的错误分类")
    lines.append("-" * 60)

    error_cat_labels = {
        "model_exploded":        "数值爆炸（模型飞出/NaN）",
        "model_not_moving":      "模型未移动（被约束/摩擦）",
        "trivial_error":         "微小误差（接近阈值边界）",
        "significant_deviation": "显著偏差（蜕变关系明显违反）",
        "rewind_state_mismatch": "回溯状态不匹配",
        "ratio_mismatch":        "比例关系不匹配",
    }

    cat_stats = defaultdict(list)
    for e in failed:
        cat_stats[e.error_category].append(e)

    lines.append(f"  {'错误分类':<36} {'数量':>6} {'占违反总数':>10} {'是否为真实Bug':>14}")
    lines.append("  " + "-" * 76)
    for cat_key in sorted(cat_stats.keys()):
        exps_in_cat = cat_stats[cat_key]
        label = error_cat_labels.get(cat_key, cat_key)
        is_bug = "✓ 是" if cat_key not in ("model_not_moving", "model_exploded", "trivial_error") else "✗ 否/待定"
        lines.append(f"  {label:<36} {len(exps_in_cat):>6} {len(exps_in_cat)/len(failed)*100:>9.1f}% {is_bug:>14}")
    lines.append("")

    # 真实Bug数量
    real_bugs = [e for e in failed if e.is_real_bug]
    lines.append(f"  总计可能的真实Bug数量: {len(real_bugs)}")
    lines.append("")

    # ===== 第四部分：按测试类型 × 错误分类交叉统计 =====
    lines.append("四、测试类型 × 错误分类交叉统计")
    lines.append("-" * 60)

    cross_stats = defaultdict(lambda: defaultdict(int))
    for e in failed:
        cross_stats[e.test_type][e.error_category] += 1

    all_cats = sorted(set(e.error_category for e in failed if e.error_category))
    # 简短标签
    short_cat_labels = {
        "model_exploded":        "爆炸",
        "model_not_moving":      "未动",
        "trivial_error":         "微小",
        "significant_deviation": "显著偏差",
        "rewind_state_mismatch": "回溯不匹",
        "ratio_mismatch":        "比例不匹",
    }
    header = f"  {'测试类型':<28}"
    for c in all_cats:
        header += f" {short_cat_labels.get(c, c):>8}"
    header += f" {'合计':>8}"
    lines.append(header)
    lines.append("  " + "-" * (28 + 9 * (len(all_cats) + 1)))

    for t_key in sorted(cross_stats.keys()):
        display = TYPE_DISPLAY_NAME.get(t_key, t_key)
        row = f"  {display:<28}"
        row_total = 0
        for c in all_cats:
            val = cross_stats[t_key].get(c, 0)
            row += f" {val:>8}"
            row_total += val
        row += f" {row_total:>8}"
        lines.append(row)
    lines.append("")

    # ===== 第五部分：真实Bug清单 =====
    lines.append("五、筛选出的真实Bug实验清单（可复现）")
    lines.append("-" * 60)
    lines.append(f"  共 {len(real_bugs)} 个实验被判定为可能触发了真实Bug")
    lines.append("")

    # 按测试类型和错误分类分组
    grouped_bugs = defaultdict(list)
    for e in real_bugs:
        key = (e.test_type, e.error_category)
        grouped_bugs[key].append(e)

    for (t_type, cat), exps in sorted(grouped_bugs.items()):
        display_type = TYPE_DISPLAY_NAME.get(t_type, t_type)
        display_cat = error_cat_labels.get(cat, cat)
        lines.append(f"  [{display_type}] - [{display_cat}] : {len(exps)} 个实验")
        lines.append("  " + "~" * 50)

        # 最多显示前5个例子
        show_count = min(5, len(exps))
        for e in exps[:show_count]:
            lines.append(f"    实验目录: {e.dir_path}")
            if e.model_name:
                lines.append(f"    模型: {e.model_name}")
            if e.world_name:
                lines.append(f"    世界: {e.world_name}")
            if e.error_magnitude is not None:
                try:
                    if e.error_magnitude < 1e10:
                        lines.append(f"    误差量级: {e.error_magnitude:.4f} m")
                    else:
                        lines.append(f"    误差量级: {e.error_magnitude:.2e} m (异常大)")
                except (OverflowError, ValueError):
                    lines.append(f"    误差量级: 极大值")
            # 显示关键的错误描述（截取前200字符）
            if e.error_info:
                brief = e.error_info[:200].replace('\n', ' | ')
                lines.append(f"    错误摘要: {brief}")
            lines.append("")

        if len(exps) > show_count:
            lines.append(f"    ... 还有 {len(exps) - show_count} 个类似实验（完整列表见CSV文件）")
            lines.append("")

    # ===== 第六部分：ERROR（执行异常）分类 =====
    lines.append("六、执行异常 (ERROR) 分类统计")
    lines.append("-" * 60)

    error_type_stats = defaultdict(int)
    for e in error:
        error_type_stats[e.test_type or "unknown"] += 1
    for t_key in sorted(error_type_stats.keys()):
        display = TYPE_DISPLAY_NAME.get(t_key, t_key)
        lines.append(f"  {display:<32} {error_type_stats[t_key]:>6}")
    lines.append("")

    # ===== 第七部分：PASSED 按测试类型统计 =====
    lines.append("七、通过测试 (PASSED) 按类型统计")
    lines.append("-" * 60)
    passed_type_stats = defaultdict(int)
    for e in passed:
        passed_type_stats[e.test_type or "unknown"] += 1
    for t_key in sorted(passed_type_stats.keys()):
        display = TYPE_DISPLAY_NAME.get(t_key, t_key)
        lines.append(f"  {display:<32} {passed_type_stats[t_key]:>6}")
    lines.append("")

    # ===== 打印并保存报告 =====
    report_text = "\n".join(lines)
    print(report_text)

    report_path = os.path.join(output_dir, "analysis_report.txt")
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    print(f"\n报告已保存到: {report_path}")

    # ===== 保存 CSV：所有 FAILED 实验 =====
    csv_path = os.path.join(output_dir, "failed_experiments.csv")
    with open(csv_path, 'w', encoding='utf-8') as f:
        f.write("实验目录,测试类型,错误分类,是否真实Bug,模型名,世界名,误差量级,实验路径\n")
        for e in sorted(failed, key=lambda x: (x.test_type or "", x.error_category or "")):
            mag_str = ""
            if e.error_magnitude is not None:
                try:
                    mag_str = f"{e.error_magnitude:.6f}" if e.error_magnitude < 1e10 else f"{e.error_magnitude:.2e}"
                except (OverflowError, ValueError):
                    mag_str = "INF"
            f.write(f"{e.dir_name},{e.test_type},{e.error_category},{e.is_real_bug},{e.model_name or ''},{e.world_name or ''},{mag_str},{e.dir_path}\n")
    print(f"FAILED实验CSV已保存到: {csv_path}")

    # ===== 保存 CSV：真实 Bug 实验 =====
    real_bug_csv_path = os.path.join(output_dir, "real_bugs.csv")
    with open(real_bug_csv_path, 'w', encoding='utf-8') as f:
        f.write("实验目录,测试类型,错误分类,模型名,世界名,误差量级,实验路径\n")
        for e in sorted(real_bugs, key=lambda x: (x.test_type or "", x.error_category or "")):
            mag_str = ""
            if e.error_magnitude is not None:
                try:
                    mag_str = f"{e.error_magnitude:.6f}" if e.error_magnitude < 1e10 else f"{e.error_magnitude:.2e}"
                except (OverflowError, ValueError):
                    mag_str = "INF"
            f.write(f"{e.dir_name},{e.test_type},{e.error_category},{e.model_name or ''},{e.world_name or ''},{mag_str},{e.dir_path}\n")
    print(f"真实Bug实验CSV已保存到: {real_bug_csv_path}")

    # ===== 保存 JSON：真实 Bug 实验详情 =====
    real_bugs_json = []
    for e in real_bugs:
        entry = {
            "dir_name": e.dir_name,
            "dir_path": e.dir_path,
            "test_type": e.test_type,
            "error_category": e.error_category,
            "model_name": e.model_name,
            "world_name": e.world_name,
            "error_magnitude": None,
            "result_text": e.result_text,
        }
        if e.error_magnitude is not None:
            try:
                entry["error_magnitude"] = float(e.error_magnitude) if e.error_magnitude < 1e300 else None
            except (OverflowError, ValueError):
                entry["error_magnitude"] = None
        real_bugs_json.append(entry)

    json_path = os.path.join(output_dir, "real_bugs_detail.json")
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(real_bugs_json, f, indent=2, ensure_ascii=False)
    print(f"真实Bug详情JSON已保存到: {json_path}")

    # ===== 保存可复现实验列表 =====
    # 按 (test_type, error_category, world_name) 去重，每组保留一个代表性实验
    reproducible_path = os.path.join(output_dir, "reproducible_bugs.txt")
    unique_bugs = {}
    for e in real_bugs:
        key = (e.test_type, e.error_category, e.world_name or "unknown")
        if key not in unique_bugs:
            unique_bugs[key] = e
        else:
            # 保留误差较大的（更明显的 bug）
            existing = unique_bugs[key]
            if (e.error_magnitude is not None and existing.error_magnitude is not None
                    and e.error_magnitude > existing.error_magnitude):
                unique_bugs[key] = e

    with open(reproducible_path, 'w', encoding='utf-8') as f:
        f.write("可复现的独立Bug实验列表\n")
        f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"去重后独立Bug数: {len(unique_bugs)}\n")
        f.write("=" * 80 + "\n\n")
        f.write("复现方法: python replay_experiment.py <实验目录>\n\n")

        for (t_type, cat, world), e in sorted(unique_bugs.items()):
            display_type = TYPE_DISPLAY_NAME.get(t_type, t_type)
            display_cat = error_cat_labels.get(cat, cat)
            f.write(f"[{display_type}] [{display_cat}]\n")
            f.write(f"  实验目录: {e.dir_path}\n")
            f.write(f"  模型: {e.model_name or 'N/A'}\n")
            f.write(f"  世界: {e.world_name or 'N/A'}\n")
            if e.error_magnitude is not None:
                try:
                    if e.error_magnitude < 1e10:
                        f.write(f"  误差: {e.error_magnitude:.4f} m\n")
                    else:
                        f.write(f"  误差: {e.error_magnitude:.2e} m\n")
                except (OverflowError, ValueError):
                    f.write(f"  误差: 极大值\n")
            f.write(f"  复现命令: python replay_experiment.py {e.dir_path}\n")
            f.write("\n")

    print(f"可复现Bug列表已保存到: {reproducible_path}")

    return experiments


# ============================================================
# 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="蜕变测试实验数据批量分析")
    parser.add_argument("test_dir", help="实验数据目录 (如 /home/liyitao/workspace/meta/test_15)")
    parser.add_argument("--output", "-o", help="分析结果输出目录（默认在test_dir下创建 analysis_results）")
    parser.add_argument("--verbose", "-v", action="store_true", help="显示详细信息")
    args = parser.parse_args()

    test_dir = args.test_dir
    if not os.path.isdir(test_dir):
        print(f"Error: 目录不存在: {test_dir}")
        sys.exit(1)

    output_dir = args.output or os.path.join(test_dir, "analysis_results")

    # 扫描所有实验目录
    print(f"正在扫描实验目录: {test_dir}")
    exp_dirs = []
    for name in os.listdir(test_dir):
        if name.startswith("_") and os.path.isdir(os.path.join(test_dir, name)):
            exp_dirs.append(os.path.join(test_dir, name))

    exp_dirs.sort(key=lambda d: int(re.search(r'_(\d+)$', d).group(1)) if re.search(r'_(\d+)$', d) else 0)
    print(f"找到 {len(exp_dirs)} 个实验目录")

    # 解析所有实验
    print("正在解析实验数据...")
    experiments = []
    for i, d in enumerate(exp_dirs):
        exp = parse_experiment(d)
        experiments.append(exp)
        if args.verbose and (i + 1) % 500 == 0:
            print(f"  已解析 {i+1}/{len(exp_dirs)} ...")

    print(f"解析完成，共 {len(experiments)} 个实验")
    print()

    # 生成报告
    generate_report(experiments, output_dir, args.verbose)


if __name__ == "__main__":
    main()
