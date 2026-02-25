#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_22 初筛脚本：从大量蜕变测试结果中筛出「可能测出 Gazebo 问题」的实验编号。

排除规则（已知非 Gazebo 根因）：
- 世界名 contact_extra_data / contact_sensor（SDF 初始穿地等缺陷）
- SDF 中 real_time_factor=0（激活阶段不可控，易误报）
- SDF 中缺少 gz-sim-physics-system（无物理引擎，行为非常规）

保留规则（优先人工复核）：
- Determinism FAILED：位置差在 0.01m ~ 500m（排除爆炸级与浮点级）
- Force Removal FAILED：撤力后速度跳变，疑似 wrench/clear 或物理 bug
- Zero-Input Stability FAILED：无力漂移（排除上述坏世界）
- Temporal Monotonicity FAILED：位移突变/非单调
- Force Isolation FAILED：若有则保留

用法:
    python3 screen_bug_candidates.py /home/liyitao/workspace/meta/test_22
    python3 screen_bug_candidates.py /home/liyitao/workspace/meta/test_22 --out candidates.csv
    python3 screen_bug_candidates.py /home/liyitao/workspace/meta/test_22 --top 200
"""

import os
import re
import sys
import argparse
from pathlib import Path


# 已知「坏世界」：SDF 缺陷导致爆炸/混沌，非 Gazebo 核心 bug
EXCLUDED_WORLDS = {"contact_extra_data", "contact_sensor"}

# Determinism：位置差在此范围内视为「可能非确定性」，超出为爆炸或浮点噪声
DETERMINISM_ERROR_MIN = 0.01   # 小于此视为浮点/无意义
DETERMINISM_ERROR_MAX = 500.0  # 大于此视为 SDF 爆炸


def extract_world_name(sdf_path):
    if not sdf_path or not os.path.exists(sdf_path):
        return None
    try:
        with open(sdf_path, "r", encoding="utf-8", errors="replace") as f:
            content = f.read(8192)
        m = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']', content)
        return m.group(1).strip() if m else None
    except Exception:
        return None


def sdf_has_rtf_zero(sdf_path):
    if not sdf_path or not os.path.exists(sdf_path):
        return False
    try:
        with open(sdf_path, "r", encoding="utf-8", errors="replace") as f:
            content = f.read(8192)
        return bool(re.search(r'<real_time_factor>\s*0\s*</real_time_factor>', content))
    except Exception:
        return False


def sdf_has_physics_plugin(sdf_path):
    if not sdf_path or not os.path.exists(sdf_path):
        return False
    try:
        with open(sdf_path, "r", encoding="utf-8", errors="replace") as f:
            content = f.read(8192)
        return "gz-sim-physics-system" in content
    except Exception:
        return False


def parse_result_file(result_path):
    """解析 metamorphic_test_result.txt，返回 dict。"""
    info = {
        "test_type": None,
        "result": None,
        "model": None,
        "error_magnitude": None,
        "raw": "",
    }
    if not result_path or not os.path.exists(result_path):
        return info
    try:
        with open(result_path, "r", encoding="utf-8", errors="replace") as f:
            content = f.read()
    except Exception:
        return info
    info["raw"] = content

    m = re.search(r"^Test Type:\s*(.+)$", content, re.MULTILINE)
    if m:
        info["test_type"] = m.group(1).strip()

    if "Result: PASSED" in content:
        info["result"] = "PASSED"
    elif "Result: FAILED" in content:
        info["result"] = "FAILED"
    elif "returned None" in content or "failed to execute" in content:
        info["result"] = "ERROR"
    else:
        info["result"] = "UNKNOWN"

    m = re.search(r"^Model:\s*(.+)$", content, re.MULTILINE)
    if m:
        info["model"] = m.group(1).strip()

    # Error magnitude (Determinism, 或通用位置差)
    m = re.search(r"Error magnitude:\s*([\d.e+-]+)", content)
    if m:
        try:
            info["error_magnitude"] = float(m.group(1))
        except ValueError:
            pass

    # Force Removal: 速度比 (coast/force)
    m_force = re.search(r"Velocity \(force phase avg\):\s*([\d.e+-]+)", content)
    m_coast = re.search(r"Velocity \(coast 1st half\):\s*([\d.e+-]+)", content)
    if m_force and m_coast:
        try:
            vf, vc = float(m_force.group(1)), float(m_coast.group(1))
            if vf > 1e-6:
                info["velocity_jump_ratio"] = vc / vf
        except ValueError:
            pass

    # Zero-Input: 水平漂移
    m = re.search(r"Horizontal drift:\s*([\d.e+-]+)\s*m", content)
    if m:
        try:
            info["horizontal_drift"] = float(m.group(1))
        except ValueError:
            pass

    return info


def normalize_test_type(raw):
    """统一测试类型名称便于分组。"""
    if not raw:
        return "unknown"
    raw = raw.strip()
    if "Determinism" in raw:
        return "determinism"
    if "Force Removal" in raw or raw == "force_removal":
        return "force_removal"
    if "Zero-Input" in raw or "zero_input" in raw:
        return "zero_input_stability"
    if "Temporal Monotonicity" in raw or raw == "temporal_monotonicity":
        return "temporal_monotonicity"
    if "force_isolation" in raw or "Force Isolation" in raw:
        return "force_isolation"
    return raw


def should_exclude(world_name, sdf_path):
    """是否因已知 SDF/世界问题排除该实验。"""
    if world_name and world_name in EXCLUDED_WORLDS:
        return True, "excluded_world"
    if sdf_path and sdf_has_rtf_zero(sdf_path):
        return True, "rtf_zero"
    if sdf_path and not sdf_has_physics_plugin(sdf_path):
        return True, "no_physics_plugin"
    return False, None


def is_candidate(exp_id, info, world_name, sdf_path):
    """
    判断是否为「可能测出 Gazebo 问题」的候选。
    返回 (is_candidate: bool, reason: str)。
    """
    if info["result"] != "FAILED":
        return False, ""

    excluded, excl_reason = should_exclude(world_name, sdf_path)
    if excluded:
        return False, f"excluded({excl_reason})"

    test_type = normalize_test_type(info.get("test_type") or "")

    if test_type == "determinism":
        err = info.get("error_magnitude")
        if err is None:
            return True, "determinism_failed"
        if err < DETERMINISM_ERROR_MIN:
            return False, "determinism_trivial"
        if err > DETERMINISM_ERROR_MAX:
            return False, "determinism_explosion"
        return True, f"determinism_err_{err:.2f}m"

    if test_type == "force_removal":
        return True, "force_removal_velocity_jump"

    if test_type == "zero_input_stability":
        return True, "zero_input_drift"

    if test_type == "temporal_monotonicity":
        return True, "temporal_monotonicity_violation"

    if test_type == "force_isolation":
        return True, "force_isolation_failed"

    return True, "failed_other"


def collect_experiment_dirs(root):
    """收集所有 _N 格式的子目录，按数字排序。"""
    root = Path(root)
    if not root.is_dir():
        return []
    dirs = []
    for p in root.iterdir():
        if p.is_dir() and re.match(r"^_\d+$", p.name):
            try:
                num = int(p.name[1:])
                dirs.append((num, p))
            except ValueError:
                pass
    dirs.sort(key=lambda x: x[0])
    return [p for _, p in dirs]


def main():
    parser = argparse.ArgumentParser(description="初筛 test_22 中可能测出 Gazebo 问题的实验")
    parser.add_argument("test_dir", default="/home/liyitao/workspace/meta/test_22", nargs="?",
                        help="test_22 目录路径")
    parser.add_argument("--out", "-o", default="", help="输出 CSV 路径，不指定则打印到 stdout")
    parser.add_argument("--top", "-n", type=int, default=0,
                        help="只输出前 N 个候选（0=全部）")
    parser.add_argument("--verbose", "-v", action="store_true", help="打印排除原因等")
    args = parser.parse_args()

    test_dir = Path(args.test_dir)
    if not test_dir.is_dir():
        print(f"错误：目录不存在 {test_dir}", file=sys.stderr)
        sys.exit(1)

    dirs = collect_experiment_dirs(test_dir)
    print(f"扫描目录数: {len(dirs)}", file=sys.stderr)

    candidates = []
    for d in dirs:
        exp_id = d.name  # _0, _1, ...
        result_path = d / "metamorphic_test_result.txt"
        sdf_path = d / "a.sdf"
        info = parse_result_file(str(result_path))
        world_name = extract_world_name(str(sdf_path))

        is_cand, reason = is_candidate(exp_id, info, world_name, str(sdf_path))
        if is_cand:
            test_type = normalize_test_type(info.get("test_type") or "")
            err_mag = info.get("error_magnitude")
            err_str = f"{err_mag:.4f}" if err_mag is not None else "N/A"
            candidates.append({
                "exp_id": exp_id,
                "test_type": test_type,
                "world": world_name or "N/A",
                "model": (info.get("model") or "N/A")[:40],
                "reason": reason,
                "error_magnitude": err_str,
            })
        elif args.verbose and info["result"] == "FAILED":
            _, excl_reason = should_exclude(world_name, str(sdf_path))
            if excl_reason:
                print(f"  {exp_id} excluded: {excl_reason} (world={world_name})", file=sys.stderr)

    if args.top > 0:
        candidates = candidates[: args.top]

    # 输出
    lines = []
    header = "exp_id,test_type,world,model,reason,error_magnitude"
    lines.append(header)
    for c in candidates:
        row = f"{c['exp_id']},{c['test_type']},{c['world']},{c['model']},{c['reason']},{c['error_magnitude']}"
        lines.append(row)

    out_text = "\n".join(lines)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(out_text)
        print(f"已写入 {len(candidates)} 条候选到 {args.out}", file=sys.stderr)
    else:
        print(out_text)

    print(f"候选实验数: {len(candidates)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
