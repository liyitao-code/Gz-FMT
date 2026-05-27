#!/usr/bin/env python3
# coding: utf-8
"""
实验复现与结果比较工具

原理：严格按照 experiment_log.json 中记录的每一步指令和等待时间，
      启动 Gazebo → 逐条执行命令 → 等待 → 最后对比复现结果与原始结果。

用法：
    # 复现单次实验
    python3 reproduce_experiment.py /path/to/experiment_dir

    # 批量复现（从某个目录下挑选指定条件的实验）
    python3 reproduce_experiment.py /path/to/test_dir --batch --filter-result FAILED --filter-type force_additivity --max 5

    # 指定输出目录
    python3 reproduce_experiment.py /path/to/experiment_dir -o /path/to/output
"""

import os
import sys
import json
import re
import time
import subprocess
import argparse
import math
from datetime import datetime

# 添加 gz Python 库路径（与 randomsmith_meta.py 保持一致）
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

import psutil


# ============================================================
# 工具函数
# ============================================================

def kill_all_gz():
    """杀掉所有残留 Gazebo 进程"""
    for pat in ['gz sim', 'gz-sim-server', 'ruby', 'parameter_bridge']:
        subprocess.run(f"pkill -9 -f '{pat}'", shell=True, timeout=5,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)


def exec_command(cmd, timeout=100):
    """执行一条 shell 命令，返回 (成功, stdout)"""
    try:
        r = subprocess.run(cmd, shell=True, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return r.returncode == 0, r.stdout.decode('utf-8', errors='replace')
    except subprocess.TimeoutExpired:
        return False, "TIMEOUT"
    except Exception as e:
        return False, str(e)


def trigger_step_then_get_pose(world_name, model_name):
    """
    发送 multi_step: 1 触发仿真发布位姿，再从 topic 读取。
    用于 Determinism 复现时在暂停状态下获取终点位置（与 randomsmith_meta 中 get_model_pose_from_scene 一致）。
    返回 (x,y,z) 或 None。
    """
    step_cmd = (f"gz service -s /world/{world_name}/control "
                f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                f"--timeout 10000 --req 'multi_step: 1'")
    ok, _ = exec_command(step_cmd)
    if not ok:
        return None
    time.sleep(0.15)
    return get_model_pose_from_topic(world_name, model_name, timeout_sec=4.0)


def get_model_pose_from_topic(world_name, model_name, timeout_sec=3.0):
    """
    通过 topic 获取模型实时位置（需要模拟在运行状态）。
    优先使用 Python gz transport API，失败时回退到 gz topic 命令行。
    返回 (x,y,z) 或 None。
    """
    # 方法1：Python gz transport API
    try:
        from gz.transport14 import Node
        from gz.msgs11.pose_v_pb2 import Pose_V

        node = Node()
        result = [None]

        def cb(msg):
            for pose in msg.pose:
                if pose.name == model_name:
                    result[0] = (pose.position.x, pose.position.y, pose.position.z)

        for topic in [f"/world/{world_name}/dynamic_pose/info",
                      f"/world/{world_name}/pose/info"]:
            result[0] = None
            sub = node.subscribe(Pose_V, topic, cb)
            if not sub:
                continue
            t0 = time.time()
            while result[0] is None and (time.time() - t0) < timeout_sec:
                time.sleep(0.01)
            if result[0] is not None:
                return result[0]
        return None
    except ImportError:
        pass  # 回退到命令行方法
    except Exception as e:
        print(f"  [WARN] get_model_pose Python API error: {e}")

    # 方法2：gz topic 命令行（回退方案）
    for topic in [f"/world/{world_name}/dynamic_pose/info",
                  f"/world/{world_name}/pose/info"]:
        try:
            r = subprocess.run(
                f"gz topic -e -t {topic} -n 1",
                shell=True, timeout=timeout_sec + 5,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            if r.returncode != 0:
                continue
            output = r.stdout.decode('utf-8', errors='replace')

            # 解析 protobuf 文本格式，按 "pose {" 分块
            blocks = re.split(r'(?=pose\s*\{)', output)
            for block in blocks:
                if f'name: "{model_name}"' in block:
                    # 提取 position 中的 x, y, z
                    pos_block = re.search(r'position\s*\{([^}]*)\}', block, re.DOTALL)
                    if pos_block:
                        px = re.search(r'x:\s*([-\d.e+]+)', pos_block.group(1))
                        py = re.search(r'y:\s*([-\d.e+]+)', pos_block.group(1))
                        pz = re.search(r'z:\s*([-\d.e+]+)', pos_block.group(1))
                        if px and py and pz:
                            return (float(px.group(1)), float(py.group(1)), float(pz.group(1)))
        except subprocess.TimeoutExpired:
            continue
        except Exception as e:
            print(f"  [WARN] gz topic -e error: {e}")

    print(f"  [WARN] 无法获取模型 {model_name} 的位置")
    return None


def trigger_step_then_get_all_poses(world_name, timeout_sec=4.0):
    """
    发送 multi_step: 1 触发仿真发布位姿，再从 pose/info 读取所有实体（模型+link）位置。
    用于关节约束稳定性复现时获取两 link 的位置以计算距离。
    返回 dict: { entity_name: (x, y, z), ... } 或 None。
    """
    step_cmd = (f"gz service -s /world/{world_name}/control "
                f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                f"--timeout 10000 --req 'multi_step: 1'")
    ok, _ = exec_command(step_cmd)
    if not ok:
        return None
    time.sleep(0.15)

    # 方法1：Python gz transport API
    try:
        from gz.transport14 import Node
        from gz.msgs11.pose_v_pb2 import Pose_V

        node = Node()
        result = [None]

        def cb(msg):
            out = {}
            for pose in msg.pose:
                out[pose.name] = (pose.position.x, pose.position.y, pose.position.z)
            result[0] = out

        for topic in [f"/world/{world_name}/dynamic_pose/info",
                      f"/world/{world_name}/pose/info"]:
            result[0] = None
            sub = node.subscribe(Pose_V, topic, cb)
            if not sub:
                continue
            t0 = time.time()
            while result[0] is None and (time.time() - t0) < timeout_sec:
                time.sleep(0.01)
            if result[0] is not None and len(result[0]) > 0:
                return result[0]
        return None
    except ImportError:
        pass
    except Exception as e:
        print(f"  [WARN] get_all_poses Python API error: {e}")

    # 方法2：gz topic 命令行
    for topic in [f"/world/{world_name}/dynamic_pose/info",
                  f"/world/{world_name}/pose/info"]:
        try:
            r = subprocess.run(
                f"gz topic -e -t {topic} -n 1",
                shell=True, timeout=timeout_sec + 5,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            if r.returncode != 0:
                continue
            output = r.stdout.decode('utf-8', errors='replace')
            poses = {}
            blocks = re.split(r'(?=pose\s*\{)', output)
            for block in blocks:
                name_m = re.search(r'name:\s*"([^"]*)"', block)
                if not name_m:
                    continue
                pos_block = re.search(r'position\s*\{([^}]*)\}', block, re.DOTALL)
                if not pos_block:
                    continue
                px = re.search(r'x:\s*([-\d.e+]+)', pos_block.group(1))
                py = re.search(r'y:\s*([-\d.e+]+)', pos_block.group(1))
                pz = re.search(r'z:\s*([-\d.e+]+)', pos_block.group(1))
                if px and py and pz:
                    try:
                        poses[name_m.group(1)] = (float(px.group(1)), float(py.group(1)), float(pz.group(1)))
                    except (ValueError, OverflowError):
                        pass
            if poses:
                return poses
        except subprocess.TimeoutExpired:
            continue
        except Exception as e:
            print(f"  [WARN] gz topic -e (all poses) error: {e}")
    return None


def _resolve_link_key(poses_dict, scoped_name):
    """pose/info 可能用 model::link 或仅 link，尝试两种格式（与 randomsmith_meta 一致）。"""
    if not poses_dict:
        return None
    if scoped_name in poses_dict:
        return scoped_name
    local = scoped_name.split("::")[-1] if "::" in scoped_name else scoped_name
    return local if local in poses_dict else None


# ============================================================
# 核心：按日志回放实验
# ============================================================

def _is_determinism_log(log_data):
    """判断是否为 Determinism 测试的日志（需双进程复现）。"""
    for entry in log_data:
        if entry.get("type") == "test_info":
            t = (entry.get("test_type") or "").strip().lower()
            if "determinism" in t:
                return True
    return False


def _is_temporal_monotonicity_log(log_data):
    """判断是否为 Temporal Monotonicity 测试的日志。"""
    for entry in log_data:
        if entry.get("type") == "test_info":
            t = (entry.get("test_type") or "").strip().lower()
            if "temporal_monotonicity" in t or "temporal monotonicity" in t:
                return True
    return False


def _determinism_second_gazebo_sleep_index(log_data):
    """返回 'Wait for second Gazebo to start' 那条 sleep 的索引。"""
    for i, entry in enumerate(log_data):
        if entry.get("type") == "sleep":
            if "second Gazebo" in (entry.get("description") or ""):
                return i
    return -1


def _run_entries(log_data, start_idx, end_idx_excl, world_name, model_name, result, output_dir):
    """
    执行 log_data[start_idx:end_idx_excl] 中的条目（command/sleep），不启动/不杀进程。
    """
    for i in range(start_idx, end_idx_excl):
        entry = log_data[i]
        entry_type = entry.get("type", "")

        if entry_type in ("experiment_info", "test_info"):
            continue
        if entry_type == "command":
            cmd_type = entry.get("command_type", "")
            cmd = entry.get("command", "")
            if cmd_type == "launch":
                wait = entry.get("wait_after", 2) or 2
                print(f"  [{i:2d}] launch → 等待 {wait}s")
                time.sleep(wait)
            else:
                print(f"  [{i:2d}] {cmd_type:7s}: {cmd[:100]}{'...' if len(cmd) > 100 else ''}")
                ok, _ = exec_command(cmd)
                if not ok:
                    result["errors"].append(f"Step {i}: failed")
            result["steps_executed"] += 1
        elif entry_type == "sleep":
            duration = entry.get("duration", 0) or 0
            desc = entry.get("description", "") or ""
            if duration > 0:
                print(f"  [{i:2d}] sleep {duration:.2f}s  {desc[:70]}")
                time.sleep(duration)
            result["steps_executed"] += 1


def _replay_determinism(sdf_path, log_data, i_second, world_name, model_name, result, output_dir):
    """
    Determinism 测试复现：Run 1 → 关进程 → 等 2s → Run 2（新进程 + 激活）→ 比较 P1/P2。
    """
    gz_out = subprocess.DEVNULL
    gz_err = subprocess.DEVNULL
    if output_dir:
        gz_out = open(os.path.join(output_dir, "gz_run1.out"), "w")
        gz_err = open(os.path.join(output_dir, "gz_run1.err"), "w")

    print("  [Determinism] Run 1: 启动第一个 Gazebo...")
    gz1 = subprocess.Popen(f"gz sim {sdf_path} --seed 12345".split(), stdout=gz_out, stderr=gz_err, start_new_session=True)
    time.sleep(2)
    print("  [Determinism] Run 1: 激活仿真引擎 (play → pause)...")
    for req in ["pause: false", "pause: true"]:
        exec_command(f"gz service -s /world/{world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req '{req}'")
    time.sleep(0.3)
    try:
        # 施力前读取初态 Q1（用于检查两轮起始是否一致）
        q1 = trigger_step_then_get_pose(world_name, model_name)
        if q1 is not None:
            result["determinism_initial_run1"] = q1
            print(f"  [Determinism] Run 1 施力前初态 Q1 = ({q1[0]:.6f}, {q1[1]:.6f}, {q1[2]:.6f})")
        # 从索引 3 开始（跳过 launch、sleep 2、test_info），执行到 Run 1 结束
        _run_entries(log_data, 3, i_second, world_name, model_name, result, output_dir)
        print("  [Determinism] Run 1 结束，读取终点位置 P1...")
        p1 = trigger_step_then_get_pose(world_name, model_name)
        if p1:
            result["positions_captured"]["Run 1 (determinism)"] = p1
            print(f"         P1 = ({p1[0]:.6f}, {p1[1]:.6f}, {p1[2]:.6f})")
        else:
            result["errors"].append("Run 1: 未捕获到位置")
    finally:
        try:
            for c in psutil.Process(gz1.pid).children(recursive=True):
                c.kill()
            gz1.kill()
            gz1.wait(timeout=5)
        except Exception:
            pass
        if output_dir:
            try:
                gz_out.close()
                gz_err.close()
            except Exception:
                pass

    # "Wait for second Gazebo to start"
    sleep_entry = log_data[i_second]
    duration = sleep_entry.get("duration", 2) or 2
    print(f"  [{i_second:2d}] sleep {duration:.2f}s  (Wait for second Gazebo to start)")
    time.sleep(duration)
    result["steps_executed"] += 1

    kill_all_gz()
    time.sleep(1)

    print("  [Determinism] Run 2: 启动第二个 Gazebo...")
    if output_dir:
        gz_out = open(os.path.join(output_dir, "gz_run2.out"), "w")
        gz_err = open(os.path.join(output_dir, "gz_run2.err"), "w")
    else:
        gz_out, gz_err = subprocess.DEVNULL, subprocess.DEVNULL
    gz2 = subprocess.Popen(f"gz sim {sdf_path} --seed 12345".split(), stdout=gz_out, stderr=gz_err, start_new_session=True)
    time.sleep(2)

    # 激活第二个 Gazebo（play → pause），与 randomsmith_meta 一致
    print("  [Determinism] Run 2: 激活仿真引擎 (play → pause)...")
    for req in ["pause: false", "pause: true"]:
        exec_command(f"gz service -s /world/{world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req '{req}'")
    time.sleep(0.3)

    try:
        # 施力前读取初态 Q2（用于检查两轮起始是否一致）
        q2 = trigger_step_then_get_pose(world_name, model_name)
        if q2 is not None:
            result["determinism_initial_run2"] = q2
            print(f"  [Determinism] Run 2 施力前初态 Q2 = ({q2[0]:.6f}, {q2[1]:.6f}, {q2[2]:.6f})")
        _run_entries(log_data, i_second + 1, len(log_data), world_name, model_name, result, output_dir)
        print("  [Determinism] Run 2 结束，读取终点位置 P2...")
        p2 = trigger_step_then_get_pose(world_name, model_name)
        if p2:
            result["positions_captured"]["Run 2 (determinism)"] = p2
            print(f"         P2 = ({p2[0]:.6f}, {p2[1]:.6f}, {p2[2]:.6f})")
        else:
            result["errors"].append("Run 2: 未捕获到位置")
    finally:
        try:
            for c in psutil.Process(gz2.pid).children(recursive=True):
                c.kill()
            gz2.kill()
            gz2.wait(timeout=5)
        except Exception:
            pass
        if output_dir:
            try:
                gz_out.close()
                gz_err.close()
            except Exception:
                pass

    return result


def replay_from_log(sdf_path, log_data, output_dir=None):
    """
    严格按照 experiment_log.json 的记录回放实验。

    Args:
        sdf_path:   a.sdf 的完整路径
        log_data:   experiment_log.json 解析后的 list
        output_dir: 可选输出目录

    Returns:
        dict: {
            'world_name': str,
            'steps_executed': int,
            'positions_captured': {description: (x,y,z)},  # 在关键 sleep 前捕获位置
            'errors': [str],
        }
    """
    result = {
        'world_name': None,
        'steps_executed': 0,
        'positions_captured': {},
        'joint_constraint_pose_snapshots': [],  # 关节约束：每步后的全实体位姿 [{name:(x,y,z),...}, ...]
        'errors': [],
    }

    # 从日志中提取 world_name（从命令中的 /world/xxx/ 路径提取）
    world_name = None
    model_name = None
    for entry in log_data:
        if entry.get('type') == 'command':
            cmd = entry.get('command', '')
            m = re.search(r'/world/([^/]+)/', cmd)
            if m:
                world_name = m.group(1)
                break
    # 提取 model_name（从 wrench 命令中的 name: "xxx" 提取）
    for entry in log_data:
        if entry.get('type') == 'command':
            cmd = entry.get('command', '')
            m = re.search(r'name:\s*"([^"]+)".*type:\s*MODEL', cmd)
            if m:
                model_name = m.group(1)
                break

    result['world_name'] = world_name
    result['model_name'] = model_name

    # ---------- 特殊测试类型检测 ----------
    is_determinism = _is_determinism_log(log_data)
    is_monotonicity = _is_temporal_monotonicity_log(log_data)
    is_force_isolation = any(
        e.get('type') == 'test_info' and e.get('test_type') == 'force_isolation'
        for e in log_data
    )
    is_joint_constraint = any(
        e.get('type') == 'test_info' and (e.get('test_type') or '').strip().lower() == 'joint_constraint_stability'
        for e in log_data
    )

    # Force Isolation：尝试识别 target / bystander 模型名
    force_iso_target = None
    force_iso_bystander = None
    if is_force_isolation:
        names = []
        for entry in log_data:
            if entry.get('type') == 'command':
                cmd = entry.get('command', '')
                m = re.search(r'name:\s*\"([^\"]+)\"\\s*,\\s*type:\\s*MODEL', cmd)
                if m:
                    n = m.group(1)
                    if n not in names:
                        names.append(n)
        if names:
            force_iso_target = names[0]
            if len(names) > 1:
                force_iso_bystander = names[1]
    result['force_iso_target'] = force_iso_target
    result['force_iso_bystander'] = force_iso_bystander

    # Determinism 测试：双进程复现
    i_second = _determinism_second_gazebo_sleep_index(log_data)
    if is_determinism and i_second >= 0 and world_name and model_name:
        return _replay_determinism(sdf_path, log_data, i_second, world_name, model_name, result, output_dir)

    # ---------- 单进程复现（非 Determinism）----------
    gz_cmd = f"gz sim {sdf_path} --seed 12345"
    print(f"  启动 Gazebo: {gz_cmd}")
    gz_out = subprocess.DEVNULL
    gz_err = subprocess.DEVNULL
    if output_dir:
        gz_out = open(os.path.join(output_dir, "gz.out"), "w")
        gz_err = open(os.path.join(output_dir, "gz.err"), "w")

    gz_process = subprocess.Popen(gz_cmd.split(), stdout=gz_out, stderr=gz_err, start_new_session=True)

    try:
        # 逐条执行日志
        monotonicity_sample_idx = 0
        for i, entry in enumerate(log_data):
            entry_type = entry.get('type', '')

            if entry_type == 'experiment_info' or entry_type == 'test_info':
                # 信息条目，跳过
                continue

            elif entry_type == 'command':
                cmd_type = entry.get('command_type', '')
                cmd = entry.get('command', '')
                desc = entry.get('description', '')

                if cmd_type == 'launch':
                    # 启动命令我们已经手动执行了，这里只做等待
                    wait = entry.get('wait_after', 0)
                    if wait > 0:
                        print(f"  [{i:2d}] launch → 等待 Gazebo 启动 {wait}s")
                        time.sleep(wait)
                    else:
                        # 默认等待
                        print(f"  [{i:2d}] launch → 等待 Gazebo 启动 2s")
                        time.sleep(2)
                    result['steps_executed'] += 1
                    continue

                # 普通 service/topic 命令
                print(f"  [{i:2d}] {cmd_type:7s}: {cmd[:120]}{'...' if len(cmd)>120 else ''}")
                ok, out = exec_command(cmd)
                if not ok:
                    result['errors'].append(f"Step {i}: command failed: {cmd[:80]}")
                result['steps_executed'] += 1

            elif entry_type == 'sleep':
                duration = entry.get('duration', 0)
                desc = entry.get('description', '')

                # 在关键 sleep 之前（"Running test" / "Test A" / "Test B"）
                # 之后捕获模型位置
                is_test_run = any(kw in desc for kw in ['Running test', 'Test A:', 'Test B:'])

                if duration > 0:
                    print(f"  [{i:2d}] sleep {duration:.2f}s  {desc[:80]}")
                    time.sleep(duration)

                # 在测试运行结束后（下一步是 pause）捕获位置（通用逻辑）
                if is_test_run and world_name and model_name:
                    pos = get_model_pose_from_topic(world_name, model_name)
                    if pos:
                        label = desc[:60]
                        result['positions_captured'][label] = pos
                        print(f"         → 捕获位置: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

                # Temporal Monotonicity：施力后、首次 Stepping 前的初始位置
                if is_monotonicity and world_name and model_name and 'force' in desc.lower() and 'monotonicity' in desc.lower() and 'Wait for force' in desc:
                    pos = get_model_pose_from_topic(world_name, model_name)
                    if pos:
                        result['positions_captured']['monotonicity initial'] = pos
                        print(f"         → 单调性初态: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

                # Temporal Monotonicity：每段 Stepping 后采样一次位置
                if is_monotonicity and world_name and model_name and 'Stepping' in desc:
                    pos = get_model_pose_from_topic(world_name, model_name)
                    if pos:
                        monotonicity_sample_idx += 1
                        label = f"monotonicity sample {monotonicity_sample_idx}: {desc[:40]}"
                        result['positions_captured'][label] = pos
                        print(f"         → 单调性采样 {monotonicity_sample_idx}: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

                # Joint Constraint Stability：每次“Wait for force”后采 d0，每次 Stepping 后采一次全实体位姿
                if is_joint_constraint and world_name:
                    if 'Wait for force (joint constraint)' in desc:
                        snap = trigger_step_then_get_all_poses(world_name)
                        if snap:
                            result['joint_constraint_pose_snapshots'].append(snap)
                            print(f"         → 关节约束 d0 快照: {len(snap)} 个实体")
                    elif 'Stepping' in desc:
                        snap = trigger_step_then_get_all_poses(world_name)
                        if snap:
                            result['joint_constraint_pose_snapshots'].append(snap)
                            print(f"         → 关节约束采样 {len(result['joint_constraint_pose_snapshots'])}: {len(snap)} 个实体")

                # Force Isolation：记录 target / bystander 初始与最终位置
                if is_force_isolation and world_name and result.get('force_iso_target'):
                    tgt = result.get('force_iso_target')
                    bys = result.get('force_iso_bystander')
                    # 初始位置：施力后、步进前
                    if 'Wait for force to be applied (force isolation)' in desc and tgt:
                        pt = get_model_pose_from_topic(world_name, tgt)
                        if pt:
                            result['positions_captured']['force_iso_target_initial'] = pt
                            print(f"         → ForceIsolation target 初始: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
                        if bys:
                            pb = get_model_pose_from_topic(world_name, bys)
                            if pb:
                                result['positions_captured']['force_iso_bystander_initial'] = pb
                                print(f"         → ForceIsolation bystander 初始: ({pb[0]:.4f}, {pb[1]:.4f}, {pb[2]:.4f})")
                    # 终点位置：Stepping 结束后
                    if 'Stepping' in desc and tgt:
                        pt = get_model_pose_from_topic(world_name, tgt)
                        if pt:
                            result['positions_captured']['force_iso_target_final'] = pt
                            print(f"         → ForceIsolation target 终点: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
                        if bys:
                            pb = get_model_pose_from_topic(world_name, bys)
                            if pb:
                                result['positions_captured']['force_iso_bystander_final'] = pb
                                print(f"         → ForceIsolation bystander 终点: ({pb[0]:.4f}, {pb[1]:.4f}, {pb[2]:.4f})")

                result['steps_executed'] += 1

            else:
                # 未知类型，跳过
                pass

    except KeyboardInterrupt:
        print("\n  中断!")
        result['errors'].append("User interrupted")
    except Exception as e:
        print(f"\n  异常: {e}")
        result['errors'].append(str(e))
    finally:
        # 关闭 Gazebo
        print("  关闭 Gazebo...")
        try:
            for child in psutil.Process(gz_process.pid).children(recursive=True):
                child.kill()
            gz_process.kill()
            gz_process.wait(timeout=5)
        except:
            pass
        if output_dir:
            try:
                gz_out.close()
                gz_err.close()
            except:
                pass

    return result


# ============================================================
# 比较原始结果与复现结果
# ============================================================

def parse_original_result(result_path):
    """解析原始 metamorphic_test_result.txt，返回 dict"""
    info = {}
    if not os.path.exists(result_path):
        return info
    with open(result_path) as f:
        text = f.read()
    info['raw_text'] = text

    m = re.search(r'^Test Type:\s*(.+)$', text, re.MULTILINE)
    if m:
        info['test_type'] = m.group(1).strip()

    m = re.search(r'^Result:\s*(\w+)', text, re.MULTILINE)
    if m:
        info['result'] = m.group(1)
    elif 'returned None' in text or 'failed to execute' in text:
        info['result'] = 'ERROR'
    else:
        info['result'] = 'UNKNOWN'

    m = re.search(r'^Model:\s*(.+)$', text, re.MULTILINE)
    if m:
        info['model'] = m.group(1).strip()

    # 提取所有位置
    for pattern, label in [
        (r'Position \(Run 1\s*-\s*first Gazebo\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_run1'),
        (r'Position \(Run 2\s*-\s*fresh Gazebo\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_run2'),
        (r'Position \(Run A\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_run_a'),
        (r'Position \(Run B\):\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_run_b'),
        (r'Position with F1\+F2.*:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_f1f2'),
        (r'Position with F_total.*:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_ftotal'),
        (r'Position with rtf=1\.00:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_rtf1'),
        (r'Position with rtf=[\d.]+:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'pos_rtf2'),
        (r'Final Position:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'final_pos'),
        (r'Expected Position:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)', 'expected_pos'),
    ]:
        m = re.search(pattern, text)
        if m:
            try:
                info[label] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            except (ValueError, OverflowError):
                pass

    # 提取 Position difference
    m = re.search(r'Position difference:\s*x=([-\d.e+]+),?\s*y=([-\d.e+]+),?\s*z=([-\d.e+]+)', text)
    if not m:
        m = re.search(r'^Error:\s*x=([-\d.e+]+),?\s*y=([-\d.e+]+),?\s*z=([-\d.e+]+)', text, re.MULTILINE)
    if m:
        try:
            info['original_error'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
        except (ValueError, OverflowError):
            pass

    # Temporal Monotonicity：提取 trajectory (x-displacement)
    if ('Temporal Monotonicity' in info.get('test_type', '') or
            ('monotonicity' in text.lower() and 'Trajectory (x-displacement)' in text)):
        trajectory = []
        for m in re.finditer(r't=([\d.]+)s:\s*x_disp=([-\d.e+]+)m', text):
            try:
                trajectory.append((float(m.group(1)), float(m.group(2))))
            except (ValueError, OverflowError):
                pass
        if trajectory:
            info['monotonicity_trajectory'] = trajectory
        m = re.search(r'Monotonic:\s*(\w+)', text)
        if m:
            info['monotonicity_monotonic'] = m.group(1).strip().lower() == 'yes'
        m = re.search(r'Smooth:\s*(\w+)', text)
        if m:
            info['monotonicity_smooth'] = m.group(1).strip().lower() == 'yes'

    # Force Isolation：提取 target / bystander 位置与漂移
    if 'Force Isolation' in info.get('test_type', ''):
        m = re.search(r'^Target Model:\s*(.+)$', text, re.MULTILINE)
        if m:
            info['fi_target_model'] = m.group(1).strip()
        m = re.search(r'^Bystander Model:\s*(.+)$', text, re.MULTILINE)
        if m:
            info['fi_bystander_model'] = m.group(1).strip()

        def _parse_vec(line_prefix):
            mm = re.search(
                rf'^{line_prefix}:\s*\(([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\)',
                text, re.MULTILINE)
            if mm:
                try:
                    return (float(mm.group(1)), float(mm.group(2)), float(mm.group(3)))
                except (ValueError, OverflowError):
                    return None
            return None

        ti = _parse_vec(r'Target Initial')
        tf = _parse_vec(r'Target Final')
        bi = _parse_vec(r'Bystander Initial')
        bf = _parse_vec(r'Bystander Final')
        if ti:
            info['fi_target_initial'] = ti
        if tf:
            info['fi_target_final'] = tf
        if bi:
            info['fi_bystander_initial'] = bi
        if bf:
            info['fi_bystander_final'] = bf

        m = re.search(r'Target Displacement:\s*([-\d.e+]+)m', text)
        if m:
            try:
                info['fi_target_displacement'] = float(m.group(1))
            except (ValueError, OverflowError):
                pass
        m = re.search(r'Bystander Drift:\s*([-\d.e+]+)m', text)
        if m:
            try:
                info['fi_bystander_drift'] = float(m.group(1))
            except (ValueError, OverflowError):
                pass

    # Joint Constraint Stability：提取 Links 与 Samples (t, distance)
    if 'Joint Constraint' in info.get('test_type', '') or 'joint_constraint' in info.get('test_type', '').lower():
        m = re.search(r'^Links:\s*(.+)$', text, re.MULTILINE)
        if m:
            links_str = m.group(1).strip()
            # 格式 "vehicle::chassis, vehicle::front_left_wheel" 或 "cylinder::cylinder_link, cylinder::cylinder_link2"
            parts = [p.strip() for p in re.split(r',\s*', links_str)]
            if len(parts) >= 2:
                info['joint_constraint_links'] = (parts[0], parts[1])
        # 从 Error Info 中解析 resolved 名（可选，用于复现时查找）
        m = re.search(r'\(resolved:\s*([^,]+),\s*([^)]+)\)', text)
        if m:
            info['joint_constraint_links_resolved'] = (m.group(1).strip(), m.group(2).strip())
        # Samples (t, distance): t=0.00s: d=0.963685m
        samples = []
        for mm in re.finditer(r't=([\d.]+)s:\s*d=([-\d.e+]+)m', text):
            try:
                samples.append((float(mm.group(1)), float(mm.group(2))))
            except (ValueError, OverflowError):
                pass
        if samples:
            info['joint_constraint_samples'] = samples
        m = re.search(r'Initial distance d0:\s*([-\d.e+]+)m', text)
        if m:
            try:
                info['joint_constraint_d0'] = float(m.group(1))
            except (ValueError, OverflowError):
                pass
        m = re.search(r'Drift tolerance:\s*([-\d.e+]+)m', text)
        if m:
            try:
                info['joint_constraint_drift_tol'] = float(m.group(1))
            except (ValueError, OverflowError):
                pass
        m = re.search(r'Jump tolerance:\s*([-\d.e+]+)m', text)
        if m:
            try:
                info['joint_constraint_jump_tol'] = float(m.group(1))
            except (ValueError, OverflowError):
                pass

    return info


def compare_and_report(original_info, replay_result, output_dir=None):
    """
    比较原始结果与复现结果，打印报告。

    核心逻辑：
    - 复现时在关键 sleep 后捕获了模型位置（positions_captured）
    - 原始结果中也有位置记录
    - 比较两者，判断复现是否与原始一致
    """
    lines = []
    lines.append("=" * 60)
    lines.append("         实验复现结果比较报告")
    lines.append("=" * 60)
    lines.append(f"  原始测试类型: {original_info.get('test_type', 'N/A')}")
    lines.append(f"  原始结果:     {original_info.get('result', 'N/A')}")
    lines.append(f"  模型:         {original_info.get('model', 'N/A')}")
    lines.append(f"  复现步骤数:   {replay_result['steps_executed']}")
    lines.append(f"  复现错误数:   {len(replay_result['errors'])}")
    if replay_result['errors']:
        for err in replay_result['errors'][:3]:
            lines.append(f"    - {err[:100]}")
    lines.append("")

    # 复现中捕获的位置
    captured = replay_result.get('positions_captured', {})
    snapshots = replay_result.get('joint_constraint_pose_snapshots', [])
    if captured:
        lines.append("  复现捕获的位置:")
        for label, pos in captured.items():
            lines.append(f"    {label}:")
            lines.append(f"      ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
    if snapshots:
        lines.append(f"  复现关节约束快照数: {len(snapshots)}（每步全实体位姿）")
    if not captured and not snapshots:
        lines.append("  复现中未捕获到位置（可能 Gazebo 未正常启动或模型名不匹配）")

    # Determinism：施力前初态 Q1/Q2 及初态差（用于判断实验误差）
    q1 = replay_result.get("determinism_initial_run1")
    q2 = replay_result.get("determinism_initial_run2")
    if q1 is not None and q2 is not None:
        lines.append("")
        lines.append("  施力前初态（激活后、施力前）:")
        lines.append(f"    Run 1 初态 Q1: ({q1[0]:.6f}, {q1[1]:.6f}, {q1[2]:.6f})")
        lines.append(f"    Run 2 初态 Q2: ({q2[0]:.6f}, {q2[1]:.6f}, {q2[2]:.6f})")
        dq = math.sqrt((q1[0]-q2[0])**2 + (q1[1]-q2[1])**2 + (q1[2]-q2[2])**2)
        lines.append(f"    初态差 |Q1−Q2|: {dq:.6f} m")
        if dq >= 0.01:
            lines.append("    → 初态不一致，部分 P1−P2 差可能来自实验误差（激活阶段未控制）")

    lines.append("")

    # ===== Joint Constraint Stability 专用比较 =====
    orig_jc_samples = original_info.get('joint_constraint_samples')
    orig_jc_links = original_info.get('joint_constraint_links') or original_info.get('joint_constraint_links_resolved')
    if orig_jc_samples and orig_jc_links:
        if not snapshots:
            lines.append("  [Joint Constraint] 原始结果含约束距离样本，但复现未捕获到位姿快照（pose/info 或 multi_step 可能失败）。")
            lines.append("  → 结论: 无法比较（复现未捕获关节约束快照）")
            lines.append("=" * 60)
            report_text = "\n".join(lines)
            print(report_text)
            if output_dir:
                report_path = os.path.join(output_dir, "reproduction_report.txt")
                with open(report_path, "w") as f:
                    f.write(report_text)
                    f.write("\n\n--- 原始结果文件 ---\n")
                    f.write(original_info.get('raw_text', ''))
                    f.write("\n\n--- 复现详情 ---\n")
                    f.write(json.dumps(replay_result, indent=2, default=str))
                print(f"\n报告已保存: {report_path}")
            return None
    if orig_jc_samples and orig_jc_links and snapshots:
        link1_scoped, link2_scoped = orig_jc_links[0], orig_jc_links[1]
        first_snap = snapshots[0]
        link1_key = _resolve_link_key(first_snap, link1_scoped)
        link2_key = _resolve_link_key(first_snap, link2_scoped)
        if link1_key is None or link2_key is None:
            lines.append("  [Joint Constraint] 复现快照中未找到原始 link 名，无法比较。")
            lines.append(f"    原始 Links: {link1_scoped}, {link2_scoped}")
            lines.append(f"    快照中实体示例: " + ", ".join(list(first_snap.keys())[:15]))
        else:
            def _dist(p1, p2):
                return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
            rep_distances = []
            for s in snapshots:
                if link1_key in s and link2_key in s:
                    rep_distances.append(_dist(s[link1_key], s[link2_key]))
                else:
                    rep_distances.append(float('nan'))
            n_compare = min(len(orig_jc_samples), len(rep_distances))
            lines.append("  [Joint Constraint Stability] 约束距离 (两 link) 序列对比:")
            lines.append(f"    Links: {link1_scoped}, {link2_scoped} (解析: {link1_key}, {link2_key})")
            max_dev = 0
            for i in range(n_compare):
                t_orig, d_orig = orig_jc_samples[i][0], orig_jc_samples[i][1]
                d_rep = rep_distances[i] if i < len(rep_distances) else float('nan')
                dev = abs(d_orig - d_rep) if not math.isnan(d_rep) else float('nan')
                if not math.isnan(dev):
                    max_dev = max(max_dev, dev)
                lines.append(f"    t={t_orig:.2f}s: 原始 d={d_orig:.6f}m, 复现 d={d_rep:.6f}m, 偏差={dev:.6f}m")
            lines.append(f"  约束距离最大偏差: {max_dev:.6f} m")
            d0_orig = original_info.get('joint_constraint_d0')
            drift_tol = original_info.get('joint_constraint_drift_tol', 0.05)
            jump_tol = original_info.get('joint_constraint_jump_tol', 0.1)
            if d0_orig is not None and rep_distances:
                rep_d0 = rep_distances[0]
                rep_drift_ok = all(abs(d - rep_d0) <= drift_tol for d in rep_distances if not math.isnan(d))
                rep_jump_ok = True
                for i in range(1, len(rep_distances)):
                    if not math.isnan(rep_distances[i]) and not math.isnan(rep_distances[i-1]):
                        if abs(rep_distances[i] - rep_distances[i-1]) > jump_tol:
                            rep_jump_ok = False
                            break
                lines.append("  复现结果是否满足约束稳定性:")
                lines.append(f"    无漂移 (|d-d0|<{drift_tol}m): {'是' if rep_drift_ok else '否'}")
                lines.append(f"    无跳变 (相邻采样差<{jump_tol}m): {'是' if rep_jump_ok else '否'}")
            lines.append("")
            JC_THRESHOLD = 0.05
            if max_dev <= JC_THRESHOLD:
                conclusion = f"一致（约束距离最大偏差 {max_dev:.4f}m <= {JC_THRESHOLD}m）"
                match = True
            else:
                conclusion = f"不一致（约束距离最大偏差 {max_dev:.4f}m > {JC_THRESHOLD}m）"
                match = False
            lines.append(f"  → 结论: {conclusion}")
            lines.append("=" * 60)
            report_text = "\n".join(lines)
            print(report_text)
            if output_dir:
                report_path = os.path.join(output_dir, "reproduction_report.txt")
                with open(report_path, "w") as f:
                    f.write(report_text)
                    f.write("\n\n--- 原始结果文件 ---\n")
                    f.write(original_info.get('raw_text', ''))
                    f.write("\n\n--- 复现详情 ---\n")
                    f.write(json.dumps(replay_result, indent=2, default=str))
                print(f"\n报告已保存: {report_path}")
            return match

    # ===== Temporal Monotonicity 专用比较 =====
    orig_traj = original_info.get('monotonicity_trajectory')
    init_pos = captured.get('monotonicity initial')
    def _mono_sample_idx(k):
        m = re.search(r'sample (\d+)', k)
        return int(m.group(1)) if m else 0
    sample_keys = sorted(
        [k for k in (captured or {}) if k.startswith('monotonicity sample')],
        key=_mono_sample_idx
    )
    if orig_traj is not None and init_pos is not None and sample_keys:
        sample_positions = [captured[k] for k in sample_keys]

        # 构造复现的 x-displacement 序列：初态为 0，之后为 相对初态的 x 位移
        rep_x_disp = [0.0] + [p[0] - init_pos[0] for p in sample_positions]

        # 1. 序列对比
        lines.append("  [Temporal Monotonicity] 轨迹序列对比:")
        orig_x_disp = [pt[1] for pt in orig_traj]
        n_compare = min(len(orig_x_disp), len(rep_x_disp))
        max_traj_dev = 0
        for i in range(n_compare):
            t_val = orig_traj[i][0] if i < len(orig_traj) else 0
            od = orig_x_disp[i]
            rd = rep_x_disp[i]
            dev = abs(od - rd)
            max_traj_dev = max(max_traj_dev, dev)
            lines.append(f"    t={t_val:.2f}s: 原始 x_disp={od:.6f}m, 复现 x_disp={rd:.6f}m, 偏差={dev:.6f}m")
        lines.append(f"  轨迹最大偏差: {max_traj_dev:.6f} m")
        lines.append("")

        # 2. 判断复现结果是否符合预期（单调且平滑）
        rep_monotonic = True
        rep_smooth = True
        rep_violations = []
        MONOTONICITY_JUMP_RATIO = 3.0

        for i in range(1, len(rep_x_disp)):
            if rep_x_disp[i] <= rep_x_disp[i - 1]:
                rep_monotonic = False
                rep_violations.append({'type': 'non_monotonic', 'index': i, 'prev': rep_x_disp[i-1], 'curr': rep_x_disp[i]})
        deltas = [rep_x_disp[i] - rep_x_disp[i-1] for i in range(1, len(rep_x_disp))]
        for i in range(1, len(deltas)):
            if deltas[i-1] > 0.001:
                ratio = deltas[i] / deltas[i-1]
                if ratio > MONOTONICITY_JUMP_RATIO:
                    rep_smooth = False
                    rep_violations.append({'type': 'jump', 'index': i+1, 'ratio': ratio, 'delta_prev': deltas[i-1], 'delta_curr': deltas[i]})

        lines.append("  复现结果是否符合预期:")
        lines.append(f"    单调性 (Monotonic): {'是' if rep_monotonic else '否'}")
        lines.append(f"    平滑性 (Smooth):    {'是' if rep_smooth else '否'}")
        if rep_violations:
            lines.append(f"    违规: {len(rep_violations)} 处")
            for v in rep_violations[:5]:
                if v['type'] == 'non_monotonic':
                    lines.append(f"      样本 {v['index']}: 位移非单调 ({v['prev']:.6f} -> {v['curr']:.6f})")
                else:
                    lines.append(f"      样本 {v['index']}: 突变比例 {v['ratio']:.2f}x (delta {v['delta_prev']:.6f} -> {v['delta_curr']:.6f})")
        lines.append("")
        meets_expectation = rep_monotonic and rep_smooth
        conclusion = "符合预期（单调且平滑）" if meets_expectation else "不符合预期（存在非单调或突变）"
        lines.append(f"  → 结论: {conclusion}")
        lines.append("=" * 60)
        report_text = "\n".join(lines)
        print(report_text)
        if output_dir:
            report_path = os.path.join(output_dir, "reproduction_report.txt")
            with open(report_path, "w") as f:
                f.write(report_text)
                f.write("\n\n--- 原始结果文件 ---\n")
                f.write(original_info.get('raw_text', ''))
                f.write("\n\n--- 复现详情 ---\n")
                f.write(json.dumps(replay_result, indent=2, default=str))
            print(f"\n报告已保存: {report_path}")
        return meets_expectation

    # ===== 通用比较逻辑 =====
    # 获取原始测试中的两组关键位置（Determinism 优先用 Run 1 / Run 2）
    orig_pos_pairs = []  # [(label, orig_pos)]
    if 'pos_run1' in original_info and 'pos_run2' in original_info:
        orig_pos_pairs.append(('Run 1', original_info['pos_run1']))
        orig_pos_pairs.append(('Run 2', original_info['pos_run2']))
    else:
        for key, label in [('pos_f1f2', 'F1+F2'), ('pos_ftotal', 'F_total'),
                           ('pos_rtf1', 'RTF1'), ('pos_rtf2', 'RTF2'),
                           ('pos_run_a', 'Run A'), ('pos_run_b', 'Run B'),
                           ('final_pos', 'Final')]:
            if key in original_info:
                orig_pos_pairs.append((label, original_info[key]))

    # 获取复现中捕获的位置（按顺序）
    captured_list = list(captured.values())

    # 尝试逐位置比较
    orig_result = original_info.get('result', 'UNKNOWN')

    # 如果原始结果是 ERROR，复现中也没有捕获到位置，则两者一致（都失败）
    if orig_result == 'ERROR' and not captured_list:
        conclusion = "一致（两次均执行异常）"
        match = True
    elif orig_result == 'UNKNOWN' or not orig_pos_pairs:
        conclusion = "无法比较（原始结果缺少位置数据）"
        match = None
    elif not captured_list:
        conclusion = "无法比较（复现未捕获到位置）"
        match = None
    else:
        # 比较捕获位置与原始位置
        lines.append("  位置比较:")
        max_deviation = 0
        comparison_count = min(len(orig_pos_pairs), len(captured_list))
        for idx in range(comparison_count):
            olabel, opos = orig_pos_pairs[idx]
            rpos = captured_list[idx]
            dx = abs(opos[0] - rpos[0])
            dy = abs(opos[1] - rpos[1])
            dz = abs(opos[2] - rpos[2])
            dev = math.sqrt(dx**2 + dy**2 + dz**2)
            max_deviation = max(max_deviation, dev)
            lines.append(f"    [{olabel}]")
            lines.append(f"      原始:  ({opos[0]:.6f}, {opos[1]:.6f}, {opos[2]:.6f})")
            lines.append(f"      复现:  ({rpos[0]:.6f}, {rpos[1]:.6f}, {rpos[2]:.6f})")
            lines.append(f"      偏差:  {dev:.6f} m (x={dx:.6f}, y={dy:.6f}, z={dz:.6f})")

        lines.append("")
        lines.append(f"  最大位置偏差: {max_deviation:.6f} m")

        # Determinism：额外报告复现时 Run 1 vs Run 2 的差异（是否仍非确定）
        if 'pos_run1' in original_info and len(captured_list) >= 2:
            r1, r2 = captured_list[0], captured_list[1]
            det_dx = abs(r1[0] - r2[0])
            det_dy = abs(r1[1] - r2[1])
            det_dz = abs(r1[2] - r2[2])
            det_dev = math.sqrt(det_dx**2 + det_dy**2 + det_dz**2)
            lines.append(f"  复现时 Run1 vs Run2 位置差: {det_dev:.6f} m (非确定性复现)" if det_dev >= 0.001 else "  复现时 Run1 vs Run2 一致（未复现非确定性）")
            lines.append("")

        # 判断是否一致（偏差小于 1m 认为基本一致）
        REPRODUCE_THRESHOLD = 1.0
        if max_deviation < REPRODUCE_THRESHOLD:
            conclusion = f"一致（最大偏差 {max_deviation:.4f}m < {REPRODUCE_THRESHOLD}m）"
            match = True
        else:
            conclusion = f"不一致（最大偏差 {max_deviation:.4f}m >= {REPRODUCE_THRESHOLD}m）"
            match = False

    lines.append("")
    symbol = "✓" if match is True else ("✗" if match is False else "?")
    lines.append(f"  {symbol} 结论: {conclusion}")
    lines.append("=" * 60)

    report_text = "\n".join(lines)
    print(report_text)

    # 保存报告
    if output_dir:
        report_path = os.path.join(output_dir, "reproduction_report.txt")
        with open(report_path, "w") as f:
            f.write(report_text)
            f.write("\n\n--- 原始结果文件 ---\n")
            f.write(original_info.get('raw_text', ''))
            f.write("\n\n--- 复现详情 ---\n")
            f.write(json.dumps(replay_result, indent=2, default=str))
        print(f"\n报告已保存: {report_path}")

    return match


# ============================================================
# 单次复现入口
# ============================================================

def reproduce_one(exp_dir, output_dir=None):
    """
    复现单次实验的完整流程。

    Args:
        exp_dir: 原始实验目录
        output_dir: 输出目录（可选）

    Returns:
        True=结果一致, False=结果不一致, None=无法比较
    """
    exp_dir = os.path.abspath(exp_dir)
    sdf_path = os.path.join(exp_dir, "a.sdf")
    log_path = os.path.join(exp_dir, "experiment_log.json")
    result_path = os.path.join(exp_dir, "metamorphic_test_result.txt")

    print(f"\n{'='*60}")
    print(f"  复现实验: {exp_dir}")
    print(f"{'='*60}")

    # 检查文件
    for fp, name in [(sdf_path, "a.sdf"), (log_path, "experiment_log.json")]:
        if not os.path.exists(fp):
            print(f"  ERROR: 缺少 {name}")
            return None

    # 读取日志
    with open(log_path) as f:
        log_data = json.load(f)

    # 读取原始结果
    original_info = parse_original_result(result_path) if os.path.exists(result_path) else {}
    print(f"  原始结果: {original_info.get('result', 'N/A')}")
    print(f"  测试类型: {original_info.get('test_type', 'N/A')}")
    print(f"  日志步骤数: {len(log_data)}")

    # 准备输出目录
    if output_dir is None:
        output_dir = os.path.join(exp_dir, f"reproduce_{int(time.time())}")
    os.makedirs(output_dir, exist_ok=True)

    # 清理
    print("\n  清理残留进程...")
    kill_all_gz()

    # 回放
    print(f"\n  开始回放实验:")
    replay_result = replay_from_log(sdf_path, log_data, output_dir)

    # 清理
    kill_all_gz()

    # 比较
    print()
    match = compare_and_report(original_info, replay_result, output_dir)

    return match


# ============================================================
# 批量复现入口
# ============================================================

def reproduce_batch(test_dir, output_dir, filter_result=None, filter_type=None, max_count=0):
    """批量复现"""
    test_dir = os.path.abspath(test_dir)
    os.makedirs(output_dir, exist_ok=True)

    # 收集实验目录
    exp_dirs = []
    for name in sorted(os.listdir(test_dir),
                       key=lambda x: int(x[1:]) if x.startswith('_') and x[1:].isdigit() else 0):
        if not name.startswith('_'):
            continue
        d = os.path.join(test_dir, name)
        if not os.path.isdir(d) or not os.path.exists(os.path.join(d, "experiment_log.json")):
            continue

        # 过滤
        if filter_result or filter_type:
            rp = os.path.join(d, "metamorphic_test_result.txt")
            info = parse_original_result(rp) if os.path.exists(rp) else {}
            if filter_result and info.get('result', '').upper() != filter_result.upper():
                continue
            if filter_type:
                tt = info.get('test_type', '').lower().replace(' ', '_')
                if filter_type.lower() not in tt:
                    continue

        exp_dirs.append(d)
        if 0 < max_count <= len(exp_dirs):
            break

    print(f"找到 {len(exp_dirs)} 个实验需要复现")
    if not exp_dirs:
        return

    results = []
    for idx, d in enumerate(exp_dirs):
        out = os.path.join(output_dir, os.path.basename(d))
        print(f"\n{'#'*60}")
        print(f"# [{idx+1}/{len(exp_dirs)}] {os.path.basename(d)}")
        print(f"{'#'*60}")
        try:
            match = reproduce_one(d, out)
        except Exception as e:
            print(f"  异常: {e}")
            match = None
        results.append((os.path.basename(d), match))
        kill_all_gz()
        time.sleep(2)

    # 汇总
    print(f"\n\n{'='*60}")
    print(f"  批量复现汇总 ({len(results)} 个实验)")
    print(f"{'='*60}")
    same = sum(1 for _, m in results if m is True)
    diff = sum(1 for _, m in results if m is False)
    fail = sum(1 for _, m in results if m is None)
    print(f"  结果一致: {same}")
    print(f"  结果不一致: {diff}")
    print(f"  无法比较: {fail}")
    print()
    for name, match in results:
        s = "✓ 一致" if match is True else ("✗ 不一致" if match is False else "? 无法比较")
        print(f"    {name}: {s}")
    print(f"{'='*60}")

    # 保存汇总
    summary_path = os.path.join(output_dir, "batch_summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"批量复现汇总\n时间: {datetime.now()}\n\n")
        f.write(f"一致: {same}, 不一致: {diff}, 无法比较: {fail}\n\n")
        for name, match in results:
            s = "一致" if match is True else ("不一致" if match is False else "无法比较")
            f.write(f"  {name}: {s}\n")
    print(f"\n汇总已保存: {summary_path}")


# ============================================================
# 主入口
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="实验复现与结果比较工具")
    parser.add_argument("path", help="实验目录（单次）或测试总目录（批量）")
    parser.add_argument("-o", "--output", default=None, help="输出目录")
    parser.add_argument("--batch", action="store_true", help="批量模式")
    parser.add_argument("--filter-result", default=None, help="过滤: PASSED/FAILED/ERROR")
    parser.add_argument("--filter-type", default=None, help="过滤: motion/force_additivity/time_scaling/rewind")
    parser.add_argument("--max", type=int, default=0, help="批量模式最大数量")
    args = parser.parse_args()

    if args.batch:
        out = args.output or os.path.join(args.path, f"reproduce_{int(time.time())}")
        reproduce_batch(args.path, out, args.filter_result, args.filter_type, args.max)
    else:
        reproduce_one(args.path, args.output)
