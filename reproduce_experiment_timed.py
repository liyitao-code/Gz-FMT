#!/usr/bin/env python3
# coding: utf-8
"""
带时间分析的实验复现工具

在 reproduce_experiment.py 基础上增加了 sim-time 追踪功能：
- 识别 experiment_log 中 description 含 [TIMING] 的命令
- 执行后解析 gz topic stats 输出中的 sim_time、real_time、iterations 等
- 最终输出时间分析报告，验证 wall-clock time 与 sim-time 的差异

用法：
    python3 reproduce_experiment_timed.py /path/to/experiment_dir
    python3 reproduce_experiment_timed.py /path/to/experiment_dir --log experiment_log_timed.json
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
from collections import OrderedDict

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


def parse_sim_time_from_stats(output):
    """
    从 gz topic stats 输出中解析 sim_time、real_time、iterations、paused、real_time_factor。

    输出格式示例：
        sim_time {
          sec: 5
          nsec: 500000000
        }
        real_time {
          sec: 5
          nsec: 800000000
        }
        iterations: 5500
        paused: true
        real_time_factor: 1.06

    返回 dict，例如：
        {'sim_time': 5.5, 'real_time': 5.8, 'iterations': 5500,
         'paused': True, 'real_time_factor': 1.06}
    """
    result = {}

    # 解析 sim_time { sec: X  nsec: Y }
    m = re.search(r'sim_time\s*\{([^}]*)\}', output, re.DOTALL)
    if m:
        block = m.group(1)
        sec = re.search(r'sec:\s*(\d+)', block)
        nsec = re.search(r'nsec:\s*(\d+)', block)
        if sec:
            s = int(sec.group(1))
            ns = int(nsec.group(1)) if nsec else 0
            result['sim_time'] = s + ns / 1e9

    # 解析 real_time { sec: X  nsec: Y }
    m = re.search(r'real_time\s*\{([^}]*)\}', output, re.DOTALL)
    if m:
        block = m.group(1)
        sec = re.search(r'sec:\s*(\d+)', block)
        nsec = re.search(r'nsec:\s*(\d+)', block)
        if sec:
            s = int(sec.group(1))
            ns = int(nsec.group(1)) if nsec else 0
            result['real_time'] = s + ns / 1e9

    # iterations
    m = re.search(r'iterations:\s*(\d+)', output)
    if m:
        result['iterations'] = int(m.group(1))

    # paused
    m = re.search(r'paused:\s*(true|false)', output)
    if m:
        result['paused'] = m.group(1) == 'true'

    # real_time_factor
    m = re.search(r'real_time_factor:\s*([\d.e+-]+)', output)
    if m:
        try:
            result['real_time_factor'] = float(m.group(1))
        except ValueError:
            pass

    return result


def get_model_pose_from_topic(world_name, model_name, timeout_sec=3.0):
    """
    通过 topic 获取模型实时位置。
    优先使用 Python gz transport API，失败时回退到 gz topic 命令行。
    返回 (x,y,z) 或 None。
    """
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
        pass
    except Exception as e:
        print(f"  [WARN] get_model_pose Python API error: {e}")

    # 回退：gz topic 命令行
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
            blocks = re.split(r'(?=pose\s*\{)', output)
            for block in blocks:
                if f'name: "{model_name}"' in block:
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


# ============================================================
# 核心：带时间追踪的日志回放
# ============================================================

def replay_from_log_timed(sdf_path, log_data, output_dir=None):
    """
    严格按照 experiment_log 回放实验，同时追踪 sim-time。

    识别 description 中含 [TIMING] 的命令，执行后解析 stats 输出。

    Returns:
        dict: {
            'world_name': str,
            'model_name': str,
            'steps_executed': int,
            'positions_captured': {description: (x,y,z)},
            'timing_data': OrderedDict of {label: {sim_time, real_time, ...}},
            'wall_clock_times': {label: float},  # wall-clock 时间戳
            'errors': [str],
        }
    """
    result = {
        'world_name': None,
        'model_name': None,
        'steps_executed': 0,
        'positions_captured': {},
        'timing_data': OrderedDict(),
        'wall_clock_times': OrderedDict(),
        'errors': [],
    }

    # 提取 world_name
    world_name = None
    model_name = None
    for entry in log_data:
        if entry.get('type') == 'command':
            cmd = entry.get('command', '')
            m = re.search(r'/world/([^/]+)/', cmd)
            if m:
                world_name = m.group(1)
                break
    for entry in log_data:
        if entry.get('type') == 'command':
            cmd = entry.get('command', '')
            m = re.search(r'name:\s*"([^"]+)".*type:\s*MODEL', cmd)
            if m:
                model_name = m.group(1)
                break

    result['world_name'] = world_name
    result['model_name'] = model_name

    # 启动 Gazebo
    gz_cmd = f"gz sim {sdf_path}"
    print(f"  启动 Gazebo: {gz_cmd}")
    gz_out = subprocess.DEVNULL
    gz_err = subprocess.DEVNULL
    if output_dir:
        gz_out = open(os.path.join(output_dir, "gz.out"), "w")
        gz_err = open(os.path.join(output_dir, "gz.err"), "w")

    gz_process = subprocess.Popen(gz_cmd.split(), stdout=gz_out, stderr=gz_err,
                                  start_new_session=True)

    try:
        for i, entry in enumerate(log_data):
            entry_type = entry.get('type', '')

            if entry_type == 'experiment_info' or entry_type == 'test_info':
                desc = entry.get('description', '')
                if desc:
                    print(f"  --- {desc} ---")
                continue

            elif entry_type == 'command':
                cmd_type = entry.get('command_type', '')
                cmd = entry.get('command', '')
                desc = entry.get('description', '')

                if cmd_type == 'launch':
                    wait = entry.get('wait_after', 0)
                    if wait > 0:
                        print(f"  [{i:2d}] launch → 等待 Gazebo 启动 {wait}s")
                        time.sleep(wait)
                    else:
                        print(f"  [{i:2d}] launch → 等待 Gazebo 启动 2s")
                        time.sleep(2)
                    result['steps_executed'] += 1
                    continue

                # 是否是 [TIMING] 命令
                is_timing = '[TIMING]' in desc

                print(f"  [{i:2d}] {cmd_type:12s}: {cmd[:100]}{'...' if len(cmd)>100 else ''}")

                # 记录 wall-clock 时间
                wall_before = time.time()
                ok, out = exec_command(cmd, timeout=15)
                wall_after = time.time()

                if not ok:
                    if is_timing:
                        print(f"         ⚠ [TIMING] 命令失败: {out[:80]}")
                    result['errors'].append(f"Step {i}: command failed: {cmd[:80]}")
                elif is_timing:
                    # 解析 stats 输出
                    stats = parse_sim_time_from_stats(out)
                    label = desc.replace('[TIMING] ', '')
                    result['timing_data'][label] = stats
                    result['wall_clock_times'][label] = wall_after

                    sim_t = stats.get('sim_time', '?')
                    real_t = stats.get('real_time', '?')
                    iters = stats.get('iterations', '?')
                    rtf = stats.get('real_time_factor', '?')
                    paused = stats.get('paused', '?')

                    print(f"         ⏱ sim_time={sim_t}s  real_time={real_t}s  "
                          f"iterations={iters}  RTF={rtf}  paused={paused}")

                result['steps_executed'] += 1

            elif entry_type == 'sleep':
                duration = entry.get('duration', 0)
                desc = entry.get('description', '')

                is_test_run = any(kw in desc for kw in ['Running test', 'Test A:', 'Test B:'])

                if duration > 0:
                    print(f"  [{i:2d}] sleep {duration:.2f}s  {desc[:80]}")
                    # 记录 sleep 开始的 wall-clock
                    if is_test_run:
                        result['wall_clock_times'][f"wall_sleep_start:{desc[:60]}"] = time.time()
                    time.sleep(duration)
                    if is_test_run:
                        result['wall_clock_times'][f"wall_sleep_end:{desc[:60]}"] = time.time()

                # 捕获位置
                if is_test_run and world_name and model_name:
                    pos = get_model_pose_from_topic(world_name, model_name)
                    if pos:
                        label = desc[:60]
                        result['positions_captured'][label] = pos
                        print(f"         → 捕获位置: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

                result['steps_executed'] += 1

            else:
                pass

    except KeyboardInterrupt:
        print("\n  中断!")
        result['errors'].append("User interrupted")
    except Exception as e:
        print(f"\n  异常: {e}")
        result['errors'].append(str(e))
    finally:
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
# 时间分析报告
# ============================================================

def print_timing_report(replay_result):
    """打印详细的时间分析报告"""
    timing = replay_result.get('timing_data', {})
    wall_times = replay_result.get('wall_clock_times', {})
    positions = replay_result.get('positions_captured', {})

    lines = []
    lines.append("")
    lines.append("=" * 70)
    lines.append("                    时间分析报告")
    lines.append("=" * 70)

    if not timing:
        lines.append("  未捕获到任何 [TIMING] 数据。")
        lines.append("  请确保 experiment_log 中包含 [TIMING] 标记的 stats 查询命令。")
        print("\n".join(lines))
        return

    # 打印所有 timing 数据
    lines.append("")
    lines.append("  ┌─ 各检测点的 sim-time ─────────────────────────────────┐")
    for label, stats in timing.items():
        sim_t = stats.get('sim_time', None)
        real_t = stats.get('real_time', None)
        iters = stats.get('iterations', None)
        rtf = stats.get('real_time_factor', None)
        sim_str = f"{sim_t:.6f}s" if sim_t is not None else "N/A"
        real_str = f"{real_t:.6f}s" if real_t is not None else "N/A"
        iter_str = f"{iters}" if iters is not None else "N/A"
        rtf_str = f"{rtf:.4f}" if rtf is not None else "N/A"
        lines.append(f"  │ {label}")
        lines.append(f"  │   sim_time={sim_str}  real_time={real_str}")
        lines.append(f"  │   iterations={iter_str}  RTF={rtf_str}")
        lines.append(f"  │")
    lines.append("  └──────────────────────────────────────────────────────┘")

    # 计算 Test A 和 Test B 的 sim-time 差值
    lines.append("")
    lines.append("  ┌─ sim-time 差值分析 ────────────────────────────────────┐")

    test_a_before = timing.get('Test A: sim-time BEFORE resume', {}).get('sim_time')
    test_a_after = timing.get('Test A: sim-time AFTER pause', {}).get('sim_time')
    test_b_before = timing.get('Test B: sim-time BEFORE resume', {}).get('sim_time')
    test_b_after = timing.get('Test B: sim-time AFTER pause', {}).get('sim_time')

    wall_clock_duration = 3.4031505696535533  # 来自 experiment_log 的 sleep duration

    if test_a_before is not None and test_a_after is not None:
        delta_a = test_a_after - test_a_before
        rtf_a = delta_a / wall_clock_duration if wall_clock_duration > 0 else 0
        lines.append(f"  │ Test A:")
        lines.append(f"  │   sim-time: {test_a_before:.6f}s → {test_a_after:.6f}s")
        lines.append(f"  │   Δsim-time  = {delta_a:.6f}s")
        lines.append(f"  │   wall-clock = {wall_clock_duration:.6f}s")
        lines.append(f"  │   有效 RTF   = {rtf_a:.4f}x (sim-time / wall-clock)")
        lines.append(f"  │")

    if test_b_before is not None and test_b_after is not None:
        delta_b = test_b_after - test_b_before
        rtf_b = delta_b / wall_clock_duration if wall_clock_duration > 0 else 0
        lines.append(f"  │ Test B:")
        lines.append(f"  │   sim-time: {test_b_before:.6f}s → {test_b_after:.6f}s")
        lines.append(f"  │   Δsim-time  = {delta_b:.6f}s")
        lines.append(f"  │   wall-clock = {wall_clock_duration:.6f}s")
        lines.append(f"  │   有效 RTF   = {rtf_b:.4f}x (sim-time / wall-clock)")
        lines.append(f"  │")

    if (test_a_before is not None and test_a_after is not None and
            test_b_before is not None and test_b_after is not None):
        delta_a = test_a_after - test_a_before
        delta_b = test_b_after - test_b_before
        diff = abs(delta_b - delta_a)
        ratio = delta_b / delta_a if delta_a > 0 else float('inf')

        lines.append(f"  │ 对比:")
        lines.append(f"  │   Test A Δsim-time = {delta_a:.6f}s")
        lines.append(f"  │   Test B Δsim-time = {delta_b:.6f}s")
        lines.append(f"  │   差异 = {diff:.6f}s  (比值 = {ratio:.4f})")
        lines.append(f"  │")

        if diff > 0.1:
            lines.append(f"  │   ⚠ Test A 和 Test B 的 sim-time 差异 {diff:.3f}s > 0.1s")
            lines.append(f"  │     这意味着相同的 wall-clock 等待时间，")
            lines.append(f"  │     两轮测试实际模拟了不同长度的 sim-time！")
            lines.append(f"  │     → wall-clock timing 问题确认存在")
        else:
            lines.append(f"  │   ✓ Test A 和 Test B 的 sim-time 差异 {diff:.3f}s ≤ 0.1s")
            lines.append(f"  │     sim-time 基本一致，误差不太可能来自 timing 问题")

    lines.append("  └──────────────────────────────────────────────────────┘")

    # 位置报告
    if positions:
        lines.append("")
        lines.append("  ┌─ 捕获的位置 ──────────────────────────────────────────┐")
        for label, pos in positions.items():
            lines.append(f"  │ {label}")
            lines.append(f"  │   ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
        lines.append("  └──────────────────────────────────────────────────────┘")

    lines.append("")
    lines.append("=" * 70)

    report = "\n".join(lines)
    print(report)
    return report


# ============================================================
# 主入口
# ============================================================

def reproduce_timed(exp_dir, log_filename="experiment_log_timed.json", output_dir=None):
    """
    执行带时间追踪的复现。
    """
    exp_dir = os.path.abspath(exp_dir)
    sdf_path = os.path.join(exp_dir, "a.sdf")
    log_path = os.path.join(exp_dir, log_filename)

    print(f"\n{'='*70}")
    print(f"  带时间追踪的实验复现")
    print(f"  实验目录: {exp_dir}")
    print(f"  日志文件: {log_filename}")
    print(f"{'='*70}")

    # 检查文件
    for fp, name in [(sdf_path, "a.sdf"), (log_path, log_filename)]:
        if not os.path.exists(fp):
            print(f"  ERROR: 缺少 {name}")
            return None

    # 读取日志
    with open(log_path) as f:
        log_data = json.load(f)

    print(f"  日志步骤数: {len(log_data)}")

    # 输出目录
    if output_dir is None:
        output_dir = os.path.join(exp_dir, f"reproduce_timed_{int(time.time())}")
    os.makedirs(output_dir, exist_ok=True)

    # 清理
    print("\n  清理残留进程...")
    kill_all_gz()

    # 回放
    print(f"\n  开始回放实验:")
    replay_result = replay_from_log_timed(sdf_path, log_data, output_dir)

    # 清理
    kill_all_gz()

    # 时间分析报告
    report = print_timing_report(replay_result)

    # 保存报告
    if output_dir and report:
        report_path = os.path.join(output_dir, "timing_report.txt")
        with open(report_path, "w") as f:
            f.write(report)
            f.write("\n\n--- 完整复现数据 ---\n")
            f.write(json.dumps(replay_result, indent=2, default=str))
        print(f"\n报告已保存: {report_path}")

    return replay_result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="带时间分析的实验复现工具")
    parser.add_argument("path", help="实验目录路径")
    parser.add_argument("--log", default="experiment_log_timed.json",
                        help="日志文件名 (默认: experiment_log_timed.json)")
    parser.add_argument("-o", "--output", default=None, help="输出目录")
    args = parser.parse_args()

    reproduce_timed(args.path, args.log, args.output)
