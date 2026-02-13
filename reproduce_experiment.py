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


# ============================================================
# 核心：按日志回放实验
# ============================================================

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

    # 启动 Gazebo（不使用日志中的 launch 命令，因为路径可能不同）
    gz_cmd = f"gz sim {sdf_path}"
    print(f"  启动 Gazebo: {gz_cmd}")
    gz_out = subprocess.DEVNULL
    gz_err = subprocess.DEVNULL
    if output_dir:
        gz_out = open(os.path.join(output_dir, "gz.out"), "w")
        gz_err = open(os.path.join(output_dir, "gz.err"), "w")

    gz_process = subprocess.Popen(gz_cmd.split(), stdout=gz_out, stderr=gz_err, start_new_session=True)

    try:
        # 逐条执行日志
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

                # 在测试运行结束后（下一步是 pause）捕获位置
                if is_test_run and world_name and model_name:
                    pos = get_model_pose_from_topic(world_name, model_name)
                    if pos:
                        label = desc[:60]
                        result['positions_captured'][label] = pos
                        print(f"         → 捕获位置: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

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
    if captured:
        lines.append("  复现捕获的位置:")
        for label, pos in captured.items():
            lines.append(f"    {label}:")
            lines.append(f"      ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
    else:
        lines.append("  复现中未捕获到位置（可能 Gazebo 未正常启动或模型名不匹配）")

    lines.append("")

    # ===== 比较逻辑 =====
    # 获取原始测试中的两组关键位置
    orig_pos_pairs = []  # [(label, orig_pos)]
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
