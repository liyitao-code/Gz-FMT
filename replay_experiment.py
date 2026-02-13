#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实验复现脚本

根据 experiment_log.json 文件自动复现实验流程

使用方法:
    python replay_experiment.py <experiment_log.json>
    
或者:
    python replay_experiment.py <test_directory>
    (会自动查找 test_directory/experiment_log.json)
"""

import json
import sys
import os
import time
import subprocess
import signal
from datetime import datetime

def execute_command(cmd, description=""):
    """
    执行命令
    
    Args:
        cmd: 命令字符串
        description: 描述信息
    
    Returns:
        命令执行结果
    """
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Executing: {description}")
    print(f"  Command: {cmd}")
    
    try:
        if cmd.startswith("gz sim"):
            # 启动命令，需要在后台运行
            process = subprocess.Popen(
                cmd.split(" "), 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                start_new_session=True
            )
            print(f"  Process started with PID: {process.pid}")
            return process
        else:
            # 普通命令
            result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=100)
            if result.returncode == 0:
                print(f"  Success")
                if result.stdout:
                    print(f"  Output: {result.stdout.decode('utf-8')[:200]}")
            else:
                print(f"  Warning: Command returned non-zero exit code {result.returncode}")
                if result.stderr:
                    print(f"  Error: {result.stderr.decode('utf-8')[:200]}")
            return result
    except subprocess.TimeoutExpired:
        print(f"  Error: Command timed out")
        return None
    except Exception as e:
        print(f"  Error: {e}")
        return None

def replay_experiment(log_file_path):
    """
    根据实验日志文件复现实验
    
    Args:
        log_file_path: 实验日志JSON文件路径
    """
    # 读取实验日志
    if not os.path.exists(log_file_path):
        print(f"Error: Log file not found: {log_file_path}")
        return False
    
    with open(log_file_path, 'r') as f:
        experiment_log = json.load(f)
    
    print(f"Loading experiment log from: {log_file_path}")
    print(f"Total steps: {len(experiment_log)}")
    
    # 统计各类型条目的数量
    type_counts = {}
    for entry in experiment_log:
        entry_type = entry.get("type", "unknown")
        type_counts[entry_type] = type_counts.get(entry_type, 0) + 1
    
    print(f"Entry types: {', '.join([f'{k}: {v}' for k, v in type_counts.items()])}")
    print("=" * 80)
    
    # 提取实验信息
    experiment_info = None
    test_type = None
    for entry in experiment_log:
        if entry.get("type") == "experiment_info" and "start_time" in entry:
            experiment_info = entry
        if entry.get("type") == "test_info":
            test_type = entry.get("test_type")
    
    if experiment_info:
        print(f"Experiment Directory: {experiment_info.get('directory', 'N/A')}")
        print(f"SDF File: {experiment_info.get('sdf_file', 'N/A')}")
        print(f"Test Type: {test_type or 'N/A'}")
        print("=" * 80)
    
    # 执行实验步骤
    process = None
    try:
        for i, entry in enumerate(experiment_log):
            entry_type = entry.get("type")
            
            if entry_type == "command":
                command = entry.get("command")
                if not command:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Warning: Command entry {i+1} has no command field, skipping")
                    continue
                
                command_type = entry.get("command_type", "unknown")
                description = entry.get("description", f"{command_type} command {i+1}")
                wait_after = entry.get("wait_after", 0.0)
                
                if command_type == "launch":
                    # 启动命令（gz sim）
                    process = execute_command(command, description)
                    if wait_after > 0:
                        print(f"  Waiting {wait_after} seconds after launch...")
                        time.sleep(wait_after)
                elif command_type in ["service", "topic"]:
                    # gz service 或 gz topic 命令
                    execute_command(command, description)
                    if wait_after > 0:
                        print(f"  Waiting {wait_after} seconds...")
                        time.sleep(wait_after)
                else:
                    # 其他类型的命令（向后兼容）
                    execute_command(command, description)
                    if wait_after > 0:
                        print(f"  Waiting {wait_after} seconds...")
                        time.sleep(wait_after)
            
            elif entry_type == "sleep":
                duration = entry.get("duration", 0.0)
                description = entry.get("description", f"Sleep {i+1}")
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Sleeping: {description}")
                print(f"  Duration: {duration} seconds")
                time.sleep(duration)
            
            elif entry_type == "experiment_info":
                # 实验信息，跳过执行
                continue
            
            elif entry_type == "test_info":
                # 测试信息，只打印
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Test Info: {entry.get('test_type', 'N/A')}")
                continue
            
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Unknown entry type: {entry_type}")
        
        print("=" * 80)
        print("Experiment replay completed!")
        
        # 如果启动了进程，等待用户中断
        if process:
            print(f"\nGazebo process is running (PID: {process.pid})")
            print("Press Ctrl+C to stop...")
            try:
                process.wait()
            except KeyboardInterrupt:
                print("\nStopping Gazebo process...")
                process.terminate()
                process.wait()
                print("Gazebo process stopped.")
        
        return True
        
    except KeyboardInterrupt:
        print("\nExperiment replay interrupted by user")
        if process:
            print("Stopping Gazebo process...")
            process.terminate()
            process.wait()
        return False
    except Exception as e:
        print(f"\nError during replay: {e}")
        import traceback
        traceback.print_exc()
        if process:
            print("Stopping Gazebo process...")
            process.terminate()
            process.wait()
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python replay_experiment.py <experiment_log.json>")
        print("   or: python replay_experiment.py <test_directory>")
        sys.exit(1)
    
    log_path = sys.argv[1]
    
    # 如果是目录，查找 experiment_log.json
    if os.path.isdir(log_path):
        log_path = os.path.join(log_path, "experiment_log.json")
    
    if not os.path.exists(log_path):
        print(f"Error: Log file not found: {log_path}")
        sys.exit(1)
    
    success = replay_experiment(log_path)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()

