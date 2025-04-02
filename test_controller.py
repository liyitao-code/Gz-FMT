#!/usr/bin/env python3

import subprocess
import json
import time
import os
import glob
import threading
import sys
import signal
import psutil
import random
import shutil
from coverage_process import CoverageInfo, CoverageDiff
import argparse

# 添加 Gazebo Python 库路径
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')

# 全局变量
BUILD_DIR = "/home/liyitao/workspace/gz_lastest/build/"  # 修改为与 randomsmith.py 相同的路径
GCOV_DIR = "gcov"

class TeeOutput:
    """同时输出到文件和终端的工具类"""
    def __init__(self, file, terminal):
        self.file = file
        self.terminal = terminal
        self._lock = threading.Lock()
        self._closed = False

    def write(self, message):
        with self._lock:
            if not self._closed:
                try:
                    self.terminal.write(message)
                    self.file.write(message)
                    self.file.flush()
                    self.terminal.flush()
                except (ValueError, IOError) as e:
                    # 如果文件已关闭，标记为closed并继续输出到终端
                    if "closed file" in str(e):
                        self._closed = True
                        self.terminal.write(message)
                        self.terminal.flush()
                    else:
                        raise

    def flush(self):
        with self._lock:
            if not self._closed:
                try:
                    self.file.flush()
                except (ValueError, IOError):
                    self._closed = True
            self.terminal.flush()
    
    def close(self):
        with self._lock:
            self._closed = True

def _run_process_with_tee(cmd, log_file, err_file, wait=False):
    """运行进程并使用Tee输出重定向
    
    Args:
        cmd: 要运行的命令
        log_file: 标准输出日志文件
        err_file: 错误输出日志文件
        wait: 是否等待进程完成。如果为False，则立即返回进程对象
    """
    # 创建Tee输出对象
    log_tee = TeeOutput(log_file, sys.stdout)
    err_tee = TeeOutput(err_file, sys.stderr)
    
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        bufsize=1
    )
    
    # 启动线程来处理输出
    def forward_output(pipe, tee):
        try:
            for line in pipe:
                tee.write(line)
        finally:
            tee.close()
    
    stdout_thread = threading.Thread(
        target=forward_output,
        args=(process.stdout, log_tee)
    )
    stderr_thread = threading.Thread(
        target=forward_output,
        args=(process.stderr, err_tee)
    )
    
    stdout_thread.daemon = True
    stderr_thread.daemon = True
    stdout_thread.start()
    stderr_thread.start()
    
    if wait:
        # 等待进程完成
        retcode = process.wait()
        
        # 等待输出线程完成
        stdout_thread.join()
        stderr_thread.join()
    else:
        retcode = None
    
    return process, retcode

class TestController:
    """测试控制器"""
    def __init__(self, sdf_dir, output_dir, rounds=1, steps_per_round=10, mode='mem', test_mode=False):
        self.sdf_dir = sdf_dir
        self.output_dir = os.path.abspath(output_dir)
        self.rounds = rounds
        self.steps_per_round = steps_per_round
        self.mode = mode  # 'cov' 或 'mem' 或 'test'
        self.test_mode = test_mode
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 查找所有SDF文件
        self.sdf_files = []
        for ext in ['*.sdf', '*.world']:
            self.sdf_files.extend(glob.glob(os.path.join(sdf_dir, ext)))
        
        if not self.sdf_files:
            raise ValueError(f"No SDF files found in {sdf_dir}")
            
        # 初始化覆盖率追踪
        self.coverage_old = None
        self.total_coverage = 0
        self.current_gazebo_process = None
        
        # 清理之前的覆盖率文件
        if self.mode == 'cov':
            print("[DEBUG] Cleaning up previous coverage files...")
            coverage_info = CoverageInfo()
            coverage_info.cleanup(rm_gcda=True, rm_json=True)
            print("[DEBUG] Coverage files cleanup completed")

    def _setup_round_dir(self, round_num):
        """设置轮次目录"""
        round_dir = os.path.join(self.output_dir, f"_{round_num}")
        os.makedirs(round_dir, exist_ok=True)
        return round_dir
        
    def _get_random_sdf(self):
        """随机选择一个SDF文件"""
        return random.choice(self.sdf_files)
        
    def _create_test_config(self, config_file):
        """创建测试配置文件"""
        if self.test_mode:
            # 在测试模式下，每个算子执行一次
            steps = [
                {"type": "add_model"},
                {"type": "list_models"},
                {"type": "remove_model"},
                {
                    "type": "exec_service",
                    "service": "/gazebo/worlds",
                    "reqtype": "gz.msgs.Empty",
                    "reptype": "gz.msgs.StringMsg_V"
                },
                {
                    "type": "exec_topic",
                    "topic": "/clock",
                    "msgtype": "gz.msgs.Clock"
                }
            ]
        else:
            # 在正常模式下，随机生成步骤
            steps = []
            for _ in range(self.steps_per_round):
                step_type = random.choice([
                    "add_model",
                    "list_models",
                    "remove_model",
                    "exec_service",
                    "exec_topic"
                ])
                
                if step_type == "add_model":
                    step = {
                        "type": step_type,
                        "model_type": random.choice(["box", "sphere", "cylinder"])
                    }
                elif step_type in ["list_models", "remove_model"]:
                    step = {"type": step_type}
                elif step_type == "exec_service":
                    step = {
                        "type": step_type,
                        "service": "/gazebo/worlds",
                        "reqtype": "gz.msgs.Empty",
                        "reptype": "gz.msgs.StringMsg_V"
                    }
                else:  # exec_topic
                    step = {
                        "type": step_type,
                        "topic": "/clock",
                        "msgtype": "gz.msgs.Clock"
                    }
                steps.append(step)
        
        # 写入配置文件
        with open(config_file, 'w') as f:
            json.dump(steps, f, indent=2)
            
        print(f"[DEBUG] Created test config with {len(steps)} steps")

    def _start_gazebo_coverage(self, round_dir):
        """以覆盖率模式启动 Gazebo"""
        world_file = os.path.join(round_dir, "world.sdf")
        log_file = os.path.join(round_dir, "gazebo.log")
        err_file = os.path.join(round_dir, "gazebo.err")

        # 构建 Gazebo 命令
        gazebo_cmd = [
            "python3",
            "gazebo_launcher.py",
            "--sdf", world_file,
            "--output", round_dir
        ]

        print("[DEBUG] Starting Gazebo process...")
        print(f"[DEBUG] Gazebo command: {' '.join(gazebo_cmd)}")
        print(f"[DEBUG] Gazebo log file: {log_file}")
        print(f"[DEBUG] Gazebo error file: {err_file}")

        # 启动进程并设置输出重定向
        with open(log_file, 'w') as log, open(err_file, 'w') as err:
            self.current_gazebo_process = subprocess.Popen(
                gazebo_cmd,
                stdout=log,
                stderr=err,
                universal_newlines=True,
                bufsize=1
            )

        print(f"[DEBUG] Gazebo process started with PID: {self.current_gazebo_process.pid}")
        return self.current_gazebo_process.pid

    def _start_gazebo_valgrind(self, round_dir):
        """使用Valgrind启动Gazebo"""
        # 创建valgrind日志目录
        valgrind_dir = os.path.join(round_dir, "valgrind")
        os.makedirs(valgrind_dir, exist_ok=True)
        
        # 准备Gazebo命令
        gazebo_cmd = [
            "valgrind",
            "--tool=memcheck",
            "--leak-check=full",
            "--show-leak-kinds=all",
            "--track-origins=yes",
            "--read-var-info=yes",
            "--verbose",
            "--log-file=" + os.path.join(valgrind_dir, "gazebo.log"),
            "python3",
            "gazebo_launcher.py",
            "--sdf", os.path.join(round_dir, "world.sdf"),
            "--output", round_dir,
            "--config", os.path.join(round_dir, "test_config.json")
        ]
        
        # 在测试模式下添加 --test-mode 参数
        if self.mode == "test":
            gazebo_cmd.append("--test-mode")
        
        print("[DEBUG] Starting Gazebo process with Valgrind...")
        print("[DEBUG] Gazebo command:", " ".join(gazebo_cmd))
        
        # 启动进程
        self.current_gazebo_process = subprocess.Popen(gazebo_cmd)
        print(f"[DEBUG] Gazebo process started with PID: {self.current_gazebo_process.pid}")
        return self.current_gazebo_process.pid

    def _cleanup_gazebo_process(self):
        """清理 Gazebo 进程"""
        if self.current_gazebo_process:
            print("[DEBUG] Attempting to terminate Gazebo process...")
            try:
                # 首先尝试发送 SIGTERM 信号
                self.current_gazebo_process.terminate()
                try:
                    # 等待5秒让进程正常退出
                    self.current_gazebo_process.wait(timeout=5)
                    print("[DEBUG] Gazebo process terminated gracefully")
                    return
                except subprocess.TimeoutExpired:
                    print("[DEBUG] Gazebo process did not respond to SIGTERM, sending SIGKILL...")
                    
                # 如果进程没有响应 SIGTERM，使用 SIGKILL
                self.current_gazebo_process.kill()
                self.current_gazebo_process.wait(timeout=2)
                print("[DEBUG] Gazebo process killed")
            except Exception as e:
                print(f"[WARNING] Error cleaning up Gazebo process: {e}")
            finally:
                self.current_gazebo_process = None

    def _run_test_round(self, round_num):
        """运行一轮测试"""
        # 创建轮次目录
        round_dir = os.path.join(self.output_dir, f"_{round_num}")
        os.makedirs(round_dir, exist_ok=True)
        
        try:
            # 选择并复制SDF文件
            sdf_file = self._get_random_sdf()
            print(f"[DEBUG] Selected SDF file: {sdf_file}")
            
            # 复制SDF文件到轮次目录
            local_sdf = os.path.join(round_dir, "world.sdf")
            shutil.copy2(sdf_file, local_sdf)
            
            # 创建测试配置文件
            config_file = os.path.join(round_dir, "test_config.json")
            self._create_test_config(config_file)
            
            # 根据模式启动Gazebo
            if self.mode == 'cov':
                self._start_gazebo_coverage(round_dir)
            else:  # mem
                self._start_gazebo_valgrind(round_dir)
            
            # 等待Gazebo完全启动
            print("[DEBUG] Waiting for Gazebo to initialize...")
            time.sleep(5)  # 给Gazebo一些启动时间
            print("[DEBUG] Gazebo initialization wait completed")
            
            # 等待Gazebo进程结束
            if self.current_gazebo_process:
                try:
                    self.current_gazebo_process.wait(timeout=300)  # 设置一个合理的超时时间，比如5分钟
                    print("[DEBUG] Gazebo process exited")
                except subprocess.TimeoutExpired:
                    print("[DEBUG] Gazebo process timed out, this should not happen")
                    self._cleanup_gazebo_process()
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Test round {round_num} failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def _collect_coverage(self, round_dir):
        """收集覆盖率数据"""
        coverage_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
        coverage_new.collect()
        
        if self.coverage_old is None:
            print("\n=== Coverage Report for Round ===")
            print(f"Initial Lines Covered: {len(coverage_new.file_cov)}")
            self.total_coverage = len(coverage_new.file_cov)
            self.coverage_old = coverage_new
        else:
            diff = CoverageDiff()
            diff.compare(coverage_new, self.coverage_old)
            self.total_coverage += diff.new_line
            print("\n=== Coverage Report for Round ===")
            print(f"New Lines Covered: {diff.new_line}")
            print(f"Total Lines Covered: {self.total_coverage}")
            self.coverage_old = coverage_new

    def run_tests(self):
        """运行所有测试轮次"""
        for round_num in range(1, self.rounds + 1):
            if not self._run_test_round(round_num):
                print(f"[ERROR] Stopping tests after round {round_num} due to failure")
                break
        
        if self.mode == 'cov' and self.coverage_old is not None:
            print("\n=== Final Coverage Report ===")
            print(f"Total Lines Covered: {self.total_coverage}")

def main():
    parser = argparse.ArgumentParser(description="Test Controller")
    parser.add_argument("--sdf-dir", type=str, required=True,
                      help="Directory containing SDF files")
    parser.add_argument("--output", type=str, required=True,
                      help="Output directory for test results")
    parser.add_argument("--rounds", type=int, default=1,
                      help="Number of test rounds to run")
    parser.add_argument("--steps", type=int, default=10,
                      help="Number of steps per round")
    parser.add_argument("--mode", type=str, default="mem",
                      choices=["mem", "cov", "test"],
                      help="Test mode: 'mem' for memory check, 'cov' for coverage, 'test' for single test")
    
    args = parser.parse_args()
    
    # 在测试模式下，强制设置为1轮
    if args.mode == "test":
        args.rounds = 1
        print("[INFO] Test mode enabled: forcing rounds=1")
    
    controller = TestController(
        args.sdf_dir,
        args.output,
        rounds=args.rounds,
        steps_per_round=args.steps,
        mode=args.mode,
        test_mode=(args.mode == "test")
    )
    
    controller.run_tests()

if __name__ == "__main__":
    main()
