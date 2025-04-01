#!/usr/bin/env python3

import subprocess
import json
import time
import os
import signal
import random
import shutil
import glob
import sys
import threading
from coverage_process import CoverageInfo, CoverageDiff, BUILD_DIR, GCOV_DIR

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
    def __init__(self, sdf_dir, output_dir, rounds=1, steps_per_round=10):
        self.sdf_dir = sdf_dir
        self.output_dir = os.path.abspath(output_dir)
        self.rounds = rounds
        self.steps_per_round = steps_per_round
        
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
        # 可用的操作类型及其参数模板
        operation_types = [
            {
                "type": "exec_topic",
                "topic": "/echo",
                "msg_type": "gz.msgs.StringMsg",
                "data": {"data": f"test_data_{random.randint(1, 100)}"}
            },
            {
                "type": "exec_service",
                "service": "/world/default/control",
                "req_type": "gz.msgs.WorldControl",
                "rep_type": "gz.msgs.Boolean",
                "data": {"pause": True}
            },
            {
                "type": "add_model",
                "model_type": random.choice(["box", "sphere", "cylinder"])
            }
        ]
        
        # 随机选择指定数量的操作
        steps = []
        for _ in range(self.steps_per_round):
            step = random.choice(operation_types).copy()
            if step["type"] == "exec_topic":
                step["data"]["data"] = f"test_data_{random.randint(1, 100)}"
            elif step["type"] == "add_model":
                step["model_type"] = random.choice(["box", "sphere", "cylinder"])
            steps.append(step)
            
        test_config = {"steps": steps}
        with open(config_file, 'w') as f:
            json.dump(test_config, f, indent=2)
            
    def _collect_coverage(self, round_dir):
        """收集覆盖率信息"""
        coverage_dir = os.path.join(round_dir, "coverage")
        os.makedirs(coverage_dir, exist_ok=True)
        
        # 收集新的覆盖率信息
        coverage_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
        coverage_new.collect()
        
        # 计算差异
        diff = CoverageDiff()
        if self.coverage_old is None:
            diff.compare(coverage_new)
            self.coverage_old = coverage_new
        else:
            diff.compare(coverage_new, self.coverage_old)
            self.coverage_old = coverage_new
            
        # 更新总覆盖率
        self.total_coverage += diff.new_line
        
        # 保存覆盖率信息
        coverage_info = {
            "new_lines": diff.new_line,
            "total_coverage": self.total_coverage,
        }
        
        coverage_file = os.path.join(coverage_dir, "coverage.json")
        with open(coverage_file, 'w') as f:
            json.dump(coverage_info, f, indent=2)
            
        print(f"\n=== Coverage Report for Round ===")
        print(f"New Lines Covered: {diff.new_line}")
        print(f"Total Lines Covered: {self.total_coverage}")
        
    def _run_test_round(self, round_num):
        """运行一轮测试"""
        # 创建轮次目录
        round_dir = os.path.join(self.output_dir, f"_{round_num}")
        os.makedirs(round_dir, exist_ok=True)
        
        # 创建valgrind日志目录
        valgrind_dir = os.path.join(round_dir, "valgrind")
        os.makedirs(valgrind_dir, exist_ok=True)
        
        # 预先创建valgrind日志文件
        valgrind_gazebo_log = os.path.join(valgrind_dir, "gazebo.log")
        valgrind_gazebo_updated_log = os.path.join(valgrind_dir, "gazebo_updated.log")
        open(valgrind_gazebo_log, 'w').close()
        open(valgrind_gazebo_updated_log, 'w').close()
        
        print(f"\n=== Starting Test Round {round_num} ===")
        
        try:
            # 选择并复制SDF文件
            sdf_file = random.choice(self.sdf_files)
            print(f"[DEBUG] Selected SDF file: {sdf_file}")
            
            # 复制SDF文件到轮次目录
            local_sdf = os.path.join(round_dir, "world.sdf")
            with open(sdf_file, 'r') as src, open(local_sdf, 'w') as dst:
                dst.write(src.read())
            
            # 创建测试配置文件
            config_file = os.path.join(round_dir, "test_config.json")
            self._create_test_config(config_file)
            
            # 启动Gazebo环境
            print("[DEBUG] Starting Gazebo process with Valgrind...")
            valgrind_cmd = [
                "valgrind",
                "--tool=memcheck",
                "--leak-check=full",
                "--show-leak-kinds=all",
                "--track-origins=yes",
                "--read-var-info=yes",
                "--verbose",
                "--log-file=" + valgrind_gazebo_log,  # 使用=而不是空格
                "python3",
                "gazebo_launcher.py",
                "--sdf", local_sdf,
                "--output", round_dir
            ]
            print("[DEBUG] Gazebo command:", " ".join(valgrind_cmd))
            
            # 创建日志文件并设置Tee输出
            gazebo_log = os.path.join(round_dir, "gazebo.log")
            gazebo_err = os.path.join(round_dir, "gazebo.err")
            print(f"[DEBUG] Gazebo log file: {gazebo_log}")
            print(f"[DEBUG] Gazebo error file: {gazebo_err}")
            print(f"[DEBUG] Valgrind log file: {valgrind_gazebo_log}")
            
            with open(gazebo_log, 'w') as log_file, open(gazebo_err, 'w') as err_file:
                print("[DEBUG] Starting Gazebo process with Tee output...")
                self.current_gazebo_process, _ = _run_process_with_tee(valgrind_cmd, log_file, err_file, wait=False)
                print("[DEBUG] _run_process_with_tee returned")
                
            print(f"[DEBUG] Gazebo process started with PID: {self.current_gazebo_process.pid}")
            
            # 等待Gazebo完全启动
            print("[DEBUG] Waiting for Gazebo to initialize...")
            time.sleep(5)  # 由于使用valgrind，可能需要更长的启动时间
            print("[DEBUG] Gazebo initialization wait completed")
            
            # 执行算子
            print("[DEBUG] Starting operator execution...")
            operator_cmd = [
                "python3",
                "operator_executor.py",
                "--config", config_file,
                "--output", round_dir
            ]
            print("[DEBUG] Running operator command:", " ".join(operator_cmd))
            
            # 执行operator并收集输出
            operator_log = os.path.join(round_dir, "operator.log")
            operator_err = os.path.join(round_dir, "operator.err")
            
            print("[DEBUG] Starting operator with output files:")
            print(f"[DEBUG] Log: {operator_log}")
            print(f"[DEBUG] Err: {operator_err}")
            
            with open(operator_log, 'w') as log_file, open(operator_err, 'w') as err_file:
                # 启动operator进程
                operator_process, retcode = _run_process_with_tee(operator_cmd, log_file, err_file, wait=False)
                operator_pid = operator_process.pid
                print(f"[DEBUG] Operator started with PID: {operator_pid}")
                
                # 更新Gazebo进程的operator PID
                update_cmd = [
                    "valgrind",
                    "--tool=memcheck",
                    "--leak-check=full",
                    "--show-leak-kinds=all",
                    "--track-origins=yes",
                    "--read-var-info=yes",
                    "--verbose",
                    "--log-file=" + valgrind_gazebo_updated_log,  # 使用=而不是空格
                    "python3",
                    "gazebo_launcher.py",
                    "--sdf", local_sdf,
                    "--output", round_dir,
                    "--operator-pid", str(operator_pid)
                ]
                print("[DEBUG] Updating Gazebo with operator PID...")
                self.current_gazebo_process.terminate()
                time.sleep(2)  # 给valgrind更多时间来完成清理
                self.current_gazebo_process, _ = _run_process_with_tee(update_cmd, log_file, err_file, wait=False)
                print(f"[DEBUG] New Gazebo process started with PID: {self.current_gazebo_process.pid}")
                
                # 等待operator完成
                operator_process.wait()
                retcode = operator_process.returncode
                print(f"[DEBUG] Operator completed with return code: {retcode}")
                if retcode != 0:
                    raise subprocess.CalledProcessError(retcode, operator_cmd)
                
            print("[DEBUG] Operator execution completed")
            
            # 等待Gazebo进程自行退出
            print("[DEBUG] Waiting for Gazebo process to exit...")
            try:
                self.current_gazebo_process.wait(timeout=20)  # 增加超时时间，因为valgrind会使进程变慢
                print("[DEBUG] Gazebo process exited normally")
            except subprocess.TimeoutExpired:
                print("[DEBUG] Gazebo process did not exit in time")
                # 如果超时，我们可以检查一下gazebo.err文件看看是否有错误
                try:
                    with open(gazebo_err, 'r') as f:
                        err_content = f.read()
                        if err_content:
                            print("[DEBUG] Content of gazebo.err:")
                            print(err_content)
                except FileNotFoundError:
                    print("[DEBUG] Gazebo error log file not found")
                    
            # 收集覆盖率信息
            self._collect_coverage(round_dir)
            
        except Exception as e:
            print(f"[ERROR] Test round {round_num} failed: {str(e)}")
            raise
        finally:
            # 确保进程被终止
            if self.current_gazebo_process:
                try:
                    self.current_gazebo_process.terminate()
                except:
                    pass
                
    def run_tests(self):
        """运行所有测试轮次"""
        for round_num in range(1, self.rounds + 1):
            self._run_test_round(round_num)
            
        # 生成最终的覆盖率报告
        print("\n=== Final Coverage Report ===")
        print(f"Total Lines Covered: {self.total_coverage}")

def main():
    """主函数"""
    import argparse
    parser = argparse.ArgumentParser(description='Run Gazebo tests with different SDF files')
    parser.add_argument('-r', '--rounds', type=int, default=1, help='Number of test rounds')
    parser.add_argument('-o', '--output', type=str, required=True, help='Output directory')
    parser.add_argument('-s', '--sdf-dir', type=str, default='models', help='Directory containing SDF files')
    parser.add_argument('-p', '--steps-per-round', type=int, default=10, help='Number of steps per round')
    args = parser.parse_args()
    
    controller = TestController(args.sdf_dir, args.output, args.rounds, args.steps_per_round)
    controller.run_tests()

if __name__ == "__main__":
    main()
