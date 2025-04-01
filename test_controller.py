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
    def __init__(self):
        self.current_gazebo_process = None
        self.output_dir = None
        self.current_round = 0
    
    def _get_random_sdf(self):
        """从 ./models 目录随机选择一个 SDF 文件"""
        models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')
        sdf_files = []
        
        # 递归搜索所有 .sdf 文件
        for root, _, files in os.walk(models_dir):
            for file in files:
                if file.endswith('.sdf'):
                    sdf_files.append(os.path.join(root, file))
        
        if not sdf_files:
            raise RuntimeError("No SDF files found in ./models directory")
            
        return random.choice(sdf_files)
    
    def _setup_round_dir(self, round_num):
        """设置当前轮次的输出目录"""
        round_dir = os.path.join(self.output_dir, f"_{round_num}")
        if os.path.exists(round_dir):
            shutil.rmtree(round_dir)
        os.makedirs(round_dir)
        return round_dir
    
    def _copy_sdf(self, src_sdf, round_dir):
        """复制SDF文件到轮次目录"""
        # 复制到 world.sdf
        dst_sdf = os.path.join(round_dir, "world.sdf")
        shutil.copy2(src_sdf, dst_sdf)
        
        # 同时保存一份原始的SDF文件
        src_name = os.path.basename(src_sdf)
        orig_sdf = os.path.join(round_dir, f"original_{src_name}")
        shutil.copy2(src_sdf, orig_sdf)
        
        return dst_sdf
    
    def run_single_test(self, round_num):
        """运行单轮测试"""
        print(f"\n=== Starting Test Round {round_num} ===")
        
        # 创建轮次目录
        round_dir = self._setup_round_dir(round_num)
        
        # 创建valgrind日志目录
        valgrind_dir = os.path.join(round_dir, "valgrind")
        os.makedirs(valgrind_dir, exist_ok=True)
        
        # 预先创建valgrind日志文件
        valgrind_gazebo_log = os.path.join(valgrind_dir, "gazebo.log")
        valgrind_gazebo_updated_log = os.path.join(valgrind_dir, "gazebo_updated.log")
        open(valgrind_gazebo_log, 'w').close()
        open(valgrind_gazebo_updated_log, 'w').close()
        
        # 随机选择 SDF 文件并复制
        sdf_path = self._get_random_sdf()
        print(f"[DEBUG] Selected SDF file: {sdf_path}")
        local_sdf = self._copy_sdf(sdf_path, round_dir)
        
        # 生成本轮测试的配置
        test_config = self.generate_operator_config()
        config_file = os.path.join(round_dir, "test_config.json")
        with open(config_file, 'w') as f:
            json.dump(test_config, f, indent=2)
        
        try:
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
            
        except Exception as e:
            print(f"Error in round {round_num}: {str(e)}")
            
    def run_tests(self, rounds, output_dir):
        """运行指定轮数的测试"""
        self.output_dir = os.path.abspath(output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        for round_num in range(1, rounds + 1):
            self.current_round = round_num
            self.run_single_test(round_num)
    
    def generate_operator_config(self):
        """生成算子配置"""
        # 这里是示例配置，您可以根据需要修改
        return {
            "steps": [
                {
                    "type": "exec_topic",
                    "topic": "/echo",
                    "msg_type": "gz.msgs.StringMsg",
                    "data": {"data": f"test_data_{random.randint(0, 100)}"}
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
        }

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Test Controller")
    parser.add_argument("-r", "--rounds", type=int, required=True,
                      help="Number of test rounds to run")
    parser.add_argument("-o", "--output", type=str, required=True,
                      help="Output directory for test artifacts")
    
    args = parser.parse_args()
    
    controller = TestController()
    controller.run_tests(args.rounds, args.output)

if __name__ == "__main__":
    main()
