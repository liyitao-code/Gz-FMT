#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import argparse
import signal
import time
import os
import traceback
import subprocess

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

from gz_test_deps.sim import TestFixture, World
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs.entity_pb2 import Entity
from gz.transport14 import Node
from sdformat15 import Root as SDFRoot
from gz.msgs11.boolean_pb2 import Boolean
from gz_test_deps.common import set_verbosity

class GazeboLauncher:
    def __init__(self, sdf_path, output_dir=None, config_file=None):
        print("[DEBUG] Initializing GazeboLauncher...")
        self.world_path = os.path.abspath(sdf_path) if sdf_path else None
        self.output_dir = output_dir
        self.config_file = config_file
        self.fixture = None
        self.server = None
        self.running = True
        
        # 设置信号处理
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        print("[DEBUG] Signal handlers registered")
    
    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        signal_name = signal.Signals(signum).name
        print(f"\n[DEBUG] Received signal: {signal_name}")
        self._cleanup()
        print("[DEBUG] Signal handler completed, exiting...")
        sys.exit(0)
    
    def _cleanup(self):
        """清理资源"""
        if not self.running:
            return
            
        print("[DEBUG] Starting cleanup process...")
        self.running = False
        
        try:
            if self.server:
                print("[DEBUG] Cleaning up server...")
                try:
                    if hasattr(self.server, 'close'):
                        print("[DEBUG] Calling server.close()...")
                        self.server.close()
                    print("[DEBUG] Setting server to None...")
                    self.server = None
                    print("[DEBUG] Server cleanup completed")
                except Exception as e:
                    print(f"[DEBUG] Error during server cleanup: {str(e)}")
                    traceback.print_exc()
            
            if self.fixture:
                print("[DEBUG] Attempting to finalize fixture...")
                try:
                    if hasattr(self.fixture, 'finalize'):
                        print("[DEBUG] Calling fixture.finalize()...")
                        self.fixture.finalize()
                        print("[DEBUG] Fixture finalized")
                    print("[DEBUG] Setting fixture to None...")
                    self.fixture = None
                except Exception as e:
                    print(f"[DEBUG] Error during fixture cleanup: {str(e)}")
                    traceback.print_exc()
                    self.fixture = None
                    
        except Exception as e:
            print(f"[DEBUG] Error during cleanup: {str(e)}")
            traceback.print_exc()
        
        print("[DEBUG] Cleanup completed")
    
    def launch(self):
        """启动Gazebo环境"""
        try:
            # 创建测试环境
            print(f"[DEBUG] Creating test fixture with world: {self.world_path}")
            print(f"[DEBUG] World path exists: {os.path.exists(self.world_path)}")
            print(f"[DEBUG] World path is file: {os.path.isfile(self.world_path)}")
            
            self.fixture = TestFixture(self.world_path)
            print("[DEBUG] TestFixture created")
            
            print("[DEBUG] Calling fixture.finalize()...")
            self.fixture.finalize()
            print("[DEBUG] TestFixture finalized")
            
            print("[DEBUG] Getting server...")
            self.server = self.fixture.server()
            print("[DEBUG] Server obtained")
            
            print("Gazebo environment started successfully")
            
            # 启动operator进程
            if self.config_file and self.output_dir:
                operator_cmd = [
                    "python3",
                    "operator_executor.py",
                    "--config", self.config_file,
                    "--output", self.output_dir
                ]
                print("[DEBUG] Starting operator process:", " ".join(operator_cmd))
                
                operator_process = subprocess.Popen(
                    operator_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True,
                    bufsize=1
                )
                print(f"[DEBUG] Operator process started with PID: {operator_process.pid}")
                
                # 创建线程来处理operator的输出
                def handle_output(pipe, prefix=""):
                    for line in pipe:
                        print(prefix + line.strip())
                
                import threading
                stdout_thread = threading.Thread(
                    target=handle_output,
                    args=(operator_process.stdout, "[OPERATOR] ")
                )
                stderr_thread = threading.Thread(
                    target=handle_output,
                    args=(operator_process.stderr, "[OPERATOR ERR] ")
                )
                
                stdout_thread.start()
                stderr_thread.start()
                
                # 等待operator进程结束
                print("[DEBUG] Waiting for operator process to complete...")
                operator_process.wait()
                stdout_thread.join()
                stderr_thread.join()
                
                retcode = operator_process.returncode
                print(f"[DEBUG] Operator process completed with return code: {retcode}")
            
            # 主动清理资源
            print("[DEBUG] Operator process has ended, initiating cleanup...")
            self._cleanup()
                
        except Exception as e:
            print(f"Error launching Gazebo: {str(e)}")
            print("[DEBUG] Exception type:", type(e).__name__)
            print("[DEBUG] Exception args:", e.args)
            traceback.print_exc()
            self._cleanup()
            raise

def main():
    parser = argparse.ArgumentParser(description="Gazebo Environment Launcher")
    parser.add_argument("--sdf", type=str, required=True,
                      help="Path to SDF file")
    parser.add_argument("--output", type=str,
                      help="Output directory for logs")
    parser.add_argument("--config", type=str,
                      help="Path to test configuration file")
    
    args = parser.parse_args()
    
    launcher = GazeboLauncher(args.sdf, args.output, args.config)
    launcher.launch()

if __name__ == "__main__":
    main()
