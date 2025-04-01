#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import argparse
import signal
import time
import os
import traceback

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
    def __init__(self, sdf_path, output_dir=None):
        print("[DEBUG] Initializing GazeboLauncher...")
        self.world_path = os.path.abspath(sdf_path) if sdf_path else None
        self.output_dir = output_dir
        self.fixture = None
        self.server = None
        self.running = True
        self.operator_pid = None
        
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
    
    def set_operator_pid(self, pid):
        """设置要监听的operator进程ID"""
        print(f"[DEBUG] Setting operator PID to: {pid}")
        self.operator_pid = pid
        
    def _check_operator_status(self):
        """检查operator进程状态"""
        if self.operator_pid is None:
            return True
            
        try:
            # 尝试获取进程信息，如果进程不存在会抛出异常
            os.kill(self.operator_pid, 0)
            return True
        except ProcessLookupError:
            print("[DEBUG] Operator process has ended")
            return False
        except Exception as e:
            print(f"[DEBUG] Error checking operator status: {str(e)}")
            return False
        
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
            
            print("[DEBUG] Entering main loop...")
            # 保持进程运行直到operator进程结束
            while self.running and self._check_operator_status():
                time.sleep(0.1)
                
            if self.running:
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
    parser.add_argument("--operator-pid", type=int,
                      help="PID of the operator process to monitor")
    
    args = parser.parse_args()
    
    launcher = GazeboLauncher(args.sdf, args.output)
    if args.operator_pid:
        launcher.set_operator_pid(args.operator_pid)
    launcher.launch()

if __name__ == "__main__":
    main()
