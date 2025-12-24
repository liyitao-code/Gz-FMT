#!/usr/bin/env python3

import sys
import time
import signal
import os
import threading

# 添加 Gazebo Python 库路径
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')

from gz.sim9 import TestFixture, World

class FreeTestFixture:
    def __init__(self, world_file):
        """初始化自由测试夹具"""
        self.world_file = world_file
        self.fixture = None
        self.world = None
        self.latest_count = 0
        self._lock = threading.Lock()
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """处理退出信号"""
        print("\nReceived signal to exit...")
        if self.fixture:
            print("Finalizing test fixture...")
            self.fixture.finalize()
        sys.exit(0)
        
    def _post_update_callback(self, update_info, ecm):
        """每次更新后的回调函数"""
        with self._lock:
            self.latest_count = self.world.model_count(ecm)
        
    def start(self):
        """启动测试夹具"""
        print(f"Creating test fixture with world: {self.world_file}")
        print(f"World path exists: {os.path.exists(self.world_file)}")
        print(f"World path is file: {os.path.isfile(self.world_file)}")
        
        # 创建测试夹具
        self.fixture = TestFixture(self.world_file)
        print("TestFixture created")
        
        # 等待初始化完成
        print("Waiting for initialization...")
        time.sleep(2)
        
        # 获取世界实例
        server = self.fixture.server()
        if not server:
            print("Failed to get server")
            return False
            
        # 获取世界实体
        world_id = 1  # 世界实体的 ID 通常是 1
        print(f"Using world id: {world_id}")
        self.world = World(world_id)
        print("Created world instance")
        
        # 注册更新回调
        self.fixture.on_post_update(self._post_update_callback)
        print("Registered post-update callback")
        
        # 完成初始化
        self.fixture.finalize()
        print("Finalized test fixture")
        
        return True
        
    def run(self):
        """运行主循环"""
        if not self.start():
            return
            
        print("\nStarting model count monitoring...")
        print("Press Ctrl+C to exit\n")
        
        try:
            while True:
                with self._lock:
                    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Current model count: {self.latest_count}")
                time.sleep(5)
                
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            if self.fixture:
                print("Finalizing test fixture...")
                self.fixture.finalize()

def main():
    """主函数"""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <world_file>")
        sys.exit(1)
        
    world_file = os.path.abspath(sys.argv[1])
    fixture = FreeTestFixture(world_file)
    fixture.run()

if __name__ == "__main__":
    main()
