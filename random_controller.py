#!/usr/bin/env python3
'''
全链路为 random_controller.py - randomsmith_fixture.py
'''


from pickle import TRUE
import subprocess
import os
import argparse
import sys

class RandomController:
    """控制器类，用于管理 randomsmith_fixture.py 的执行"""
    
    def __init__(self, output_dir, num_seq=10, timeout=10000):
        """初始化控制器
        
        Args:
            output_dir: 输出目录
            num_seq: 每轮测试的命令序列长度
            timeout: 命令超时时间（毫秒）
        """
        self.output_dir = os.path.abspath(output_dir)
        self.num_seq = num_seq
        self.timeout = timeout
        self.test_counter = 0
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
    def run_test(self):
        """运行一次测试"""
        test_dir = self.output_dir
        id_dir = os.path.join(test_dir, f"_{self.test_counter}")
        os.makedirs(id_dir, exist_ok=True)
        test_dir += '/'

        cmd = [
            "valgrind",
            "--tool=memcheck",
            "--leak-check=full",
            "--show-leak-kinds=all",
            "--track-origins=yes",
            "--read-var-info=yes",
            "--verbose",
            "--log-file=" + os.path.join(id_dir, "valgrind.log"),
            "python3",
            "randomsmith_fixture.py",
            "--directory", test_dir,
            "--num-seq", str(self.num_seq),
            "--mode", "one_shot",
            "--timeout", str(self.timeout),
            "--id", str(self.test_counter)
        ]
        
        print(f"\n[INFO] Running test #{self.test_counter}")
        # print(cmd)
        process = subprocess.run(cmd)
        
        self.test_counter += 1
        return process.returncode == 0

def main():
    parser = argparse.ArgumentParser(description="Random Controller for randomsmith_fixture")
    parser.add_argument("--output-dir", type=str, required=True,
                      help="Output directory for test results")
    parser.add_argument("--num-seq", type=int, default=10,
                      help="Number of commands in the test sequence")
    parser.add_argument("--timeout", type=int, default=10000,
                      help="Command timeout in milliseconds")
    parser.add_argument("--num-tests", type=int, default=1,
                      help="Number of tests to run")
    
    args = parser.parse_args()
    
    controller = RandomController(
        args.output_dir,
        num_seq=args.num_seq,
        timeout=args.timeout
    )
    
    for i in range(args.num_tests):
        success = controller.run_test()
        if not success:
            print(f"\n[ERROR] Test {i} failed")
            sys.exit(1)
    
    print("\n[INFO] All tests completed successfully")

if __name__ == "__main__":
    main()
