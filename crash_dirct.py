import os
import re
from collections import defaultdict

class ErrorLog:
    def __init__(self, log_file="gz.err"):
        self.log_file = log_file
        self.trace = []
        if not os.path.exists(log_file):
            return

        with open(log_file) as f:
            self.content = f.read()
        self.get_stack_trace()
        self.trace = tuple(self.trace)

    def get_stack_trace(self):
        """用来从err文件里筛出错误栈"""
        for line in self.content.splitlines():
            if line.startswith("Stack trace"):
                continue
            elif line.startswith("Segmentation fault"):
                continue
            m = re.match(r'#\d\s+Object ".*?", at .*?, in (.*\(.*\))', line)
            if m:
                self.trace.append(m.group(1))

class ErrorLogManager:
    def __init__(self):
        # 使用字典来保存不同错误栈的出现次数
        self.error_counts = defaultdict(int)

    def process_error_file(self, log_file):
        """
        处理一个新的错误文件，更新错误栈计数，并返回该错误栈的总出现次数。
        
        :param log_file: 要处理的错误日志文件路径。
        :return: 该错误栈的总出现次数。
        """
        error_log = ErrorLog(log_file)
        error_trace = error_log.trace

        if error_trace:
            # 更新错误栈计数
            self.error_counts[error_trace] += 1
            return self.error_counts[error_trace]
        else:
            # 如果没有有效的错误栈，返回0
            return 0

def find_gz_err_files(directory):
    """递归寻找目录下所有的 gz.err 文件"""
    gz_err_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file == "gz.err":
                gz_err_files.append(os.path.join(root, file))
    return gz_err_files

# 使用示例
if __name__ == "__main__":
    manager = ErrorLogManager()
    
    # 假设有多个错误日志文件待处理
    # error_files = ["err1.log", "err2.log", "err3.log"]
    directory = "/home/liyitao/workspace/exp/random_1_crash/12/"
    error_files = find_gz_err_files(directory)
    # for gz_err_file in error_files:
    #     e = ErrorLog(gz_err_file)
    #     if e.trace:
    #         traces.add(e.trace)
    #     print(gz_err_file)
    #     print(e.trace)
    #     # print('\n')

    for file in error_files:
        count = manager.process_error_file(file)
        print(f"Error trace in {file} has occurred {count} times.")
