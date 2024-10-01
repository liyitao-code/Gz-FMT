#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import os
import shutil
import sys

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

def find_gz_err_files(directory):
    """递归寻找目录下所有的 gz.err 文件"""
    gz_err_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file == "gz.err":
                gz_err_files.append(os.path.join(root, file))
    return gz_err_files

def compare_directories(dir1, dir2, target_dir):
    """比较两个目录，找出第二个目录中与第一个目录不同的 gz.err 文件"""
    traces_dir1 = process_directory(dir1)
    traces_dir2 = process_directory(dir2)

    unique_traces = set()
    unique_directories = []

    gz_err_files_dir2 = find_gz_err_files(dir2)
    for gz_err_file in gz_err_files_dir2:
        e = ErrorLog(gz_err_file)
        if e.trace and e.trace not in unique_traces and e.trace not in traces_dir1:
            unique_traces.add(e.trace)
            source_dir = os.path.dirname(gz_err_file)
            if source_dir not in unique_directories:
                unique_directories.append(source_dir)
                target_subdir = os.path.join(target_dir, os.path.basename(source_dir))
                shutil.copytree(source_dir, target_subdir)
    
    return unique_directories, len(unique_traces)

def process_directory(directory):
    """处理一个目录，返回其 gz.err 文件的堆栈跟踪集合"""
    traces = set()
    gz_err_files = find_gz_err_files(directory)
    for gz_err_file in gz_err_files:
        e = ErrorLog(gz_err_file)
        if e.trace:
            traces.add(e.trace)
        print(gz_err_file)
        print(e.trace)
        # print('\n')
    return traces

def main(mode, path1, path2, target_path=None):
    """mode参数为0则为从大量数据中筛出crash，1则是对比和合并功能
    mode = 0 是从path1中筛出crash存放在path2
    mode = 1 是在path2中找和path1不同的crash，存在target_path中
    """
    if mode == 0:
        # Original functionality
        if not os.path.exists(path2):
            os.makedirs(path2)

        all_trace_dir = os.path.join(path2, "all_crash/")
        unique_trace_dir = os.path.join(path2, "unique_crash")
        seen_traces = set()
        new_trace_count = 0
        all_trace_count = 0

        gz_err_files = find_gz_err_files(path1)
        for gz_err_file in gz_err_files:
            e = ErrorLog(gz_err_file)
            if e.trace :
                source_dir = os.path.dirname(gz_err_file)
                target_dir = os.path.join(all_trace_dir, os.path.basename(source_dir))
                shutil.copytree(source_dir, target_dir)
                all_trace_count += 1
                if e.trace not in seen_traces: 
                    # 如果是新的 trace，保存并拷贝目录
                    seen_traces.add(e.trace)
                    print(e.trace)
                    new_trace_count += 1

                    source_dir = os.path.dirname(gz_err_file)
                    target_dir = os.path.join(unique_trace_dir, os.path.basename(source_dir))
                    shutil.copytree(source_dir, target_dir)
                    print(f"New trace found, copied directory: {source_dir} to {target_dir}")
                
        print(f"Total traces found: {all_trace_count}")
        print(f"Total unique traces found: {new_trace_count}")

    elif mode == 1:
        # New functionality: Compare two directories
        if target_path is None:
            print("Error: Target directory must be provided for mode 1.")
            sys.exit(1)
        unique_directories, unique_count = compare_directories(path1, path2, target_path)
        print(f"Unique directories copied to {target_path}: {unique_directories}")
        print(f"Total unique err files: {unique_count}")

    else:
        print("Invalid mode. Use 0 for original functionality or 1 for comparison mode.")
        sys.exit(1)

if __name__ == "__main__":
    
    # dir1 = "/home/liyitao/workspace/exp/crash_ren/"
    # dir2 = "/home/liyitao/workspace/exp/crash_random/"
    # dir3 = "/home/liyitao/workspace/exp/muti_5_out/unique_crash"
    # a = process_directory(dir3)
    # print(len(a))
    # for i in a:
    #     print(str(i))
    #     print("\n")

    
    if len(sys.argv) < 4:
        print("Usage: python script_name.py <mode> <path1> <path2> [<target_path>]")
        sys.exit(1)

    mode = int(sys.argv[1])
    path1 = sys.argv[2]
    path2 = sys.argv[3]
    target_path = sys.argv[4] if len(sys.argv) > 4 else None

    main(mode, path1, path2, target_path)
