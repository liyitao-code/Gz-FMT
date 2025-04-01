import os
import re
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

def find_matching_gz_err_files(reference_err_file, search_directory):
    """在指定目录中寻找与参考错误栈相同的 gz.err 文件"""
    reference_log = ErrorLog(reference_err_file)
    print(reference_log.trace)
    if not reference_log.trace:
        print(f"No trace found in {reference_err_file}.")
        return []

    matching_files = []
    gz_err_files = find_gz_err_files(search_directory)
    for gz_err_file in gz_err_files:
        log = ErrorLog(gz_err_file)
        if log.trace == reference_log.trace:
            matching_files.append(gz_err_file)

    return matching_files

def main():
    if len(sys.argv) < 3:
        print("Usage: python script_name.py <reference_err_file> <search_directory>")
        sys.exit(1)

    reference_err_file = sys.argv[1]
    search_directory = sys.argv[2]

    matching_files = find_matching_gz_err_files(reference_err_file, search_directory)
    if matching_files:
        print("Files with matching stack trace:")
        for file in matching_files:
            print(file)
    else:
        print("No matching gz.err files found.")

if __name__ == "__main__":
    main()
