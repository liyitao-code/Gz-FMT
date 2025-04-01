#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cxxfilt
import glob
import gzip
import subprocess
import json
import os
import json
from pathlib import Path
import time

BUILD_DIR = "/home/liyitao/workspace/gz_lastest/build/"
# BUILD_DIR = "/new/workspace/build/"
GCOV_DIR = "gcov"


class CoverageDiff:
    def __init__(self, new_line=0, new_file=0):
        self.new_line = new_line
        self.new_file = new_file

    def compare(self, cov_a, cov_b=None):
        if cov_b is None:
            # first run, compared against None
            self.new_file = len(set(cov_a.file_cov))
            self.new_line = 0
            for f in cov_a.file_cov:
                self.new_line += len(cov_a.file_cov[f])
        else:
            self.new_file = len(set(cov_a.file_cov) - set(cov_b.file_cov))
            self.new_line = 0
            for f in cov_a.file_cov:
                if f in cov_b.file_cov:
                    self.new_line += len(cov_a.file_cov[f] - cov_b.file_cov[f])
                else:
                    self.new_line += len(cov_a.file_cov[f])

class CoverageInfo:
    def __init__(self, build_dir=BUILD_DIR, gcov_dir=GCOV_DIR):
        self.build_dir = build_dir
        self.gcov_dir = gcov_dir
        self.file_cov = dict()
        # TODO: should reconsider this
        self.function_branch = dict()

    def collect(self):
        coverage_list = self.run_gcov()
        for c in coverage_list:
            self.process_coverage(c)

    def run_gcov(self):
        coverage_list = list()
        if not os.path.exists(self.gcov_dir):
            os.mkdir(self.gcov_dir)
        owd = os.getcwd()
        os.chdir(self.gcov_dir)

        # 运行 gcov 命令并等待完成
        print("[DEBUG] Running gcov command...")
        try:
            subprocess.run(
                f"find {self.build_dir} -name '*.gcda' | parallel gcov --json-format -p",
                shell=True,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("[DEBUG] gcov command completed")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] gcov command failed: {e}")
            print(f"stdout: {e.stdout.decode()}")
            print(f"stderr: {e.stderr.decode()}")
            raise

        # 等待一会儿确保文件都写入完成
        time.sleep(1)
        
        # 读取生成的 JSON 文件
        print("[DEBUG] Reading gcov JSON files...")
        for json_file in glob.glob("*.gcov.json.gz"):
            try:
                with gzip.open(json_file, "rb") as f:
                    content = f.read()
                    coverage = json.loads(content)
                    coverage_list.append(coverage)
            except Exception as e:
                print(f"[ERROR] Failed to read {json_file}: {e}")
                continue

        os.chdir(owd)
        return coverage_list

    def cleanup(self, rm_gcda=True, rm_json=True):
        if rm_gcda:
            subprocess.run(f"find {self.build_dir} -name '*.gcda' -delete", shell=True)
        if rm_json:
            subprocess.run(f'rm {self.gcov_dir}/*.json.gz', shell=True)

    def process_coverage(self, cov_dict):
        total_branches = 0
        executed_branches = 0
        for file in cov_dict['files']:
            filename = file['file']
            for function in file['functions']:
                # total_branches += function['blocks']
                blocks = function['blocks_executed']
                if filename not in self.function_branch:
                    self.function_branch[filename] = blocks
                else:
                    self.function_branch[filename] += blocks

            for line in file['lines']:
                line_number = line['line_number']
                count = line['count']
                # function = cxxfilt.demangle(line['function_name'])
                # unexecuted_block = line['unexecuted_block']
                # branches = line['branches']
                if count != 0:
                    if filename not in self.file_cov:
                        self.file_cov[filename] = {line_number}
                    else:
                        self.file_cov[filename].add(line_number)

    def calculate_total_coverage(self):
        total_lines = 0
        covered_lines = 0
        for filename in self.file_cov:
            total_lines += len(self.file_cov[filename])
            covered_lines += len(self.file_cov[filename])
        return covered_lines / total_lines if total_lines > 0 else 0

if __name__ == "__main__":
    cov = CoverageInfo(BUILD_DIR, GCOV_DIR)
    cov.collect()
    print("Coverage:", cov.calculate_total_coverage())
    print("Files:", len(cov.file_cov))
