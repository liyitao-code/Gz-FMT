#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cxxfilt
from glob import glob
import gzip
import subprocess
import json
import os
import json
from pathlib import Path

BUILD_DIR = "/home/liyitao/workspace/build/"
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
        # 总行数
        self.total_lines_dict = dict()  

    def collect(self):
        coverage_list = self.run_gcov()
        for c in coverage_list:
            self.process_coverage(c)

    def run_gcov(self):
        coverage_list = list()
        if not os.path.exists(self.gcov_dir):
            os.mkdir(self.gcov_dir)
        owd = os.getcwd()
        # print("!!!!!" + owd)
        os.chdir(self.gcov_dir)
        
        # owd1 = os.getcwd()
        # print("!!!!!" + owd1)

        # dirty yet fast...
        subprocess.run(f"find {self.build_dir} -name '*.gcda' | parallel gcov --json -p > /dev/null", shell=True)
        for gz_file in glob("*.json.gz"):
            with gzip.open(gz_file, "rb") as f:
                content = f.read()
                coverage = json.loads(content)
                coverage_list.append(coverage)
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
            total_lines = len(file['lines'])
            self.total_lines_dict[filename] = total_lines  # 更新总行数
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


    def merge(self, cov_info):
        for f in cov_info.file_cov:
            if f in self.cov_info.file_cov:
                self.file_cov[f] = self.file_cov[f].union(cov_info.file_cov[f])
            else:
                self.file_cov[f] = cov_info.file_cov[f].copy()
    
    def calculate_total_coverage(self):
        total_lines = sum(self.total_lines_dict.values())
        covered_lines = sum(len(lines) for lines in self.file_cov.values())

        # 如果总行数为零，避免除以零错误
        if total_lines == 0:
            return 0.0
        # print(f"all line: {total_lines}")
        return (covered_lines / total_lines) * 100
    
    def get_total_line(self):
        return sum(self.total_lines_dict.values())


if __name__ == "__main__":
    cov = CoverageInfo(BUILD_DIR, GCOV_DIR)
    cov.collect()
    print("run success")
    # for filename, lines in cov.file_cov.items():
    #     print(f"File: {filename}")
    #     print(f"Covered Lines: {sorted(lines)}")
    #     print(f"Number of Covered Lines: {len(lines)}")
    #     print("----------")


    total_coverage = cov.calculate_total_coverage()
    print(f"Total Coverage: {total_coverage:.2f}%")
    

    # cov.cleanup(False, False)
