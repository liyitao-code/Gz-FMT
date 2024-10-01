import os
import shutil
import argparse
import re

def categorize_directories(base_path, output_path):
    # 定义需要检查的条件和对应的目录名
    conditions = [
        ("collide", "collide"),
        ("must be greater than zero", "greater_zero"),
        ("> 0.0", "greater_zero_1"),
        (">= 0.0", "greater_zero_1"),
        # 使用正则表达式来匹配断言错误
        (re.compile(r"Assertion `.* < .*' failed"), "greater_other"),
        (re.compile(r"Assertion `.* <= .*' failed"), "greater_other"),
        ("Segmentation fault", "segmentation"),
        ("Aborted", "aborted"),
    ]

    # 确保输出目录存在
    for condition, directory in conditions:
        os.makedirs(os.path.join(output_path, directory), exist_ok=True)
    os.makedirs(os.path.join(output_path, "other"), exist_ok=True)

    # 遍历每一个 _x 目录
    for dir_name in os.listdir(base_path):
        dir_path = os.path.join(base_path, dir_name)
        if not os.path.isdir(dir_path):
            continue  # 跳过非目录

        gz_err_path = os.path.join(dir_path, "gz.err")
        if not os.path.exists(gz_err_path):
            continue  # 如果 gz.err 文件不存在，跳过

        with open(gz_err_path, 'r') as file:
            content = file.read()

        # 检查条件并复制目录
        copied = False
        for condition, target_dir in conditions:
            if isinstance(condition, str) and condition in content:
                dest_dir = os.path.join(output_path, target_dir)
                shutil.copytree(dir_path, os.path.join(dest_dir, dir_name))
                copied = True
                break
            elif isinstance(condition, re.Pattern) and condition.search(content):
                dest_dir = os.path.join(output_path, target_dir)
                shutil.copytree(dir_path, os.path.join(dest_dir, dir_name))
                copied = True
                break

        # 如果没有命中任何条件且文件有内容，放入 other 目录
        if not copied and content.strip():
            dest_dir = os.path.join(output_path, "other")
            shutil.copytree(dir_path, os.path.join(dest_dir, dir_name))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Categorize directories based on the contents of gz.err files.")
    parser.add_argument("input_dir", help="The base input directory containing _x directories.")
    parser.add_argument("output_dir", help="The base output directory to copy categorized directories into.")

    args = parser.parse_args()

    categorize_directories(args.input_dir, args.output_dir)
