import os
import subprocess
import argparse

def find_and_run_tests(root_dir):
    """
    查找并运行所有符合 `*/python/test/*.py` 路径规则的测试脚本
    """
    total_files = 0
    success_files = 0
    failed_files = []

    # 遍历根目录下的所有子文件夹
    for entry in os.scandir(root_dir):
        if entry.is_dir():
            # 构建目标路径：子文件夹/python/test
            test_dir = os.path.join(entry.path, "python", "test")
            
            if os.path.exists(test_dir) and os.path.isdir(test_dir):
                print(f"\n 在 {entry.name} 中发现测试目录: {test_dir}")

                # 查找所有 .py 文件
                py_files = [
                    os.path.join(test_dir, f)
                    for f in os.listdir(test_dir)
                    if f.endswith(".py") and os.path.isfile(os.path.join(test_dir, f))
                ]

                # 运行每个测试文件
                for py_file in py_files:
                    total_files += 1
                    print(f"\n🚀 正在运行测试: {os.path.relpath(py_file, root_dir)}")

                    try:
                        # 使用子进程运行测试文件
                        result = subprocess.run(
                            ["python3", py_file],
                            check=True,
                            capture_output=True,
                            text=True
                        )
                        
                        # 打印输出结果
                        if result.stdout:
                            print(f" 输出:\n{result.stdout}")
                        success_files += 1

                    except subprocess.CalledProcessError as e:
                        print(f" 运行失败 (退出码 {e.returncode})")
                        print(f" 错误信息:\n{e.stderr}")
                        failed_files.append(py_file)

    # 打印汇总报告
    print("\n" + "="*50)
    print(f" 执行完成汇总:")
    print(f"- 总测试文件: {total_files}")
    print(f"- 成功数量: {success_files}")
    print(f"- 失败数量: {len(failed_files)}")
    
    if failed_files:
        print("\n 失败文件列表:")
        for f in failed_files:
            print(f"  - {os.path.relpath(f, root_dir)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="递归运行 python/test 目录下的测试脚本")
    parser.add_argument(
        "-d", "--directory",
        default=".",
        help="要搜索的根目录（默认当前目录）"
    )
    args = parser.parse_args()

    print(f" 开始扫描目录: {os.path.abspath(args.directory)}")
    find_and_run_tests(os.path.abspath(args.directory))