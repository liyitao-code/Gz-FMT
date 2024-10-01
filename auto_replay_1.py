import os
import subprocess
import argparse

def process_directories(base_dir):
    # 定义需要检查的关键字
    keywords = [
        "collide",
        "> 0.0",
        ">= 0.0",
        "Segmentation fault",
        "must be greater than zero",
        "Aborted"
    ]

    # 递归遍历输入目录的所有子目录
    for root, dirs, files in os.walk(base_dir):
        for dir_name in dirs:
            if dir_name.startswith('_'):  # 仅处理以 '_' 开头的目录
                dir_path = os.path.join(root, dir_name)

                # 构建命令
                command = f"python3 replay.py -S 0 -E -1 -d {dir_path} -s 12345"
                print("now dir is " + dir_path)
                flag = 0
                
                try:
                    # 执行命令并获取输出
                    result = subprocess.run(command, shell=True, capture_output=True, text=True)
                    output = result.stdout + result.stderr  # 获取标准输出和错误输出

                    # 检查输出内容
                    if any(keyword in output for keyword in keywords):
                        # 如果输出中包含任何关键字，将其保存到 replay_output.txt
                        with open(os.path.join(dir_path, "replay_output.txt"), "w") as f:
                            f.write(output)
                    else:
                        # 如果没有任何关键字，将空文件保存为 cant_replay.txt
                        open(os.path.join(dir_path, "cant_replay.txt"), "w").close()

                except Exception as e:
                    print(f"Error processing directory {dir_path}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process directories and execute replay commands.")
    parser.add_argument("input_dir", help="The base input directory to search for _x directories.")

    args = parser.parse_args()

    process_directories(args.input_dir)
