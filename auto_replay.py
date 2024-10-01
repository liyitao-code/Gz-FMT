import os
import subprocess
import argparse
import time
import select
import psutil

def kill_ruby_processes():
    """Kill all Ruby processes."""
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.info['name'] == 'ruby':
            try:
                proc.kill()
                print(f"Killed Ruby process with PID {proc.info['pid']}")
            except psutil.NoSuchProcess:
                continue

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
                command = f"python3 replay.py -S 0 -E -1 -d {dir_path} -s 12345"

                # 检查父目录名称是否为 'collide'
                if os.path.basename(root) == 'collide':
                    print(f"Skipping directory {dir_path} as it is under 'collide'.")
                    continue  # 跳过此目录

                print("now dir is " + dir_path)
                try:
                    # 启动命令并设置管道
                    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

                    flag = 0
                    output_complete = ""
                    last_output_time = time.time()

                    while True:
                        # 使用 select 来监视 stdout 和 stderr
                        reads = [process.stdout.fileno(), process.stderr.fileno()]
                        ret = select.select(reads, [], [], 60)  # 1分钟超时

                        if ret[0]:  # 如果有数据可读
                            for fd in ret[0]:
                                if fd == process.stdout.fileno():
                                    output = process.stdout.readline()
                                elif fd == process.stderr.fileno():
                                    output = process.stderr.readline()

                                if output:
                                    output_complete += output
                                    last_output_time = time.time()  # 更新最后输出时间

                                    # 检查输出中的关键词
                                    if any(keyword in output for keyword in keywords):
                                        with open(os.path.join(dir_path, "replay_output.txt"), "w") as f:
                                            f.write(output_complete)
                                        process.kill()  # 终止进程
                                        flag = 1
                                        break
                        if flag == 1:
                            break

                        # 检查是否超时
                        if time.time() - last_output_time > 60:  # 1分钟无输出
                            break

                    # 如果在超时后仍然没有找到关键词，则保存空文件
                    if flag == 0:
                        open(os.path.join(dir_path, "cant_replay.txt"), "w").close()

                except Exception as e:
                    print(f"Error processing directory {dir_path}: {e}")

                kill_ruby_processes()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process directories and execute replay commands.")
    parser.add_argument("input_dir", help="The base input directory to search for _x directories.")

    args = parser.parse_args()

    process_directories(args.input_dir)
