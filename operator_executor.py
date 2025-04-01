#!/usr/bin/env python3

import json
import argparse
import subprocess
import os
import time
import datetime
import sys
import random

class OperatorExecutor:
    def __init__(self, config_path, output_dir=None):
        self.config_path = config_path
        self.output_dir = output_dir
        self.command_log = []
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
    
    def _log_command(self, cmd, result):
        """记录命令执行结果"""
        timestamp = datetime.datetime.now().isoformat()
        self.command_log.append({
            "timestamp": timestamp,
            "command": cmd,
            "result": result
        })

    def _save_command_log(self):
        """保存命令日志"""
        if self.output_dir:
            log_file = os.path.join(self.output_dir, "command_log.json")
            with open(log_file, 'w') as f:
                json.dump(self.command_log, f, indent=2)

    def _print_and_log(self, message, stderr=False):
        """同时打印到终端和日志文件"""
        print(message, file=sys.stderr if stderr else sys.stdout)
        sys.stdout.flush()
        sys.stderr.flush()

    def exec_topic(self, step):
        """执行主题命令"""
        topic = step["topic"]
        msg_type = step["msg_type"]
        data = json.dumps(step["data"])
        
        cmd = ["gz", "topic", "-p", data, "-t", topic, "-m", msg_type]
        cmd_str = " ".join(cmd)
        self._print_and_log(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        self._print_and_log(f"Executed exec_topic: {'Success' if success else 'Failed'}")
        
        if not success:
            self._print_and_log(f"Error: {result.stderr}", stderr=True)
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "topic": topic
        })
        
        return success
    
    def exec_service(self, step):
        """执行服务命令"""
        service = step["service"]
        req_type = step["req_type"]
        rep_type = step["rep_type"]
        data = step["data"]
        
        # 构建请求字符串
        req_str = ""
        for key, value in data.items():
            req_str += f"{key}: {value} "
        
        cmd = [
            "gz",
            "service",
            "-s", service,
            "--reqtype", req_type,
            "--reptype", rep_type,
            "--timeout", "5000",
            "--req", req_str.strip()
        ]
        cmd_str = " ".join(cmd)
        self._print_and_log(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        self._print_and_log(f"Executed exec_service: {'Success' if success else 'Failed'}")
        
        if not success:
            self._print_and_log(f"Error: {result.stderr}", stderr=True)
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "service": service
        })
        
        return success
    
    def add_model(self, step):
        """添加模型"""
        model_type = step["model_type"]
        model_name = f"model_{random.randint(1, 100)}"
        
        # 创建SDF内容
        sdf_content = f"""<sdf version='1.6'>
            <model name='{model_name}'>
                <pose>0 0 0 0 0 0</pose>
                <link name='link'>
                    <visual name='visual'>
                        <geometry>
                            <{model_type}/>
                        </geometry>
                    </visual>
                    <collision name='collision'>
                        <geometry>
                            <{model_type}/>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>"""
        
        cmd = [
            "gz",
            "service",
            "-s", "/world/default/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", f"sdf: \"{sdf_content}\""
        ]
        cmd_str = " ".join(cmd)
        self._print_and_log(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        self._print_and_log(f"Executed add_model: {'Success' if success else 'Failed'}")
        
        if not success:
            self._print_and_log(f"Error: {result.stderr}", stderr=True)
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "model_name": model_name
        })
        
        return success
    
    def execute_steps(self):
        """执行所有步骤"""
        success = True
        total_steps = len(self.config.get("steps", []))
        self._print_and_log(f"\n=== Starting to execute {total_steps} steps ===\n")
        
        for i, step in enumerate(self.config.get("steps", []), 1):
            step_type = step["type"]
            self._print_and_log(f"\n[Step {i}/{total_steps}] Executing {step_type}...")
            
            try:
                if step_type == "exec_topic":
                    success &= self.exec_topic(step)
                elif step_type == "exec_service":
                    success &= self.exec_service(step)
                elif step_type == "add_model":
                    success &= self.add_model(step)
                else:
                    self._print_and_log(f"Unknown step type: {step_type}", stderr=True)
                    success = False
            except Exception as e:
                self._print_and_log(f"Error executing step {step_type}: {str(e)}", stderr=True)
                success = False
            
            # 在每个步骤后等待一小段时间
            time.sleep(0.5)
        
        self._print_and_log(f"\n=== Completed executing all steps with {'success' if success else 'failure'} ===\n")
        
        # 保存命令日志
        self._save_command_log()
        
        return success

def main():
    parser = argparse.ArgumentParser(description="Operator Executor")
    parser.add_argument("--config", type=str, required=True,
                      help="Path to operator configuration file")
    parser.add_argument("--output", type=str,
                      help="Output directory for command logs")
    
    args = parser.parse_args()
    
    executor = OperatorExecutor(args.config, args.output)
    success = executor.execute_steps()
    
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
