#!/usr/bin/env python3

import json
import argparse
import subprocess
import os
import time
import datetime

class OperatorExecutor:
    def __init__(self, config_path, output_dir=None):
        self.config_path = config_path
        self.output_dir = output_dir
        self.command_log = []
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
    
    def _log_command(self, cmd, result):
        """记录执行的命令和结果"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = {
            "timestamp": timestamp,
            "command": cmd,
            "result": result
        }
        self.command_log.append(log_entry)
    
    def _save_command_log(self):
        """保存命令日志到文件"""
        if self.output_dir:
            log_file = os.path.join(self.output_dir, "commands.json")
            with open(log_file, 'w') as f:
                json.dump(self.command_log, f, indent=2)
    
    def exec_topic(self, step):
        """执行topic命令"""
        cmd = [
            "gz",
            "topic",
            "-p", f"{step['data']}",
            "-t", step["topic"],
            "-m", step["msg_type"]
        ]
        cmd_str = " ".join(cmd)
        print(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        print(f"Executed exec_topic: {'Success' if success else 'Failed'}")
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr
        })
        
        return success
    
    def exec_service(self, step):
        """执行service命令"""
        cmd = [
            "gz",
            "service",
            "-s", step["service"],
            "--reqtype", step["req_type"],
            "--reptype", step["rep_type"],
            "--timeout", "5000",
            "--req", " ".join(f"{k}: {v}" for k, v in step["data"].items())
        ]
        cmd_str = " ".join(cmd)
        print(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        print(f"Executed exec_service: {'Success' if success else 'Failed'}")
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr
        })
        
        return success
    
    def add_model(self, step):
        """添加模型"""
        model_type = step.get("model_type", "box")
        model_name = f"model_{int(time.time() * 1000) % 100}"
        
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
        print(f"Executing command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        print(f"Executed add_model: {'Success' if success else 'Failed'}")
        
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
        for step in self.config.get("steps", []):
            step_type = step["type"]
            try:
                if step_type == "exec_topic":
                    success &= self.exec_topic(step)
                elif step_type == "exec_service":
                    success &= self.exec_service(step)
                elif step_type == "add_model":
                    success &= self.add_model(step)
                else:
                    print(f"Unknown step type: {step_type}")
                    success = False
            except Exception as e:
                print(f"Error executing step {step_type}: {str(e)}")
                success = False
        
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
