#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')

from gz.msgs11.stringmsg_pb2 import StringMsg
from gz.msgs11.stringmsg_v_pb2 import StringMsg_V
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.empty_pb2 import Empty
from gz.msgs11.scene_pb2 import Scene
from gz.msgs11.entity_pb2 import Entity
from gz.msgs11.sdf_generator_config_pb2 import SdfGeneratorConfig
from gz.msgs11.stringmsg_pb2 import StringMsg
from gz.transport14 import Node
import random
import subprocess
import argparse
import json
import os
import time
import tempfile
import xml.etree.ElementTree as ET

class OperatorExecutor:
    def __init__(self, config_file, output_dir):
        self.config_file = config_file
        self.output_dir = output_dir
        self.node = Node()
        self.world_name = None
        self.timeout = 5000
        self.model_types = ['box', 'sphere', 'cylinder']
        
        # 加载配置文件
        with open(config_file, 'r') as f:
            self.config = json.load(f)
    
    def get_world(self):
        """获取当前世界的名称"""
        # gz service -s /gazebo/worlds --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V --timeout 300 --req ''
        service_name = "/gazebo/worlds"
        request = Empty()
        result, response = self.node.request(service_name, request, Empty, StringMsg_V, self.timeout)
        if result and response.data:
            self.world_name = response.data[0]
            return True
        return False
    
    def _get_scene_from_sdf(self):
        """通过生成和解析 SDF 文件来获取场景信息"""
        if not self.world_name:
            if not self.get_world():
                return None, None
                
        print(f"[DEBUG] Trying to get scene via generate_world_sdf: /world/{self.world_name}/generate_world_sdf")
        
        try:
            # 调用 generate_world_sdf 服务
            service_name = f"/world/{self.world_name}/generate_world_sdf"
            request = SdfGeneratorConfig()
            result, response = self.node.request(service_name, request, SdfGeneratorConfig, StringMsg, self.timeout)
            
            if not result or not response:
                print("[DEBUG] Failed to get world SDF")
                return None, None
                
            # 解析 SDF 字符串
            sdf_str = str(response).encode("utf-8").decode("unicode_escape")[7:-3]  # skip data: ""
            print("[DEBUG] Got world SDF, parsing models...")
            
            # 创建临时文件来解析 XML
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as temp_file:
                temp_file.write(sdf_str)
                temp_path = temp_file.name
            
            try:
                # 使用 xml.etree.ElementTree 解析 SDF
                tree = ET.parse(temp_path)
                root = tree.getroot()
                
                # 查找所有的 model 元素
                models = []
                reserved_models = {"ground_plane", "ground_model", "ceiling_model", "west_model", "east_model", "north_model", "south_model"}
                
                class Model:
                    def __init__(self, name):
                        self.name = name
                
                # 遍历所有 model 元素
                for model in root.findall(".//model"):
                    name = model.get('name')
                    if name:
                        models.append(Model(name))
                        
                print(f"[DEBUG] Found {len(models)} models in SDF")
                
                class Scene:
                    def __init__(self, models):
                        self.model = models
                
                return Scene(models), reserved_models
                
            finally:
                # 清理临时文件
                try:
                    os.unlink(temp_path)
                except:
                    pass
                    
        except Exception as e:
            print(f"[DEBUG] Error during SDF parsing: {str(e)}")
            return None, None

    def get_scene(self):
        """获取当前场景信息"""
        return self._get_scene_from_sdf()

    def _print_model_list(self, prefix=""):
        """打印当前世界中的模型列表"""
        scene, _ = self.get_scene()
        if scene and scene.model:
            print(f"{prefix}Current models in world:")
            for model in scene.model:
                print(f"{prefix}  - {model.name}")
        else:
            print(f"{prefix}No models in world")

    def add_model(self, step):
        """添加模型"""
        self._print_model_list("[Before add_model] ")
        
        # 生成一个随机的模型名称
        model_name = f"model_{random.randint(1, 1000)}"
        
        # 随机选择一个基本形状
        shapes = ["box", "sphere", "cylinder"]
        shape = random.choice(shapes)
        
        # 构建 SDF 字符串
        sdf = f"""<sdf version='1.6'>
<model name='{model_name}'>
<pose>0 0 0 0 0 0</pose>
<link name='link'>
<visual name='visual'>
<geometry>
<{shape}/>
</geometry>
</visual>
<collision name='collision'>
<geometry>
<{shape}/>
</geometry>
</collision>
</link>
</model>
</sdf>"""
        
        # 构建命令
        cmd = [
            "gz",
            "service",
            "-s", f"/world/{self.world_name}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.timeout),
            "--req", f'sdf: "{sdf}"'
        ]
        
        cmd_str = " ".join(cmd)
        print(f"[DEBUG] Executing add_model command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr
        })
        
        if not success:
            print(f"Failed to add model: {result.stderr}")
            return False
            
        # 检查命令的实际输出
        print(f"[DEBUG] Add model response: {result.stdout.strip()}")
        
        # 等待一小段时间让模型真正加入到世界中
        time.sleep(0.5)
        
        self._print_model_list("[After add_model] ")
        
        # 再次获取场景，检查模型是否真的添加了
        scene, _ = self.get_scene()
        if scene and scene.model:
            model_names = [m.name for m in scene.model]
            if model_name not in model_names:
                print(f"[WARNING] Model {model_name} was not found in the world after adding")
                return False
        
        return True

    def remove_model(self, step):
        """删除模型"""
        self._print_model_list("[Before remove_model] ")
        
        # 获取当前场景中的模型
        scene, reserved_models = self.get_scene()
        if not scene or not scene.model:
            print("No available models to remove")
            return False
            
        # 过滤掉保留的模型
        available_models = [m.name for m in scene.model if m.name not in reserved_models]
        if not available_models:
            print("No available models to remove")
            return False
            
        # 随机选择一个模型删除
        model_to_remove = random.choice(available_models)
        
        # 构建命令
        cmd = [
            "gz",
            "service",
            "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.timeout),
            "--req", f'name: "{model_to_remove}"'
        ]
        
        cmd_str = " ".join(cmd)
        print(f"[DEBUG] Executing remove_model command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr
        })
        
        if not success:
            print(f"Failed to remove model: {result.stderr}")
            return False
            
        # 检查命令的实际输出
        print(f"[DEBUG] Remove model response: {result.stdout.strip()}")
        
        # 等待一小段时间让模型真正从世界中移除
        time.sleep(0.5)
        
        self._print_model_list("[After remove_model] ")
        
        # 再次获取场景，检查模型是否真的被删除了
        scene, _ = self.get_scene()
        if scene and scene.model:
            model_names = [m.name for m in scene.model]
            if model_to_remove in model_names:
                print(f"[WARNING] Model {model_to_remove} is still in the world after removal")
                return False
        
        return True
    
    def list_models(self, step):
        """列出当前世界中的所有模型"""
        self._print_model_list("[list_models] ")
        return True
    
    def exec_service(self, step):
        """执行service指令"""
        service = step.get("service", "")
        if not service:
            print("No service specified")
            return False
            
        cmd = [
            "gz",
            "service",
            "-s", service,
            "--reqtype", step.get("reqtype", "gz.msgs.Empty"),
            "--reptype", step.get("reptype", "gz.msgs.Boolean"),
            "--timeout", str(step.get("timeout", "5000"))
        ]
        
        req = step.get("req", "")
        if req:
            cmd.extend(["--req", req])
            
        cmd_str = " ".join(cmd)
        print(f"[DEBUG] Executing exec_service command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "service": service
        })
        
        return success
    
    def exec_topic(self, step):
        """执行topic指令"""
        topic = step.get("topic", "")
        if not topic:
            print("No topic specified")
            return False
            
        cmd = [
            "gz",
            "topic",
            "-t", topic,
            "-m", step.get("msgtype", "gz.msgs.StringMsg")
        ]
        
        msg = step.get("msg", "")
        if msg:
            cmd.extend(["-p", msg])
            
        cmd_str = " ".join(cmd)
        print(f"[DEBUG] Executing exec_topic command: {cmd_str}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        success = result.returncode == 0
        
        self._log_command(cmd_str, {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "topic": topic
        })
        
        return success
    
    def _print_and_log(self, message):
        """同时打印和记录消息"""
        print(message)
        with open(os.path.join(self.output_dir, "operator.log"), "a") as f:
            f.write(message + "\n")
    
    def _log_command(self, cmd_str, result):
        """记录命令执行结果"""
        log_entry = {
            "command": cmd_str,
            "result": result
        }
        with open(os.path.join(self.output_dir, "commands.log"), "a") as f:
            json.dump(log_entry, f)
            f.write("\n")
    
    def run(self):
        """执行所有步骤"""
        if not self.get_world():
            print("Failed to get world name")
            return False
            
        print("\n=== Starting to execute", len(self.config), "steps ===\n")
        
        for i, step in enumerate(self.config, 1):
            print(f"\n[Step {i}/{len(self.config)}] Executing {step['type']}...")
            
            if step["type"] == "add_model":
                success = self.add_model(step)
            elif step["type"] == "remove_model":
                success = self.remove_model(step)
            elif step["type"] == "list_models":
                success = self.list_models(step)
            elif step["type"] == "exec_service":
                success = self.exec_service(step)
            elif step["type"] == "exec_topic":
                success = self.exec_topic(step)
            else:
                print(f"Unknown step type: {step['type']}")
                continue
                
            print(f"Executed {step['type']}: {'Success' if success else 'Failed'}")
        
        print("\n=== Completed executing all steps ===\n")
        return True

def main():
    parser = argparse.ArgumentParser(description='Execute test steps')
    parser.add_argument('--config', type=str, required=True,
                      help='Path to config file')
    parser.add_argument('--output', type=str, required=True,
                      help='Output directory for logs')
    
    args = parser.parse_args()
    
    executor = OperatorExecutor(args.config, args.output)
    executor.run()

if __name__ == "__main__":
    main()
