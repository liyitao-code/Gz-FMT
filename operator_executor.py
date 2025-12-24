#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')

import re
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
from enum import Enum

class GzCommandType(Enum):
    SERVICE = 1
    TOPIC = 2

# 表示和执行gazebo命令
class GzCommand:
    def __init__(self, gz_type, cmd, use_text=True):
        self.gz_type = gz_type
        self.use_text = use_text
        self.cmd = cmd


    @staticmethod
    def dump_empty(filename):
        with open(filename, "w") as f:
            f.write("")

    def dump(self, filename):
        if self.use_text:
            with open(filename, "w") as f:
                if type(self.cmd) == list:
                    f.write("\n".join(self.cmd))
                else:
                    f.write(self.cmd)

    def execute(self):
        if self.use_text:
            if self.gz_type == GzCommandType.SERVICE:
                if self.cmd:
                    try:
                        ret = subprocess.run(self.cmd, shell=True, stdout=subprocess.PIPE, timeout=100) # TODO: check this
                        # what to return here?
                        return ret.stdout.decode("utf-8")
                    except:
                        print("DEBUG: subprocess error")
                        return ""
                else:
                    return None
            elif self.gz_type == GzCommandType.TOPIC:
                # traverse gz_topics
                rets = list()
                for topic in self.cmd:
                    try:
                        ret = subprocess.run(topic, shell=True, stdout=subprocess.PIPE, timeout=100)
                        rets.append(ret.stdout.decode("utf-8"))
                    except:
                        print("DEBUG: subprocess error")

                return "\n".join(rets)
        else:

            if self.gz_type == GzCommandType.SERVICE:
                node = Node()
                msg_type_convert = MessageTypeConvert()
                service_name = self.cmd.service_name
                request = self.cmd.request
                req_type = self.cmd.req_type
                rep_type = self.cmd.rep_type
                timeout = self.cmd.timeout

                info_list = node.service_info(service_name)
                if not info_list:
                    return ""
                info = random.choice(info_list)
                rep_type = msg_type_convert.get_class_type(info.rep_type_name)
                req_type = msg_type_convert.get_class_type(info.req_type_name)
                if req_type:
                    try:
                        random_req = func_timeout.func_timeout(RANDPROTO_TIMEOUT, randomproto.randproto, args=[req_type])
                        req_text = str(random_req).strip()
                        cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype {info.rep_type_name} --reqtype {info.req_type_name} --req '{req_text}'"
                        result, response = self.node.request(service_name, random_req, req_type, rep_type, self.timeout)

                        return response
                    except:
                        return ""
                else:
                    return ""
            elif self.gz_type == GzCommandType.TOPIC:
                for topic_param in self.cmd:
                    publisher = topic_param.publisher
                    publisher.publish(topic_param.message)

                return ""

class OperatorExecutor:
    def __init__(self, config_file, output_dir):
        """初始化执行器"""
        self.config_file = config_file
        self.output_dir = output_dir
        self.node = Node()
        self.world_name = None
        self.timeout = 5000
        
        # 从配置文件加载测试步骤
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        # 创建输出目录
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # 创建日志文件
        self.log_file = os.path.join(output_dir, "operator_log.json")
        self.logs = []
        
        # 加载活动模型列表
        self.models_file = os.path.join(os.path.dirname(config_file), "active_models.json")
        self._load_active_models()

    def _load_active_models(self):
        """加载活动模型列表"""
        try:
            with open(self.models_file, 'r') as f:
                data = json.load(f)
                self.active_models = set(data.get("active_models", []))
        except FileNotFoundError:
            print(f"[WARNING] Active models file not found: {self.models_file}")
            self.active_models = set()

    def _save_active_models(self):
        """保存活动模型列表"""
        with open(self.models_file, 'w') as f:
            json.dump({"active_models": list(self.active_models)}, f, indent=2)

    def _print_model_info(self, prefix=""):
        """打印当前模型信息"""
        active_models = sorted(list(self.active_models))
        print(f"{prefix}Current model count: {len(active_models)}")
        if active_models:
            print(f"{prefix}Active models: {', '.join(active_models)}")
        else:
            print(f"{prefix}No active models")

    def get_world(self):
        """获取当前world名称"""
        service_name = "/gazebo/worlds"
        request = Empty()
        result, response = self.node.request(service_name, request, Empty, StringMsg_V, self.timeout)
        if result and response.data:
            self.world_name = response.data[0]
            return True
        return False

    def add_model(self, step):
        """添加模型"""
        self._print_model_info("[Before add_model] ")
        
        # 生成一个随机的模型名称
        model_name = f"model_{random.randint(1, 1000)}"
        while model_name in self.active_models:  # 确保名称唯一
            model_name = f"model_{random.randint(1, 1000)}"
        
        # 从 models_all.txt 中读取指定 ID 的模型
        model_id = step.get("model_id", 0)
        model_sdf = ""
        model_found = False
        current_model = ""
        current_id = 0
        service_name = f"/world/{self.world_name}/create"
        try:
            with open("models_all.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("<model"):
                        if model_found:  # 已找到目标模型，结束当前模型
                            # model_sdf += "</model>\n"
                            break
                        elif current_id == model_id:  # 找到目标模型
                            model_found = True
                            # 替换模型名称
                            model_sdf = line.replace(line.split('"')[1], model_name)
                            model_sdf += "\n"
                        else:  # 未找到目标模型，继续计数
                            current_id += 1
                    elif model_found:  # 在目标模型内，继续添加内容
                        model_sdf += line + "\n"
        except FileNotFoundError:
            print("[ERROR] models_all.txt not found")
            return False
            
        if not model_found:
            print(f"[ERROR] Model with ID {model_id} not found")
            return False
        
        # 添加随机位置
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = random.uniform(0, 5)
        # model_sdf = "<sdf version=\"1.12\">\n" + model_sdf + "</sdf>"
        model_sdf = "<sdf version=\"1.12\">\n<model name=\"model_8\">\n<pose>-159.0 -1.5 0.5 0 0 0</pose>\n<link name=\"cylinder_link\">\n<inertial>\n<inertia>\n<ixx>2</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>2</iyy>\n<iyz>0</iyz>\n<izz>2</izz>\n</inertia>\n<mass>2.0</mass>\n</inertial>\n<collision name=\"cylinder_collision\">\n<geometry>\n<cylinder>\n<radius>0.5</radius>\n<length>1.0</length>\n</cylinder>\n</geometry>\n</collision>\n\n<visual name=\"cylinder_visual\">\n<geometry>\n<cylinder>\n<radius>0.5</radius>\n<length>1.0</length>\n</cylinder>\n</geometry>\n<material>\n<ambient>0 1 0 1</ambient>\n<diffuse>0 1 0 1</diffuse>\n<specular>0 1 0 1</specular>\n</material>\n</visual>\n</link>\n</model>\n\n</sdf>"
        # 构建请求参数
        model_request = EntityFactory()
        model_request.sdf = model_sdf
        model_request.pose.position.x = x
        model_request.pose.position.y = y
        model_request.pose.position.z = z
        model_request.name = model_name
        model_request.allow_renaming = True
        req_txt = str(model_request).replace(r"\'", r'\"')
        
        # 构建命令
        cmd = f"gz service --timeout {self.timeout} -s {service_name} --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req '{req_txt}'"
        
        print(f"[DEBUG] Executing add_model command: {cmd}")
        
        result = GzCommand(GzCommandType.SERVICE, cmd, True).execute()
        # result = subprocess.run(cmd, capture_output=True, text=True)
        # success = result.returncode == 0
        
        self._log_command(cmd, {
            "result": result,
        })

        # if result:
        #     print(f"[DEBUG] Add model response: {result.stdout.strip()}")
        #     time.sleep(0.5)  # 等待模型加载
        #     self.active_models.add(model_name)
        #     self._save_active_models()
        #     print(f"[DEBUG] Model {model_name} added successfully")
        # else:
        #     print(f"Failed to add model: {result.stderr}")
            
        # self._print_model_info("[After add_model] ")
        return result

    def remove_model(self, step):
        """删除模型"""
        self._print_model_info("[Before remove_model] ")
        scene, reserved_models = self.get_scene()
        self.active_models = scene
        # 检查是否有可删除的模型
        if not self.active_models:
            print("No available models to remove")
            return False
            
        # 随机选择一个模型删除
        model_to_remove = random.choice(list(self.active_models))
        
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
        
        self._log_command(cmd_str.replace('\\', ''), {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr
        })
        
        if success:
            print(f"[DEBUG] Remove model response: {result.stdout.strip()}")
            time.sleep(0.5)  # 等待模型删除
            self.active_models.remove(model_to_remove)
            self._save_active_models()
            print(f"[DEBUG] Model {model_to_remove} removed successfully")
        else:
            print(f"Failed to remove model: {result.stderr}")
            
        self._print_model_info("[After remove_model] ")
        return success

    def list_models(self, step):
        """列出当前世界中的所有模型"""
        self._print_model_info("[list_models] ")
        return True

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
        # gz service -s /world/gravity/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 300 --req ''
        # try:
        #     result, response = self.get_world()
        #     world_name = response.data[0]
        # except:
        #     print("DEBUG: gz process not alive")
        #     return None, None
        # self.world_name = world_name
        node = Node()
        service_name = f"/world/{self.world_name}/scene/info"
        reserved_models = {"ground_model", "ceiling_model", "west_model", "east_model", "north_model", "south_model"}
        request = Empty()
        # request = safe_utf8_encode(request)
        result, response = node.request(service_name, request, Empty, Scene, self.timeout)
        model_blocks = re.findall(r'model\s*{([^}]+)}', str(response), re.DOTALL)
        model_names = []
        for block in model_blocks:
            # 在每个model块中查找name字段
            name_match = re.search(r'name:\s*"([^"]+)"', block)
            if name_match:
                model_names.append(name_match.group(1))
        
        # return names
        # models = [m for m in response.model if m.name not in reserved_models]
        print("DEBUG: now models is ", model_names)
        return model_names, reserved_models
        # return self._get_scene_from_sdf()

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
        
        self._log_command(cmd_str.replace('\\', ''), {
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
        
        self._log_command(cmd_str.replace('\\', ''), {
            "success": success,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "topic": topic
        })
        
        return success

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
