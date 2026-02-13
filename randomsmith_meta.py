#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#   

# valgrind --tool=memcheck          --leak-check=full          --show-leak-kinds=all          --track-origins=yes          --verbose          --log-file=valgrind.log          python3 testfixture_smith_1.py DynamicTestRunner.test_dynamic_workflow -v

# valgrind --tool=memcheck \
#          --leak-check=full \
#          --show-leak-kinds=all \
#          --track-origins=yes \
#          --read-var-info=yes \
#          --verbose \
#          --log-file=valgrind-debug.log \
#          python3 /home/liyitao/workspace/rezilla-modelsmith-sim9/src/sdformat/python/test/pyWorld_TEST.py
#          python3 testfixture_smith_1.py DynamicTestRunner.test_dynamic_workflow -v

import sys
sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

from gz.msgs11.stringmsg_pb2 import StringMsg
from gz.msgs11.stringmsg_v_pb2 import StringMsg_V
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.pose_v_pb2 import Pose_V
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.empty_pb2 import Empty
from gz.msgs11.scene_pb2 import Scene
from gz.msgs11.entity_pb2 import Entity
from gz.msgs11.sdf_generator_config_pb2 import SdfGeneratorConfig
from gz.msgs11.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs11.plugin_pb2 import Plugin
from gz.msgs11.entity_wrench_pb2 import EntityWrench
from gz.msgs11.vector3d_pb2 import Vector3d
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.world_stats_pb2 import WorldStatistics
from gz.msgs11.time_pb2 import Time
from gz.msgs11.log_playback_control_pb2 import LogPlaybackControl
from gz.msgs11.serialized_pb2 import SerializedState
from gz.msgs11.serialized_map_pb2 import SerializedStepMap
from gz.msgs11.world_control_state_pb2 import WorldControlState
from gz.msgs11.physics_pb2 import Physics
# from gz.msgs11.entity_pb2 import Entity_Type
from gz.transport14 import Node
from modelsmith import RootGen, ModelGen, POSE, PLUGIN_DIR
from lxml.etree import tostring
import random
import re
import randomproto
import subprocess
from glob import glob
from os.path import basename
import os
import importlib
from optparse import OptionParser
from datetime import datetime
# from coverage_process import CoverageInfo, CoverageDiff, BUILD_DIR, GCOV_DIR
import copy
from enum import Enum
import psutil
import shutil
import time
from plugin_mining import PluginMiner, SdfMiner
from sdf_diversity import SdfDiversity
from crash_result import ErrorLog

import logging
import logging.config
import func_timeout

from lxml import etree
import string

import xml.etree.ElementTree as ET

from search_plugin_in_model import retrieve_plugin_by_index
from search_plugin_in_world import retrieve_plugin_in_world_by_index
from search_model_with_plugin import retrieve_model_by_index

FIRST_DIR = ['/home/liyitao/workspace/gz_lastest', '/home/liyitao/gazebo/800']
DIR_FLAG = 0

DIR = FIRST_DIR[DIR_FLAG] + '/install/lib/python/gz/msgs11'
MAX_MODEL_NUM = 20
NUM_ARM = 7 # dirty, should be calculated, not assigned
RANDPROTO_TIMEOUT = 10

actions = ['add_model', 'remove_model', 'modify_position', 'add_component']

import fcntl

# 保证data的utf8编码正确
def safe_utf8_encode(data):
    if isinstance(data, str):
        return data.encode('utf-8', errors='replace').decode('utf-8')
    return data

# 定义每个算子的参数数量
action_param_counts = {
    0: 1,    # RANDOM_LOAD_MODEL
    1: 1,    # RANDOM_LOAD_MODEL_XML
    2: 117,  # RANDOM_ADD_PLUGIN
    3: 117,  # RANDOM_ADD_PLUGIN_XML
    4: 1,    # RANDOM_REMOVE_MODEL
    5: 1,   # RANDOM_EXEC_SERVICE (动态获取)
    6: 1,   # RANDOM_EXEC_TOPIC (动态获取)
    7: 1,    # RANDOM_SET_POSE
    8: 123,  # RANDOM_ADD_MODEL_WITH_PLUGIN
    9: 123   # RANDOM_ADD_MODEL_WITH_PLUGIN_XML
}

class SimulatorAction:
    '''动作空间'''
    RANDOM_LOAD_MODEL = 0
    RANDOM_LOAD_MODEL_XML = 1
    RANDOM_ADD_PLUGIN = 2
    RANDOM_ADD_PLUGIN_XML = 3
    RANDOM_REMOVE_MODEL = 4
    RANDOM_EXEC_SERVICE = 5
    RANDOM_EXEC_TOPIC = 6
    RANDOM_SET_POSE = 7
    RANDOM_ADD_MODEL_WITH_PLUGIN = 8
    RANDOM_ADD_MODEL_WITH_PLUGIN_XML = 9

    @staticmethod
    def perform_action(action):
        if action == SimulatorAction.RANDOM_LOAD_MODEL:
            return "func_add_random_model"
        elif action == SimulatorAction.RANDOM_LOAD_MODEL_XML:
            return "func_add_random_model_xml"
        elif action == SimulatorAction.RANDOM_ADD_PLUGIN:
            return "func_add_random_plugin_to_model"
        elif action == SimulatorAction.RANDOM_ADD_PLUGIN_XML:
            return "func_add_random_plugin_to_model_xml"
        elif action == SimulatorAction.RANDOM_REMOVE_MODEL:
            return "func_remove_random_model"
        elif action == SimulatorAction.RANDOM_EXEC_SERVICE:
            return "func_random_service"
        elif action == SimulatorAction.RANDOM_EXEC_TOPIC:
            return "func_random_topic"
        elif action == SimulatorAction.RANDOM_SET_POSE:
            return "func_random_pose"
        elif action == SimulatorAction.RANDOM_ADD_MODEL_WITH_PLUGIN:
            return "fund_add_random_model_with_plugin"
        elif action == SimulatorAction.RANDOM_ADD_MODEL_WITH_PLUGIN_XML:
            return "fund_add_random_model_with_plugin_xml"

# list形式定义每个算子的参数数量
operator_parameter_counts = [1, 1, 117, 117, 1, 1, 1, 1, 123, 123]

# 生成随机字符串
def random_string(length=6):
    return ''.join(random.choices(string.ascii_letters + string.digits, k=length))

# 生成与原始数字有显著差别的随机数字
def random_number_like(original):

    if '.' in original:
        # 浮点数处理
        decimals = len(original.split('.')[1])
        original_value = float(original)
        
        # 策略选择
        strategy = random.choice(['max_min', 'large_offset', 'scale', 'negative'])
        
        if strategy == 'max_min':
            # 使用极值
            # random_number = random.choice([sys.float_info.max, sys.float_info.min])
            random_number = random.choice([500000 + random.uniform(-1e5, 1e5), 500000 + random.uniform(-1e5, 1e5)])
        elif strategy == 'large_offset':
            # 大随机偏移
            random_number = original_value + random.uniform(-1e4, 1e4)
        elif strategy == 'scale':
            # 倍增与缩小
            random_number = original_value * random.choice([10.0, 100.0, 0.1])
        elif strategy == 'negative':
            # 取反
            random_number = original_value * -1

        # 保留小数点后位数
        random_number = round(random_number, decimals)
    
    else:
        # 整数处理
        original_value = int(original)
        
        # 策略选择
        strategy = random.choice(['max_min', 'large_offset', 'scale', 'negative'])
        
        if strategy == 'max_min':
            # 使用极值
            random_number = random.choice([500000 + random.randint(-10000, 10000), -500000 + random.randint(-10000, 10000)])
        elif strategy == 'large_offset':
            # 大随机偏移
            random_number = original_value + random.randint(-10000, 10000)
        elif strategy == 'scale':
            # 倍增与缩小
            random_number = original_value * random.choice([10, 100, 0.1])
        elif strategy == 'negative':
            # 取反
            random_number = original_value * -1

    return str(random_number)

# 解析plugin文本
def parse_plugin(xml_str):
    
    try:
        # 解析XML字符串
        plugin_element = ET.fromstring(xml_str)
    except ET.ParseError as e:
        raise ValueError(f"Invalid XML format: {e}")

    # 提取filename和name属性
    filename = plugin_element.get("filename")
    name = plugin_element.get("name")

    if filename is None or name is None:
        raise ValueError("Missing 'filename' or 'name' attribute in plugin XML.")

    # 提取内部的其他元素（转换回字符串格式）
    internal_elements = "".join(ET.tostring(child, encoding='unicode') for child in plugin_element if child.tag not in ['filename', 'name'])

    return filename, name, internal_elements

def perturb_xml(xml_fragment):
    """
    对 XML 片段进行解析并扰动。
    :param xml_fragment: 输入的 XML 片段字符串
    :return: 扰动后的 XML 片段字符串
    """
    # 包装输入的XML片段为一个完整的XML文档
    wrapped_xml = f"<root>{xml_fragment}</root>"
    
    try:
        # 解析 XML 片段
        root = etree.fromstring(wrapped_xml)

        # 找到所有叶节点
        leaf_nodes = [elem for elem in root.iter() if len(elem) == 0]

        if leaf_nodes:
            # 随机选择一个叶节点
            random_leaf = random.choice(leaf_nodes)

            if random_leaf.text is not None:
                original_text = random_leaf.text.strip()

                # 判断文本内容是否是单一的数字
                if re.match(r"^-?\d+(\.\d+)?$", original_text):
                    # 如果是单一数字，生成相似格式的随机数字
                    mutated_text = random_number_like(original_text)
                    
                    # 输出变异信息
                    # print(f"Node: <{random_leaf.tag}>")
                    # print(f"Original: {original_text}")
                    # print(f"Mutated: {mutated_text}")

                    # 更新节点文本
                    random_leaf.text = mutated_text

                else:
                    # 处理包含多个数字的情况
                    numbers = re.findall(r"-?\d+(?:\.\d+)?", original_text)
                    mutated_numbers = [random_number_like(number) for number in numbers if number]

                    # 将变异后的数字重新组合成字符串
                    mutated_text = ' '.join(mutated_numbers) if mutated_numbers else random_string(len(original_text))
                    
                    # 输出变异信息
                    # print(f"Node: <{random_leaf.tag}>")
                    # print(f"Original: {original_text}")
                    # print(f"Mutated: {mutated_text}")

                    # 更新节点文本
                    random_leaf.text = mutated_text

            else:
                print("Selected leaf node has no text to perturb.")

        # 返回修改后的 XML 片段
        perturbed_xml = etree.tostring(root, pretty_print=True, encoding='unicode')
        # 移除包装的根节点
        return perturbed_xml.replace('<root>', '').replace('</root>', '').strip()

    except etree.XMLSyntaxError as e:
        print(f"XML 解析错误: {e}")
        return None

# 对Protobuf格式的文本进行扰动
def perturb_protobuf_like_text(data):
    # 定义正则表达式模式来匹配键值对
    pattern = r'(\b\w+\b):\s*"([^"]*?)"'
    print("DEBUG: begin random string")
    print(data)

    # 对匹配的内容进行替换，保留键名，扰动值
    def replace_match(match):
        key = match.group(1)
        value = match.group(2)
        new_value = random_string(len(value))
        return f'{key}: "{new_value}"'

    # 对所有满足条件的字符串进行替换
    perturbed_data = re.sub(pattern, replace_match, data)
    # print("DEBUG: end random string")
    # print(perturbed_data)
    # print("DEBUG: perturbed data end")
    return perturbed_data

def load_builtin(filename):
    with open(filename) as f:
        l = f.read().splitlines()

    return {re.sub(r"/world/.*?/", r"/world/{world_name}/", entry) for entry in l}

def non_blocking_read(fd):
    # Make the file non-blocking
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    output = []
    while True:
        try:
            chunk = os.read(fd, 4096).decode()
            if chunk:
                output.append(chunk)
            else:
                break
        except BlockingIOError:
            # No more data available
            break
    return ''.join(output)

# 存储service的参数
class ServiceParam:
    def __init__(self, service_name, request, req_type, rep_type, timeout=10000):
        self.service_name = service_name
        self.request = request
        self.req_type = req_type
        self.rep_type = rep_type
        self.timeout = timeout

# 存储topic的参数
class TopicParam:
    def __init__(self, topic_name, type_name, type_class, publisher, message, timeout=10000):
        self.topic_name = topic_name
        self.type_name = type_name
        self.type_class = type_class
        self.publisher = publisher
        self.message = message
        self.timeout = timeout

# 将消息类型名称转换成类名称
class MessageTypeConvert:
    def __init__(self, directory=DIR):
        self.file_prefix_list = [basename(i)[:-3] for i in glob(f"{DIR}/*.py")]
        self.pb2_modules = list()
        for file in self.file_prefix_list:
            if "__init__" in file:
                continue
            try:
                self.pb2_modules.append(importlib.import_module(f"gz.msgs11.{file}"))
            except:
                print(f"error processing gz.msgs11.{file}")

    def get_class_type(self, type_name):
        if type_name.startswith("gz.msgs."):
            type_name_stripped = type_name.split(".")[-1]
            class_type = None
            for module in self.pb2_modules:
                try:
                    class_type = getattr(module, type_name_stripped)
                    break
                except:
                    continue

            return class_type
        else:
            return None

# 枚举类，定义gazebo命令的类型（service / topic）
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

    def execute(self, experiment_log=None):
        """
        执行命令，并可选地记录到实验日志中
        
        Args:
            experiment_log: 如果提供，会将命令记录到日志中
        """
        if self.use_text:
            if self.gz_type == GzCommandType.SERVICE:
                if self.cmd:
                    # 记录命令
                    if experiment_log is not None:
                        if isinstance(self.cmd, list):
                            for cmd in self.cmd:
                                experiment_log.append({
                                    "type": "command",
                                    "command_type": "service",
                                    "command": cmd,
                                    "timestamp": time.time()
                                })
                        else:
                            experiment_log.append({
                                "type": "command",
                                "command_type": "service",
                                "command": self.cmd,
                                "timestamp": time.time()
                            })
                    
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
                # 记录命令
                if experiment_log is not None:
                    if isinstance(self.cmd, list):
                        for cmd in self.cmd:
                            experiment_log.append({
                                "type": "command",
                                "command_type": "topic",
                                "command": cmd,
                                "timestamp": time.time()
                            })
                    else:
                        experiment_log.append({
                            "type": "command",
                            "command_type": "topic",
                            "command": self.cmd,
                            "timestamp": time.time()
                        })
                
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

# 测试用的类
class SmithUnit:
    def __init__(self, directory="exp", sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0, sdf_miner=None, bandits=None, diversity=None, crashes=None, enable_playback=True):
        self.directory = directory
        self.sdf_name = sdf_name
        self.num_seq = num_seq
        self.gz_cmds = list()
        self.use_text = use_text
        self.node = Node()
        self.sdf_miner = sdf_miner
        self.plugin_miner = PluginMiner(FIRST_DIR[DIR_FLAG] + "/install/share/gz/gz-sim9/worlds/")
        self.diversity = diversity
        self.diversity_rewards = [0] * self.num_seq
        self.crash_rewards = [0] * self.num_seq

        if not os.path.exists(directory):
            os.mkdir(directory)

        self.timeout = timeout
        self.skipped = skipped
        self.world_name = None
        # result, response = self.get_world()
        # self.world_name = response.data[0]
        self.funcs = [member for member in dir(self) if member.startswith("func_")]
        # self.cov_old = CoverageInfo(BUILD_DIR, GCOV_DIR)
        # self.cov_new = None
        self.root_gen = RootGen(self.sdf_miner)
        self.sdf = None
        self.seed = seed
        self.bandits = bandits
        self.crashes = crashes
        self.enable_playback = enable_playback  # 控制是否执行 playback 回溯测试
        self.experiment_log = []  # 记录实验流程，用于复现

    def log_command_execute(self, command_type, command_str, wait_after=0.0, description=""):
        """
        记录命令执行到实验日志
        
        Args:
            command_type: 命令类型 ("launch", "service", "topic")
            command_str: 命令字符串
            wait_after: 执行后等待时间（秒）
            description: 命令描述
        """
        if not hasattr(self, 'experiment_log'):
            self.experiment_log = []
        
        self.experiment_log.append({
            "type": "command",
            "command_type": command_type,
            "command": command_str,
            "wait_after": wait_after,
            "description": description,
            "timestamp": time.time()
        })
    
    def log_sleep(self, duration, description=""):
        """
        记录等待时间到实验日志
        
        Args:
            duration: 等待时间（秒）
            description: 等待原因描述
        """
        if not hasattr(self, 'experiment_log'):
            self.experiment_log = []
        
        self.experiment_log.append({
            "type": "sleep",
            "duration": duration,
            "description": description,
            "timestamp": time.time()
        })

    # 检测是否有新崩溃
    def check_new_crash(self, err_file):
        if not os.path.exists(err_file):
            return False
        error_log = ErrorLog(err_file)
        if not error_log.trace:
            return False
        if error_log.trace in self.crashes:
            return False
        else:
            print(f"new crash: {error_log.trace}")
            self.crashes.add(error_log.trace)
            return True

    # 生成服务或话题
    def generate_service_topic(self, st_name, service_list, topic_list, hardcode_list):
        if st_name in service_list:
            # def func_random_service(self, service_name=""):
            print("1", st_name)
            return self.func_random_service(service_name=st_name)
        elif st_name in topic_list:
            print("2")
            return self.func_random_topic(topic_name=st_name)
        elif st_name in hardcode_list:
            print("3")
            func = getattr(self, st_name)
            return func()
        else:
            print("4")
            return None

    # 启动gazebo模拟器
    def launch_gazebo(self, filename=""):
        # gz_sim = f"gz sim {self.directory}/{self.sdf_name} --seed {self.seed} -v 0 -r -s --headless-rendering" 
        if filename:
            gz_sim = f"gz sim {filename} --seed {self.seed} -v 0 -r -s --headless-rendering" 
        else:
            gz_sim = f"gz sim empty.sdf --seed {self.seed} -v 0 -r -s --headless-rendering" 
        print("DEBUG: before subprocess gz sim")
        try:
            self.process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        except:
            print("DEBUG: subprocess launch gz error")
            return False
        return True

    # 停止gazebo模拟器并存储输出
    def stop_gazebo(self, dump=False):
        if not hasattr(self, "process"):
            return False

        if dump:
            out = non_blocking_read(self.process.stdout.fileno())
            err = non_blocking_read(self.process.stderr.fileno())
            with open(f"{self.directory}/gz.out", "a") as f:
                # f.write(process.stdout.read().decode("utf-8"))
                f.write(out)
            with open(f"{self.directory}/gz.err", "a") as f:
                # f.write(process.stderr.read().decode("utf-8"))
                f.write(err)

        # actually stop gazebo

        print("DEBUG: before psutil")
        with open(f"./terminate", "w") as f:
            f.write(f"{self.process.pid}")

        try:
            for child in psutil.Process(self.process.pid).children(recursive=True):
                print(f"DEBUG: terminating child: {child.pid}")
                child.terminate()
                child.wait()
                # child.kill()
        except:
            print("DEBUG: something happens during terminating children")

        print("DEBUG: before process.terminate")
        self.process.terminate()
        print("DEBUG: before process.wait")
        self.process.wait()
        print("DEBUG: after process.wait")


        try:
            os.remove(f"./terminate")
        except:
            print("DEBUG: exception removing terminate file")

        return True

    # 进行topic的fuzzing
    def topic_fuzzing(self):

        SRC_DIR = FIRST_DIR[DIR_FLAG] + "/src/gz-sim/"
        # traverse all the models with gz topic 
        i = 0
        for idx in range(100):
            for filename in glob(f"{SRC_DIR}/**/*.sdf", recursive=True):
            # for filename in ["/home/ren/play/robot/workspace/src/gz-sim/test/worlds/imu.sdf"]:
                with open(filename) as f:
                    c = f.read()
                # if "gz topic" not in c and "gz service" not in c:
                #     continue

                # TODO: mkdir here
                old_directory = self.directory
                self.directory = f"{old_directory}/{i}"
                os.mkdir(self.directory)
                cmd_id = 0
                shutil.copyfile(filename, f"{self.directory}/a.sdf")


                print(f"DEBUG: {filename}")
                retcode = self.launch_gazebo(filename)
                if not retcode:
                    self.directory = old_directory
                    i += 1
                    continue
                process_status = psutil.Process(self.process.pid)
                time.sleep(5)

                try:
                    result, response = self.get_world()
                    self.world_name = response.data[0]
                except:
                    print("DEBUG: gz process not alive")
                    self.directory = old_directory
                    i += 1
                    continue

                node = Node()
                topic_list = node.topic_list()
                service_list = node.service_list()

                c0 = self.func_add_random_plugin_to_model()
                with open(f"{self.directory}/cmd_{cmd_id}.sh", "w") as f:
                    if type(c0) is list:
                        for c in c0.cmd:
                            f.write(f"{c}\n")
                    elif c0:
                        f.write(c0.cmd)
                    else:
                        f.write("")
                cmd_id += 1
                if c0:
                    c0.execute()
                print(f"DEBUG: topic list: {topic_list}")
                for topic in topic_list + service_list:
                    print(f"DEBUG: topic: {topic}")
                    # FIXME: temporarily skip here
                    if topic.endswith("playback/control"):
                        continue
                    if topic in topic_list:
                        topic_cmd = self.func_random_topic(topic_name=topic)
                    else:
                        topic_cmd = self.func_random_service(service_name=topic)
                    if topic_cmd:
                        logging.debug(f"topic cmd: {topic_cmd.cmd}")
                        with open(f"{self.directory}/cmd_{cmd_id}.sh", "w") as f:
                            if type(topic_cmd.cmd) is list:
                                for c in topic_cmd.cmd:
                                    f.write(f"{c}\n")
                            else:
                                f.write(topic_cmd.cmd)
                        cmd_id += 1
                        topic_cmd.execute()
                        if process_status.status() == psutil.STATUS_ZOMBIE:
                            print("DEBUG: gz process not alive")


                # c1 = self.func_remove_random_model()
                # if c1:
                #     with open(f"{self.directory}/cmd_{cmd_id}.sh", "w") as f:
                #         if type(c1) is list:
                #             for c in c1.cmd:
                #                 f.write(f"{c}\n")
                #         else:
                #             f.write(c1.cmd)
                #     cmd_id += 1
                #     c1.execute()
                print("DEBUG: after execution")
                if process_status.status() == psutil.STATUS_ZOMBIE:
                    print("DEBUG: gz process not alive")
                self.stop_gazebo(True)
                # TODO: restore dir
                self.directory = old_directory
                i += 1


    # 成对地 生成命令并测试
    def pairwise_generate_and_test_commands(self, iteration=1000000):
        # 0. collect coverage info
        # self.cov_old.collect()
        # 1. run gz sim a.sdf and sleep for a few seconds, dirty

        retcode = self.launch_gazebo()
        if not retcode:
            return 
        process_status = psutil.Process(self.process.pid)
        time.sleep(5)

        try:
            result, response = self.get_world()
            self.world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return None

        builtin_services = load_builtin("builtin_services")
        builtin_topics = load_builtin("builtin_topics")

        builtin_services = {re.sub(r"{world_name}", self.world_name, i) for i in builtin_services}
        builtin_topics = {re.sub(r"{world_name}", self.world_name, i) for i in builtin_topics}
        # TODO: shall we add the pre-build functions? seems it's ok
        hardcoded = ["func_add_random_model",
                "func_add_mined_random_model",
                "func_remove_random_model",
                "func_random_pose",
                "func_random_topic",
                "func_add_random_plugin_to_model"]

        # TODO: test service/topic in a pairwise paradigm

        # 3. obtain the topics and services of current gz execution
        node = Node()
        service_list = node.service_list()
        # topic_list = self.node.topic_list()
        topic_list = []
        services_n_topics = set(service_list) | set(topic_list) # | set(hardcoded)

        # return services_n_topics, service_list, topic_list, builtin_services, builtin_topics

        # 4. optionally, filter out the builtin services and topics


        self.stop_gazebo(True)

        # 5. for each combination of i, j in service/topic
        # for i in range(iteration):
        dir_id = 0
        print(len(self.plugin_miner.models_with_plugin))
        for it in range(10):
            for model in self.plugin_miner.models_with_plugin:

                sdf_content = '<sdf version="1.11">' + tostring(model).decode("utf-8").strip() + '</sdf>'
                old_directory = self.directory
                self.directory = f"{old_directory}/{dir_id}"
                os.mkdir(self.directory)
                st1 = "func_add_random_model"
                st2 = random.choice(list(services_n_topics))
                st3 = "func_remove_random_model"

                self.root_gen.id = 0
                self.create_sdf()

                retcode = self.launch_gazebo()
                if not retcode:
                    return 
                time.sleep(1)
                process_status = psutil.Process(self.process.pid)

                node = Node()
                i = 0
                c1 = self.generate_service_topic(st1, service_list, topic_list, hardcoded)
                # c1 = self.func_add_mined_random_model(sdf_content=sdf_content)
                print("DEBUG: before execution")
                if c1:
                    print("c1")
                    with open(f"{self.directory}/cmd_{i}.sh", "w") as f:
                        f.write(c1.cmd)
                    print(c1.execute())
                    content = self.dump_sdf("world_0")
                    with open(f"{self.directory}/world_{i}.sdf", "w") as f:
                        f.write(content)
                service_list = node.service_list()
                topic_list = node.topic_list()
                i += 1


                for s in range(1):
                    # st2 = random.choice(list(services_n_topics))
                    st2 = "func_add_random_plugin_to_model"
                    c2 = self.generate_service_topic(st2, service_list, topic_list, hardcoded)

                    print("DEBUG: before execution")
                    if c2:
                        print(f"c2: {c2}")
                        with open(f"{self.directory}/cmd_{i}.sh", "a") as f:
                            if type(c2.cmd) is list:
                                for c in c2.cmd:
                                    f.write(f"{c}\n")
                            else:
                                f.write(c2.cmd)
                        print(c2.execute())
                        service_list2 = node.service_list()
                        topic_list2 = node.topic_list()
                        logging.debug(f"iter {dir_id}, service orig: {service_list}")
                        logging.debug(f"iter {dir_id}, service diff: {set(service_list2) - set(service_list)}")
                        logging.debug(f"iter {dir_id}, topic orig: {topic_list}")
                        logging.debug(f"iter {dir_id}, topic diff: {set(topic_list2) - set(topic_list)}")
                        content = self.dump_sdf("world_0")
                        with open(f"{self.directory}/world_{i}.sdf", "a") as f:
                            f.write(content)
                    i += 1

                # deliberately exercising services and topics diff
                service_diff = set(service_list2) - set(service_list)
                topic_diff = set(topic_list2) - set(topic_list)

                for service in service_diff:
                    service_cmd = self.func_random_service(service_name=service)
                    logging.debug(f"service: {service}")
                    if service_cmd:
                        logging.debug(f"service cmd: {service_cmd.cmd}")
                        service_cmd.execute()

                for topic in topic_diff:
                    topic_cmd = self.func_random_topic(topic_name=topic)
                    logging.debug(f"topic: {topic}")
                    if topic_cmd:
                        logging.debug(f"topic cmd: {topic_cmd.cmd}")
                        topic_cmd.execute()

                c3 = self.generate_service_topic(st3, service_list, topic_list, hardcoded)
                if c3:
                    print("c3")
                    with open(f"{self.directory}/cmd_{i}.sh", "w") as f:
                        f.write(c3.cmd)
                    print(c3.execute())
                    content = self.dump_sdf("world_0")
                    with open(f"{self.directory}/world_{i}.sdf", "w") as f:
                        f.write(content)
                i += 1

                print("DEBUG: after execution")
                if process_status.status() == psutil.STATUS_ZOMBIE:
                    print("DEBUG: gz process not alive")
                self.stop_gazebo(True)
                #     # should collect error message, and 

                #     break
                self.directory = old_directory
                dir_id += 1
        # stop gz 
        # print("DEBUG: before stop_gazebo")
        # self.stop_gazebo(True)
        # print("DEBUG: after stop_gazebo")



    # 生成和测试命令（修改为执行蜕变测试）
    def generate_and_test_commands(self):
        # 初始化实验日志
        self.experiment_log = []
        experiment_start_time = time.time()
        
        # 记录实验基本信息
        self.experiment_log.append({
            "type": "experiment_info",
            "directory": self.directory,
            "sdf_file": self.sdf_name,
            "start_time": experiment_start_time,
            "timestamp": time.time()
        })
        
        # 0. collect coverage info
        print("DEBUG: before cov")
        # 在启动新的 Gazebo 之前，确保所有残留的 Gazebo 相关进程都已关闭
        try:
            subprocess.run("pkill -9 -f 'gz sim'", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run("pkill -9 ruby", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run("pkill -9 -f 'gz-sim-server'", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except:
            pass
        
        # self.cov_old.collect()  # Coverage collection disabled
        # 1. run gz sim a.sdf with recording enabled
        # 设置录像路径为测试目录下的 log 子目录
        record_path = os.path.join(self.directory, "log")
        if not os.path.exists(record_path):
            os.makedirs(record_path)
        
        # 使用 --record-path 参数启动 gazebo 并录像
        gz_sim = f"gz sim {self.directory}/{self.sdf_name} --record-path {record_path}"
        print(f"DEBUG: gz_sim: {gz_sim}")
        print(f"DEBUG: Recording to: {record_path}")
        
        # 记录启动命令
        self.log_command_execute("launch", gz_sim, wait_after=5.0, description="Launch Gazebo with recording")
        
        print("DEBUG: before subprocess gz sim")
        try:
            process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        except:
            print("DEBUG: subprocess launch gz error")
            return 0

        process_status = psutil.Process(process.pid)
        
        # 等待 Gazebo 启动，使用重试机制确保 SDF 完全加载
        max_retries = 5
        wait_per_retry = 2  # 每次重试间隔秒数
        world_found = False
        for retry_i in range(max_retries):
            self.log_sleep(wait_per_retry, f"Wait for Gazebo to start (attempt {retry_i + 1}/{max_retries})")
            time.sleep(wait_per_retry)
            
            # 检查进程是否还活着
            try:
                proc_status = psutil.Process(process.pid).status()
                if proc_status == psutil.STATUS_ZOMBIE:
                    print(f"DEBUG: gz process became zombie at attempt {retry_i + 1}")
                    break
            except psutil.NoSuchProcess:
                print(f"DEBUG: gz process died at attempt {retry_i + 1}")
                break
            
            try:
                result, response = self.get_world()
                self.world_name = response.data[0]
                world_found = True
                print(f"DEBUG: get_world() succeeded at attempt {retry_i + 1}, world_name={self.world_name}")
                break
            except Exception as e:
                print(f"DEBUG: get_world() failed at attempt {retry_i + 1}: {e}")
        
        if not world_found:
            print("DEBUG: gz process not alive or get_world() failed after all retries, cleaning up...")
            # 清理 Gazebo 进程，防止泄漏
            try:
                for child in psutil.Process(process.pid).children(recursive=True):
                    child.kill()
                process.kill()
                process.wait(timeout=5)
            except Exception as e:
                print(f"DEBUG: Error cleaning up gz process: {e}")
            return 0

        # 校验世界名：包含空格或特殊字符的世界名会导致 service/topic 路径无效
        import re
        if not re.match(r'^[a-zA-Z0-9_\-]+$', self.world_name):
            print(f"DEBUG: Invalid world name '{self.world_name}' (contains spaces or special characters), skipping this SDF")
            try:
                for child in psutil.Process(process.pid).children(recursive=True):
                    child.kill()
                process.kill()
                process.wait(timeout=5)
            except Exception as e:
                print(f"DEBUG: Error cleaning up gz process: {e}")
            return 0

        print("DEBUG: starting metamorphic test")
        
        # 随机选择一种蜕变测试关系
        test_type = random.choice(['determinism', 'zero_input_stability', 'force_isolation', 'force_removal', 'temporal_monotonicity'])  # 新范式蜕变关系
        # test_type = random.choice(['motion', 'force_additivity', 'time_scaling', 'mass_scaling', 'determinism', 'symmetry'])  # 旧蜕变关系（reset-compare 范式）
        print(f"DEBUG: Selected metamorphic test type: {test_type}")
        # test_type = 'time_scaling'  # 临时固定测试类型（用于调试）
        
        # 记录测试类型
        self.experiment_log.append({
            "type": "test_info",
            "test_type": test_type,
            "timestamp": time.time()
        })
        
        test_result = None
        
        if test_type == 'motion':
            # 第一种蜕变测试：运动测试
            # 随机生成测试参数
            test_duration = random.uniform(3.0, 8.0)  # 测试持续时间：3-8秒
            
            # 执行蜕变测试，添加异常处理以避免崩溃
            try:
                test_result = self.metamorphic_test_example(velocity_x=None, test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during motion metamorphic test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
                
        elif test_type == 'rewind':
            # 第二种蜕变测试：回溯测试
            # 随机生成测试参数
            record_time_a = random.uniform(2.0, 5.0)  # 记录时间点：2-5秒
            run_time_b = random.uniform(2.0, 5.0)  # 运行时间：2-5秒
            
            # 执行回溯测试，添加异常处理以避免崩溃
            try:
                test_result = self.metamorphic_test_rewind(record_time_a=record_time_a, run_time_b=run_time_b)
            except Exception as e:
                print(f"DEBUG: Exception during rewind metamorphic test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
                
        elif test_type == 'force_additivity':
            # 第三种蜕变测试：力的可加性测试
            # 随机生成测试参数
            test_duration = random.uniform(3.0, 6.0)  # 测试持续时间：3-6秒
            
            # 执行力的可加性测试，添加异常处理以避免崩溃
            try:
                test_result = self.metamorphic_test_force_additivity(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during force additivity test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
                
        elif test_type == 'time_scaling':
            # 第四种蜕变测试：时间比例测试
            # 随机生成测试参数
            test_duration = random.uniform(3.0, 6.0)  # 测试持续时间：3-6秒
            
            # 执行时间比例测试，添加异常处理以避免崩溃
            try:
                test_result = self.metamorphic_test_time_scaling(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during time scaling test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'mass_scaling':
            # 第五种蜕变测试：质量系数测试
            # 随机生成测试参数
            test_duration = random.uniform(3.0, 6.0)  # 测试持续时间：3-6秒
            
            # 执行质量系数测试，添加异常处理以避免崩溃
            try:
                test_result = self.metamorphic_test_mass_scaling(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during mass scaling test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'determinism':
            # 第六种蜕变测试：确定性重复测试
            # 在两个独立的 Gazebo 实例中执行完全相同的实验，验证结果一致
            test_duration = random.uniform(3.0, 6.0)
            
            try:
                # ===== Run 1：在当前 Gazebo 实例中运行 =====
                print("DEBUG: Determinism test - Run 1 (current Gazebo)...")
                run1_result = self._determinism_single_run(test_duration=test_duration)
                
                if run1_result is None:
                    print("DEBUG: Determinism test Run 1 failed (model may be constrained)")
                    test_result = None
                else:
                    d_model_name, d_force_x, d_force_y, d_force_z, pos_run1, d_num_steps = run1_result
                    print(f"Run 1 complete - model: {d_model_name}, pos: {pos_run1}")
                    
                    # ===== 关闭第一个 Gazebo =====
                    print("DEBUG: Shutting down first Gazebo for determinism test...")
                    # 先读取 stdout/stderr（之后第一个进程就没了）
                    out_run1 = non_blocking_read(process.stdout.fileno())
                    err_run1 = non_blocking_read(process.stderr.fileno())
                    
                    try:
                        for child in psutil.Process(process.pid).children(recursive=True):
                            child.terminate()
                            child.wait()
                        process.terminate()
                        process.wait(timeout=10)
                    except Exception as e:
                        print(f"DEBUG: Error terminating first Gazebo: {e}")
                        try:
                            process.kill()
                            process.wait(timeout=5)
                        except:
                            pass
                    
                    time.sleep(2)
                    
                    # 彻底清理残留进程
                    try:
                        subprocess.run("pkill -9 -f 'gz sim'", shell=True, timeout=5,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        subprocess.run("pkill -9 ruby", shell=True, timeout=5,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        subprocess.run("pkill -9 -f 'gz-sim-server'", shell=True, timeout=5,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        time.sleep(1)
                    except:
                        pass
                    
                    # ===== 启动第二个 Gazebo（相同 SDF，不录像） =====
                    print("DEBUG: Starting second Gazebo for determinism test...")
                    gz_sim2 = f"gz sim {self.directory}/{self.sdf_name}"
                    try:
                        process = subprocess.Popen(gz_sim2.split(" "),
                                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                                  start_new_session=True)
                    except Exception as e:
                        print(f"DEBUG: Failed to start second Gazebo: {e}")
                        test_result = None
                        # 创建一个虚拟的 process 以避免后续清理代码报错
                        process = None
                    
                    if process is not None:
                        process_status = psutil.Process(process.pid)
                        
                        # 等待第二个 Gazebo 启动
                        world_found2 = False
                        for retry_i in range(max_retries):
                            self.log_sleep(wait_per_retry, f"Wait for second Gazebo to start (attempt {retry_i + 1}/{max_retries})")
                            time.sleep(wait_per_retry)
                            
                            try:
                                proc_status = psutil.Process(process.pid).status()
                                if proc_status == psutil.STATUS_ZOMBIE:
                                    print(f"DEBUG: Second gz process became zombie at attempt {retry_i + 1}")
                                    break
                            except psutil.NoSuchProcess:
                                print(f"DEBUG: Second gz process died at attempt {retry_i + 1}")
                                break
                            
                            try:
                                result, response = self.get_world()
                                self.world_name = response.data[0]
                                world_found2 = True
                                print(f"DEBUG: Second Gazebo ready, world_name={self.world_name}")
                                break
                            except Exception as e:
                                print(f"DEBUG: get_world() failed for second Gazebo at attempt {retry_i + 1}: {e}")
                        
                        if not world_found2:
                            print("DEBUG: Second Gazebo failed to start")
                            test_result = None
                        else:
                            # ===== Run 2：在第二个 Gazebo 中用相同参数运行 =====
                            print("DEBUG: Determinism test - Run 2 (fresh Gazebo)...")
                            run2_result = self._determinism_single_run(
                                model_name=d_model_name,
                                force_x=d_force_x, force_y=d_force_y, force_z=d_force_z,
                                test_duration=test_duration
                            )
                            
                            if run2_result is None:
                                print("DEBUG: Determinism test Run 2 failed")
                                test_result = None
                            else:
                                _, _, _, _, pos_run2, _ = run2_result
                                print(f"Run 2 complete - pos: {pos_run2}")
                                
                                # ===== 比较两次运行结果 =====
                                import math as _math
                                error_x = abs(pos_run1[0] - pos_run2[0])
                                error_y = abs(pos_run1[1] - pos_run2[1])
                                error_z = abs(pos_run1[2] - pos_run2[2])
                                error_magnitude = _math.sqrt(error_x**2 + error_y**2 + error_z**2)
                                
                                # 确定性仿真的容差应该非常小
                                # 使用 0.001m (1mm) 作为主要容差
                                # 考虑到浮点精度和可能的微小差异
                                tolerance = 0.001
                                success = error_magnitude < tolerance
                                
                                error_info = f"Run 1 position: ({pos_run1[0]:.6f}, {pos_run1[1]:.6f}, {pos_run1[2]:.6f})\n"
                                error_info += f"Run 2 position: ({pos_run2[0]:.6f}, {pos_run2[1]:.6f}, {pos_run2[2]:.6f})\n"
                                error_info += f"Position difference: x={error_x:.6f}, y={error_y:.6f}, z={error_z:.6f} m\n"
                                error_info += f"Error magnitude: {error_magnitude:.6f} m\n"
                                error_info += f"Tolerance: {tolerance} m (1mm)\n"
                                error_info += f"Force: ({d_force_x:.6f}, {d_force_y:.6f}, {d_force_z:.6f}) N\n"
                                error_info += f"Steps: {d_num_steps}, Duration: {test_duration:.2f} s\n"
                                error_info += f"Note: Two independent Gazebo instances with identical SDF and forces.\n"
                                error_info += f"      Results should be bit-for-bit identical for deterministic simulation."
                                
                                print(f"Determinism error: {error_magnitude:.6f} m (tolerance: {tolerance} m)")
                                print(f"Test {'PASSED' if success else 'FAILED'}")
                                
                                test_result = (d_model_name, pos_run1, pos_run2, success, error_info)
                    else:
                        test_result = None
                        
            except Exception as e:
                print(f"DEBUG: Exception during determinism test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'symmetry':
            # 第七种蜕变测试：对称性/方向不变性测试
            test_duration = random.uniform(3.0, 6.0)
            
            try:
                test_result = self.metamorphic_test_symmetry(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during symmetry test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'zero_input_stability':
            # 范式 B：零输入稳定性测试 — 不施力，检查模型是否保持静止
            test_duration = random.uniform(3.0, 6.0)
            
            try:
                test_result = self.metamorphic_test_zero_input_stability(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during zero-input stability test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'force_isolation':
            # 范式 C：多模型力隔离测试 — 施力到 A，检查 B 是否不受影响
            test_duration = random.uniform(3.0, 6.0)
            
            try:
                test_result = self.metamorphic_test_force_isolation(test_duration=test_duration)
            except Exception as e:
                print(f"DEBUG: Exception during force isolation test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'force_removal':
            # 范式 D：撤力响应测试 — 施力后撤除，检查是否停止加速
            force_duration = random.uniform(1.5, 3.0)
            coast_duration = random.uniform(1.5, 3.0)
            
            try:
                test_result = self.metamorphic_test_force_removal(
                    force_duration=force_duration, coast_duration=coast_duration)
            except Exception as e:
                print(f"DEBUG: Exception during force removal test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        elif test_type == 'temporal_monotonicity':
            # 范式 B：时序单调性测试 — 恒定力下位移必须单调递增
            test_duration = random.uniform(3.0, 6.0)
            num_samples = random.choice([8, 10, 12])
            
            try:
                test_result = self.metamorphic_test_temporal_monotonicity(
                    test_duration=test_duration, num_samples=num_samples)
            except Exception as e:
                print(f"DEBUG: Exception during temporal monotonicity test: {e}")
                print(f"DEBUG: Exception type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
                test_result = None
        
        # 记录测试结果
        test_passed = False
        if test_result:
            if test_type == 'motion':
                # 运动确定性测试结果格式: (model_name, initial_pos, pos_A, pos_B, success)
                # pos_A = 第一次运动最终位置, pos_B = 第二次运动最终位置
                model_name, initial_pos, pos_A, pos_B, success = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Motion Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Position (Run A): ({pos_A[0]:.3f}, {pos_A[1]:.3f}, {pos_A[2]:.3f})\n")
                    f.write(f"Position (Run B): ({pos_B[0]:.3f}, {pos_B[1]:.3f}, {pos_B[2]:.3f})\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    
                    # 计算两次运动的位置差异
                    error_x = abs(pos_A[0] - pos_B[0])
                    error_y = abs(pos_A[1] - pos_B[1])
                    error_z = abs(pos_A[2] - pos_B[2])
                    error_magnitude = (error_x**2 + error_y**2 + error_z**2)**0.5
                    disp_a = ((pos_A[0]-initial_pos[0])**2 + (pos_A[1]-initial_pos[1])**2 + (pos_A[2]-initial_pos[2])**2)**0.5
                    disp_b = ((pos_B[0]-initial_pos[0])**2 + (pos_B[1]-initial_pos[1])**2 + (pos_B[2]-initial_pos[2])**2)**0.5
                    max_displacement = max(disp_a, disp_b)
                    relative_error = error_magnitude / max_displacement if max_displacement > 0.001 else 0.0
                    f.write(f"\nError Info:\n")
                    f.write(f"Position difference: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f} m\n")
                    f.write(f"Error magnitude: {error_magnitude:.4f} m\n")
                    f.write(f"Max displacement: {max_displacement:.4f} m\n")
                    f.write(f"Relative error: {relative_error*100:.2f}%\n")
                    f.write(f"Threshold: abs < 0.5 m OR rel < 10%\n")
                    if hasattr(self, '_test_force_x'):
                        f.write(f"Force F: ({self._test_force_x:.6f}, {self._test_force_y:.6f}, {self._test_force_z:.6f}) N\n")
                    f.write(f"Note: Same force applied twice from same initial state. Positions should match.\n")
                    
            elif test_type == 'rewind':
                # 回溯测试结果格式: (success, record_time_a, run_time_b, state_before, state_after, differences)
                success, record_time_a, run_time_b, state_before, state_after, differences = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Rewind Test\n")
                    f.write(f"Record Time (a): {record_time_a:.2f} s\n")
                    f.write(f"Run Time (b): {run_time_b:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\n")
                    
                    if state_before:
                        f.write(f"Models recorded before: {len(state_before)}\n")
                    if state_after:
                        f.write(f"Models recorded after rewind: {len(state_after)}\n")
                    
                    if differences:
                        f.write(f"\nDifferences found: {len(differences)}\n")
                        for diff in differences:
                            f.write(f"  - {diff}\n")
                    else:
                        f.write(f"\nNo differences found - states match perfectly!\n")
                        
            elif test_type == 'force_additivity':
                # 力的可加性测试结果格式: (model_name, initial_pos, pos_with_f1_f2, pos_with_f_total, success, error_info)
                model_name, initial_pos, pos_with_f1_f2, pos_with_f_total, success, error_info = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Force Additivity Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Position with F1+F2 (simultaneous): ({pos_with_f1_f2[0]:.3f}, {pos_with_f1_f2[1]:.3f}, {pos_with_f1_f2[2]:.3f})\n")
                    f.write(f"Position with F_total (combined): ({pos_with_f_total[0]:.3f}, {pos_with_f_total[1]:.3f}, {pos_with_f_total[2]:.3f})\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'time_scaling':
                # 时间比例测试结果格式: (model_name, initial_pos, pos_with_rtf1, pos_with_rtf2, rtf1, rtf2, success, error_info)
                model_name, initial_pos, pos_with_rtf1, pos_with_rtf2, rtf1, rtf2, success, error_info = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Time Scaling Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Position with rtf={rtf1:.2f}: ({pos_with_rtf1[0]:.3f}, {pos_with_rtf1[1]:.3f}, {pos_with_rtf1[2]:.3f})\n")
                    f.write(f"Position with rtf={rtf2:.2f}: ({pos_with_rtf2[0]:.3f}, {pos_with_rtf2[1]:.3f}, {pos_with_rtf2[2]:.3f})\n")
                    f.write(f"Test Duration (physical time): {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'mass_scaling':
                # 质量系数测试结果格式: (model_name, initial_pos, initial_mass, mass_scale_factor, pos_with_m1, pos_with_m2, d1, d2, success, error_info)
                model_name, initial_pos, initial_mass, mass_scale_factor, pos_with_m1, pos_with_m2, d1, d2, success, error_info = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Mass Scaling Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Initial Mass: {initial_mass:.6f} kg\n")
                    f.write(f"Mass Scale Factor k: {mass_scale_factor:.6f}\n")
                    f.write(f"New Mass (k*m): {initial_mass * mass_scale_factor:.6f} kg\n")
                    f.write(f"Position with mass m: ({pos_with_m1[0]:.3f}, {pos_with_m1[1]:.3f}, {pos_with_m1[2]:.3f})\n")
                    f.write(f"Position with mass k*m: ({pos_with_m2[0]:.3f}, {pos_with_m2[1]:.3f}, {pos_with_m2[2]:.3f})\n")
                    f.write(f"Displacement d1: ({d1[0]:.3f}, {d1[1]:.3f}, {d1[2]:.3f})\n")
                    f.write(f"Displacement d2: ({d2[0]:.3f}, {d2[1]:.3f}, {d2[2]:.3f})\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'determinism':
                # 确定性测试结果格式: (model_name, pos_run1, pos_run2, success, error_info)
                model_name, pos_run1, pos_run2, success, error_info = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Determinism Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Position (Run 1 - first Gazebo): ({pos_run1[0]:.6f}, {pos_run1[1]:.6f}, {pos_run1[2]:.6f})\n")
                    f.write(f"Position (Run 2 - fresh Gazebo): ({pos_run2[0]:.6f}, {pos_run2[1]:.6f}, {pos_run2[2]:.6f})\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'symmetry':
                # 对称性测试结果格式: (model_name, initial_pos, pos_after_x, pos_after_y, dx, dy, force_magnitude, success, error_info)
                model_name, initial_pos, pos_after_x, pos_after_y, dx, dy, force_magnitude, success, error_info = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Symmetry Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Position after x-force: ({pos_after_x[0]:.3f}, {pos_after_x[1]:.3f}, {pos_after_x[2]:.3f})\n")
                    f.write(f"Position after y-force: ({pos_after_y[0]:.3f}, {pos_after_y[1]:.3f}, {pos_after_y[2]:.3f})\n")
                    f.write(f"x-displacement (dx): {dx:.4f} m\n")
                    f.write(f"y-displacement (dy): {dy:.4f} m\n")
                    f.write(f"Force Magnitude: {force_magnitude:.2f} N\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'zero_input_stability':
                # 零输入稳定性测试结果格式: (model_name, initial_pos, final_pos, drift_x, drift_y, drift_z, success, error_info)
                model_name, initial_pos, final_pos, drift_x, drift_y, drift_z, success, error_info = test_result
                test_passed = success
                
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Zero-Input Stability Test (Paradigm B)\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f})\n")
                    f.write(f"Final Position: ({final_pos[0]:.6f}, {final_pos[1]:.6f}, {final_pos[2]:.6f})\n")
                    f.write(f"Drift: x={drift_x:.6f}m, y={drift_y:.6f}m, z={drift_z:.6f}m\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'force_isolation':
                # 力隔离测试结果格式: (target_name, bystander_name, target_initial, bystander_initial,
                #                      target_final, bystander_final, target_displacement, bystander_drift,
                #                      force_magnitude, success, error_info)
                (target_name, bystander_name, target_initial, bystander_initial,
                 target_final, bystander_final, target_displacement, bystander_drift,
                 force_magnitude, success, error_info) = test_result
                test_passed = success
                
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Force Isolation Test (Paradigm C)\n")
                    f.write(f"Target Model: {target_name}\n")
                    f.write(f"Bystander Model: {bystander_name}\n")
                    f.write(f"Target Initial: ({target_initial[0]:.6f}, {target_initial[1]:.6f}, {target_initial[2]:.6f})\n")
                    f.write(f"Target Final: ({target_final[0]:.6f}, {target_final[1]:.6f}, {target_final[2]:.6f})\n")
                    f.write(f"Bystander Initial: ({bystander_initial[0]:.6f}, {bystander_initial[1]:.6f}, {bystander_initial[2]:.6f})\n")
                    f.write(f"Bystander Final: ({bystander_final[0]:.6f}, {bystander_final[1]:.6f}, {bystander_final[2]:.6f})\n")
                    f.write(f"Target Displacement: {target_displacement:.6f}m\n")
                    f.write(f"Bystander Drift: {bystander_drift:.6f}m\n")
                    f.write(f"Force Magnitude: {force_magnitude:.2f} N\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'force_removal':
                # 撤力响应测试结果格式: (model_name, initial_pos, pos_after_force, pos_coast_mid, pos_coast_end,
                #                        v_force, v_coast1, v_coast2, force_magnitude, success, error_info)
                (model_name, initial_pos, pos_after_force, pos_coast_mid, pos_coast_end,
                 v_force, v_coast1, v_coast2, force_magnitude, success, error_info) = test_result
                test_passed = success
                
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Force Removal Response Test (Paradigm D)\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Force Magnitude: {force_magnitude:.2f} N\n")
                    f.write(f"Velocity (force phase avg): {v_force:.6f} m/s\n")
                    f.write(f"Velocity (coast 1st half):  {v_coast1:.6f} m/s\n")
                    f.write(f"Velocity (coast 2nd half):  {v_coast2:.6f} m/s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
                    
            elif test_type == 'temporal_monotonicity':
                # 时序单调性测试结果格式: (model_name, initial_pos, trajectory, force_magnitude,
                #                         monotonic, smooth, violations, success, error_info)
                (model_name, initial_pos, trajectory, force_magnitude,
                 monotonic, smooth, violations, success, error_info) = test_result
                test_passed = success
                
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Temporal Monotonicity Test (Paradigm B)\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Force Magnitude: {force_magnitude:.2f} N\n")
                    f.write(f"Monotonic: {'Yes' if monotonic else 'No'}\n")
                    f.write(f"Smooth: {'Yes' if smooth else 'No'}\n")
                    f.write(f"Violations: {len(violations)}\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    f.write(f"\nError Info:\n{error_info}\n")
        else:
            print("DEBUG: metamorphic test returned None")
            with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                f.write(f"Test Type: {test_type}\n")
                f.write("Test failed to execute (returned None)\n")
        
        # 如果测试失败，打标签（在目录中创建标记文件）
        if not test_passed:
            with open(f"{self.directory}/METAMORPHIC_TEST_FAILED", "w") as f:
                f.write(f"Metamorphic test failed at {datetime.now().isoformat()}\n")
            print("DEBUG: Metamorphic test FAILED - tag created")
        
        # 记录实验结束信息
        experiment_end_time = time.time()
        self.experiment_log.append({
            "type": "experiment_info",
            "end_time": experiment_end_time,
            "duration": experiment_end_time - experiment_start_time,
            "test_passed": test_passed,
            "test_type": test_type,
            "timestamp": time.time()
        })
        
        # 保存实验日志到JSON文件
        experiment_log_file = os.path.join(self.directory, "experiment_log.json")
        import json
        with open(experiment_log_file, "w") as f:
            json.dump(self.experiment_log, f, indent=2)
        print(f"DEBUG: Experiment log saved to {experiment_log_file}")
        
        # 检查进程状态（determinism 测试中 process 可能为 None）
        if process is None:
            print("DEBUG: process is None (likely determinism test cleanup), skipping process cleanup")
            out = ""
            err = ""
        else:
            try:
                if process_status.status() == psutil.STATUS_ZOMBIE:
                    print("DEBUG: gz process not alive")
            except:
                pass

            out = non_blocking_read(process.stdout.fileno())
            err = non_blocking_read(process.stderr.fileno())
            # 3. terminate gz sim
            print("DEBUG: before psutil")
            with open(f"./terminate", "w") as f:
                f.write(f"{process.pid}")

            for child in psutil.Process(process.pid).children(recursive=True):
                print(f"DEBUG: terminating child: {child.pid}")
                child.terminate()
                child.wait()
                # child.kill()
            print("DEBUG: before process.terminate")
            process.terminate()
            print("DEBUG: before process.wait")
            process.wait()
            print("DEBUG: after process.wait")
        
        # 等待录像文件完全写入
        print("DEBUG: Waiting for log files to be written...")
        time.sleep(2)
        
        # 4. 执行 playback 模式下的回溯测试（如果启用）
        # 注意：determinism 测试使用两个 Gazebo 实例，第一个的录像可能不完整，跳过 playback
        if self.enable_playback and test_type != 'determinism':
            print("DEBUG: Starting playback rewind test...")
            playback_result = None
            try:
                playback_result = self.metamorphic_test_playback_rewind(record_path, test_type, test_result)
            except Exception as e:
                print(f"DEBUG: Exception during playback rewind test: {e}")
                import traceback
                traceback.print_exc()
                playback_result = None
            
            # 记录 playback 测试结果
            if playback_result:
                playback_success, playback_details = playback_result
                with open(f"{self.directory}/playback_test_result.txt", "w") as f:
                    f.write(f"Test Type: Playback Rewind Test\n")
                    f.write(f"Log Path: {record_path}\n")
                    f.write(f"Result: {'PASSED' if playback_success else 'FAILED'}\n")
                    f.write(f"\nDetails:\n{playback_details}\n")
                
                if not playback_success:
                    with open(f"{self.directory}/PLAYBACK_TEST_FAILED", "w") as f:
                        f.write(f"Playback test failed at {datetime.now().isoformat()}\n")
                    print("DEBUG: Playback test FAILED - tag created")
            else:
                print("DEBUG: Playback test returned None")
                with open(f"{self.directory}/playback_test_result.txt", "w") as f:
                    f.write(f"Test Type: Playback Rewind Test\n")
                    f.write("Test failed to execute (returned None)\n")
        else:
            if test_type == 'determinism':
                print("DEBUG: Playback test skipped for determinism test (second Gazebo has no recording)")
            else:
                print("DEBUG: Playback test is disabled (enable_playback=False)")

        # 5. collect coverage
        try:
            os.remove(f"./terminate")
        except:
            print("DEBUG: exception removing terminate file")

        if process is None:
            # determinism 测试中第二个 Gazebo 启动失败，无需进一步清理
            return 0

        try:
            process_status = psutil.Process(process.pid)
            print("DEBUG: before kill")
            for child in process_status.children(recursive=True):
                print(f"DEBUG: killing child: {child.pid}")
                child.kill()
            process.kill()
            return 0
        except:
            # Coverage collection disabled
            # print("DEBUG: before coverage")
            # self.cov_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
            # self.cov_new.collect()
            # diff = CoverageDiff()
            # diff.compare(self.cov_new, self.cov_old)

            with open(f"{self.directory}/gz.out", "w") as f:
                # f.write(process.stdout.read().decode("utf-8"))
                f.write(out)
            with open(f"{self.directory}/gz.err", "w") as f:
                # f.write(process.stderr.read().decode("utf-8"))
                f.write(err)
            # print(f"Diff new line: {diff.new_line}, new file: {diff.new_file}")

            if self.check_new_crash(f"{self.directory}/gz.err"):
                print(f"DEBUG: new crash detected")
                # 如果同时有崩溃和蜕变测试失败，创建双重标记
                if not test_passed:
                    with open(f"{self.directory}/BOTH_CRASH_AND_METAMORPHIC_FAIL", "w") as f:
                        f.write(f"Both crash and metamorphic test failure at {datetime.now().isoformat()}\n")

            # if self.bandits:
            #     ### for i in range(len(func_names)):
            #     ###     index = self.funcs.index(func_names[i])  
            #     ###     self.bandits[i].reward(index)
            #     if diff.new_line > 0:
            #         rewards = [(1 if idx <= i else 0, self.diversity_rewards[idx], self.crash_rewards[idx]) for idx in range(self.num_seq)]
            #     else:
            #         rewards = [(0, self.diversity_rewards[idx], self.crash_rewards[idx]) for idx in range(self.num_seq)]

            #     print(rewards)
            #     self.bandits.update(actions, rewards=rewards)

            # return diff.new_line
            return 0

    # 复制随机的sdf文件
    def copy_random_sdf(self):
        filenames = glob("./models/*.sdf")
        # print(f"DEBUG: filenames: {filenames}")
        random_filename = random.choice(filenames)
        shutil.copyfile(random_filename, f"{self.directory}/a.sdf")
        print(f"DEBUG: copied {random_filename} to {self.directory}/a.sdf")
        
        # 添加必需的插件：gz-sim-apply-link-wrench-system
        # self.add_required_plugin(f"{self.directory}/a.sdf")
    
    # 在SDF文件中添加必需的插件
    def add_required_plugin(self, sdf_file):
        """
        在SDF文件的<world>标签后添加必需的插件
        """
        try:
            # 读取SDF文件
            tree = etree.parse(sdf_file)
            root = tree.getroot()
            
            # 查找world元素
            world = root.find('.//world')
            if world is None:
                # 如果没有world标签，尝试查找sdf下的world
                sdf = root if root.tag == 'sdf' else root.find('sdf')
                if sdf is not None:
                    world = sdf.find('world')
            
            if world is None:
                print(f"DEBUG: Warning - No <world> tag found in {sdf_file}, cannot add plugin")
                return
            
            # 检查是否已经存在该插件（避免重复添加）
            existing_plugins = world.findall('.//plugin[@filename="gz-sim-apply-link-wrench-system"]')
            if existing_plugins:
                print(f"DEBUG: Plugin gz-sim-apply-link-wrench-system already exists in {sdf_file}")
                return
            
            # 创建插件元素（使用SubElement确保正确的父子关系）
            plugin = etree.SubElement(world, 'plugin')
            plugin.set('filename', 'gz-sim-apply-link-wrench-system')
            plugin.set('name', 'gz::sim::systems::ApplyLinkWrench')
            
            # 将插件移动到第一个位置（如果world已有其他子元素）
            if len(world) > 1:
                # 将新添加的插件移动到第一个位置
                world.insert(0, world[-1])
            
            # 保存修改后的文件
            # 使用tostring然后手动写入，以确保格式正确
            xml_string = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='utf-8').decode('utf-8')
            
            # 使用正则表达式替换自闭合的plugin标签为完整的标签对
            # 处理可能的换行和空格变化
            pattern = r'<plugin\s+filename="gz-sim-apply-link-wrench-system"\s+name="gz::sim::systems::ApplyLinkWrench"\s*/>'
            replacement = '<plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"></plugin>'
            xml_string = re.sub(pattern, replacement, xml_string)
            
            with open(sdf_file, 'w', encoding='utf-8') as f:
                f.write(xml_string)
            
            print(f"DEBUG: Added gz-sim-apply-link-wrench-system plugin to {sdf_file}")
            
        except Exception as e:
            print(f"DEBUG: Error adding plugin to {sdf_file}: {e}")
            import traceback
            traceback.print_exc()

    # 创建sdf文件
    def create_sdf(self, dump=True):
        self.sdf = self.root_gen.generate()
        if dump:
            with open(f"{self.directory}/{self.sdf_name}", "w") as f:
                f.write(f"<!-- seed: {self.seed} -->\n")
                f.write(self.sdf.to_string())

    # 获取gazebo世界的名称
    def get_world(self):
        # gz service -s /gazebo/worlds --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V --timeout 300 --req ''
        service_name = "/gazebo/worlds"
        request = Empty()
        result, response = self.node.request(service_name, request, Empty, StringMsg_V, self.timeout)
        return result, response

    # 启动模拟器（确保模拟器处于运行状态，而不是暂停状态）
    def pause_simulation(self, max_retries=3):
        """
        暂停模拟器
        通过调用 /world/<world_name>/control 服务，设置 pause: true
        会多次尝试以确保成功
        """
        if not self.world_name:
            print("DEBUG: world_name not set, cannot pause simulation")
            return False
        
        service_name = f"/world/{self.world_name}/control"
        request = WorldControl()
        request.pause = True  # 设置为 true 表示暂停模拟器
        
        # 多次尝试，确保模拟器暂停
        for attempt in range(max_retries):
            try:
                result, response = self.node.request(service_name, request, WorldControl, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Simulation paused for world {self.world_name} (attempt {attempt + 1})")
                    # 等待一小段时间让模拟器完全暂停
                    time.sleep(0.2)
                    return True
                else:
                    print(f"DEBUG: Failed to pause simulation for world {self.world_name} (attempt {attempt + 1})")
                    if attempt < max_retries - 1:
                        time.sleep(0.2)  # 等待后重试
            except Exception as e:
                print(f"DEBUG: Exception when trying to pause simulation (attempt {attempt + 1}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.2)
        
        return False

    def play_simulation(self, max_retries=3):
        """
        启动模拟器，确保模拟器处于运行状态
        通过调用 /world/<world_name>/control 服务，设置 pause: false
        会多次尝试以确保成功
        """
        if not self.world_name:
            print("DEBUG: world_name not set, cannot play simulation")
            return False
        
        service_name = f"/world/{self.world_name}/control"
        request = WorldControl()
        request.pause = False  # 设置为 false 表示运行模拟器
        
        # 多次尝试，确保模拟器启动
        for attempt in range(max_retries):
            try:
                result, response = self.node.request(service_name, request, WorldControl, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Simulation started (play) for world {self.world_name} (attempt {attempt + 1})")
                    # 等待一小段时间让模拟器完全启动
                    time.sleep(0.5)
                    return True
                else:
                    print(f"DEBUG: Failed to start simulation for world {self.world_name} (attempt {attempt + 1})")
                    if attempt < max_retries - 1:
                        time.sleep(0.5)  # 等待后重试
            except Exception as e:
                print(f"DEBUG: Exception when trying to play simulation (attempt {attempt + 1}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.5)  # 等待后重试
        
        print(f"DEBUG: Failed to start simulation after {max_retries} attempts")
        return False

    def step_simulation(self, num_steps):
        """
        精确推进仿真指定步数。使用 WorldControl.multi_step 替代原来的
        'resume → time.sleep → pause' 模式，确保仿真时间精确可控，
        消除 wall-clock timing 导致的不确定性。
        
        multi_step 完成后仿真自动暂停。
        
        Args:
            num_steps: 要推进的仿真步数
                       (step_size=0.001 时，1000步=1秒 sim-time)
        
        Returns:
            bool: 是否成功
        """
        if not hasattr(self, 'world_name') or not self.world_name:
            print("DEBUG: world_name not set, cannot step simulation")
            return False
        
        step_size = 0.001  # Gazebo 默认步长
        sim_time = num_steps * step_size
        
        print(f"DEBUG: Stepping simulation {num_steps} steps ({sim_time:.3f}s sim-time)...")
        
        # 发送 multi_step 命令（仿真从暂停状态开始推进，完成后自动暂停）
        if self.use_text:
            cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                       f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                       f"--timeout {self.timeout} "
                       f"--req 'multi_step: {num_steps}'")
            cmd = GzCommand(GzCommandType.SERVICE, [cmd_txt], True)
            cmd.execute(self.experiment_log if hasattr(self, 'experiment_log') else None)
        else:
            service_name = f"/world/{self.world_name}/control"
            request = WorldControl()
            request.multi_step = num_steps
            result, response = self.node.request(
                service_name, request, WorldControl, Boolean, self.timeout)
            if not (result and response.data):
                print(f"DEBUG: Failed to send multi_step command")
                return False
        
        # 记录步进操作到实验日志
        if hasattr(self, 'experiment_log'):
            self.log_sleep(sim_time,
                f"Stepping {num_steps} steps ({sim_time:.3f}s sim-time)")
        
        # 等待步进完成
        # multi_step 完成后自动暂停。等待时间取决于系统性能和 RTF 设置。
        # 保守估计：sim_time * 3（覆盖 RTF < 0.33 的极端情况）+ 基础等待时间
        wait_time = max(sim_time * 3.0, 8.0)
        time.sleep(wait_time)
        
        # 安全措施：发送 pause 指令确保仿真已暂停
        # （如果 multi_step 已完成，这是一个无害的 no-op）
        if self.use_text:
            pause_txt = (f"gz service -s /world/{self.world_name}/control "
                         f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                         f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_txt], True)
            pause_cmd.execute(None)  # 不记录到实验日志（安全措施）
        else:
            service_name = f"/world/{self.world_name}/control"
            request = WorldControl()
            request.pause = True
            self.node.request(service_name, request, WorldControl, Boolean, self.timeout)
        
        time.sleep(0.3)  # 等待暂停状态稳定
        
        print(f"DEBUG: Stepped {num_steps} steps ({sim_time:.3f}s sim-time) - complete")
        return True

    # 导出世界的sdf文件
    def dump_sdf(self, world):
        print("DEBUG: before dump_sdf")
        service_name = f"/world/{world}/generate_world_sdf"
        request = SdfGeneratorConfig()
        service_name = safe_utf8_encode(service_name)
        # StringMsg = safe_utf8_encode(StringMsg)
        request = safe_utf8_encode(request)
        # 目前问题都出在这里
        result, response = self.node.request(service_name, request, SdfGeneratorConfig, safe_utf8_encode(StringMsg), self.timeout)
        print("DEBUG: before")
        return str(response).encode("utf-8").decode("unicode_escape")[7:-3]  # skip data: ""

    # 添加随机模型
    def func_add_random_model(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, False, False)

    # 添加带扰动的随机模型
    def func_add_random_model_xml(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, False, True)
    
    # 添加带有plugin的不带扰动的随机模型
    def fund_add_random_model_with_plugin(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, True, False)
    
    # 添加带有plugin的带扰动的随机模型
    def fund_add_random_model_with_plugin_xml(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, True, True)

    # 添加模型
    def helper_func_add_random_model(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content="", from_mined=False, xml_random = False):
        # def create_model(world, name, x, y, z, sdf_content=None):

        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        if len(scene.model) > MAX_MODEL_NUM:
            return None
        service_name = f"/world/{self.world_name}/create"
        request = EntityFactory()

        if model_id == -1:
            model_gen = ModelGen(self.sdf_miner)
            if not from_mined:
                root = model_gen.generate_with_root_wrapper(name, from_mined)
                # TODO: check for exception
                sdf_content = root.to_string().encode("utf-8")
            else:
                sdf_content = self.plugin_miner.random_model_with_root()
        else:
            sdf_content = retrieve_model_by_index(model_id)
        
        # 是否扰动
        # if xml_random is True:
        #     new_sdf = perturb_xml(str(sdf_content))
        #     if new_sdf is not None:
        #         request.sdf = new_sdf
        #         # print("!!!DEBUG: add model request change success")
        #     else:
        #         request.sdf = sdf_content
        # else:
        #     request.sdf = sdf_content

        request.sdf = sdf_content
        request.pose.position.x = random.random() * (pose_max - pose_min) + pose_min
        request.pose.position.y = random.random() * (pose_max - pose_min) + pose_min
        request.pose.position.z = random.random() * pose_max
        request.name = name
        request.allow_renaming = True
        req_txt = str(request).replace(r"\'", r'\"')
        # print("!!!DEBUG: old add model sdf\n%s",str(request.sdf))
        # print("!!!DEBUG: add model request type:")
        # print(type(request))
        
        # print("!!!DEBUG: new sdf is \n%s", str(request.sdf))
        if self.use_text:
            cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req '{req_txt}'"
            return GzCommand(GzCommandType.SERVICE, cmd_txt, True)
        else:
            gz_service = ServiceParam(service_name, request, EntityFactory, Boolean, self.timeout)
            return GzCommand(GzCommandType.SERVICE, gz_service, False)
        # result, response = self.node.request(service_name, request, EntityFactory, Boolean, self.timeout)
        # return cmd_txt, result, response

# 
    def get_scene(self):
        # gz service -s /world/gravity/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 300 --req ''
        try:
            result, response = self.get_world()
            world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return None, None
        self.world_name = world_name
        node = Node()
        service_name = f"/world/{self.world_name}/scene/info"
        reserved_models = {"ground_model", "ground_plane", "ground plane", "ceiling_model", "west_model", "east_model", "north_model", "south_model"}
        request = Empty()
        # request = safe_utf8_encode(request)
        result, response = node.request(service_name, request, Empty, Scene, self.timeout)
        return response, reserved_models

    def is_model_testable(self, model):
        """
        判断模型是否适合用于蜕变测试。
        排除静态模型、地面/墙壁/天花板等环境模型。
        
        Args:
            model: scene 中的 model 对象
        
        Returns:
            True 如果模型可用于测试，False 否则
        """
        # 排除名字匹配保留关键字的模型（不区分大小写）
        reserved_keywords = ["ground", "ceiling", "wall", "floor", "sun", "light", "camera"]
        name_lower = model.name.lower().replace(" ", "_")
        for keyword in reserved_keywords:
            if keyword in name_lower:
                return False
        return True

    def get_testable_models(self, scene, reserved_models):
        """
        从场景中获取可用于蜕变测试的模型列表。
        
        Args:
            scene: 场景对象
            reserved_models: 保留模型名称集合
        
        Returns:
            可测试模型列表
        """
        if scene is None:
            return []
        return [m for m in scene.model if m.name not in reserved_models and self.is_model_testable(m)]

    def get_model_pose_from_scene(self, model_name):
        """
        通过 scene/info 服务获取模型的位置信息。
        此函数在仿真暂停状态下也可正常工作（使用 service 而非 topic）。
        
        Args:
            model_name: 模型名称
        
        Returns:
            (x, y, z, qw, qx, qy, qz) 元组，如果失败返回 None
        """
        try:
            scene, reserved_models = self.get_scene()
            if scene is None:
                print(f"DEBUG: get_model_pose_from_scene: get_scene() returned None")
                return None
            
            for model in scene.model:
                if model.name == model_name:
                    pose = model.pose
                    return (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    )
            
            print(f"DEBUG: get_model_pose_from_scene: model '{model_name}' not found in scene")
            return None
            
        except Exception as e:
            print(f"DEBUG: Exception in get_model_pose_from_scene: {e}")
            import traceback
            traceback.print_exc()
            return None

    def record_all_models_state_from_scene(self):
        """
        通过 scene/info 服务记录所有模型的当前状态（位置和角度）。
        此函数在仿真暂停状态下也可正常工作（使用 service 而非 topic）。
        
        Returns:
            dict: {model_name: {'position': (x, y, z), 'orientation': (w, x, y, z)}}, 如果失败返回 None
        """
        try:
            scene, reserved_models = self.get_scene()
            if scene is None:
                print(f"DEBUG: record_all_models_state_from_scene: get_scene() returned None")
                return None
            
            models_state = {}
            for model in scene.model:
                pose = model.pose
                models_state[model.name] = {
                    'position': (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z
                    ),
                    'orientation': (
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    )
                }
            
            return models_state
            
        except Exception as e:
            print(f"DEBUG: Exception in record_all_models_state_from_scene: {e}")
            import traceback
            traceback.print_exc()
            return None

    def get_model_pose_from_topic(self, model_name):
        """
        通过 topic 获取模型的实时位置信息。
        注意：此函数必须在模拟运行（非暂停）状态下调用，因为暂停时服务器循环频率
        大幅降低，topic 可能无法在超时时间内发布消息。
        
        Args:
            model_name: 模型名称
        
        Returns:
            (x, y, z, w, qx, qy, qz) 位置和四元数元组，如果获取失败返回 None
        """
        try:
            # 确保 world_name 已设置
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    print("DEBUG: Cannot get world name")
                    return None
                self.world_name = response.data[0]
            
            # dynamic_pose/info 发布非静态模型的实时位姿（从 ECM 读取）
            # pose/info 发布所有模型的实时位姿（从 ECM 读取）
            # 两者都提供实时位姿，但都依赖服务器循环发布，暂停时可能不及时
            topic_name = f"/world/{self.world_name}/dynamic_pose/info"
            
            # 使用 Node API 方式获取 topic 消息
            node = Node()
            received_msg = None
            received = False
            
            def pose_callback(msg):
                nonlocal received_msg, received
                received_msg = msg
                received = True
            
            subscriber = node.subscribe(Pose_V, topic_name, pose_callback)
            if not subscriber:
                print(f"DEBUG: Failed to subscribe to topic {topic_name}")
                # 尝试使用 pose/info topic（包含所有模型，包括静态模型）
                topic_name = f"/world/{self.world_name}/pose/info"
                subscriber = node.subscribe(Pose_V, topic_name, pose_callback)
                if not subscriber:
                    print(f"DEBUG: Failed to subscribe to topic {topic_name}")
                    return None
            
            # 等待消息，最多等待 3 秒
            import time
            start_time = time.time()
            while not received and (time.time() - start_time) < 3.0:
                time.sleep(0.01)
            
            if not received or received_msg is None:
                print("DEBUG: No pose message received (simulation may be paused - this function requires running simulation)")
                return None
            
            # 在 Pose_V 消息中查找指定模型
            for pose in received_msg.pose:
                if pose.name == model_name:
                    return (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    )
            
            print(f"DEBUG: Model {model_name} not found in pose message")
            return None
            
        except Exception as e:
            print(f"DEBUG: Exception in get_model_pose_from_topic: {e}")
            import traceback
            traceback.print_exc()
            return None

    def get_simulation_time(self):
        """
        获取当前的模拟时间
        
        Returns:
            (sec, nsec) 元组，表示模拟时间的秒和纳秒部分，如果获取失败返回 None
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return None
                self.world_name = response.data[0]
            
            # 从 /stats topic 获取模拟时间
            topic_name = "/stats"  # 全局 stats topic
            node = Node()
            received_msg = None
            received = False
            
            def stats_callback(msg):
                nonlocal received_msg, received
                received_msg = msg
                received = True
            
            subscriber = node.subscribe(WorldStatistics, topic_name, stats_callback)
            if not subscriber:
                print("DEBUG: Failed to subscribe to /stats topic")
                return None
            
            # 等待消息，最多等待 1 秒
            import time
            start_time = time.time()
            while not received and (time.time() - start_time) < 1.0:
                time.sleep(0.01)
            
            if not received or received_msg is None:
                print("DEBUG: No stats message received")
                return None
            
            if received_msg.HasField('sim_time'):
                return (received_msg.sim_time.sec, received_msg.sim_time.nsec)
            else:
                print("DEBUG: Stats message does not contain sim_time")
                return None
                
        except Exception as e:
            print(f"DEBUG: Exception in get_simulation_time: {e}")
            import traceback
            traceback.print_exc()
            return None

    def record_all_models_state(self):
        """
        记录所有模型的当前状态（位置和角度）
        
        Returns:
            dict: {model_name: {'position': (x, y, z), 'orientation': (w, x, y, z)}}, 如果失败返回 None
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return None
                self.world_name = response.data[0]
            
            # 使用 pose/info topic 获取所有模型的状态
            topic_name = f"/world/{self.world_name}/pose/info"
            node = Node()
            received_msg = None
            received = False
            
            def pose_callback(msg):
                nonlocal received_msg, received
                received_msg = msg
                received = True
            
            subscriber = node.subscribe(Pose_V, topic_name, pose_callback)
            if not subscriber:
                print(f"DEBUG: Failed to subscribe to topic {topic_name}")
                return None
            
            # 等待消息，最多等待 2 秒
            import time
            start_time = time.time()
            while not received and (time.time() - start_time) < 2.0:
                time.sleep(0.01)
            
            if not received or received_msg is None:
                print("DEBUG: No pose message received")
                return None
            
            # 记录所有模型的状态
            models_state = {}
            for pose in received_msg.pose:
                model_name = pose.name
                models_state[model_name] = {
                    'position': (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z
                    ),
                    'orientation': (
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    )
                }
            
            return models_state
            
        except Exception as e:
            print(f"DEBUG: Exception in record_all_models_state: {e}")
            import traceback
            traceback.print_exc()
            return None

    def save_simulation_state(self):
        """
        保存完整的模拟状态（包括所有模型的位置、速度等）
        
        Returns:
            SerializedState 对象，如果失败返回 None
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return None
                self.world_name = response.data[0]
            
            # 使用 /world/<world_name>/state 服务获取完整状态
            service_name = f"/world/{self.world_name}/state"
            request = Empty()  # state 服务通常不需要请求参数，或使用 Empty
            
            result, response = self.node.request(service_name, request, Empty, SerializedStepMap, self.timeout)
            if result and response.HasField('state'):
                print(f"DEBUG: Successfully saved simulation state")
                return response.state
            else:
                print(f"DEBUG: Failed to get simulation state")
                return None
                
        except Exception as e:
            print(f"DEBUG: Exception in save_simulation_state: {e}")
            import traceback
            traceback.print_exc()
            return None

    def restore_simulation_state(self, saved_state):
        """
        恢复完整的模拟状态（包括所有模型的位置、速度等）
        
        Args:
            saved_state: SerializedState 对象
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 WorldControlState 消息来恢复状态
            # 服务可以是 /world/<world_name>/control/state 或 /world/<world_name>/control
            service_name = f"/world/{self.world_name}/control/state"
            
            request = WorldControlState()
            # 设置状态
            request.state.CopyFrom(saved_state)
            # 不设置 world_control，只恢复状态
            
            result, response = self.node.request(service_name, request, WorldControlState, Boolean, self.timeout)
            if result and response.data:
                print(f"DEBUG: Successfully restored simulation state")
                import time
                time.sleep(0.5)  # 等待状态恢复完成
                return True
            else:
                # 如果 /control/state 不可用，尝试使用 /control
                print(f"DEBUG: /control/state not available, trying /control")
                service_name = f"/world/{self.world_name}/control"
                result, response = self.node.request(service_name, request, WorldControlState, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Successfully restored simulation state via /control")
                    import time
                    time.sleep(0.5)
                    return True
                else:
                    print(f"DEBUG: Failed to restore simulation state")
                    return False
                
        except Exception as e:
            print(f"DEBUG: Exception in restore_simulation_state: {e}")
            import traceback
            traceback.print_exc()
            return False

    def restore_models_state(self, models_state):
        """
        恢复所有模型的状态（位置和角度）
        
        Args:
            models_state: 模型状态字典 {model_name: {'position': (x, y, z), 'orientation': (w, x, y, z)}}
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 set_pose 服务逐个恢复模型位置
            service_name = f"/world/{self.world_name}/set_pose"
            success_count = 0
            
            for model_name, state in models_state.items():
                try:
                    request = Pose()
                    request.name = model_name
                    request.position.x = state['position'][0]
                    request.position.y = state['position'][1]
                    request.position.z = state['position'][2]
                    request.orientation.w = state['orientation'][0]
                    request.orientation.x = state['orientation'][1]
                    request.orientation.y = state['orientation'][2]
                    request.orientation.z = state['orientation'][3]
                    
                    result, response = self.node.request(service_name, request, Pose, Boolean, self.timeout)
                    if result and response.data:
                        success_count += 1
                    else:
                        print(f"DEBUG: Failed to restore pose for model {model_name}")
                except Exception as e:
                    print(f"DEBUG: Exception restoring pose for model {model_name}: {e}")
            
            print(f"DEBUG: Restored pose for {success_count}/{len(models_state)} models")
            return success_count > 0
            
        except Exception as e:
            print(f"DEBUG: Exception in restore_models_state: {e}")
            import traceback
            traceback.print_exc()
            return False

    def seek_to_simulation_time(self, target_sec, target_nsec=0):
        """
        回溯到指定的模拟时间（仅改变时间，不恢复状态）
        
        注意：这个方法只改变模拟时间，不会恢复模型状态。
        如果需要恢复模型状态，应该使用 restore_models_state() 方法。
        
        Args:
            target_sec: 目标时间的秒部分
            target_nsec: 目标时间的纳秒部分（默认0）
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 注意：seek 功能主要用于 log playback，对于实时模拟可能不适用
            # 而且即使成功，也只改变时间，不会恢复模型状态
            # 这里保留这个方法，但实际使用时应该配合 restore_models_state 使用
            
            # 尝试使用 reset 功能重置到初始状态，然后恢复状态
            # 或者直接使用 restore_models_state 恢复状态（不改变时间）
            
            # 由于 seek 功能在实时模拟中可能不可用，这里先尝试 reset
            service_name = f"/world/{self.world_name}/control"
            request = WorldControl()
            # 重置时间到初始状态
            request.reset.all = True
            
            result, response = self.node.request(service_name, request, WorldControl, Boolean, self.timeout)
            if result and response.data:
                print(f"DEBUG: Reset simulation to initial state")
                import time
                time.sleep(0.5)
                return True
            else:
                print(f"DEBUG: Failed to reset simulation")
                return False
                
        except Exception as e:
            print(f"DEBUG: Exception in seek_to_simulation_time: {e}")
            import traceback
            traceback.print_exc()
            return False

    def compare_models_state(self, state1, state2, position_tolerance=0.01, orientation_tolerance=0.01):
        """
        对比两个模型状态字典
        
        Args:
            state1: 第一个状态字典
            state2: 第二个状态字典
            position_tolerance: 位置容差（默认0.01m，即1cm）
            orientation_tolerance: 角度容差（默认0.01，四元数分量差）
        
        Returns:
            (bool, list): (是否相同, 差异列表)
        """
        if state1 is None or state2 is None:
            return (False, ["One or both states are None"])
        
        differences = []
        
        # 检查所有模型
        all_models = set(state1.keys()) | set(state2.keys())
        
        for model_name in all_models:
            if model_name not in state1:
                differences.append(f"Model {model_name} missing in state1")
                continue
            if model_name not in state2:
                differences.append(f"Model {model_name} missing in state2")
                continue
            
            pos1 = state1[model_name]['position']
            pos2 = state2[model_name]['position']
            ori1 = state1[model_name]['orientation']
            ori2 = state2[model_name]['orientation']
            
            # 检查位置
            pos_diff = (
                abs(pos1[0] - pos2[0]),
                abs(pos1[1] - pos2[1]),
                abs(pos1[2] - pos2[2])
            )
            if any(d > position_tolerance for d in pos_diff):
                differences.append(
                    f"Model {model_name} position differs: "
                    f"state1={pos1}, state2={pos2}, diff={pos_diff}"
                )
            
            # 检查角度
            ori_diff = (
                abs(ori1[0] - ori2[0]),
                abs(ori1[1] - ori2[1]),
                abs(ori1[2] - ori2[2]),
                abs(ori1[3] - ori2[3])
            )
            if any(d > orientation_tolerance for d in ori_diff):
                differences.append(
                    f"Model {model_name} orientation differs: "
                    f"state1={ori1}, state2={ori2}, diff={ori_diff}"
                )
        
        return (len(differences) == 0, differences)

    # 第二种蜕变测试：利用回溯功能
    def metamorphic_test_rewind(self, record_time_a=2.0, run_time_b=3.0):
        """
        蜕变测试：利用 Gazebo 的回溯功能
        
        流程：
        1. 在 a 秒时记录所有模型的动态信息（坐标和角度）
        2. 继续运行 b 秒
        3. 回溯 b 秒（回到 a 秒时的状态）
        4. 将回溯后的所有模型的动态信息和之前记录的数据对比
        5. 如果相同就正常，如果不对就检测出错误
        
        Args:
            record_time_a: 记录状态的时间点（秒），从模拟开始计算
            run_time_b: 继续运行的时间（秒）
        
        Returns:
            (success, record_time, rewind_time, state_before, state_after, differences) 或 None
        """
        try:
            import time as time_module
            
            # 获取初始模拟时间
            initial_time = self.get_simulation_time()
            if initial_time is None:
                print("DEBUG: Failed to get initial simulation time")
                return None
            
            initial_sim_time_sec = initial_time[0] + initial_time[1] / 1e9
            print(f"DEBUG: Initial simulation time: {initial_sim_time_sec:.6f} s")
            
            # 1. 等待到 record_time_a 秒（基于模拟时间）
            target_sim_time = initial_sim_time_sec + record_time_a
            print(f"DEBUG: Waiting for simulation time to reach {target_sim_time:.6f} s (record_time_a={record_time_a} s)...")
            
            # 等待模拟时间达到目标时间（使用真实时间等待，但检查模拟时间）
            max_wait_time = record_time_a * 2  # 最多等待真实时间的2倍
            start_wait = time_module.time()
            while True:
                current_time = self.get_simulation_time()
                if current_time is None:
                    print("DEBUG: Failed to get simulation time during wait")
                    return None
                
                current_sim_time_sec = current_time[0] + current_time[1] / 1e9
                if current_sim_time_sec >= target_sim_time:
                    print(f"DEBUG: Reached target simulation time: {current_sim_time_sec:.6f} s")
                    break
                
                if time_module.time() - start_wait > max_wait_time:
                    print(f"DEBUG: Timeout waiting for simulation time to reach {target_sim_time:.6f} s")
                    return None
                
                self.log_sleep(0.1, "Wait for simulation time to reach target")
                self.log_sleep(0.1, "Wait for simulation time during record wait")
                time_module.sleep(0.1)
            
            # 获取当前模拟时间（用于验证）
            current_time = self.get_simulation_time()
            if current_time is None:
                print("DEBUG: Failed to get current simulation time")
                return None
            
            print(f"DEBUG: Current simulation time: {current_time[0]}.{current_time[1]:09d} s")
            
            # 2. 记录模型位置用于对比（从 pose topic 获取）
            print("DEBUG: Recording models pose for comparison...")
            state_before = self.record_all_models_state()
            if state_before is None:
                print("DEBUG: Failed to record models pose")
                return None
            
            print(f"DEBUG: Saved simulation state and recorded pose for {len(state_before)} models")
            for model_name in list(state_before.keys())[:3]:  # 只打印前3个模型
                print(f"  {model_name}: pos={state_before[model_name]['position']}")
            
            # 3. 继续运行 run_time_b 秒（基于模拟时间）
            target_after_run = current_sim_time_sec + run_time_b
            print(f"DEBUG: Running simulation for {run_time_b} seconds (to {target_after_run:.6f} s)...")
            
            # 等待模拟时间达到运行后的目标时间
            max_wait_time = run_time_b * 2
            start_wait = time_module.time()
            while True:
                current_time = self.get_simulation_time()
                if current_time is None:
                    print("DEBUG: Failed to get simulation time during run")
                    return None
                
                current_sim_time_sec = current_time[0] + current_time[1] / 1e9
                if current_sim_time_sec >= target_after_run:
                    print(f"DEBUG: Reached target simulation time after run: {current_sim_time_sec:.6f} s")
                    break
                
                if time_module.time() - start_wait > max_wait_time:
                    print(f"DEBUG: Timeout waiting for simulation time to reach {target_after_run:.6f} s")
                    return None
                
                self.log_sleep(0.1, "Wait for simulation time during run wait")
                time_module.sleep(0.1)
            
            # 获取运行后的模拟时间
            time_after_run = self.get_simulation_time()
            if time_after_run is None:
                print("DEBUG: Failed to get simulation time after running")
                return None
            
            print(f"DEBUG: Simulation time after running: {time_after_run[0]}.{time_after_run[1]:09d} s")
            
            # 4. 回溯到 record_time_a 秒时的状态
            # 先暂停模拟，防止恢复过程中物理引擎导致漂移
            print(f"DEBUG: Pausing simulation before restoring state...")
            self.pause_simulation()
            self.log_sleep(0.5, "Wait after pause before restore")
            time_module.sleep(0.5)
            
            # 使用 restore_models_state 逐个恢复模型的位置和角度
            print(f"DEBUG: Restoring all models state using set_pose...")
            restore_success = self.restore_models_state(state_before)
            if not restore_success:
                print("DEBUG: Failed to restore models state")
                self.play_simulation()
                return None
            
            # 等待一小段时间让 set_pose 生效
            self.log_sleep(0.5, "Wait for set_pose to take effect")
            time_module.sleep(0.5)
            
            # 5. 恢复模拟运行，然后立即获取回溯后的状态
            # 注：必须在运行状态下读取 topic，因为暂停时服务器循环频率降低，topic 可能无法及时发布
            print("DEBUG: Resuming simulation briefly to read pose from topic...")
            self.play_simulation()
            self.log_sleep(0.1, "Brief wait after resume for topic to publish")
            time_module.sleep(0.1)
            
            print("DEBUG: Getting models state after rewind (simulation running)...")
            state_after = self.record_all_models_state()
            if state_after is None:
                print("DEBUG: Failed to get models state after rewind")
                return None
            
            print(f"DEBUG: Got state for {len(state_after)} models after rewind")
            
            # 6. 对比状态
            print("DEBUG: Comparing states...")
            is_same, differences = self.compare_models_state(state_before, state_after)
            
            if is_same:
                print("DEBUG: States match! Rewind test PASSED")
            else:
                print(f"DEBUG: States differ! Rewind test FAILED")
                print(f"DEBUG: Found {len(differences)} differences:")
                for diff in differences[:5]:  # 只打印前5个差异
                    print(f"  {diff}")
            
            return (is_same, record_time_a, run_time_b, state_before, state_after, differences)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_rewind: {e}")
            import traceback
            traceback.print_exc()
            return None

    def metamorphic_test_playback_rewind(self, log_path, original_test_type=None, original_test_result=None):
        """
        在 playback 模式下进行回溯测试
        
        流程：
        1. 使用录像文件启动 playback 模式的 gazebo
        2. 在 playback 模式下，使用 seek 功能回溯到指定时间点
        3. 记录回溯后的模型状态
        4. 与原始测试中记录的状态进行对比
        
        Args:
            log_path: 录像文件路径
            original_test_type: 原始测试类型（'motion' 或 'rewind'）
            original_test_result: 原始测试结果
        
        Returns:
            (success, details_string) 或 None
        """
        try:
            import time as time_module
            
            # 检查录像文件是否存在
            if not os.path.exists(log_path):
                print(f"DEBUG: Log path does not exist: {log_path}")
                return (False, f"Log path does not exist: {log_path}")
            
            # 1. 启动 playback 模式的 gazebo
            print(f"DEBUG: Starting playback mode with log: {log_path}")
            gz_playback = f"gz sim -r --playback {log_path}"
            print(f"DEBUG: gz_playback: {gz_playback}")
            
            try:
                playback_process = subprocess.Popen(
                    gz_playback.split(" "), 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    start_new_session=True
                )
            except Exception as e:
                print(f"DEBUG: Failed to start playback process: {e}")
                return (False, f"Failed to start playback: {e}")
            
            # 等待 playback 模式启动
            time_module.sleep(5)
            
            # 获取 world 名称
            try:
                result, response = self.get_world()
                if not result:
                    print("DEBUG: Failed to get world name in playback mode")
                    playback_process.terminate()
                    playback_process.wait()
                    return (False, "Failed to get world name")
                playback_world_name = response.data[0]
                self.world_name = playback_world_name
            except Exception as e:
                print(f"DEBUG: Exception getting world name: {e}")
                playback_process.terminate()
                playback_process.wait()
                return (False, f"Exception getting world name: {e}")
            
            print(f"DEBUG: Playback world name: {playback_world_name}")
            time_module.sleep(2)  # 等待 playback 完全启动
            
            # 2. 根据原始测试类型选择回溯时间点
            if original_test_type == 'motion' and original_test_result:
                # 对于运动测试，回溯到测试开始时间（或中间某个时间点）
                # 简化处理：回溯到测试开始后 1 秒
                seek_time_sec = 1.0
                seek_time_nsec = 0
            elif original_test_type == 'rewind' and original_test_result:
                # 对于回溯测试，使用原始测试的记录时间点
                success, record_time_a, run_time_b, state_before, state_after, differences = original_test_result
                seek_time_sec = record_time_a
                seek_time_nsec = 0
            else:
                # 默认回溯到 2 秒
                seek_time_sec = 2.0
                seek_time_nsec = 0
            
            print(f"DEBUG: Seeking to time: {seek_time_sec}.{seek_time_nsec:09d} s")
            
            # 3. 使用 seek 功能回溯到指定时间
            seek_success = self.seek_in_playback_mode(playback_world_name, seek_time_sec, seek_time_nsec)
            if not seek_success:
                print("DEBUG: Failed to seek in playback mode")
                playback_process.terminate()
                playback_process.wait()
                return (False, "Failed to seek in playback mode")
            
            # 等待 seek 完成
            self.log_sleep(1.0, "Wait after seek in playback mode")
            time_module.sleep(1.0)
            
            # 4. 记录回溯后的模型状态
            print("DEBUG: Recording models state after seek...")
            state_after_seek = self.record_all_models_state()
            if state_after_seek is None:
                print("DEBUG: Failed to record models state after seek")
                playback_process.terminate()
                playback_process.wait()
                return (False, "Failed to record models state after seek")
            
            print(f"DEBUG: Recorded state for {len(state_after_seek)} models after seek")
            
            # 5. 如果有原始测试的状态记录，进行对比
            if original_test_type == 'rewind' and original_test_result:
                success, record_time_a, run_time_b, state_before, state_after, differences = original_test_result
                # 与原始测试中记录的状态对比
                if state_before:
                    print("DEBUG: Comparing with original test state...")
                    is_same, new_differences = self.compare_models_state(state_before, state_after_seek)
                    
                    details = f"Seek time: {seek_time_sec} s\n"
                    details += f"Models recorded: {len(state_after_seek)}\n"
                    if is_same:
                        details += "States match with original test!\n"
                    else:
                        details += f"Found {len(new_differences)} differences:\n"
                        for diff in new_differences[:10]:  # 最多显示10个差异
                            details += f"  - {diff}\n"
                    
                    # 停止 playback 进程
                    playback_process.terminate()
                    playback_process.wait()
                    
                    return (is_same, details)
            
            # 如果没有原始状态对比，只检查状态是否成功获取
            details = f"Seek time: {seek_time_sec} s\n"
            details += f"Models recorded: {len(state_after_seek)}\n"
            details += "No original state to compare with.\n"
            
            # 停止 playback 进程
            playback_process.terminate()
            playback_process.wait()
            
            return (True, details)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_playback_rewind: {e}")
            import traceback
            traceback.print_exc()
            # 确保进程被终止
            try:
                if 'playback_process' in locals():
                    playback_process.terminate()
                    playback_process.wait()
            except:
                pass
            return (False, f"Exception: {e}")

    def seek_in_playback_mode(self, world_name, target_sec, target_nsec=0):
        """
        在 playback 模式下使用 seek 功能回溯到指定时间
        
        Args:
            world_name: 世界名称
            target_sec: 目标时间的秒部分
            target_nsec: 目标时间的纳秒部分（默认0）
        
        Returns:
            bool: 是否成功
        """
        try:
            # 使用 /world/<world_name>/playback/control 服务进行 seek
            service_name = f"/world/{world_name}/playback/control"
            
            # 创建 LogPlaybackControl 消息
            request = LogPlaybackControl()
            request.seek.sec = int(target_sec)
            request.seek.nsec = int(target_nsec)
            
            result, response = self.node.request(service_name, request, LogPlaybackControl, Boolean, self.timeout)
            if result and response.data:
                print(f"DEBUG: Successfully seeked to {target_sec}.{target_nsec:09d} s")
                return True
            else:
                print(f"DEBUG: Failed to seek in playback mode")
                return False
                
        except Exception as e:
            print(f"DEBUG: Exception in seek_in_playback_mode: {e}")
            import traceback
            traceback.print_exc()
            return False

    def func_add_random_plugin_to_model(self, plugin_id):
        return self.helper_func_add_random_plugin_to_model(False, plugin_id)
    
    def func_add_random_plugin_to_model_xml(self, plugin_id):
        return self.helper_func_add_random_plugin_to_model(True, plugin_id)

    # 随机给模型添加组件
    def helper_func_add_random_plugin_to_model(self, random_xml = False, plugin_id = -1):
        if plugin_id == -1:
            plugin_id = random(0, action_param_counts[2])
        # print("DEBUG: begin func_add_random_plugin_to_model")
        # 0. get world name
        try:
            result, response = self.get_world()
            world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return None
        scene, reserved_models = self.get_scene()
        # print(f"scene.model: {scene.model}")
        # 1. get random model
        if not reserved_models:
            return None
        available_models = self.get_testable_models(scene, reserved_models)
        # print(f"available_models: {available_models}, world_name: {self.world_name}")
        print(f"world_name: {self.world_name}")
        if available_models:
            model = random.choice(available_models)
            model_id = model.id
        else:
            return None
        # 2. get random plugin
        # plugin = random.choice(self.plugin_miner.plugins_within_model)

        # get plugin though id
        if plugin_id == -1:
            plugin = random.choice(self.plugin_miner.plugins_within_model)
            filename = plugin.get("filename")
            name = plugin.get("name")
            innerxml = "\n".join([tostring(c).decode("utf-8") for c in plugin.getchildren()])
        else:
            plugin = retrieve_plugin_by_index(plugin_id)
            filename, name, innerxml = parse_plugin(plugin)
        
        

        entity_plugin_pb = EntityPlugin_V()
        plugin_pb = Plugin()
        plugin_pb.filename = filename
        # print("!!!DEBUG: plugin_pb filename is %s" %(plugin_pb.filename))
        plugin_pb.name = name
        if random_xml is True:
            new_innerxml = perturb_xml(str(innerxml))
            if(new_innerxml is not None) :
                print("DEBUG: change plugin success")
                plugin_pb.innerxml = new_innerxml
            else:
                plugin_pb.innerxml = innerxml
        else:
            plugin_pb.innerxml = innerxml
        entity_plugin_pb.entity.id = model_id
        entity_plugin_pb.plugins.append(plugin_pb)

        # 3. generate gz command
        service_name = f"/world/{world_name}/entity/system/add"
        gz_command = f"gz service --timeout {self.timeout} -s {service_name} --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req '{str(entity_plugin_pb)}'"
        print("DEBUG: gz_command :" + gz_command)
        # 4. return gz command

        if self.use_text:
            return GzCommand(GzCommandType.SERVICE, gz_command, True)
        else:
            gz_service = ServiceParam(service_name, entity_plugin_pb, EntityPlugin_V, Boolean, self.timeout)
            return GzCommand(GzCommandType.SERVICE, gz_service, False)

    # 移除随机模型
    def func_remove_random_model(self):
        # remove
        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        # print(f"scene.model: {scene.model}")
        available_models = self.get_testable_models(scene, reserved_models)
        # print(f"available_models: {available_models}, world_name: {self.world_name}")
        print(f"world_name: {self.world_name}")
        if available_models:
            model_to_remove = random.choice(available_models)
            service_name = f"/world/{self.world_name}/remove"
            request = Entity()
            request.name = model_to_remove.name
            request.id = model_to_remove.id
            cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype gz.msgs.Boolean --reqtype gz.msgs.Entity --req '{request}'"
            if self.use_text:
                return GzCommand(GzCommandType.SERVICE, cmd_txt, True)
            else:
                gz_service = ServiceParam(service_name, request, Entity, Boolean, self.timeout)
                return GzCommand(GzCommandType.SERVICE, gz_service, False)
            # result, response = self.node.request(service_name, request, Entity, Boolean, self.timeout)
            # return cmd_txt, result, response
        else:
            return None
            # return "", None, None

    # 随机生成service请求
    def func_random_service(self, service_name=""):
        print("DEBUG: got here", service_name)
        node = Node()
        print("DEBUG: got here2")
        msg_type_convert = MessageTypeConvert()
        if not service_name:
            service_list = node.service_list()
            if not service_list:
                return None
            service_name = random.choice(service_list)
        print("DEBUG: got here3")
        if self.skipped and service_name in self.skipped:
            print(self.skipped, service_name)
            print("DEBUG: should not be here")
            return None
        # print(f"\tservice name: {service_name}")
        service_list = node.service_list()
        info_list = node.service_info(service_name)
        print("DEBUG: got here x", service_list, info_list)
        if not info_list:
            return None
        print("DEBUG: got here4")
        info = random.choice(info_list)
        rep_type = msg_type_convert.get_class_type(info.rep_type_name)
        req_type = msg_type_convert.get_class_type(info.req_type_name)
        print("DEBUG: got here5")
        if req_type:
            try:
                print("DEBUG: got here6")
                random_req = randomproto.randproto(req_type)
                print("DEBUG: got here7")
                req_text = str(random_req).strip()
                print("DEBUG: got here8")
                cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype {info.rep_type_name} --reqtype {info.req_type_name} --req '{req_text}'"
                gz_service = ServiceParam(service_name, random_req, req_type, rep_type, self.timeout)
                print("DEBUG: got here9")

                if self.use_text:
                    return GzCommand(GzCommandType.SERVICE, cmd_txt, True)
                else:
                    return GzCommand(GzCommandType.SERVICE, gz_service, False)

                # TODO: remove explicit call, just return something like gz_service
                # result, response = self.node.request(service_name, random_req, req_type, rep_type, self.timeout)

                # return cmd_txt, result, response
            except:
                return None
                # return "", None, None
        else:
            return None
            # return "", None, None

    # 随机生成topic请求
    def func_random_topic(self, topic_name=""):
        msg_type_convert = MessageTypeConvert()
        if not topic_name:
            topic_list = self.node.topic_list()
            if not topic_list:
                return None
            topic_name = random.choice(topic_list)
        if "/scene/info" in topic_name:
            # Scene string triggers argument list too long error
            return None
        info_list = self.node.topic_info(topic_name)
        if not info_list:
            return None
        info = random.choice(info_list)
        cmd_txts = list()
        gz_topics = list()

        for publisher in info:
            type_name = publisher.msg_type_name
            type_class = msg_type_convert.get_class_type(type_name)
            print(f"DEBUG: type_name: {type_name} type_class: {type_class}")
            pub = self.node.advertise(topic_name, type_class)
            try:
                message = randomproto.randproto(type_class)
                # def __init__(self, topic_name, type_name, type_class, publisher, message, timeout=10000):
                gz_topic = TopicParam(topic_name, type_name, type_class, pub, message)
                gz_topics.append(gz_topic)

                # TODO: remove explicit call, just return something like gz_topics at the end
                # print(message)
                cmd_txt = f"gz topic -t {topic_name} -m {type_name} -p '{message}'"
                pub.publish(message)
                cmd_txts.append(cmd_txt)

            except:
                pass

        if self.use_text:
            return GzCommand(GzCommandType.TOPIC, cmd_txts, True)
        else:
            # gz_topics = safe_utf8_encode(gz_topics)
            return GzCommand(GzCommandType.TOPIC, gz_topics, False)
        # return cmd_txts, None, None

    # 随机设置模型的位置
    def func_random_pose(self, pose_min=-POSE, pose_max=POSE):
        service_name = f"/world/{self.world_name}/set_pose"
        request = Pose()
        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        available_models = self.get_testable_models(scene, reserved_models)
        # print(f"available_models: {available_models}, scene.model: {scene.model}")
        if available_models:
            model_to_move = random.choice(available_models)
            request.name = model_to_move.name
            request.position.x = random.random() * (pose_max - pose_min) + pose_min
            request.position.y = random.random() * (pose_max - pose_min) + pose_min
            request.position.z = random.random() * pose_max
            cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype gz.msgs.Boolean --reqtype gz.msgs.Pose --req '{request}'"
            gz_service = ServiceParam(service_name, request, Pose, Boolean, self.timeout)

            if self.use_text:
                return GzCommand(GzCommandType.SERVICE, cmd_txt, True)
            else:
                return GzCommand(GzCommandType.SERVICE, gz_service, False)

            # result, response = self.node.request(service_name, request, Pose, Boolean, self.timeout)
            # return cmd_txt, result, response
        else:
            return None
            # return "", None, None

    # 对模型施加力，使其按指定方向运动
    def func_apply_model_force(self, model_name=None, force_x=0.0, force_y=0.0, force_z=0.0, 
                                torque_x=0.0, torque_y=0.0, torque_z=0.0, persistent=False):
        """
        对模型施加力或力矩，使其按指定方向运动
        
        Args:
            model_name: 模型名称，如果为None则随机选择一个模型
            force_x, force_y, force_z: 沿x、y、z轴的力（牛顿）
            torque_x, torque_y, torque_z: 绕x、y、z轴的力矩
            persistent: 如果为True，力将持续施加；如果为False，只施加一次
        
        Returns:
            GzCommand对象，可以通过execute()方法执行
        """
        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        
        # 获取模型名称
        if model_name is None:
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                return None
            model_name = random.choice(available_models).name
        
        # 创建EntityWrench消息
        entity_wrench = EntityWrench()
        entity_wrench.entity.name = model_name
        # entity_wrench.entity.type = Entity_Type.MODEL  # 设置为MODEL类型
        
        # 设置力
        entity_wrench.wrench.force.x = force_x
        entity_wrench.wrench.force.y = force_y
        entity_wrench.wrench.force.z = force_z
        
        # 设置力矩
        entity_wrench.wrench.torque.x = torque_x
        entity_wrench.wrench.torque.y = torque_y
        entity_wrench.wrench.torque.z = torque_z
        
        # 确定topic名称
        # 注意：需要确保世界中加载了 gz-sim-apply-link-wrench-system 插件
        if persistent:
            topic_name = f"/world/{self.world_name}/wrench/persistent"
        else:
            topic_name = f"/world/{self.world_name}/wrench"
        
        if self.use_text:
            # 手动构建正确格式的命令行文本
            # 格式：entity: {name: "model_name", type: MODEL}, wrench: {force: {x: ..., y: ..., z: ...}, torque: {x: ..., y: ..., z: ...}}
            wrench_str = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force_x}, y: {force_y}, z: {force_z}}}, torque: {{x: {torque_x}, y: {torque_y}, z: {torque_z}}}}}'
            cmd_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str}'"
            print(f"DEBUG: cmd_txt: {cmd_txt}")
            return GzCommand(GzCommandType.TOPIC, [cmd_txt], True)
        else:
            # 使用Node API发布消息
            # 注意：需要先advertise topic，然后publish
            publisher = self.node.advertise(topic_name, EntityWrench)
            time.sleep(0.1)  # 等待连接建立
            publisher.publish(entity_wrench)
            # 返回一个简单的命令对象用于记录
            return GzCommand(GzCommandType.TOPIC, None, False)

    # 设置模型的线性速度（如果模型支持速度控制）
    def func_set_model_velocity(self, model_name=None, velocity_x=0.0, velocity_y=0.0, velocity_z=0.0):
        """
        设置模型的线性速度，使其按指定速度运动
        
        注意：此方法需要模型具有free_group（自由刚体组），通常只有没有关节约束的模型才支持
        
        Args:
            model_name: 模型名称，如果为None则随机选择一个模型
            velocity_x, velocity_y, velocity_z: 沿x、y、z轴的速度（m/s）
        
        Returns:
            GzCommand对象或None
        """
        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        
        # 获取模型名称
        if model_name is None:
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                return None
            model_name = random.choice(available_models).name
        
        
        # 注意：gz-sim中设置模型速度通常需要通过组件系统，而不是直接的服务
        # 这里提供一个通过施加持续力来近似实现速度控制的方法
        # 或者可以通过设置模型的pose来实现瞬间移动（但不适合持续运动）
        
        # 方法1：通过施加力来实现速度控制（需要根据模型质量调整力的大小）
        # 这里简化处理，使用一个固定的力值
        # 实际应用中可能需要根据模型质量和期望速度计算合适的力
        
        # 使用施加力的方法来实现速度控制
        # 注意：这只是一个近似方法，实际效果取决于模型的物理属性
        force_magnitude = 100.0  # 可以根据需要调整
        force_x = velocity_x * force_magnitude
        force_y = velocity_y * force_magnitude
        force_z = velocity_z * force_magnitude
        
        return self.func_apply_model_force(
            model_name=model_name,
            force_x=force_x,
            force_y=force_y,
            force_z=force_z,
            persistent=True  # 持续施加力以维持速度
        )

    # 蜕变测试：运动确定性测试（同样条件下两次运动结果应一致）
    def metamorphic_test_example(self, velocity_x=None, test_duration=5.0):
        """
        蜕变测试：运动确定性测试
        
        蜕变关系：对同一模型，从相同初始状态施加相同的力F，运行相同时间t，
        两次实验应产生相同的位移。这是物理引擎确定性的基本要求。
        
        测试流程：
        1. 选择模型，获取初始位置
        2. 测试A：暂停→施力→恢复→等待t秒→记录位置P1
        3. 清除力，重置模型到初始状态
        4. 测试B：暂停→施同样的力→恢复→等待t秒→记录位置P2
        5. 验证P1 ≈ P2
        
        Args:
            velocity_x: 已废弃参数，保留接口兼容性
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, initial_pos, final_pos, expected_pos, success) 或 None
            其中 final_pos = 测试A的最终位置, expected_pos = 测试B的最终位置
        """
        # 1. 获取场景并随机选择一个模型
        scene, reserved_models = self.get_scene()
        if not reserved_models or scene is None:
            print("DEBUG: get_scene() returned None, skipping this test")
            return None
        
        available_models = self.get_testable_models(scene, reserved_models)
        if not available_models:
            print("No available models for metamorphic test")
            return None
        
        target_model = random.choice(available_models)
        model_name = target_model.name
        print(f"Selected model for metamorphic test: {model_name}")
        
        # Gazebo 以暂停状态启动（无 -r 参数），模型处于 SDF 定义的初始位置且速度为零
        # 通过 scene/info 服务获取初始位置（暂停状态下 topic 不可靠，但 service 可用）
        initial_pos = self.get_model_pose_from_scene(model_name)
        if initial_pos is None:
            print(f"DEBUG: Failed to get initial position for model {model_name}")
            return None
        print(f"Initial position: {initial_pos}")
        
        # 2. 获取模型质量，生成全向随机力（保存为实例变量确保两次测试相同）
        model_mass = self.get_model_mass(model_name)
        if model_mass is None or model_mass <= 0:
            print(f"DEBUG: Invalid mass for model {model_name}, using default 1.0")
            model_mass = 1.0
        
        force_x, force_y, force_z = self.generate_omnidirectional_force(model_mass)
        self._test_force_x = force_x
        self._test_force_y = force_y
        self._test_force_z = force_z
        print(f"Force F: ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N")
        print(f"Test Duration: {test_duration:.2f} s")
        
        # ===== 测试A：第一次施加力 =====
        print("DEBUG: Test A: Applying force (first run)...")
        
        # 暂停模拟
        pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
        pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
        pause_cmd.execute(self.experiment_log)
        self.log_sleep(0.3, "Wait for pause (test A)")
        time.sleep(0.3)
        
        # 清除残余力并施加力
        self.clear_model_wrench(model_name)
        force_cmd = self.func_apply_model_force(
            model_name=model_name,
            force_x=self._test_force_x,
            force_y=self._test_force_y,
            force_z=self._test_force_z,
            persistent=True
        )
        if force_cmd:
            force_cmd.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait for force to be applied (test A)")
            time.sleep(0.1)
        else:
            print("DEBUG: Warning - force_cmd is None")
            return None
        
        # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
        num_steps = int(test_duration / 0.001)
        print(f"Running test A for {test_duration:.2f} seconds ({num_steps} steps)...")
        self.step_simulation(num_steps)
        
        # 获取最终位置（仿真已暂停，使用 scene 服务获取）
        print("DEBUG: Getting position after test A...")
        pos_after_A = self.get_model_pose_from_scene(model_name)
        if pos_after_A is None:
            print(f"DEBUG: Failed to get position after test A for model {model_name}")
            return None
        print(f"Position after test A: {pos_after_A}")
        
        # 计算测试A的位移
        d1 = (pos_after_A[0] - initial_pos[0],
              pos_after_A[1] - initial_pos[1],
              pos_after_A[2] - initial_pos[2])
        d1_magnitude = (d1[0]**2 + d1[1]**2 + d1[2]**2)**0.5
        print(f"Displacement A: ({d1[0]:.4f}, {d1[1]:.4f}, {d1[2]:.4f}), magnitude: {d1_magnitude:.4f} m")
        
        # 位移检测：如果模型几乎没动，跳过测试
        MIN_DISPLACEMENT_THRESHOLD = 0.01  # 1cm
        if d1_magnitude < MIN_DISPLACEMENT_THRESHOLD:
            error_info = f"Model '{model_name}' did not move (displacement: {d1_magnitude:.6f} m < {MIN_DISPLACEMENT_THRESHOLD} m). Skipping."
            print(f"DEBUG: {error_info}")
            self.clear_model_wrench(model_name)
            return None
        
        # ===== 重置模型状态（仿真保持暂停状态） =====
        # Test A 结束后仿真已暂停，在暂停状态下完成所有重置操作
        # 这样模型不会因为重力等外力而在重置后发生位移
        print("DEBUG: Resetting model for test B (simulation stays paused)...")
        self.clear_model_wrench(model_name)
        self.log_sleep(0.2, "Wait after clearing wrench")
        time.sleep(0.2)
        
        self.reset_simulation()
        self.log_sleep(0.5, "Wait after world reset")
        time.sleep(0.5)
        
        self.set_model_pose(model_name, initial_pos[0], initial_pos[1], initial_pos[2],
                            initial_pos[3], initial_pos[4], initial_pos[5], initial_pos[6])
        self.log_sleep(0.5, "Wait after setting model pose")
        time.sleep(0.5)
        
        # 验证重置成功（通过 scene 服务验证，无需恢复仿真）
        pos_after_reset = self.get_model_pose_from_scene(model_name)
        if pos_after_reset is not None:
            print(f"Position after reset: {pos_after_reset}")
        
        # ===== 测试B：第二次施加相同的力 =====
        print("DEBUG: Test B: Applying same force (second run)...")
        
        # 显式暂停模拟（确保处于暂停状态，与 Test A 的 pause→force→resume 流程一致）
        pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
        pause_cmd.execute(self.experiment_log)
        self.log_sleep(0.3, "Wait for pause (test B)")
        time.sleep(0.3)
        
        # 施加同样的力
        self.clear_model_wrench(model_name)
        force_cmd_B = self.func_apply_model_force(
            model_name=model_name,
            force_x=self._test_force_x,
            force_y=self._test_force_y,
            force_z=self._test_force_z,
            persistent=True
        )
        if force_cmd_B:
            force_cmd_B.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait for force to be applied (test B)")
            time.sleep(0.1)
        else:
            print("DEBUG: Warning - force_cmd_B is None")
            return None
        
        # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
        print(f"Running test B for {test_duration:.2f} seconds ({num_steps} steps)...")
        self.step_simulation(num_steps)
        
        # 获取最终位置（仿真已暂停，使用 scene 服务获取）
        print("DEBUG: Getting position after test B...")
        pos_after_B = self.get_model_pose_from_scene(model_name)
        if pos_after_B is None:
            print(f"DEBUG: Failed to get position after test B for model {model_name}")
            return None
        print(f"Position after test B: {pos_after_B}")
        
        # 清除力
        self.clear_model_wrench(model_name)
        
        # ===== 比较结果 =====
        # 蜕变关系：两次运动应产生相同的位移 pos_after_A ≈ pos_after_B
        # 混合阈值：绝对误差 < 0.5m 或 相对误差 < 10%，满足其一即通过
        abs_threshold = 0.5  # 绝对阈值：0.5米
        rel_threshold = 0.10  # 相对阈值：10%
        
        error_x = abs(pos_after_A[0] - pos_after_B[0])
        error_y = abs(pos_after_A[1] - pos_after_B[1])
        error_z = abs(pos_after_A[2] - pos_after_B[2])
        error_magnitude = (error_x**2 + error_y**2 + error_z**2)**0.5
        
        # 计算位移（取两次中较大的位移作为基准）
        disp_a = ((pos_after_A[0]-initial_pos[0])**2 + (pos_after_A[1]-initial_pos[1])**2 + (pos_after_A[2]-initial_pos[2])**2)**0.5
        disp_b = ((pos_after_B[0]-initial_pos[0])**2 + (pos_after_B[1]-initial_pos[1])**2 + (pos_after_B[2]-initial_pos[2])**2)**0.5
        max_displacement = max(disp_a, disp_b)
        relative_error = error_magnitude / max_displacement if max_displacement > 0.001 else 0.0
        
        # 混合判定：绝对误差小于阈值 或 相对误差小于阈值
        abs_pass = (error_x < abs_threshold and error_y < abs_threshold and error_z < abs_threshold)
        rel_pass = (relative_error < rel_threshold)
        success = abs_pass or rel_pass
        
        print(f"Position error (A vs B): x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}")
        print(f"Error magnitude: {error_magnitude:.4f} m, Max displacement: {max_displacement:.4f} m, Relative error: {relative_error*100:.2f}%")
        print(f"Test {'PASSED' if success else 'FAILED'} (abs_pass={abs_pass}, rel_pass={rel_pass})")
        
        # 返回格式保持兼容: (model_name, initial_pos, final_pos, expected_pos, success)
        # final_pos = 测试A结果, expected_pos = 测试B结果
        return (model_name, initial_pos, pos_after_A, pos_after_B, success)

    def metamorphic_test_force_additivity(self, test_duration=5.0):
        """
        蜕变测试：力的可加性测试
        
        测试原理：对同一模型同时施加多个力 F1, F2, ..., Fn，结果应该等于这些力的矢量和 F_total = F1 + F2 + ... + Fn
        
        测试流程：
        1. 随机选择一个模型，获取初始位置
        2. 测试A：同时施加力 F1 和 F2，运行 t 秒，记录最终位置 P1
        3. 测试B：重置模型到初始状态，施加力 F_total = F1 + F2，运行 t 秒，记录最终位置 P2
        4. 验证 P1 ≈ P2（在容差范围内）
        
        Args:
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, initial_pos, pos_with_f1_f2, pos_with_f_total, success, error_info) 或 None
        """
        try:
            import time as time_module
            
            # 1. 获取场景并随机选择一个模型
            scene, reserved_models = self.get_scene()
            if not reserved_models or scene is None:
                print("DEBUG: get_scene() returned None, skipping this test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("No available models for force additivity test")
                return None
            
            target_model = random.choice(available_models)
            model_name = target_model.name
            print(f"Selected model for force additivity test: {model_name}")
            
            # Gazebo 以暂停状态启动（无 -r 参数），模型处于 SDF 定义的初始位置且速度为零
            # 通过 scene/info 服务获取初始位置和角度（暂停状态下 service 可用）
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: {initial_pos}")
            
            # 获取初始角度（用于后续恢复）
            initial_state_dict = self.record_all_models_state_from_scene()
            if initial_state_dict is None or model_name not in initial_state_dict:
                print("DEBUG: Failed to record initial state")
                return None
            initial_orientation = initial_state_dict[model_name]['orientation']
            
            # 3. 获取模型质量，生成与质量成比例的全向随机力 F1 和 F2
            model_mass = self.get_model_mass(model_name)
            
            # F1: 全向随机，力大小与质量成比例
            force1_x, force1_y, force1_z = self.generate_omnidirectional_force(model_mass)
            
            # F2: 全向随机，力大小与质量成比例
            force2_x, force2_y, force2_z = self.generate_omnidirectional_force(model_mass)
            
            # 计算合力
            force_total_x = force1_x + force2_x
            force_total_y = force1_y + force2_y
            force_total_z = force1_z + force2_z
            
            print(f"Force F1: ({force1_x:.2f}, {force1_y:.2f}, {force1_z:.2f}) N")
            print(f"Force F2: ({force2_x:.2f}, {force2_y:.2f}, {force2_z:.2f}) N")
            print(f"Force F_total = F1 + F2: ({force_total_x:.2f}, {force_total_y:.2f}, {force_total_z:.2f}) N")
            
            # 4. 测试A：同时施加 F1 和 F2
            print("DEBUG: Test A: Applying F1 and F2 simultaneously...")
            
            # 清除之前的力（如果有）
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench")
            time_module.sleep(0.2)
            
            # 步骤1：暂停模拟（使用 gz service 指令）
            print("DEBUG: Pausing simulation using gz service command...")
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete")
            time_module.sleep(0.3)  # 等待暂停完成
            
            # 步骤2：使用 gz topic 指令添加 F1
            print("DEBUG: Applying F1 using gz topic command...")
            topic_name = f"/world/{self.world_name}/wrench/persistent"
            wrench_str1 = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force1_x}, y: {force1_y}, z: {force1_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd1_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str1}'"
            force_cmd1 = GzCommand(GzCommandType.TOPIC, [force_cmd1_txt], True)
            force_cmd1.execute(self.experiment_log)
            self.log_sleep(0.05, "Brief wait after applying F1")
            time_module.sleep(0.05)  # 短暂等待
            
            # 步骤3：使用 gz topic 指令添加 F2
            print("DEBUG: Applying F2 using gz topic command...")
            wrench_str2 = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force2_x}, y: {force2_y}, z: {force2_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd2_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str2}'"
            force_cmd2 = GzCommand(GzCommandType.TOPIC, [force_cmd2_txt], True)
            force_cmd2.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait for F2 message to be received")
            time_module.sleep(0.1)  # 等待消息被接收
            
            # 步骤4：精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            num_steps = int(test_duration / 0.001)
            print(f"Running with F1+F2 for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取最终位置（仿真已暂停，使用 scene 服务获取）
            print("DEBUG: Getting position after F1+F2...")
            pos_with_f1_f2 = self.get_model_pose_from_scene(model_name)
            if pos_with_f1_f2 is None:
                print(f"DEBUG: Failed to get position after F1+F2")
                return None
            print(f"Position with F1+F2: {pos_with_f1_f2}")
            
            # 检测模型是否真正发生了位移（排除被约束的模型）
            import math
            displacement_a = math.sqrt(
                (pos_with_f1_f2[0] - initial_pos[0])**2 +
                (pos_with_f1_f2[1] - initial_pos[1])**2 +
                (pos_with_f1_f2[2] - initial_pos[2])**2
            )
            min_displacement = 0.01  # 最小可接受位移 1cm
            if displacement_a < min_displacement:
                print(f"DEBUG: Model '{model_name}' barely moved (displacement={displacement_a:.6f}m < {min_displacement}m). "
                      f"Model is likely constrained by ground contact/joints. Skipping this test.")
                self.clear_model_wrench(model_name)
                return None
            
            # 5. 重置到初始状态（仿真保持暂停状态）
            # Test A 结束后仿真已暂停，在暂停状态下完成所有重置操作
            # 这样模型不会因为重力等外力而在重置后发生位移
            print("DEBUG: Resetting to initial state (simulation stays paused)...")
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench before reset")
            time_module.sleep(0.2)
            
            # 使用 reset 功能重置模拟到初始状态
            reset_success = self.reset_simulation()
            if not reset_success:
                print("DEBUG: Failed to reset simulation")
                return None
            
            self.log_sleep(1.0, "Wait for reset to complete")
            time_module.sleep(1.0)  # 等待重置完成
            
            # 使用 set_pose 恢复模型到初始位置和角度
            restore_pose_success = self.set_model_pose(
                model_name, 
                initial_pos[0], initial_pos[1], initial_pos[2],
                initial_orientation[0], initial_orientation[1], 
                initial_orientation[2], initial_orientation[3]
            )
            if not restore_pose_success:
                print("DEBUG: Failed to restore model pose")
                return None
            
            self.log_sleep(0.5, "Wait for position restore")
            time_module.sleep(0.5)  # 等待位置恢复
            
            # 验证位置是否重置（通过 scene 服务验证，无需恢复仿真）
            reset_pos = self.get_model_pose_from_scene(model_name)
            if reset_pos is None:
                print("DEBUG: Failed to get position after reset")
                return None
            print(f"Position after reset: {reset_pos}")
            
            # 6. 测试B：施加 F_total = F1 + F2
            print("DEBUG: Test B: Applying F_total = F1 + F2...")
            
            # 显式暂停模拟（确保处于暂停状态，与 Test A 的 pause→force→resume 流程一致）
            print("DEBUG: Pausing simulation before applying force (test B)...")
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete (test B)")
            time_module.sleep(0.3)
            
            # 清除之前的力（如果有）
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench before test B")
            time_module.sleep(0.2)
            
            # 使用 gz topic 指令添加 F_total
            print("DEBUG: Applying F_total using gz topic command...")
            topic_name = f"/world/{self.world_name}/wrench/persistent"
            wrench_str_total = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force_total_x}, y: {force_total_y}, z: {force_total_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd_total_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str_total}'"
            force_cmd_total = GzCommand(GzCommandType.TOPIC, [force_cmd_total_txt], True)
            force_cmd_total.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait for F_total message to be received")
            time_module.sleep(0.1)  # 等待消息被接收
            
            # 步骤3：精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            print(f"Running with F_total for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取最终位置（仿真已暂停，使用 scene 服务获取）
            print("DEBUG: Getting position after F_total...")
            pos_with_f_total = self.get_model_pose_from_scene(model_name)
            if pos_with_f_total is None:
                print(f"DEBUG: Failed to get position after F_total")
                return None
            print(f"Position with F_total: {pos_with_f_total}")
            
            # 7. 验证结果
            # 混合阈值：绝对误差 < 0.5m 或 相对误差 < 10%，满足其一即通过
            abs_threshold = 0.5  # 绝对阈值：0.5米
            rel_threshold = 0.10  # 相对阈值：10%
            
            error_x = abs(pos_with_f1_f2[0] - pos_with_f_total[0])
            error_y = abs(pos_with_f1_f2[1] - pos_with_f_total[1])
            error_z = abs(pos_with_f1_f2[2] - pos_with_f_total[2])
            error_magnitude = (error_x**2 + error_y**2 + error_z**2)**0.5
            
            # 计算位移（取两次中较大的位移作为基准）
            disp_f1f2 = ((pos_with_f1_f2[0]-initial_pos[0])**2 + (pos_with_f1_f2[1]-initial_pos[1])**2 + (pos_with_f1_f2[2]-initial_pos[2])**2)**0.5
            disp_ftotal = ((pos_with_f_total[0]-initial_pos[0])**2 + (pos_with_f_total[1]-initial_pos[1])**2 + (pos_with_f_total[2]-initial_pos[2])**2)**0.5
            max_displacement = max(disp_f1f2, disp_ftotal)
            relative_error = error_magnitude / max_displacement if max_displacement > 0.001 else 0.0
            
            # 混合判定：绝对误差小于阈值 或 相对误差小于阈值
            abs_pass = (error_x < abs_threshold and error_y < abs_threshold and error_z < abs_threshold)
            rel_pass = (relative_error < rel_threshold)
            success = abs_pass or rel_pass
            
            error_info = f"Position difference: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f} m\n"
            error_info += f"Error magnitude: {error_magnitude:.4f} m\n"
            error_info += f"Max displacement: {max_displacement:.4f} m\n"
            error_info += f"Relative error: {relative_error*100:.2f}%\n"
            error_info += f"Threshold: abs < {abs_threshold} m OR rel < {rel_threshold*100:.0f}%\n"
            error_info += f"F1: ({force1_x:.2f}, {force1_y:.2f}, {force1_z:.2f}) N\n"
            error_info += f"F2: ({force2_x:.2f}, {force2_y:.2f}, {force2_z:.2f}) N\n"
            error_info += f"F_total: ({force_total_x:.2f}, {force_total_y:.2f}, {force_total_z:.2f}) N"
            
            print(f"Position error: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}")
            print(f"Error magnitude: {error_magnitude:.4f} m, Max displacement: {max_displacement:.4f} m, Relative error: {relative_error*100:.2f}%")
            print(f"Test {'PASSED' if success else 'FAILED'} (abs_pass={abs_pass}, rel_pass={rel_pass})")
            
            # 清除力
            self.clear_model_wrench(model_name)
            
            return (model_name, initial_pos, pos_with_f1_f2, pos_with_f_total, success, error_info)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_force_additivity: {e}")
            import traceback
            traceback.print_exc()
            return None

    def clear_model_wrench(self, model_name):
        """
        清除模型上施加的持续力
        
        Args:
            model_name: 模型名称
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 /world/<world_name>/wrench/clear topic 清除持续力
            topic_name = f"/world/{self.world_name}/wrench/clear"
            
            # 创建 Entity 消息（Entity 已经在文件顶部导入）
            entity_msg = Entity()
            entity_msg.name = model_name
            # entity_msg.type = Entity_Type.MODEL  # 如果需要可以设置类型
            
            if self.use_text:
                # 使用命令行方式
                cmd_txt = f"gz topic -t {topic_name} -m gz.msgs.Entity -p 'name: \"{model_name}\", type: MODEL'"
                cmd = GzCommand(GzCommandType.TOPIC, [cmd_txt], True)
                cmd.execute(self.experiment_log if hasattr(self, 'experiment_log') else None)
            else:
                # 使用 Node API
                publisher = self.node.advertise(topic_name, Entity)
                time.sleep(0.1)
                publisher.publish(entity_msg)
            
            return True
            
        except Exception as e:
            print(f"DEBUG: Exception in clear_model_wrench: {e}")
            return False

    def reset_simulation(self):
        """
        重置模拟到初始状态
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 /world/<world_name>/control 服务进行重置
            service_name = f"/world/{self.world_name}/control"
            
            if self.use_text:
                # 使用命令行方式
                cmd_txt = f"gz service -s {service_name} --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'reset: {{all: true}}, pause: true'"
                cmd = GzCommand(GzCommandType.SERVICE, [cmd_txt], True)
                cmd.execute(self.experiment_log if hasattr(self, 'experiment_log') else None)
                print(f"DEBUG: Successfully reset simulation (command logged)")
                return True
            else:
                # 使用 Node API
                request = WorldControl()
                request.reset.all = True
                request.pause = True
                
                result, response = self.node.request(service_name, request, WorldControl, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Successfully reset simulation")
                    return True
                else:
                    print(f"DEBUG: Failed to reset simulation")
                    return False
                
        except Exception as e:
            print(f"DEBUG: Exception in reset_simulation: {e}")
            import traceback
            traceback.print_exc()
            return False

    def set_model_pose(self, model_name, x, y, z, w, qx, qy, qz):
        """
        设置模型的位置和角度
        
        Args:
            model_name: 模型名称
            x, y, z: 位置坐标
            w, qx, qy, qz: 四元数（w, x, y, z）
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 /world/<world_name>/set_pose 服务设置模型位置
            service_name = f"/world/{self.world_name}/set_pose"
            
            if self.use_text:
                # 使用命令行方式
                # 注意：四元数的顺序是 w, x, y, z
                cmd_txt = f"gz service -s {service_name} --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'name: \"{model_name}\", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: {w}, x: {qx}, y: {qy}, z: {qz}}}'"
                cmd = GzCommand(GzCommandType.SERVICE, [cmd_txt], True)
                cmd.execute(self.experiment_log if hasattr(self, 'experiment_log') else None)
                print(f"DEBUG: Successfully set pose for model {model_name} (command logged)")
                return True
            else:
                # 使用 Node API
                request = Pose()
                request.name = model_name
                request.position.x = x
                request.position.y = y
                request.position.z = z
                request.orientation.w = w
                request.orientation.x = qx
                request.orientation.y = qy
                request.orientation.z = qz
                
                result, response = self.node.request(service_name, request, Pose, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Successfully set pose for model {model_name}")
                    return True
                else:
                    print(f"DEBUG: Failed to set pose for model {model_name}")
                    return False
                
        except Exception as e:
            print(f"DEBUG: Exception in set_model_pose: {e}")
            import traceback
            traceback.print_exc()
            return False

    def set_real_time_factor(self, rtf_value):
        """
        设置模拟的 real_time_factor 参数
        
        Args:
            rtf_value: real_time_factor 值（例如 0.5, 1.0, 2.0）
        
        Returns:
            bool: 是否成功
        """
        try:
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    return False
                self.world_name = response.data[0]
            
            # 使用 /world/<world_name>/set_physics 服务设置 real_time_factor
            service_name = f"/world/{self.world_name}/set_physics"
            
            if self.use_text:
                # 使用命令行方式
                # 需要获取当前的 max_step_size，保持它不变
                # 但为了简化，我们可以只设置 real_time_factor，max_step_size 使用默认值或保持原值
                # 实际上，我们需要先获取当前的 physics 参数，但为了简化，我们假设 max_step_size 为 0.001
                cmd_txt = f"gz service -s {service_name} --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'max_step_size: 0.001, real_time_factor: {rtf_value}'"
                cmd = GzCommand(GzCommandType.SERVICE, [cmd_txt], True)
                cmd.execute(self.experiment_log if hasattr(self, 'experiment_log') else None)
                if hasattr(self, 'experiment_log'):
                    self.log_sleep(0.2, f"Wait for real_time_factor={rtf_value} to take effect")
                time.sleep(0.2)  # 等待设置生效
                return True
            else:
                # 使用 Node API
                request = Physics()
                request.max_step_size = 0.001  # 保持默认值，只改变 real_time_factor
                request.real_time_factor = rtf_value
                
                result, response = self.node.request(service_name, request, Physics, Boolean, self.timeout)
                if result and response.data:
                    print(f"DEBUG: Successfully set real_time_factor to {rtf_value}")
                    if hasattr(self, 'experiment_log'):
                        self.log_sleep(0.2, f"Wait for real_time_factor={rtf_value} to take effect")
                    time.sleep(0.2)  # 等待设置生效
                    return True
                else:
                    print(f"DEBUG: Failed to set real_time_factor")
                    return False
                
        except Exception as e:
            print(f"DEBUG: Exception in set_real_time_factor: {e}")
            import traceback
            traceback.print_exc()
            return False

    def metamorphic_test_time_scaling(self, test_duration=5.0):
        """
        蜕变测试：时间比例测试
        
        测试原理：通过调整 real_time_factor 参数，调节仿真速度比例，物理行为应该有比例关系或性质相同
        如果 real_time_factor = r，那么在相同物理时间内，模型的位置应该相同
        
        测试流程：
        1. 默认参数（real_time_factor = 1.0）下，给一个力 F1，计时统计位置 P1
        2. 恢复模型位置（不重置模拟）
        3. 调整时间参数 real_time_factor（范围 0.5-2.0）
        4. 如果调整时间参数必须重置模拟，那就重置模拟
        5. 调整后给同样的力 F1，计时统计位置 P2
        6. 比较位置确认结果是否正确
        
        Args:
            test_duration: 测试持续时间（秒，物理时间）
        
        Returns:
            (model_name, initial_pos, pos_with_rtf1, pos_with_rtf2, rtf1, rtf2, success, error_info) 或 None
        """
        try:
            import time as time_module
            
            # 1. 获取场景并随机选择一个模型
            scene, reserved_models = self.get_scene()
            if not reserved_models or scene is None:
                print("DEBUG: get_scene() returned None, skipping this test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("No available models for time scaling test")
                return None
            
            target_model = random.choice(available_models)
            model_name = target_model.name
            print(f"Selected model for time scaling test: {model_name}")
            
            # Gazebo 以暂停状态启动（无 -r 参数），模型处于 SDF 定义的初始位置且速度为零
            # 通过 scene/info 服务获取初始位置和角度（暂停状态下 service 可用）
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: {initial_pos}")
            
            # 获取初始角度（用于后续恢复）
            initial_state_dict = self.record_all_models_state_from_scene()
            if initial_state_dict is None or model_name not in initial_state_dict:
                print("DEBUG: Failed to record initial state")
                return None
            initial_orientation = initial_state_dict[model_name]['orientation']
            
            # 3. 获取模型质量，生成与质量成比例的全向随机力 F1（保存为实例变量确保两次测试使用完全相同的力）
            model_mass = self.get_model_mass(model_name)
            force1_x, force1_y, force1_z = self.generate_omnidirectional_force(model_mass)
            
            # 保存力值，确保两次测试使用完全相同的力
            self._test_force_x = force1_x
            self._test_force_y = force1_y
            self._test_force_z = force1_z
            
            print(f"Force F1: ({force1_x:.6f}, {force1_y:.6f}, {force1_z:.6f}) N")
            print(f"DEBUG: Force values saved for both tests")
            
            # 4. 测试A：默认 real_time_factor = 1.0，施加 F1
            print("DEBUG: Test A: Applying F1 with default real_time_factor (1.0)...")
            
            # 确保 real_time_factor 为 1.0
            self.set_real_time_factor(1.0)
            self.log_sleep(0.3, "Wait after setting rtf=1.0")
            time_module.sleep(0.3)
            
            # 清除之前的力（如果有）
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench (test A)")
            time_module.sleep(0.2)
            
            # 暂停模拟
            print("DEBUG: Pausing simulation...")
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete (test A)")
            time_module.sleep(0.3)
            
            # 添加 F1（使用保存的力值）
            print("DEBUG: Applying F1 using gz topic command...")
            topic_name = f"/world/{self.world_name}/wrench/persistent"
            # 使用保存的力值，确保两次测试使用完全相同的力
            wrench_str1 = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {self._test_force_x}, y: {self._test_force_y}, z: {self._test_force_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd1_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str1}'"
            print(f"DEBUG: Force command for Test A: {force_cmd1_txt}")
            print(f"DEBUG: Force values for Test A - x: {self._test_force_x:.6f}, y: {self._test_force_y:.6f}, z: {self._test_force_z:.6f}")
            force_cmd1 = GzCommand(GzCommandType.TOPIC, [force_cmd1_txt], True)
            force_cmd1.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait after applying F1 (test A)")
            time_module.sleep(0.1)
            
            # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            num_steps = int(test_duration / 0.001)
            print(f"Running with F1 and rtf=1.0 for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取最终位置（仿真已暂停，使用 scene 服务获取）
            print("DEBUG: Getting position after F1 with rtf=1.0...")
            pos_with_rtf1 = self.get_model_pose_from_scene(model_name)
            if pos_with_rtf1 is None:
                print(f"DEBUG: Failed to get position after F1 with rtf=1.0")
                return None
            print(f"Position with rtf=1.0: {pos_with_rtf1}")
            
            # 检测模型是否真正发生了位移（排除被约束的模型）
            import math
            displacement_a = math.sqrt(
                (pos_with_rtf1[0] - initial_pos[0])**2 +
                (pos_with_rtf1[1] - initial_pos[1])**2 +
                (pos_with_rtf1[2] - initial_pos[2])**2
            )
            min_displacement = 0.01  # 最小可接受位移 1cm
            if displacement_a < min_displacement:
                print(f"DEBUG: Model '{model_name}' barely moved (displacement={displacement_a:.6f}m < {min_displacement}m). "
                      f"Model is likely constrained by ground contact/joints. Skipping this test.")
                self.clear_model_wrench(model_name)
                return None
            
            # 5. 重置模型状态（仿真保持暂停状态）
            # Test A 结束后仿真已暂停，在暂停状态下完成所有重置操作
            # 这样模型不会因为重力等外力而在重置后发生位移
            print("DEBUG: Resetting model state for test B (simulation stays paused)...")
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench before restore")
            time_module.sleep(0.2)
            
            # 重置模拟（清除所有速度，确保模型状态完全重置）
            reset_success = self.reset_simulation()
            if not reset_success:
                print("DEBUG: Failed to reset simulation")
                return None
            
            self.log_sleep(1.0, "Wait for reset to complete")
            time_module.sleep(1.0)  # 等待重置完成
            
            # 6. 调整时间参数 real_time_factor（范围 0.5-2.0）
            rtf2 = random.uniform(0.5, 2.0)
            print(f"DEBUG: Setting real_time_factor to {rtf2:.2f}...")
            
            # 设置新的 real_time_factor
            set_rtf_success = self.set_real_time_factor(rtf2)
            if not set_rtf_success:
                print("DEBUG: Failed to set real_time_factor after reset")
                return None
            
            self.log_sleep(0.3, "Wait for rtf parameter to take effect")
            time_module.sleep(0.3)  # 等待参数生效
            
            # 恢复模型位置（重置后需要重新设置位置）
            restore_pose_success = self.set_model_pose(
                model_name, 
                initial_pos[0], initial_pos[1], initial_pos[2],
                initial_orientation[0], initial_orientation[1], 
                initial_orientation[2], initial_orientation[3]
            )
            if not restore_pose_success:
                print("DEBUG: Failed to restore model pose after reset")
                return None
            
            self.log_sleep(0.5, "Wait for position restore after reset")
            time_module.sleep(0.5)  # 等待位置恢复
            
            # 验证位置是否正确恢复（通过 scene 服务验证，无需恢复仿真）
            verify_pos = self.get_model_pose_from_scene(model_name)
            if verify_pos:
                print(f"DEBUG: Model position after reset and restore: {verify_pos}")
                print(f"DEBUG: Expected position: {initial_pos}")
                pos_error = (
                    abs(verify_pos[0] - initial_pos[0]) +
                    abs(verify_pos[1] - initial_pos[1]) +
                    abs(verify_pos[2] - initial_pos[2])
                )
                if pos_error > 0.1:
                    print(f"DEBUG: WARNING - Position not fully restored, error: {pos_error:.3f}")
            
            # 7. 测试B：使用新的 real_time_factor，施加同样的 F1
            print(f"DEBUG: Test B: Applying F1 with real_time_factor={rtf2:.2f}...")
            
            # 显式暂停模拟（确保处于暂停状态，与 Test A 的 pause→force→resume 流程一致）
            print("DEBUG: Pausing simulation before applying force (test B)...")
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete (test B)")
            time_module.sleep(0.3)
            
            # 清除之前的力（如果有）
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench (test B)")
            time_module.sleep(0.2)
            
            # 添加 F1（使用保存的力值，确保与测试A完全相同）
            print("DEBUG: Applying F1 using gz topic command...")
            # 使用保存的力值，确保两次测试使用完全相同的力
            wrench_str1 = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {self._test_force_x}, y: {self._test_force_y}, z: {self._test_force_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd1_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str1}'"
            print(f"DEBUG: Force command for Test B: {force_cmd1_txt}")
            print(f"DEBUG: Force values for Test B - x: {self._test_force_x:.6f}, y: {self._test_force_y:.6f}, z: {self._test_force_z:.6f}")
            force_cmd1 = GzCommand(GzCommandType.TOPIC, [force_cmd1_txt], True)
            force_cmd1.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait after applying F1 (test B)")
            time_module.sleep(0.1)
            
            # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            print(f"Running with F1 and rtf={rtf2:.2f} for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取最终位置（仿真已暂停，使用 scene 服务获取）
            print(f"DEBUG: Getting position after F1 with rtf={rtf2:.2f}...")
            pos_with_rtf2 = self.get_model_pose_from_scene(model_name)
            if pos_with_rtf2 is None:
                print(f"DEBUG: Failed to get position after F1 with rtf={rtf2:.2f}")
                return None
            print(f"Position with rtf={rtf2:.2f}: {pos_with_rtf2}")
            
            # 8. 验证结果
            # 理论上，在相同的物理时间内，无论 real_time_factor 如何，模型的位置应该相同
            # 混合阈值：绝对误差 < 0.5m 或 相对误差 < 10%，满足其一即通过
            abs_threshold = 0.5  # 绝对阈值：0.5米
            rel_threshold = 0.10  # 相对阈值：10%
            
            error_x = abs(pos_with_rtf1[0] - pos_with_rtf2[0])
            error_y = abs(pos_with_rtf1[1] - pos_with_rtf2[1])
            error_z = abs(pos_with_rtf1[2] - pos_with_rtf2[2])
            error_magnitude = (error_x**2 + error_y**2 + error_z**2)**0.5
            
            # 计算位移（取两次中较大的位移作为基准）
            disp_rtf1 = ((pos_with_rtf1[0]-initial_pos[0])**2 + (pos_with_rtf1[1]-initial_pos[1])**2 + (pos_with_rtf1[2]-initial_pos[2])**2)**0.5
            disp_rtf2 = ((pos_with_rtf2[0]-initial_pos[0])**2 + (pos_with_rtf2[1]-initial_pos[1])**2 + (pos_with_rtf2[2]-initial_pos[2])**2)**0.5
            max_displacement = max(disp_rtf1, disp_rtf2)
            relative_error = error_magnitude / max_displacement if max_displacement > 0.001 else 0.0
            
            # 混合判定：绝对误差小于阈值 或 相对误差小于阈值
            abs_pass = (error_x < abs_threshold and error_y < abs_threshold and error_z < abs_threshold)
            rel_pass = (relative_error < rel_threshold)
            success = abs_pass or rel_pass
            
            error_info = f"Position difference: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f} m\n"
            error_info += f"Error magnitude: {error_magnitude:.4f} m\n"
            error_info += f"Max displacement: {max_displacement:.4f} m\n"
            error_info += f"Relative error: {relative_error*100:.2f}%\n"
            error_info += f"Threshold: abs < {abs_threshold} m OR rel < {rel_threshold*100:.0f}%\n"
            error_info += f"Real Time Factor 1: 1.0\n"
            error_info += f"Real Time Factor 2: {rtf2:.2f}\n"
            error_info += f"Test Duration (physical time): {test_duration:.2f} s\n"
            error_info += f"Force F1: ({self._test_force_x:.6f}, {self._test_force_y:.6f}, {self._test_force_z:.6f}) N\n"
            error_info += f"Note: Both tests used the exact same force values to ensure fairness."
            
            print(f"Position error: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}")
            print(f"Error magnitude: {error_magnitude:.4f} m, Max displacement: {max_displacement:.4f} m, Relative error: {relative_error*100:.2f}%")
            print(f"Test {'PASSED' if success else 'FAILED'} (abs_pass={abs_pass}, rel_pass={rel_pass})")
            
            # 清除力
            self.clear_model_wrench(model_name)
            
            return (model_name, initial_pos, pos_with_rtf1, pos_with_rtf2, 1.0, rtf2, success, error_info)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_time_scaling: {e}")
            import traceback
            traceback.print_exc()
            return None

    def generate_omnidirectional_force(self, model_mass=None, min_acceleration=5.0, max_acceleration=50.0):
        """
        生成全向随机力（均匀球面分布），力的大小与模型质量成比例。
        
        Args:
            model_mass: 模型质量(kg)。如果为None，使用默认力范围(50-500N)
            min_acceleration: 最小目标加速度(m/s²)，默认5.0，确保可见位移
            max_acceleration: 最大目标加速度(m/s²)，默认50.0，避免力过大
        
        Returns:
            (force_x, force_y, force_z) 元组
        """
        import math
        
        # 计算力的大小范围
        if model_mass is not None and model_mass > 0:
            # F = m * a，确保加速度在 [min_acceleration, max_acceleration] 之间
            min_force = model_mass * min_acceleration
            max_force = model_mass * max_acceleration
            # 设置绝对下限，避免力太小（对于极轻的模型）
            min_force = max(min_force, 50.0)
        else:
            # 未知质量时使用默认范围
            min_force = 50.0
            max_force = 500.0
        
        # 随机力大小
        magnitude = random.uniform(min_force, max_force)
        
        # 均匀球面分布的随机方向
        theta = random.uniform(0, 2 * math.pi)       # 方位角 [0, 2π]
        cos_phi = random.uniform(-1, 1)               # 极角余弦 [-1, 1]
        sin_phi = math.sqrt(1 - cos_phi**2)
        
        force_x = magnitude * sin_phi * math.cos(theta)
        force_y = magnitude * sin_phi * math.sin(theta)
        force_z = magnitude * cos_phi
        
        print(f"DEBUG: Generated force: ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N, "
              f"magnitude={magnitude:.2f} N, model_mass={model_mass}")
        
        return force_x, force_y, force_z

    def get_model_mass(self, model_name):
        """
        获取模型的总质量（所有link的质量之和）
        
        Args:
            model_name: 模型名称
        
        Returns:
            模型总质量（kg），如果获取失败返回None
        """
        try:
            scene, reserved_models = self.get_scene()
            if scene is None:
                print(f"DEBUG: Failed to get scene for model {model_name}")
                return None
            
            # 查找模型
            target_model = None
            for model in scene.model:
                if model.name == model_name:
                    target_model = model
                    break
            
            if target_model is None:
                print(f"DEBUG: Model {model_name} not found in scene")
                return None
            
            # 累加所有link的质量
            # 注意：proto3 中 mass 是标量字段(double)，不能使用 HasField()
            # 只需检查 inertial（消息字段）是否存在，然后直接读取 mass（默认值为0.0）
            total_mass = 0.0
            for link in target_model.link:
                if link.HasField('inertial') and link.inertial.mass > 0:
                    total_mass += link.inertial.mass
            
            if total_mass <= 0:
                print(f"DEBUG: Model {model_name} has invalid mass: {total_mass}")
                return None
            
            print(f"DEBUG: Model {model_name} total mass: {total_mass} kg")
            return total_mass
            
        except Exception as e:
            print(f"DEBUG: Exception in get_model_mass: {e}")
            import traceback
            traceback.print_exc()
            return None

    def modify_model_mass(self, model_name, new_mass, mass_scale_factor):
        """
        修改模型的质量系数
        
        策略：
        1. 获取模型的当前SDF（通过dump_sdf）
        2. 解析SDF，找到模型的所有link，修改每个link的质量
        3. 删除旧模型
        4. 使用修改后的SDF重新创建模型
        
        Args:
            model_name: 模型名称
            new_mass: 新的总质量（kg）
            mass_scale_factor: 质量缩放因子（用于记录）
        
        Returns:
            True if successful, False otherwise
        """
        try:
            import xml.etree.ElementTree as ET
            import re
            
            # 1. 获取当前世界的SDF
            world_sdf = self.dump_sdf(self.world_name)
            if not world_sdf:
                print(f"DEBUG: Failed to dump SDF for world {self.world_name}")
                return False
            
            # 2. 解析SDF，找到目标模型
            try:
                root = ET.fromstring(world_sdf)
            except ET.ParseError as e:
                print(f"DEBUG: Failed to parse SDF: {e}")
                return False
            
            # 查找模型（可能在world下，也可能直接是model）
            model_elem = None
            if root.tag == 'sdf':
                world_elem = root.find('world')
                if world_elem is not None:
                    for model in world_elem.findall('model'):
                        if model.get('name') == model_name:
                            model_elem = model
                            break
            elif root.tag == 'world':
                for model in root.findall('model'):
                    if model.get('name') == model_name:
                        model_elem = model
                        break
            
            if model_elem is None:
                print(f"DEBUG: Model {model_name} not found in SDF")
                return False
            
            # 3. 计算当前总质量，确定缩放因子
            current_total_mass = 0.0
            link_masses = []
            for link in model_elem.findall('link'):
                inertial = link.find('inertial')
                if inertial is not None:
                    mass_elem = inertial.find('mass')
                    if mass_elem is not None:
                        try:
                            mass_val = float(mass_elem.text)
                            current_total_mass += mass_val
                            link_masses.append((link, mass_elem, mass_val))
                        except ValueError:
                            pass
            
            if current_total_mass <= 0:
                print(f"DEBUG: Model {model_name} has invalid current mass: {current_total_mass}")
                return False
            
            # 计算每个link的新质量（按比例缩放）
            scale_factor = new_mass / current_total_mass
            print(f"DEBUG: Scaling model mass from {current_total_mass:.6f} kg to {new_mass:.6f} kg (scale factor: {scale_factor:.6f})")
            
            # 4. 修改每个link的质量
            for link, mass_elem, old_mass in link_masses:
                new_link_mass = old_mass * scale_factor
                mass_elem.text = str(new_link_mass)
                print(f"DEBUG: Link {link.get('name')}: {old_mass:.6f} kg -> {new_link_mass:.6f} kg")
                
                # 同时需要按比例修改惯性矩阵（保持形状不变）
                inertia = inertial.find('inertia')
                if inertia is not None:
                    for inertia_elem in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                        elem = inertia.find(inertia_elem)
                        if elem is not None:
                            try:
                                old_inertia = float(elem.text)
                                new_inertia = old_inertia * scale_factor
                                elem.text = str(new_inertia)
                            except ValueError:
                                pass
            
            # 5. 将修改后的模型SDF转换为字符串
            model_sdf_str = ET.tostring(model_elem, encoding='unicode')
            # 添加XML声明
            model_sdf_str = f'<sdf version="1.6">\n{model_sdf_str}\n</sdf>'
            
            # 6. 获取模型的当前位置和姿态（使用 scene 服务，暂停状态下也可工作）
            current_pose = self.get_model_pose_from_scene(model_name)
            if current_pose is None:
                print(f"DEBUG: Failed to get current pose for model {model_name}")
                return False
            
            # 获取模型的姿态（四元数）
            scene, _ = self.get_scene()
            if scene is None:
                return False
            
            model_orientation = None
            for model in scene.model:
                if model.name == model_name:
                    if model.HasField('pose'):
                        model_orientation = (
                            model.pose.orientation.w,
                            model.pose.orientation.x,
                            model.pose.orientation.y,
                            model.pose.orientation.z
                        )
                    break
            
            if model_orientation is None:
                model_orientation = (1.0, 0.0, 0.0, 0.0)  # 默认无旋转
            
            # 7. 删除旧模型
            print(f"DEBUG: Removing old model {model_name}...")
            remove_cmd_txt = f"gz service -s /world/{self.world_name}/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'name: \"{model_name}\"'"
            remove_cmd = GzCommand(GzCommandType.SERVICE, [remove_cmd_txt], True)
            remove_cmd.execute(self.experiment_log)
            self.log_sleep(0.5, "Wait for model removal")
            time.sleep(0.5)
            
            # 8. 使用修改后的SDF重新创建模型
            print(f"DEBUG: Creating model {model_name} with new mass...")
            service_name = f"/world/{self.world_name}/create"
            
            # 构建EntityFactory请求
            request_sdf = model_sdf_str.replace('\n', '\\n').replace('"', '\\"')
            cmd_txt = f"gz service -s {service_name} --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'sdf: \"{request_sdf}\", pose: {{position: {{x: {current_pose[0]}, y: {current_pose[1]}, z: {current_pose[2]}}}, orientation: {{w: {model_orientation[0]}, x: {model_orientation[1]}, y: {model_orientation[2]}, z: {model_orientation[3]}}}}}, name: \"{model_name}\", allow_renaming: false'"
            
            create_cmd = GzCommand(GzCommandType.SERVICE, [cmd_txt], True)
            create_cmd.execute(self.experiment_log)
            self.log_sleep(1.0, "Wait for model creation with new mass")
            time.sleep(1.0)
            
            print(f"DEBUG: Successfully modified model {model_name} mass to {new_mass:.6f} kg")
            return True
            
        except Exception as e:
            print(f"DEBUG: Exception in modify_model_mass: {e}")
            import traceback
            traceback.print_exc()
            return False

    def metamorphic_test_mass_scaling(self, test_duration=5.0):
        """
        蜕变测试：基于质量系数的蜕变关系
        
        测试原理：根据F=ma，在相同力F下，质量m1时位移d1，质量m2=k*m1时位移d2，应该有d2 = d1/k
        
        测试流程：
        1. 随机选择一个模型，获取初始位置和初始质量m
        2. 测试A：原始质量m，施加力F1，运行t秒，记录位移d1
        3. 重置模型位置
        4. 修改质量系数为km（k是比例因子，如0.5或2.0）
        5. 测试B：新质量km，施加相同的力F1，运行t秒，记录位移d2
        6. 验证d1/d2 ≈ k（在容差范围内）
        
        Args:
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, initial_pos, initial_mass, mass_scale_factor, pos_with_m1, pos_with_m2, d1, d2, success, error_info) 或 None
        """
        try:
            import time as time_module
            import random
            
            # 1. 获取场景并随机选择一个模型
            scene, reserved_models = self.get_scene()
            if not reserved_models or scene is None:
                print("DEBUG: get_scene() returned None, skipping this test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("No available models for mass scaling test")
                return None
            
            # 质量系数测试需要模型有有效质量，筛选出有质量的模型
            models_with_mass = []
            for model in available_models:
                mass = self.get_model_mass(model.name)
                if mass is not None and mass > 0:
                    models_with_mass.append((model, mass))
            
            if not models_with_mass:
                print("No available models with valid mass for mass scaling test")
                return None
            
            target_model, initial_mass = random.choice(models_with_mass)
            model_name = target_model.name
            print(f"Selected model for mass scaling test: {model_name} (mass={initial_mass:.6f} kg)")
            
            # Gazebo 以暂停状态启动（无 -r 参数），模型处于 SDF 定义的初始位置且速度为零
            # 通过 scene/info 服务获取初始位置（暂停状态下 service 可用）
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: {initial_pos}")
            print(f"Initial mass: {initial_mass:.6f} kg")
            
            # 3. 生成与质量成比例的全向随机力F1（保存为实例变量确保两次测试使用相同的力）
            # 注意：这里使用初始质量 initial_mass 来计算力大小
            force1_x, force1_y, force1_z = self.generate_omnidirectional_force(initial_mass)
            self._test_force_x = force1_x
            self._test_force_y = force1_y
            self._test_force_z = force1_z
            print(f"Force F1: ({force1_x:.2f}, {force1_y:.2f}, {force1_z:.2f}) N")
            
            # 4. 随机生成质量缩放因子k（范围0.5-2.0）
            mass_scale_factor = random.uniform(0.5, 2.0)
            new_mass = initial_mass * mass_scale_factor
            print(f"Mass scale factor k: {mass_scale_factor:.6f}")
            print(f"New mass (k*m): {new_mass:.6f} kg")
            
            # 5. 测试A：原始质量m，施加力F1
            print("DEBUG: Test A: Applying F1 with original mass m...")
            
            # 清除之前的力
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench (test A)")
            time_module.sleep(0.2)
            
            # 暂停模拟
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete (test A)")
            time_module.sleep(0.3)
            
            # 施加力F1
            topic_name = f"/world/{self.world_name}/wrench/persistent"
            wrench_str = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force1_x}, y: {force1_y}, z: {force1_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str}'"
            force_cmd = GzCommand(GzCommandType.TOPIC, [force_cmd_txt], True)
            force_cmd.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait after applying F1 (test A)")
            time_module.sleep(0.1)
            
            # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            num_steps = int(test_duration / 0.001)
            print(f"Running with F1 and mass m={initial_mass:.6f} kg for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取位置（仿真已暂停，使用 scene 服务获取）
            print("DEBUG: Getting position after F1 with mass m...")
            pos_with_m1 = self.get_model_pose_from_scene(model_name)
            if pos_with_m1 is None:
                print(f"DEBUG: Failed to get position after F1 with mass m")
                return None
            print(f"Position with mass m={initial_mass:.6f} kg: {pos_with_m1}")
            
            # 计算位移d1
            d1 = (
                pos_with_m1[0] - initial_pos[0],
                pos_with_m1[1] - initial_pos[1],
                pos_with_m1[2] - initial_pos[2]
            )
            d1_magnitude = (d1[0]**2 + d1[1]**2 + d1[2]**2)**0.5
            print(f"Displacement d1: ({d1[0]:.6f}, {d1[1]:.6f}, {d1[2]:.6f}) m, magnitude: {d1_magnitude:.6f} m")
            
            # 检测模型是否真正发生了位移（排除被约束的模型）
            min_displacement = 0.01  # 最小可接受位移 1cm
            if d1_magnitude < min_displacement:
                print(f"DEBUG: Model '{model_name}' barely moved (displacement={d1_magnitude:.6f}m < {min_displacement}m). "
                      f"Model is likely constrained by ground contact/joints. Skipping this test.")
                self.clear_model_wrench(model_name)
                return None
            
            # 6. 重置模型状态（仿真保持暂停状态）
            # Test A 结束后仿真已暂停，在暂停状态下完成所有重置操作
            # 这样模型不会因为重力等外力而在重置后发生位移
            print("DEBUG: Resetting model state for test B (simulation stays paused)...")
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench before reset")
            time_module.sleep(0.2)
            
            # 重置模拟以清除速度
            reset_success = self.reset_simulation()
            if not reset_success:
                print("DEBUG: Failed to reset simulation")
                return None
            
            self.log_sleep(1.0, "Wait for reset to complete")
            time_module.sleep(1.0)
            
            # 7. 修改质量系数为km
            print(f"DEBUG: Modifying model mass from {initial_mass:.6f} kg to {new_mass:.6f} kg...")
            modify_success = self.modify_model_mass(model_name, new_mass, mass_scale_factor)
            if not modify_success:
                print("DEBUG: Failed to modify model mass")
                return None
            
            # 恢复模型到初始位置（重置后需要重新设置位置）
            restore_pose_success = self.set_model_pose(
                model_name,
                initial_pos[0], initial_pos[1], initial_pos[2],
                1.0, 0.0, 0.0, 0.0
            )
            if not restore_pose_success:
                print("DEBUG: Failed to restore model pose after mass modification")
                return None
            
            self.log_sleep(0.5, "Wait for position restore after mass modification")
            time_module.sleep(0.5)
            
            # 8. 测试B：新质量km，施加相同的力F1
            print("DEBUG: Test B: Applying F1 with new mass k*m...")
            
            # 显式暂停模拟（确保处于暂停状态，与 Test A 的 pause→force→resume 流程一致）
            print("DEBUG: Pausing simulation before applying force (test B)...")
            pause_cmd_txt = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout {self.timeout} --req 'pause: true'"
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause to complete (test B)")
            time_module.sleep(0.3)
            
            # 清除之前的力（如果有）
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench (test B)")
            time_module.sleep(0.2)
            
            # 施加力F1（与测试A完全相同）
            wrench_str = f'entity: {{name: "{model_name}", type: MODEL}}, wrench: {{force: {{x: {force1_x}, y: {force1_y}, z: {force1_z}}}, torque: {{x: 0.0, y: 0.0, z: 0.0}}}}'
            force_cmd_txt = f"gz topic -t {topic_name} -m gz.msgs.EntityWrench -p '{wrench_str}'"
            force_cmd = GzCommand(GzCommandType.TOPIC, [force_cmd_txt], True)
            force_cmd.execute(self.experiment_log)
            self.log_sleep(0.1, "Wait after applying F1 (test B)")
            time_module.sleep(0.1)
            
            # 精确推进仿真（使用 multi_step，消除 wall-clock timing 影响）
            print(f"Running with F1 and mass k*m={new_mass:.6f} kg for {test_duration} seconds ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取位置（仿真已暂停，使用 scene 服务获取）
            print("DEBUG: Getting position after F1 with mass k*m...")
            pos_with_m2 = self.get_model_pose_from_scene(model_name)
            if pos_with_m2 is None:
                print(f"DEBUG: Failed to get position after F1 with mass k*m")
                return None
            print(f"Position with mass k*m={new_mass:.6f} kg: {pos_with_m2}")
            
            # 计算位移d2
            d2 = (
                pos_with_m2[0] - initial_pos[0],
                pos_with_m2[1] - initial_pos[1],
                pos_with_m2[2] - initial_pos[2]
            )
            d2_magnitude = (d2[0]**2 + d2[1]**2 + d2[2]**2)**0.5
            print(f"Displacement d2: ({d2[0]:.6f}, {d2[1]:.6f}, {d2[2]:.6f}) m, magnitude: {d2_magnitude:.6f} m")
            
            # 9. 验证结果
            # 理论上：d2 = d1/k，即 d1/d2 = k
            # 或者：d1 * k = d2（如果k>1，质量变大，位移应该变小）
            # 实际上：根据F=ma，a = F/m，所以a2 = F/(k*m) = a1/k
            # 位移 d = 0.5 * a * t^2，所以 d2 = 0.5 * (a1/k) * t^2 = d1/k
            # 因此：d1/d2 = k
            
            if d2_magnitude > 1e-6:  # 避免除零
                ratio_actual = d1_magnitude / d2_magnitude
                ratio_expected = mass_scale_factor
                ratio_error = abs(ratio_actual - ratio_expected) / ratio_expected if ratio_expected > 0 else abs(ratio_actual - ratio_expected)
                
                # 容差：允许20%的误差（因为可能有摩擦等非线性效应）
                error_threshold = 0.2
                success = ratio_error < error_threshold
                
                error_info = f"Mass scaling test results:\n"
                error_info += f"Initial mass m: {initial_mass:.6f} kg\n"
                error_info += f"New mass k*m: {new_mass:.6f} kg\n"
                error_info += f"Mass scale factor k: {mass_scale_factor:.6f}\n"
                error_info += f"Displacement d1 (mass m): magnitude = {d1_magnitude:.6f} m\n"
                error_info += f"Displacement d2 (mass k*m): magnitude = {d2_magnitude:.6f} m\n"
                error_info += f"Actual ratio d1/d2: {ratio_actual:.6f}\n"
                error_info += f"Expected ratio d1/d2: {ratio_expected:.6f}\n"
                error_info += f"Ratio error: {ratio_error*100:.2f}%\n"
                error_info += f"Error threshold: {error_threshold*100:.2f}%\n"
                error_info += f"Force F1: ({force1_x:.6f}, {force1_y:.6f}, {force1_z:.6f}) N\n"
                error_info += f"Test duration: {test_duration:.2f} s\n"
                
                print(f"Ratio d1/d2: actual={ratio_actual:.6f}, expected={ratio_expected:.6f}, error={ratio_error*100:.2f}%")
                print(f"Test {'PASSED' if success else 'FAILED'}")
            else:
                print("DEBUG: d2 magnitude is too small, cannot compute ratio")
                success = False
                error_info = f"Displacement d2 is too small: {d2_magnitude:.6f} m"
            
            # 清除力
            self.clear_model_wrench(model_name)
            
            return (model_name, initial_pos, initial_mass, mass_scale_factor, 
                   pos_with_m1, pos_with_m2, d1, d2, success, error_info)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_mass_scaling: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 辅助函数：从 SDF 文件中解析重力向量
    # ===================================================================
    def _get_gravity_from_sdf(self):
        """
        从当前实验的 SDF 文件中解析重力向量。
        
        Returns:
            (gx, gy, gz) 元组，如果解析失败返回 (0.0, 0.0, -9.81) 作为默认值
        """
        try:
            sdf_path = os.path.join(self.directory, self.sdf_name)
            if not os.path.exists(sdf_path):
                print(f"DEBUG: SDF file not found: {sdf_path}")
                return (0.0, 0.0, -9.81)
            
            tree = etree.parse(sdf_path)
            root = tree.getroot()
            
            # 查找 <world>/<gravity> 元素
            gravity_elem = root.find('.//world/gravity')
            if gravity_elem is None:
                # 也尝试直接在 sdf 下查找
                gravity_elem = root.find('.//gravity')
            
            if gravity_elem is None or gravity_elem.text is None:
                print("DEBUG: No <gravity> element found in SDF, using default (0, 0, -9.81)")
                return (0.0, 0.0, -9.81)
            
            parts = gravity_elem.text.strip().split()
            if len(parts) >= 3:
                gx = float(parts[0])
                gy = float(parts[1])
                gz = float(parts[2])
                print(f"DEBUG: Parsed gravity from SDF: ({gx}, {gy}, {gz})")
                return (gx, gy, gz)
            else:
                print(f"DEBUG: Invalid gravity format in SDF: '{gravity_elem.text}', using default")
                return (0.0, 0.0, -9.81)
                
        except Exception as e:
            print(f"DEBUG: Exception parsing gravity from SDF: {e}")
            return (0.0, 0.0, -9.81)

    # ===================================================================
    # 确定性重复测试 - 辅助函数
    # ===================================================================
    def _determinism_single_run(self, model_name=None, force_x=None, force_y=None, force_z=None,
                                 test_duration=5.0):
        """
        确定性测试的单次运行：选择模型、施力、推进仿真、获取最终位姿。
        
        如果 model_name/force 为 None，则随机选择/生成（用于第一次运行）。
        如果提供了这些参数，则精确复用（用于第二次运行）。
        
        Args:
            model_name: 模型名称（None=随机选择）
            force_x, force_y, force_z: 力分量（None=随机生成）
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, force_x, force_y, force_z, final_pos, num_steps) 或 None
        """
        try:
            import time as time_module
            
            # 1. 获取场景和模型
            scene, reserved_models = self.get_scene()
            if not reserved_models or scene is None:
                print("DEBUG: _determinism_single_run: get_scene() returned None")
                return None
            
            if model_name is None:
                # 第一次运行：随机选择模型
                available_models = self.get_testable_models(scene, reserved_models)
                if not available_models:
                    print("DEBUG: No available models for determinism test")
                    return None
                target_model = random.choice(available_models)
                model_name = target_model.name
            
            print(f"Determinism run - model: {model_name}")
            
            # 获取初始位姿
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: {initial_pos}")
            
            # 2. 生成力（或使用提供的力）
            if force_x is None:
                model_mass = self.get_model_mass(model_name)
                if model_mass is None or model_mass <= 0:
                    model_mass = 1.0
                force_x, force_y, force_z = self.generate_omnidirectional_force(model_mass)
            
            num_steps = int(test_duration / 0.001)
            print(f"Force: ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N, Steps: {num_steps}")
            
            # 3. 暂停 → 施力 → 推进仿真 → 获取最终位姿
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (determinism run)")
            time_module.sleep(0.3)
            
            self.clear_model_wrench(model_name)
            force_cmd = self.func_apply_model_force(
                model_name=model_name,
                force_x=force_x, force_y=force_y, force_z=force_z,
                persistent=True
            )
            if force_cmd:
                force_cmd.execute(self.experiment_log)
                self.log_sleep(0.1, "Wait for force to be applied (determinism run)")
                time_module.sleep(0.1)
            else:
                print("DEBUG: Warning - force_cmd is None in determinism run")
                return None
            
            print(f"Stepping simulation {num_steps} steps...")
            self.step_simulation(num_steps)
            
            # 获取最终位姿（仿真已暂停）
            final_pos = self.get_model_pose_from_scene(model_name)
            if final_pos is None:
                print(f"DEBUG: Failed to get final position for model {model_name}")
                return None
            print(f"Final position: {final_pos}")
            
            # 检查是否有位移（排除被约束的模型）
            import math
            displacement = math.sqrt(
                (final_pos[0] - initial_pos[0])**2 +
                (final_pos[1] - initial_pos[1])**2 +
                (final_pos[2] - initial_pos[2])**2
            )
            if displacement < 0.01:
                print(f"DEBUG: Model barely moved ({displacement:.6f} m). Skipping.")
                self.clear_model_wrench(model_name)
                return None
            
            self.clear_model_wrench(model_name)
            
            return (model_name, force_x, force_y, force_z, final_pos, num_steps)
            
        except Exception as e:
            print(f"DEBUG: Exception in _determinism_single_run: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 对称性/方向不变性测试
    # ===================================================================
    def metamorphic_test_symmetry(self, test_duration=5.0):
        """
        蜕变测试：对称性/方向不变性测试
        
        测试原理：对任意模型，施加相同大小但方向分别为 +x 和 +y 的力，
        对应方向的位移大小应相同（物理空间的各向同性 isotropy）。
        
        蜕变关系：
          Force = (F, 0, 0) → x 方向位移 dx
          Force = (0, F, 0) → y 方向位移 dy
          验证：|dx| ≈ |dy|
        
        这可以检测：
          - 插件中硬编码了特定轴方向的行为
          - 碰撞检测在某些方向上有偏差
          - 坐标系变换错误
        
        Args:
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, initial_pos, pos_after_x, pos_after_y,
             dx, dy, force_magnitude, success, error_info) 或 None
        """
        try:
            import time as time_module
            import math
            
            # 0. 检查重力方向：对称性测试要求测试的两个轴有相同的重力分量
            # 默认比较 x 和 y 轴，因此需要 gx ≈ gy（通常都为 0）
            gravity = self._get_gravity_from_sdf()
            gx, gy, gz = gravity
            
            GRAVITY_THRESHOLD = 0.01  # 小于此值视为零
            
            # 确定可用的测试轴对：选择两个重力分量相等的轴
            # axis_a 和 axis_b 分别表示 (力的方向索引, 位移读取索引)
            # 0=x, 1=y, 2=z
            test_axis_a = None
            test_axis_b = None
            axis_names = ['x', 'y', 'z']
            gravity_components = [gx, gy, gz]
            
            # 优先选择两个重力分量都为零的轴（最常见情况：gx=0, gy=0）
            zero_gravity_axes = [i for i in range(3) if abs(gravity_components[i]) < GRAVITY_THRESHOLD]
            if len(zero_gravity_axes) >= 2:
                test_axis_a = zero_gravity_axes[0]
                test_axis_b = zero_gravity_axes[1]
                print(f"DEBUG: Gravity ({gx}, {gy}, {gz}) - using axes "
                      f"{axis_names[test_axis_a]} and {axis_names[test_axis_b]} "
                      f"(both have ~zero gravity)")
            else:
                # 退而求其次：找两个重力分量近似相等的轴
                for i in range(3):
                    for j in range(i+1, 3):
                        if abs(gravity_components[i] - gravity_components[j]) < GRAVITY_THRESHOLD:
                            test_axis_a = i
                            test_axis_b = j
                            break
                    if test_axis_a is not None:
                        break
            
            if test_axis_a is None or test_axis_b is None:
                print(f"DEBUG: Gravity ({gx}, {gy}, {gz}) has no two axes with equal gravity components. "
                      f"Symmetry test not applicable for this world. Skipping.")
                return None
            
            print(f"DEBUG: Symmetry test will compare axis {axis_names[test_axis_a]} "
                  f"vs axis {axis_names[test_axis_b]}")
            
            # 1. 获取场景并选择模型
            scene, reserved_models = self.get_scene()
            if not reserved_models or scene is None:
                print("DEBUG: get_scene() returned None, skipping symmetry test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("No available models for symmetry test")
                return None
            
            target_model = random.choice(available_models)
            model_name = target_model.name
            print(f"Selected model for symmetry test: {model_name}")
            
            # 获取初始位姿
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: {initial_pos}")
            
            # 获取模型质量，计算力大小
            model_mass = self.get_model_mass(model_name)
            if model_mass is None or model_mass <= 0:
                print(f"DEBUG: Invalid mass for model {model_name}, using default 1.0")
                model_mass = 1.0
            
            # 生成力的大小（方向由测试决定）
            min_acceleration = 5.0
            max_acceleration = 50.0
            min_force = max(model_mass * min_acceleration, 50.0)
            max_force = model_mass * max_acceleration
            force_magnitude = random.uniform(min_force, max_force)
            
            num_steps = int(test_duration / 0.001)
            
            print(f"Force magnitude: {force_magnitude:.2f} N (mass: {model_mass:.2f} kg)")
            print(f"Test Duration: {test_duration:.2f} s ({num_steps} steps)")
            
            # 构建力向量：Test A 沿 axis_a 方向，Test B 沿 axis_b 方向
            force_a = [0.0, 0.0, 0.0]
            force_a[test_axis_a] = force_magnitude
            force_b = [0.0, 0.0, 0.0]
            force_b[test_axis_b] = force_magnitude
            
            # ===== Test A：沿 axis_a 方向施力 =====
            axis_a_name = axis_names[test_axis_a]
            axis_b_name = axis_names[test_axis_b]
            print(f"DEBUG: Test A: Applying force in +{axis_a_name} direction...")
            
            # 暂停
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (symmetry test A)")
            time_module.sleep(0.3)
            
            # 施加 axis_a 方向力
            self.clear_model_wrench(model_name)
            force_cmd_a = self.func_apply_model_force(
                model_name=model_name,
                force_x=force_a[0],
                force_y=force_a[1],
                force_z=force_a[2],
                persistent=True
            )
            if force_cmd_a:
                force_cmd_a.execute(self.experiment_log)
                self.log_sleep(0.1, f"Wait for {axis_a_name}-force to be applied")
                time_module.sleep(0.1)
            else:
                print(f"DEBUG: Warning - force_cmd for axis {axis_a_name} is None")
                return None
            
            # 推进仿真
            print(f"Running test A ({axis_a_name}-force) for {test_duration:.2f}s ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取 axis_a 方向力后的位姿
            pos_after_a = self.get_model_pose_from_scene(model_name)
            if pos_after_a is None:
                print(f"DEBUG: Failed to get position after {axis_a_name}-force")
                return None
            print(f"Position after {axis_a_name}-force: {pos_after_a}")
            
            # 计算 axis_a 方向位移
            da = pos_after_a[test_axis_a] - initial_pos[test_axis_a]
            da_mag = abs(da)
            print(f"{axis_a_name}-displacement: {da:.4f} m (|d{axis_a_name}|={da_mag:.4f})")
            
            # 检查最小位移
            MIN_DISP = 0.01
            if da_mag < MIN_DISP:
                print(f"DEBUG: Model barely moved in {axis_a_name} ({da_mag:.6f} m). Skipping.")
                self.clear_model_wrench(model_name)
                return None
            
            # ===== 重置到初始状态 =====
            print("DEBUG: Resetting for symmetry Test B...")
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait after clearing wrench (symmetry reset)")
            time_module.sleep(0.2)
            
            self.reset_simulation()
            self.log_sleep(1.0, "Wait for reset to complete (symmetry)")
            time_module.sleep(1.0)
            
            self.set_model_pose(model_name, initial_pos[0], initial_pos[1], initial_pos[2],
                                initial_pos[3], initial_pos[4], initial_pos[5], initial_pos[6])
            self.log_sleep(0.5, "Wait after setting model pose (symmetry)")
            time_module.sleep(0.5)
            
            # 验证重置
            pos_after_reset = self.get_model_pose_from_scene(model_name)
            if pos_after_reset is not None:
                print(f"Position after reset: {pos_after_reset}")
            
            # ===== Test B：沿 axis_b 方向施力（相同大小） =====
            print(f"DEBUG: Test B: Applying force in +{axis_b_name} direction...")
            
            # 暂停
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (symmetry test B)")
            time_module.sleep(0.3)
            
            # 施加 axis_b 方向力（大小与 Test A 完全相同）
            self.clear_model_wrench(model_name)
            force_cmd_b = self.func_apply_model_force(
                model_name=model_name,
                force_x=force_b[0],
                force_y=force_b[1],
                force_z=force_b[2],
                persistent=True
            )
            if force_cmd_b:
                force_cmd_b.execute(self.experiment_log)
                self.log_sleep(0.1, f"Wait for {axis_b_name}-force to be applied")
                time_module.sleep(0.1)
            else:
                print(f"DEBUG: Warning - force_cmd for axis {axis_b_name} is None")
                return None
            
            # 推进仿真（相同步数）
            print(f"Running test B ({axis_b_name}-force) for {test_duration:.2f}s ({num_steps} steps)...")
            self.step_simulation(num_steps)
            
            # 获取 axis_b 方向力后的位姿
            pos_after_b = self.get_model_pose_from_scene(model_name)
            if pos_after_b is None:
                print(f"DEBUG: Failed to get position after {axis_b_name}-force")
                return None
            print(f"Position after {axis_b_name}-force: {pos_after_b}")
            
            # 计算 axis_b 方向位移
            db = pos_after_b[test_axis_b] - initial_pos[test_axis_b]
            db_mag = abs(db)
            print(f"{axis_b_name}-displacement: {db:.4f} m (|d{axis_b_name}|={db_mag:.4f})")
            
            # 检查最小位移
            if db_mag < MIN_DISP:
                print(f"DEBUG: Model barely moved in {axis_b_name} ({db_mag:.6f} m). Skipping.")
                self.clear_model_wrench(model_name)
                return None
            
            # 清除力
            self.clear_model_wrench(model_name)
            
            # ===== 比较结果 =====
            # 蜕变关系：|da| ≈ |db|（物理空间各向同性）
            max_disp = max(da_mag, db_mag)
            disp_diff = abs(da_mag - db_mag)
            relative_error = disp_diff / max_disp if max_disp > 0.001 else 0.0
            
            # 附加检查：第三轴位移应相同（重力等外部效果一致）
            third_axis = [i for i in range(3) if i != test_axis_a and i != test_axis_b][0]
            third_name = axis_names[third_axis]
            dthird_a = pos_after_a[third_axis] - initial_pos[third_axis]
            dthird_b = pos_after_b[third_axis] - initial_pos[third_axis]
            dthird_diff = abs(dthird_a - dthird_b)
            
            # 附加检查：交叉轴位移（测试 A 在 axis_b 上的位移，测试 B 在 axis_a 上的位移）
            cross_a = abs(pos_after_a[test_axis_b] - initial_pos[test_axis_b])  # A 测试中 axis_b 位移
            cross_b = abs(pos_after_b[test_axis_a] - initial_pos[test_axis_a])  # B 测试中 axis_a 位移
            
            # 判定：相对误差 < 20% 或 绝对差 < 0.5m
            abs_threshold = 0.5
            rel_threshold = 0.20
            abs_pass = disp_diff < abs_threshold
            rel_pass = relative_error < rel_threshold
            success = abs_pass or rel_pass
            
            # 为了保持返回值兼容性，使用 dx/dy 变量名（映射到实际测试轴）
            dx = da
            dy = db
            pos_after_x = pos_after_a
            pos_after_y = pos_after_b
            
            error_info = f"Gravity: ({gx}, {gy}, {gz})\n"
            error_info += f"Test axes: {axis_a_name} vs {axis_b_name}\n"
            error_info += f"Force magnitude: {force_magnitude:.2f} N\n"
            error_info += f"{axis_a_name}-direction displacement (d{axis_a_name}): {da:.4f} m (|d{axis_a_name}|={da_mag:.4f})\n"
            error_info += f"{axis_b_name}-direction displacement (d{axis_b_name}): {db:.4f} m (|d{axis_b_name}|={db_mag:.4f})\n"
            error_info += f"Displacement difference: {disp_diff:.4f} m\n"
            error_info += f"Relative error: {relative_error*100:.2f}%\n"
            error_info += f"{third_name}-displacement (test A): {dthird_a:.4f} m\n"
            error_info += f"{third_name}-displacement (test B): {dthird_b:.4f} m\n"
            error_info += f"{third_name}-displacement difference: {dthird_diff:.4f} m\n"
            error_info += f"Cross-axis {axis_b_name}-disp (test A): {cross_a:.4f} m\n"
            error_info += f"Cross-axis {axis_a_name}-disp (test B): {cross_b:.4f} m\n"
            error_info += f"Threshold: abs < {abs_threshold} m OR rel < {rel_threshold*100:.0f}%\n"
            error_info += f"Test Duration: {test_duration:.2f} s, Steps: {num_steps}\n"
            error_info += f"Model mass: {model_mass:.2f} kg\n"
            error_info += f"Note: Physics should be isotropic - same force magnitude in {axis_a_name} and {axis_b_name} should produce same displacement magnitude."
            
            print(f"Symmetry comparison: |d{axis_a_name}|={da_mag:.4f}, |d{axis_b_name}|={db_mag:.4f}, "
                  f"diff={disp_diff:.4f}, rel_error={relative_error*100:.2f}%")
            print(f"Test {'PASSED' if success else 'FAILED'} (abs_pass={abs_pass}, rel_pass={rel_pass})")
            
            return (model_name, initial_pos, pos_after_x, pos_after_y,
                    dx, dy, force_magnitude, success, error_info)
            
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_symmetry: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 零输入稳定性测试 (Zero-Input Stability Test)
    # 范式 B: Single-Run Invariant — 无力、无 reset、无对比运行
    # ===================================================================
    def metamorphic_test_zero_input_stability(self, test_duration=5.0):
        """
        蜕变测试：零输入稳定性测试
        
        测试原理：不施加任何外力，仅推进仿真，检查模型是否保持静止。
        任何在水平方向（x, y）的显著位移都表明存在幽灵力、能量注入或
        插件初始化 bug。
        
        蜕变关系：
          No force → |Δx| ≈ 0 AND |Δy| ≈ 0
          （允许 z 轴变化，因为重力可能导致模型下沉/稳定）
        
        范式：B — Single-Run Invariant（不需要 reset、不需要施力、不需要对比运行）
        
        Returns:
            (model_name, initial_pos, final_pos, drift_x, drift_y, drift_z,
             success, error_info) 或 None
        """
        try:
            import time as time_module
            import math
            
            print("=" * 60)
            print("METAMORPHIC TEST: Zero-Input Stability (Paradigm B)")
            print("=" * 60)
            
            # 1. 获取场景和可测试模型
            scene, reserved_models = self.get_scene()
            if scene is None or not reserved_models:
                print("DEBUG: get_scene() failed in zero-input stability test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("DEBUG: No available models for zero-input stability test")
                return None
            
            model = random.choice(available_models)
            model_name = model.name
            print(f"Selected model: {model_name}")
            
            # 2. 获取初始位置
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: ({initial_pos[0]:.4f}, {initial_pos[1]:.4f}, {initial_pos[2]:.4f})")
            
            # 3. 暂停仿真（确保仿真是暂停的）
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (zero-input stability)")
            time_module.sleep(0.3)
            
            # 4. 不施加任何力！直接推进仿真
            num_steps = int(test_duration / 0.001)
            print(f"Advancing simulation for {test_duration:.1f}s ({num_steps} steps) with NO force applied...")
            self.step_simulation(num_steps)
            
            # 5. 获取最终位置
            final_pos = self.get_model_pose_from_scene(model_name)
            if final_pos is None:
                print(f"DEBUG: Failed to get final position for model {model_name}")
                return None
            print(f"Final position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})")
            
            # 6. 计算各轴漂移
            drift_x = final_pos[0] - initial_pos[0]
            drift_y = final_pos[1] - initial_pos[1]
            drift_z = final_pos[2] - initial_pos[2]
            drift_horizontal = math.sqrt(drift_x**2 + drift_y**2)
            
            print(f"Drift: x={drift_x:.6f}m, y={drift_y:.6f}m, z={drift_z:.6f}m")
            print(f"Horizontal drift: {drift_horizontal:.6f}m")
            
            # 7. 判断通过/失败
            # 水平方向容差：0.05m（考虑模型可能因重力而微小移动/稳定）
            HORIZONTAL_TOLERANCE = 0.05
            success = drift_horizontal < HORIZONTAL_TOLERANCE
            
            # 构建错误信息
            error_info = f"Model: {model_name}\n"
            error_info += f"Test duration: {test_duration:.1f}s ({num_steps} steps)\n"
            error_info += f"Initial position: ({initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f})\n"
            error_info += f"Final position: ({final_pos[0]:.6f}, {final_pos[1]:.6f}, {final_pos[2]:.6f})\n"
            error_info += f"Drift: x={drift_x:.6f}m, y={drift_y:.6f}m, z={drift_z:.6f}m\n"
            error_info += f"Horizontal drift: {drift_horizontal:.6f}m\n"
            error_info += f"Horizontal tolerance: {HORIZONTAL_TOLERANCE}m\n"
            error_info += f"Note: No force was applied. Any horizontal drift indicates phantom forces,\n"
            error_info += f"      energy injection, or plugin initialization bugs."
            
            print(f"Test {'PASSED' if success else 'FAILED'} "
                  f"(horizontal drift {drift_horizontal:.6f}m vs tolerance {HORIZONTAL_TOLERANCE}m)")
            
            return (model_name, initial_pos, final_pos, drift_x, drift_y, drift_z,
                    success, error_info)
        
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_zero_input_stability: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 多模型力隔离测试 (Multi-Model Force Isolation Test)
    # 范式 C: Same-Run Multi-Model — 无 reset，同一运行中比较不同模型
    # ===================================================================
    def metamorphic_test_force_isolation(self, test_duration=5.0):
        """
        蜕变测试：多模型力隔离测试
        
        测试原理：仅对一个模型施力，检查远处的另一个模型是否不受影响。
        如果旁观者模型在水平方向产生显著位移，说明存在力泄漏、
        实体路由错误或插件间干扰。
        
        蜕变关系：
          Force on M_target only → M_bystander should not move horizontally
        
        范式：C — Same-Run Multi-Model（不需要 reset，同一运行中比较两个模型）
        
        Returns:
            (target_name, bystander_name, target_initial, bystander_initial,
             target_final, bystander_final, target_displacement, bystander_drift,
             force_magnitude, success, error_info) 或 None
        """
        try:
            import time as time_module
            import math
            
            print("=" * 60)
            print("METAMORPHIC TEST: Force Isolation (Paradigm C)")
            print("=" * 60)
            
            # 1. 获取场景和可测试模型
            scene, reserved_models = self.get_scene()
            if scene is None or not reserved_models:
                print("DEBUG: get_scene() failed in force isolation test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if len(available_models) < 2:
                print(f"DEBUG: Need at least 2 testable models, found {len(available_models)}. Skipping.")
                return None
            
            # 2. 选择两个模型并检查它们的距离
            random.shuffle(available_models)
            target_model = None
            bystander_model = None
            
            for i in range(len(available_models)):
                for j in range(i + 1, len(available_models)):
                    m_a = available_models[i]
                    m_b = available_models[j]
                    # 检查距离
                    pos_a = self.get_model_pose_from_scene(m_a.name)
                    pos_b = self.get_model_pose_from_scene(m_b.name)
                    if pos_a is None or pos_b is None:
                        continue
                    dist = math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)
                    # 需要足够远（> 2m）以避免碰撞传播
                    if dist > 2.0:
                        target_model = m_a
                        bystander_model = m_b
                        break
                if target_model is not None:
                    break
            
            if target_model is None or bystander_model is None:
                print("DEBUG: No suitable model pair found (need > 2m apart). Skipping.")
                return None
            
            target_name = target_model.name
            bystander_name = bystander_model.name
            print(f"Target model: {target_name}")
            print(f"Bystander model: {bystander_name}")
            
            # 3. 记录初始位置
            target_initial = self.get_model_pose_from_scene(target_name)
            bystander_initial = self.get_model_pose_from_scene(bystander_name)
            if target_initial is None or bystander_initial is None:
                print("DEBUG: Failed to get initial positions")
                return None
            
            dist = math.sqrt((target_initial[0] - bystander_initial[0])**2 +
                           (target_initial[1] - bystander_initial[1])**2)
            print(f"Target initial: ({target_initial[0]:.4f}, {target_initial[1]:.4f}, {target_initial[2]:.4f})")
            print(f"Bystander initial: ({bystander_initial[0]:.4f}, {bystander_initial[1]:.4f}, {bystander_initial[2]:.4f})")
            print(f"Distance between models: {dist:.2f}m")
            
            # 4. 暂停仿真
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (force isolation)")
            time_module.sleep(0.3)
            
            # 5. 仅对 target 施力
            model_mass = self.get_model_mass(target_name)
            if model_mass is None or model_mass <= 0:
                model_mass = 1.0
            
            # 生成一个足够大的水平力
            min_acceleration = 5.0
            max_acceleration = 50.0
            min_force = max(model_mass * min_acceleration, 50.0)
            max_force = model_mass * max_acceleration
            force_magnitude = random.uniform(min_force, max_force)
            
            # 随机选择 x 或 y 方向（远离 bystander 的方向更好）
            force_x = force_magnitude
            force_y = 0.0
            force_z = 0.0
            
            print(f"Applying force ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N to {target_name} ONLY")
            
            self.clear_model_wrench(target_name)
            self.clear_model_wrench(bystander_name)  # 确保 bystander 也没有残留力
            
            force_cmd = self.func_apply_model_force(
                model_name=target_name,
                force_x=force_x, force_y=force_y, force_z=force_z,
                persistent=True
            )
            if force_cmd:
                force_cmd.execute(self.experiment_log)
                self.log_sleep(0.1, "Wait for force to be applied (force isolation)")
                time_module.sleep(0.1)
            else:
                print("DEBUG: Warning - force_cmd is None in force isolation test")
                return None
            
            # 6. 推进仿真
            num_steps = int(test_duration / 0.001)
            print(f"Stepping simulation {num_steps} steps ({test_duration:.1f}s)...")
            self.step_simulation(num_steps)
            
            # 7. 获取最终位置
            target_final = self.get_model_pose_from_scene(target_name)
            bystander_final = self.get_model_pose_from_scene(bystander_name)
            if target_final is None or bystander_final is None:
                print("DEBUG: Failed to get final positions")
                self.clear_model_wrench(target_name)
                return None
            
            self.clear_model_wrench(target_name)
            
            # 8. 计算位移
            target_displacement = math.sqrt(
                (target_final[0] - target_initial[0])**2 +
                (target_final[1] - target_initial[1])**2
            )
            bystander_drift_x = bystander_final[0] - bystander_initial[0]
            bystander_drift_y = bystander_final[1] - bystander_initial[1]
            bystander_drift = math.sqrt(bystander_drift_x**2 + bystander_drift_y**2)
            
            print(f"Target displacement (horizontal): {target_displacement:.4f}m")
            print(f"Bystander drift (horizontal): {bystander_drift:.6f}m "
                  f"(x={bystander_drift_x:.6f}, y={bystander_drift_y:.6f})")
            
            # 9. Sanity check：target 必须有显著位移
            if target_displacement < 0.01:
                print("DEBUG: Target barely moved. Force may not have been applied. Skipping.")
                return None
            
            # 10. 判断通过/失败
            BYSTANDER_TOLERANCE = 0.05  # 旁观者水平漂移容差
            success = bystander_drift < BYSTANDER_TOLERANCE
            
            error_info = f"Target model: {target_name}\n"
            error_info += f"Bystander model: {bystander_name}\n"
            error_info += f"Inter-model distance: {dist:.2f}m\n"
            error_info += f"Force on target: ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N\n"
            error_info += f"Test duration: {test_duration:.1f}s ({num_steps} steps)\n"
            error_info += f"Target displacement (horizontal): {target_displacement:.4f}m\n"
            error_info += f"Bystander drift (horizontal): {bystander_drift:.6f}m\n"
            error_info += f"  Bystander drift x: {bystander_drift_x:.6f}m\n"
            error_info += f"  Bystander drift y: {bystander_drift_y:.6f}m\n"
            error_info += f"Bystander tolerance: {BYSTANDER_TOLERANCE}m\n"
            error_info += f"Note: Force was applied ONLY to {target_name}. "
            error_info += f"Any significant bystander movement indicates force leakage or targeting bug."
            
            print(f"Test {'PASSED' if success else 'FAILED'} "
                  f"(bystander drift {bystander_drift:.6f}m vs tolerance {BYSTANDER_TOLERANCE}m)")
            
            return (target_name, bystander_name, target_initial, bystander_initial,
                    target_final, bystander_final, target_displacement, bystander_drift,
                    force_magnitude, success, error_info)
        
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_force_isolation: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 撤力响应测试 (Force Removal Response Test)
    # 范式 D: Sequential No-Reset — 同一运行分阶段，无 reset
    # ===================================================================
    def metamorphic_test_force_removal(self, force_duration=2.0, coast_duration=2.0):
        """
        蜕变测试：撤力响应测试
        
        测试原理：先施加持续力，然后撤除力，检查撤力后模型是否停止加速。
        如果撤力后速度仍在增加，说明 clear_model_wrench 未正确生效，
        或存在力持续性泄漏 bug。
        
        蜕变关系：
          Remove persistent force → acceleration in force direction must become <= 0
        
        范式：D — Sequential No-Reset（同一运行中两个连续阶段，不使用 reset）
        
        Returns:
            (model_name, initial_pos, pos_after_force, pos_coast_mid, pos_coast_end,
             v_force, v_coast1, v_coast2, force_magnitude, success, error_info) 或 None
        """
        try:
            import time as time_module
            import math
            
            print("=" * 60)
            print("METAMORPHIC TEST: Force Removal Response (Paradigm D)")
            print("=" * 60)
            
            # 1. 获取场景和模型
            scene, reserved_models = self.get_scene()
            if scene is None or not reserved_models:
                print("DEBUG: get_scene() failed in force removal test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("DEBUG: No available models for force removal test")
                return None
            
            model = random.choice(available_models)
            model_name = model.name
            print(f"Selected model: {model_name}")
            
            # 2. 获取初始位置
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: ({initial_pos[0]:.4f}, {initial_pos[1]:.4f}, {initial_pos[2]:.4f})")
            
            # 3. 计算力
            model_mass = self.get_model_mass(model_name)
            if model_mass is None or model_mass <= 0:
                model_mass = 1.0
            
            min_acceleration = 5.0
            max_acceleration = 30.0
            min_force = max(model_mass * min_acceleration, 50.0)
            max_force = model_mass * max_acceleration
            force_magnitude = random.uniform(min_force, max_force)
            
            # 使用 +x 方向的力
            force_x = force_magnitude
            force_y = 0.0
            force_z = 0.0
            
            # 4. 暂停仿真
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (force removal)")
            time_module.sleep(0.3)
            
            # ===== Phase 1: 施力阶段 =====
            print(f"\n--- Phase 1: Applying force ({force_x:.2f}, 0, 0) N for {force_duration:.1f}s ---")
            
            self.clear_model_wrench(model_name)
            force_cmd = self.func_apply_model_force(
                model_name=model_name,
                force_x=force_x, force_y=force_y, force_z=force_z,
                persistent=True
            )
            if force_cmd:
                force_cmd.execute(self.experiment_log)
                self.log_sleep(0.1, "Wait for force to be applied (force removal)")
                time_module.sleep(0.1)
            else:
                print("DEBUG: Warning - force_cmd is None in force removal test")
                return None
            
            force_steps = int(force_duration / 0.001)
            print(f"Stepping {force_steps} steps with force applied...")
            self.step_simulation(force_steps)
            
            pos_after_force = self.get_model_pose_from_scene(model_name)
            if pos_after_force is None:
                print("DEBUG: Failed to get position after force phase")
                self.clear_model_wrench(model_name)
                return None
            print(f"Position after force phase: ({pos_after_force[0]:.4f}, {pos_after_force[1]:.4f}, {pos_after_force[2]:.4f})")
            
            # 检查 target 是否有位移
            force_disp = pos_after_force[0] - initial_pos[0]
            if abs(force_disp) < 0.01:
                print("DEBUG: Model barely moved during force phase. Skipping.")
                self.clear_model_wrench(model_name)
                return None
            
            # ===== Phase 2: 撤力并滑行 =====
            print(f"\n--- Phase 2: Removing force, coasting for {coast_duration:.1f}s ---")
            
            self.clear_model_wrench(model_name)
            self.log_sleep(0.2, "Wait for wrench clear")
            time_module.sleep(0.2)
            
            # 滑行阶段分两半，用于检测速度变化趋势
            coast_half_steps = int((coast_duration / 2.0) / 0.001)
            coast_half_time = coast_duration / 2.0
            
            print(f"Coast phase 1: {coast_half_steps} steps ({coast_half_time:.1f}s)...")
            self.step_simulation(coast_half_steps)
            
            pos_coast_mid = self.get_model_pose_from_scene(model_name)
            if pos_coast_mid is None:
                print("DEBUG: Failed to get position at coast midpoint")
                return None
            print(f"Position at coast midpoint: ({pos_coast_mid[0]:.4f}, {pos_coast_mid[1]:.4f}, {pos_coast_mid[2]:.4f})")
            
            print(f"Coast phase 2: {coast_half_steps} steps ({coast_half_time:.1f}s)...")
            self.step_simulation(coast_half_steps)
            
            pos_coast_end = self.get_model_pose_from_scene(model_name)
            if pos_coast_end is None:
                print("DEBUG: Failed to get position at coast end")
                return None
            print(f"Position at coast end: ({pos_coast_end[0]:.4f}, {pos_coast_end[1]:.4f}, {pos_coast_end[2]:.4f})")
            
            # ===== 速度估算和判断 =====
            # 使用 x 方向（施力方向）估算速度
            v_force = (pos_after_force[0] - initial_pos[0]) / force_duration
            v_coast1 = (pos_coast_mid[0] - pos_after_force[0]) / coast_half_time
            v_coast2 = (pos_coast_end[0] - pos_coast_mid[0]) / coast_half_time
            
            print(f"\nVelocity estimates (x-direction):")
            print(f"  v_force (avg during force phase): {v_force:.4f} m/s")
            print(f"  v_coast1 (1st half of coast):     {v_coast1:.4f} m/s")
            print(f"  v_coast2 (2nd half of coast):     {v_coast2:.4f} m/s")
            
            # 判断逻辑：
            # 1. 滑行速度不应显著超过施力阶段的平均速度
            #    (施力阶段平均速度 < 最终瞬时速度，所以允许一些裕量)
            #    v_coast1 <= v_force * 2.5 (施力结束时瞬时速度 ≈ 2*平均速度)
            # 2. 滑行第二段速度不应超过第一段（不应加速）
            #    v_coast2 <= v_coast1 * 1.1 (允许 10% 浮点误差)
            
            # 主要检查：撤力后不应持续加速
            coast_acceleration = (v_coast2 - v_coast1) / coast_half_time
            
            # 失败条件：滑行阶段仍在明显加速（> 1 m/s²）
            # 撤力后允许微小的数值波动，但不应有持续的正加速度
            MAX_COAST_ACCELERATION = 1.0  # m/s²
            success = coast_acceleration < MAX_COAST_ACCELERATION
            
            # 额外检查：如果滑行速度远超施力期望值，也标记失败
            if v_force > 0 and v_coast1 > v_force * 3.0:
                success = False
            
            error_info = f"Model: {model_name}\n"
            error_info += f"Force: ({force_x:.2f}, {force_y:.2f}, {force_z:.2f}) N\n"
            error_info += f"Force duration: {force_duration:.1f}s, Coast duration: {coast_duration:.1f}s\n"
            error_info += f"Positions:\n"
            error_info += f"  Initial:     ({initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f})\n"
            error_info += f"  After force: ({pos_after_force[0]:.6f}, {pos_after_force[1]:.6f}, {pos_after_force[2]:.6f})\n"
            error_info += f"  Coast mid:   ({pos_coast_mid[0]:.6f}, {pos_coast_mid[1]:.6f}, {pos_coast_mid[2]:.6f})\n"
            error_info += f"  Coast end:   ({pos_coast_end[0]:.6f}, {pos_coast_end[1]:.6f}, {pos_coast_end[2]:.6f})\n"
            error_info += f"Velocity estimates (x-direction):\n"
            error_info += f"  v_force:  {v_force:.6f} m/s (avg during force phase)\n"
            error_info += f"  v_coast1: {v_coast1:.6f} m/s (1st half of coast)\n"
            error_info += f"  v_coast2: {v_coast2:.6f} m/s (2nd half of coast)\n"
            error_info += f"Coast acceleration: {coast_acceleration:.6f} m/s²\n"
            error_info += f"Max allowed coast acceleration: {MAX_COAST_ACCELERATION} m/s²\n"
            error_info += f"Note: After clearing persistent force, the model should decelerate (with friction)\n"
            error_info += f"      or maintain constant velocity (frictionless). It should NEVER accelerate."
            
            print(f"\nCoast acceleration: {coast_acceleration:.4f} m/s² (max allowed: {MAX_COAST_ACCELERATION})")
            print(f"Test {'PASSED' if success else 'FAILED'}")
            
            return (model_name, initial_pos, pos_after_force, pos_coast_mid, pos_coast_end,
                    v_force, v_coast1, v_coast2, force_magnitude, success, error_info)
        
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_force_removal: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ===================================================================
    # 时序单调性测试 (Temporal Monotonicity Test)
    # 范式 B: Single-Run Invariant — 检查轨迹形状的物理不变量
    # ===================================================================
    def metamorphic_test_temporal_monotonicity(self, test_duration=5.0, num_samples=10):
        """
        蜕变测试：时序单调性测试
        
        测试原理：施加恒定力，在多个时间点采样位移。力方向上的位移
        必须随时间单调递增。如果出现位移回退或突变，说明存在求解器不稳定、
        力间歇性失效或碰撞传送 bug。
        
        蜕变关系：
          Constant force in +x → x(t1) < x(t2) < ... < x(tK) for t1 < t2 < ... < tK
        
        范式：B — Single-Run Invariant（多时间点采样检查轨迹不变量）
        
        Returns:
            (model_name, initial_pos, trajectory, force_magnitude,
             monotonic, smooth, violations, success, error_info) 或 None
        """
        try:
            import time as time_module
            import math
            
            print("=" * 60)
            print("METAMORPHIC TEST: Temporal Monotonicity (Paradigm B)")
            print("=" * 60)
            
            # 1. 获取场景和模型
            scene, reserved_models = self.get_scene()
            if scene is None or not reserved_models:
                print("DEBUG: get_scene() failed in temporal monotonicity test")
                return None
            
            available_models = self.get_testable_models(scene, reserved_models)
            if not available_models:
                print("DEBUG: No available models for temporal monotonicity test")
                return None
            
            model = random.choice(available_models)
            model_name = model.name
            print(f"Selected model: {model_name}")
            
            # 2. 获取初始位置
            initial_pos = self.get_model_pose_from_scene(model_name)
            if initial_pos is None:
                print(f"DEBUG: Failed to get initial position for model {model_name}")
                return None
            print(f"Initial position: ({initial_pos[0]:.4f}, {initial_pos[1]:.4f}, {initial_pos[2]:.4f})")
            
            # 3. 计算力
            model_mass = self.get_model_mass(model_name)
            if model_mass is None or model_mass <= 0:
                model_mass = 1.0
            
            min_acceleration = 5.0
            max_acceleration = 30.0
            min_force = max(model_mass * min_acceleration, 50.0)
            max_force = model_mass * max_acceleration
            force_magnitude = random.uniform(min_force, max_force)
            
            # 使用 +x 方向
            force_x = force_magnitude
            force_y = 0.0
            force_z = 0.0
            
            # 4. 暂停仿真
            pause_cmd_txt = (f"gz service -s /world/{self.world_name}/control "
                           f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
                           f"--timeout {self.timeout} --req 'pause: true'")
            pause_cmd = GzCommand(GzCommandType.SERVICE, [pause_cmd_txt], True)
            pause_cmd.execute(self.experiment_log)
            self.log_sleep(0.3, "Wait for pause (monotonicity)")
            time_module.sleep(0.3)
            
            # 5. 施加力
            self.clear_model_wrench(model_name)
            force_cmd = self.func_apply_model_force(
                model_name=model_name,
                force_x=force_x, force_y=force_y, force_z=force_z,
                persistent=True
            )
            if force_cmd:
                force_cmd.execute(self.experiment_log)
                self.log_sleep(0.1, "Wait for force to be applied (monotonicity)")
                time_module.sleep(0.1)
            else:
                print("DEBUG: Warning - force_cmd is None in monotonicity test")
                return None
            
            # 6. 分段推进并采样
            steps_per_sample = int(test_duration / (num_samples * 0.001))
            time_per_sample = test_duration / num_samples
            
            trajectory = []  # [(time, x, y, z), ...]
            trajectory.append((0.0, initial_pos[0], initial_pos[1], initial_pos[2]))
            
            print(f"\nApplying force ({force_x:.2f}, 0, 0) N, sampling {num_samples} points over {test_duration:.1f}s")
            print(f"Steps per sample: {steps_per_sample} ({time_per_sample:.2f}s)")
            
            for i in range(num_samples):
                self.step_simulation(steps_per_sample)
                pos = self.get_model_pose_from_scene(model_name)
                if pos is None:
                    print(f"DEBUG: Failed to get position at sample {i+1}")
                    self.clear_model_wrench(model_name)
                    return None
                t = time_per_sample * (i + 1)
                trajectory.append((t, pos[0], pos[1], pos[2]))
                x_disp = pos[0] - initial_pos[0]
                print(f"  t={t:.2f}s: x_disp={x_disp:.4f}m")
            
            self.clear_model_wrench(model_name)
            
            # 7. 检查单调性
            x_displacements = [pt[1] - initial_pos[0] for pt in trajectory]
            
            # 第一个检查：最终位移需要足够大（排除被约束的模型）
            final_disp = x_displacements[-1]
            if abs(final_disp) < 0.01:
                print("DEBUG: Model barely moved. May be constrained. Skipping.")
                return None
            
            monotonic = True
            violations = []
            
            for i in range(1, len(x_displacements)):
                if x_displacements[i] <= x_displacements[i - 1]:
                    monotonic = False
                    violations.append({
                        'index': i,
                        'time': trajectory[i][0],
                        'disp_prev': x_displacements[i - 1],
                        'disp_curr': x_displacements[i],
                        'type': 'non_monotonic'
                    })
            
            # 8. 检查平滑性（无突变/传送）
            smooth = True
            deltas = []
            for i in range(1, len(x_displacements)):
                deltas.append(x_displacements[i] - x_displacements[i - 1])
            
            for i in range(1, len(deltas)):
                if deltas[i - 1] > 0.001:  # 只在增量有意义时检查
                    ratio = deltas[i] / deltas[i - 1]
                    if ratio > 3.0:  # 增量突然变为前一个的 3 倍以上
                        smooth = False
                        violations.append({
                            'index': i + 1,
                            'time': trajectory[i + 1][0],
                            'delta_prev': deltas[i - 1],
                            'delta_curr': deltas[i],
                            'ratio': ratio,
                            'type': 'jump'
                        })
            
            # 9. 判断通过/失败
            success = monotonic and smooth
            
            error_info = f"Model: {model_name}\n"
            error_info += f"Force: ({force_x:.2f}, 0, 0) N\n"
            error_info += f"Test duration: {test_duration:.1f}s, Samples: {num_samples}\n"
            error_info += f"Final x-displacement: {final_disp:.6f}m\n"
            error_info += f"Monotonic: {'Yes' if monotonic else 'No'}\n"
            error_info += f"Smooth: {'Yes' if smooth else 'No'}\n"
            
            if violations:
                error_info += f"Violations ({len(violations)}):\n"
                for v in violations:
                    if v['type'] == 'non_monotonic':
                        error_info += (f"  t={v['time']:.2f}s: displacement DECREASED from "
                                     f"{v['disp_prev']:.6f}m to {v['disp_curr']:.6f}m\n")
                    elif v['type'] == 'jump':
                        error_info += (f"  t={v['time']:.2f}s: displacement JUMPED by ratio "
                                     f"{v['ratio']:.2f}x (delta: {v['delta_prev']:.6f} -> {v['delta_curr']:.6f})\n")
            
            error_info += f"\nTrajectory (x-displacement):\n"
            for i, pt in enumerate(trajectory):
                error_info += f"  t={pt[0]:.2f}s: x_disp={x_displacements[i]:.6f}m\n"
            
            error_info += f"\nNote: With constant force in +x, x-displacement should increase monotonically\n"
            error_info += f"      and smoothly over time. Any reversal or sudden jump indicates a bug."
            
            print(f"\nMonotonic: {'Yes' if monotonic else 'No'}, Smooth: {'Yes' if smooth else 'No'}")
            print(f"Violations: {len(violations)}")
            print(f"Test {'PASSED' if success else 'FAILED'}")
            
            return (model_name, initial_pos, trajectory, force_magnitude,
                    monotonic, smooth, violations, success, error_info)
        
        except Exception as e:
            print(f"DEBUG: Exception in metamorphic_test_temporal_monotonicity: {e}")
            import traceback
            traceback.print_exc()
            return None

def DEBUG_PRINT():
    # Coverage disabled, BUILD_DIR not available
    # print("BUILD DIR = " + BUILD_DIR)
    print(type(StringMsg))

if __name__ == "__main__":
    exp_dir = "/tmp/exp"
    if not os.path.exists("/tmp/exp"):
        os.mkdir("/tmp/exp")
    logging.basicConfig(level=logging.DEBUG, filename='/tmp/exp/log', filemode='a')
    logging.config.dictConfig({
        'version': 1,
        'disable_existing_loggers': True,
    })
    skipped = [
        # "/gazebo/resource_paths/resolve",
        # "/world/world_0/enable_collision",
        # "/world/world_0/disable_collision",
        # "/world/world_0/set_physics",
        # "/world/world_0/playback/control",
        "/server_control",
    ]
    parser = OptionParser()
    parser.add_option("-d", "--directory", dest="directory", default="exp", help="directory to store test cases")
    parser.add_option("-i", "--iteration", dest="iteration", type="int", default=10, help="max iteration")
    parser.add_option("-m", "--mode", dest="mode", help="one_shot or loop", default="loop")
    parser.add_option("-s", "--seed", dest="seed", type="int", default=0, help="seed for RNG")
    parser.add_option("-n", "--num-seq", dest="num_seq", type="int", default=10, help="number of gz commands")
    parser.add_option("-p", "--plugin", dest="plugin", action="store_true", help="enable mined plugin")
    parser.add_option("-t", "--timeout", dest="timeout", type="int", default=10000, help="timeout")
    parser.add_option("--enable-playback", dest="enable_playback", action="store_true", default=True, help="enable playback rewind test after each metamorphic test (default: True)")
    parser.add_option("--disable-playback", dest="enable_playback", action="store_false", help="disable playback rewind test after each metamorphic test")

    (options, args) = parser.parse_args()

    if options.seed:
        seed = options.seed
    else:
        seed = int(datetime.now().timestamp())

    print(seed)
    # Coverage disabled, BUILD_DIR not needed
    # BUILD_DIR = FIRST_DIR[DIR_FLAG] + "/build/"
    DEBUG_PRINT()
    
    with open("seed", "w") as f:
        f.write(f"seed: {seed}")

    # def __init__(self, sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0):

    # if options.plugin:
    #     sdf_miner = SdfMiner(PLUGIN_DIR)
    # else:
    #     sdf_miner = None
    start = datetime.now().timestamp()
    stop = False
    ### bandits = [ThompsomSampling(NUM_ARM) for i in range(options.num_seq)]
    # 输入有问题
    # mab = create_smab_bernoulli_mo_cold_start(action_ids=list(range(NUM_ARM)), n_objectives=3)
    # mab = create_smab_bernoulli_mo_cold_start(action_ids=[str(i) for i in range(NUM_ARM)], n_objectives=3)
    diversity = SdfDiversity("./models")
    crashes = set()
    i = 0
    ##### unit = SmithUnit(exp_dir, "a.sdf", options.num_seq, True, skipped, options.timeout, seed, None, mab, diversity, crashes) # bandits)
    ##### unit.create_sdf()
    ##### unit.pairwise_generate_and_test_commands()
    ##### unit.topic_fuzzing()
    # Coverage tracking disabled
    # cov_line_time = 0
    # cov_line_turn = 0
    turn = -1
    # while i < options.iteration:
    while not stop:
        now = datetime.now().timestamp()
        if (now - start) > 600 * turn:
            # Coverage file writing disabled
            # with open(f"{options.directory}cov_time.txt", "a") as file:
            #     file.write(f"time is {now - start}, cover line is {cov_line_time}\n")
            turn = (now - start) / 600
        if now - start >= 60 * 60 * 240:
            stop = True
        exp_dir = f"{options.directory}_{i}"
        
        # 每轮迭代前清理残留的 Gazebo 相关进程，防止进程泄漏导致后续实验失败
        try:
            subprocess.run("pkill -9 -f 'gz sim'", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run("pkill -9 ruby", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run("pkill -9 -f 'gz-sim-server'", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run("pkill -9 -f 'parameter_bridge'", shell=True, timeout=5,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1)  # 等待进程完全退出
        except:
            pass
        
        unit = SmithUnit(exp_dir, "a.sdf", options.num_seq, True, skipped, options.timeout, seed, None, None, diversity, crashes, enable_playback=options.enable_playback) # bandits)
        # unit.create_sdf()
        unit.copy_random_sdf()
        print("DEBUG: before generate_and_test_commands")
        print("id = " + str(i + 1))
        # Coverage tracking disabled
        # cov_line = unit.generate_and_test_commands()
        unit.generate_and_test_commands()
        # cov_line_time += cov_line
        # cov_line_turn += cov_line
        # with open(f"{options.directory}cov_turn.txt", "a") as file:
        #     file.write(f"turn is {i}, cover line is {cov_line_turn}\n")
        i += 1
        # very dirty, just try it for now
        subprocess.run("pkill -9 ruby", shell=True)



    print("end of servicesmith.py")
    # if options.mode == "one_shot":
    #     service_smith = ServiceSmith(skipped=skipped)
    #     service_smith.one_shot_fuzz(directory=options.directory, iteration=options.iteration)
    # elif options.mode == "loop":
    #     service_smith = ServiceSmith(skipped=skipped)
    #     service_smith.fuzz(directory=options.directory)
    # else:
    #     print(f"not implemented mode {options.mode} yet")

    # generate sdf
    # world = dump_sdf("world_0")
    # with open("b.sdf", "w") as f:
    #     f.write(world)
