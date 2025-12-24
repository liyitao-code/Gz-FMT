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

# 测试用的类
class SmithUnit:
    def __init__(self, directory="exp", sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0, sdf_miner=None, bandits=None, diversity=None, crashes=None):
        self.directory = directory
        self.sdf_name = sdf_name
        self.num_seq = num_seq
        self.gz_cmds = list()
        self.use_text = use_text
        self.node = Node()
        self.sdf_miner = sdf_miner
        self.plugin_miner = PluginMiner(FIRST_DIR[DIR_FLAG] + "/install/share/gz/gz-sim8/worlds/")
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
        # 0. collect coverage info
        print("DEBUG: before cov")
        # self.cov_old.collect()  # Coverage collection disabled
        # 1. run gz sim a.sdf and sleep for a few seconds, dirty
        # gz_sim = f"gz sim {self.directory}/{self.sdf_name} --seed {self.seed} -v 0 -r -s --headless-rendering"
        gz_sim = f"gz sim {self.directory}/{self.sdf_name} -r"
        print(f"DEBUG: gz_sim: {gz_sim}")
        print("DEBUG: before subprocess gz sim")
        try:
            process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        except:
            print("DEBUG: subprocess launch gz error")
            return 0

        process_status = psutil.Process(process.pid)
        time.sleep(5)

        try:
            result, response = self.get_world()
            self.world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return 0

        # 注意：使用 -r 参数启动的 Gazebo 已经自动运行，不需要额外调用 play_simulation()
        # 等待一段时间确保模拟器完全启动
        print("DEBUG: waiting for simulation to fully start...")
        time.sleep(2)  # 等待模拟器完全启动

        print("DEBUG: starting metamorphic test")
        
        # 随机选择一种蜕变测试关系
        test_type = random.choice(['motion', 'rewind'])  # 随机选择运动测试或回溯测试
        test_type = 'motion' # 先测试运动部分
        print(f"DEBUG: Selected metamorphic test type: {test_type}")
        
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
        
        # 记录测试结果
        test_passed = False
        if test_result:
            if test_type == 'motion':
                # 运动测试结果格式: (model_name, initial_pos, final_pos, expected_pos, success)
                model_name, initial_pos, final_pos, expected_pos, success = test_result
                test_passed = success
                
                # 保存测试结果到文件
                with open(f"{self.directory}/metamorphic_test_result.txt", "w") as f:
                    f.write(f"Test Type: Motion Test\n")
                    f.write(f"Model: {model_name}\n")
                    f.write(f"Initial Position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})\n")
                    f.write(f"Final Position: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})\n")
                    f.write(f"Expected Position: ({expected_pos[0]:.3f}, {expected_pos[1]:.3f}, {expected_pos[2]:.3f})\n")
                    f.write(f"Test Duration: {test_duration:.2f} s\n")
                    f.write(f"Result: {'PASSED' if success else 'FAILED'}\n")
                    
                    # 计算误差
                    error_x = abs(final_pos[0] - expected_pos[0])
                    error_y = abs(final_pos[1] - expected_pos[1])
                    error_z = abs(final_pos[2] - expected_pos[2])
                    f.write(f"Error: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}\n")
                    
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
        
        # 检查进程状态
        if process_status.status() == psutil.STATUS_ZOMBIE:
            print("DEBUG: gz process not alive")

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



        # 4. collect coverage
        try:
            os.remove(f"./terminate")
        except:
            print("DEBUG: exception removing terminate file")

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
        print(f"DEBUG: filenames: {filenames}")
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
        reserved_models = {"ground_model", "ceiling_model", "west_model", "east_model", "north_model", "south_model"}
        request = Empty()
        # request = safe_utf8_encode(request)
        result, response = node.request(service_name, request, Empty, Scene, self.timeout)
        return response, reserved_models

    def get_model_pose_from_topic(self, model_name):
        """
        通过 topic 获取模型的实时位置信息
        
        Args:
            model_name: 模型名称
        
        Returns:
            (x, y, z) 位置元组，如果获取失败返回 None
        """
        try:
            # 确保 world_name 已设置
            if not hasattr(self, 'world_name') or not self.world_name:
                result, response = self.get_world()
                if not result:
                    print("DEBUG: Cannot get world name")
                    return None
                self.world_name = response.data[0]
            
            # 使用 dynamic_pose/info topic 获取动态模型的实时位置
            # 如果模型不在 dynamic_pose 中，也可以尝试 pose/info
            topic_name = f"/world/{self.world_name}/dynamic_pose/info"
            
            # 使用 Node API 方式获取 topic 消息（最可靠的方法）
            node = Node()
            # 订阅 topic 并等待一条消息
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
            
            # 等待消息，最多等待 2 秒
            import time
            start_time = time.time()
            while not received and (time.time() - start_time) < 2.0:
                time.sleep(0.01)
            
            if not received or received_msg is None:
                print("DEBUG: No pose message received")
                return None
            
            # 在 Pose_V 消息中查找指定模型
            for pose in received_msg.pose:
                if pose.name == model_name:
                    return (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z
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

    def compare_models_state(self, state1, state2, position_tolerance=1e-6, orientation_tolerance=1e-6):
        """
        对比两个模型状态字典
        
        Args:
            state1: 第一个状态字典
            state2: 第二个状态字典
            position_tolerance: 位置容差（默认1e-6）
            orientation_tolerance: 角度容差（默认1e-6）
        
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
                
                time_module.sleep(0.1)
            
            # 获取当前模拟时间（用于验证）
            current_time = self.get_simulation_time()
            if current_time is None:
                print("DEBUG: Failed to get current simulation time")
                return None
            
            print(f"DEBUG: Current simulation time: {current_time[0]}.{current_time[1]:09d} s")
            
            # 2. 保存完整的模拟状态（包括所有模型的位置、速度等）
            print("DEBUG: Saving complete simulation state...")
            saved_state = self.save_simulation_state()
            if saved_state is None:
                print("DEBUG: Failed to save simulation state")
                return None
            
            # 同时记录模型位置用于对比（从 pose topic 获取）
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
                
                time_module.sleep(0.1)
            
            # 获取运行后的模拟时间
            time_after_run = self.get_simulation_time()
            if time_after_run is None:
                print("DEBUG: Failed to get simulation time after running")
                return None
            
            print(f"DEBUG: Simulation time after running: {time_after_run[0]}.{time_after_run[1]:09d} s")
            
            # 4. 回溯到 record_time_a 秒时的状态
            # 使用保存的完整状态来恢复模拟（这会恢复所有模型的位置、速度等）
            print(f"DEBUG: Restoring simulation state to saved state...")
            restore_success = self.restore_simulation_state(saved_state)
            if not restore_success:
                print("DEBUG: Failed to restore simulation state")
                return None
            
            # 等待一段时间让状态恢复完成并稳定
            time_module.sleep(1.0)
            
            # 5. 获取回溯后的状态
            print("DEBUG: Getting models state after rewind...")
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
        available_models = [m for m in scene.model if m.name not in reserved_models]
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
        available_models = [m for m in scene.model if m.name not in reserved_models]
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
        available_models = [m for m in scene.model if m.name not in reserved_models]
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
            available_models = [m for m in scene.model if m.name not in reserved_models]
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
            available_models = [m for m in scene.model if m.name not in reserved_models]
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

    # 蜕变测试：让模型沿x轴运动，然后验证位置
    def metamorphic_test_example(self, velocity_x=None, test_duration=5.0):
        """
        蜕变测试：随机选择一个模型，让它从当前位置沿x轴正方向运动，验证t秒后的位置
        
        预期：如果模型从初始位置(x0,y0,z0)开始，以v m/s的速度沿x轴运动t秒，
              那么最终位置应该是(x0+v*t, y0, z0)附近（考虑物理引擎的误差）
        
        Args:
            velocity_x: x轴方向的速度（m/s），如果为None则随机生成
            test_duration: 测试持续时间（秒）
        
        Returns:
            (model_name, initial_position, final_position, expected_position, success) 或 None
        """
        # 1. 获取场景并随机选择一个模型（用于选择模型，但不用于获取位置）
        scene, reserved_models = self.get_scene()
        if not reserved_models or scene is None:
            print("DEBUG: get_scene() returned None, skipping this test")
            return None
        
        # 随机选择一个可用模型
        available_models = [m for m in scene.model if m.name not in reserved_models]
        if not available_models:
            print("No available models for metamorphic test")
            return None
        
        target_model = random.choice(available_models)
        # model_name = target_model.name
        model_name = "ellipsoid"
        print(f"Selected model for metamorphic test: {model_name}")
        
        # 使用 topic 获取模型的实时初始位置
        initial_pos = self.get_model_pose_from_topic(model_name)
        if initial_pos is None:
            print(f"DEBUG: Failed to get initial position for model {model_name}")
            return None
        print(f"Initial position: {initial_pos}")
        
        # 2. 如果没有指定速度，随机生成一个速度
        if velocity_x is None:
            velocity_x = random.uniform(0.5, 2.0)  # 随机速度范围：0.5-2.0 m/s
        print(f"Velocity: {velocity_x} m/s, Duration: {test_duration} s")
        
        # 3. 对模型施加力，使其沿x轴运动
        # 注意：使用 -r 参数启动的 Gazebo 已经自动运行，不需要额外调用 play_simulation()
        # 计算需要的力（简化处理，假设模型质量为1kg）
        # 使用较大的力值以确保模型能够运动（实际效果取决于模型质量）
        force_x = velocity_x * 50.0  # 根据模型质量调整，这里使用较大的系数
        print(f"DEBUG: Applying force: x={force_x}, y=0.0, z=0.0 to model {model_name}")
        force_cmd = self.func_apply_model_force(
            model_name=model_name,
            force_x=force_x,
            force_y=0.0,
            force_z=0.0,
            persistent=True
        )
        
        if force_cmd:
            print("DEBUG: Executing force command...")
            force_cmd.execute()
            # 等待一小段时间确保力被应用
            time.sleep(0.2)
        else:
            print("DEBUG: Warning - force_cmd is None, cannot apply force")
        
        # 4. 等待指定时间
        print(f"Applying force for {test_duration} seconds...")
        time.sleep(test_duration)
        
        # 5. 使用 topic 获取模型的实时最终位置
        final_pos = self.get_model_pose_from_topic(model_name)
        if final_pos is None:
            print(f"DEBUG: Failed to get final position for model {model_name}")
            return None
        print(f"Final position: {final_pos}")
        
        # 6. 计算预期位置
        expected_pos = (
            initial_pos[0] + velocity_x * test_duration,
            initial_pos[1],
            initial_pos[2]
        )
        print(f"Expected position: {expected_pos}")
        
        # 7. 验证结果（允许一定的误差）
        error_threshold = 0.5  # 允许0.5米的误差
        error_x = abs(final_pos[0] - expected_pos[0])
        error_y = abs(final_pos[1] - expected_pos[1])
        error_z = abs(final_pos[2] - expected_pos[2])
        
        success = (error_x < error_threshold and 
                   error_y < error_threshold and 
                   error_z < error_threshold)
        
        print(f"Position error: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}")
        print(f"Test {'PASSED' if success else 'FAILED'}")
        
        return (model_name, initial_pos, final_pos, expected_pos, success)

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
        unit = SmithUnit(exp_dir, "a.sdf", options.num_seq, True, skipped, options.timeout, seed, None, None, diversity, crashes) # bandits)
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
