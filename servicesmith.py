#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gz.msgs11.stringmsg_pb2 import StringMsg
from gz.msgs11.stringmsg_v_pb2 import StringMsg_V
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.empty_pb2 import Empty
from gz.msgs11.scene_pb2 import Scene
from gz.msgs11.entity_pb2 import Entity
from gz.msgs11.sdf_generator_config_pb2 import SdfGeneratorConfig
from gz.msgs11.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs11.plugin_pb2 import Plugin
from gz.transport13 import Node
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
from coverage_process import CoverageInfo, CoverageDiff, BUILD_DIR, GCOV_DIR
import copy
from enum import Enum
import psutil
import shutil
import time
from plugin_mining import PluginMiner, SdfMiner
from sdf_diversity import SdfDiversity
from crash_result import ErrorLog
### from mab.algs import ThompsomSampling, UCB1, UCBTuned

from pybandits.smab import SmabBernoulli, create_smab_bernoulli_cold_start
from pybandits.smab import SmabBernoulliMO, create_smab_bernoulli_mo_cold_start
from pybandits.model import Beta
import logging
import logging.config
import func_timeout

from lxml import etree
import string
import sys

FIRST_DIR = ['/home/liyitao/workspace', '/home/liyitao/gz_lastest']
DIR_FLAG = 0

DIR = FIRST_DIR[DIR_FLAG] + '/install/lib/python/gz/msgs10'
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
                    print(f"Node: <{random_leaf.tag}>")
                    print(f"Original: {original_text}")
                    print(f"Mutated: {mutated_text}")

                    # 更新节点文本
                    random_leaf.text = mutated_text

                else:
                    # 处理包含多个数字的情况
                    numbers = re.findall(r"-?\d+(?:\.\d+)?", original_text)
                    mutated_numbers = [random_number_like(number) for number in numbers if number]

                    # 将变异后的数字重新组合成字符串
                    mutated_text = ' '.join(mutated_numbers) if mutated_numbers else random_string(len(original_text))
                    
                    # 输出变异信息
                    print(f"Node: <{random_leaf.tag}>")
                    print(f"Original: {original_text}")
                    print(f"Mutated: {mutated_text}")

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
    print("DEBUG: end random string")
    print(perturbed_data)
    print("DEBUG: perturbed data end")
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
        self.cov_old = CoverageInfo(BUILD_DIR, GCOV_DIR)
        self.cov_new = None
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



    # 生成和测试命令
    def generate_and_test_commands(self):
        # 0. collect coverage info
        print("DEBUG: before cov")
        self.cov_old.collect()
        # print("DEBUG: now coverage is " + str(self.cov_old.calculate_total_coverage()))
        # 1. run gz sim a.sdf and sleep for a few seconds, dirty
        gz_sim = f"gz sim {self.directory}/{self.sdf_name} --seed {self.seed} -v 0 -r -s --headless-rendering"
        print("DEBUG: before subprocess gz sim")
        try:
            process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        except:
            print("DEBUG: subprocess launch gz error")
            return None

        process_status = psutil.Process(process.pid)
        time.sleep(5)

        try:
            result, response = self.get_world()
            self.world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return None

        print("DEBUG: before loop gz commands")
        func_names = []
        if self.bandits:
            actions, probs = self.bandits.predict(n_samples=self.num_seq)
            # print(probs)
        print("DEBUG: before command range")
        for i in range(self.num_seq):

            # 1. choose function to apply
            if self.bandits:
                # change this to select with bandits
                func_name = self.funcs[int(actions[i])]
            else:
                func_name = random.choice(self.funcs)
            func_names.append(func_name)
            func = getattr(self, func_name)
            # 2. apply the action
            print(func_name)
            command = func()
            self.gz_cmds.append(command)
            if self.use_text:
                cmd_filename = f"{self.directory}/cmd_{i}.sh"
                if command:
                    command.dump(cmd_filename)
                else:
                    GzCommand.dump_empty(cmd_filename)
            if command:
                print(f"DEBUG: before execute command {i}")
                ret = command.execute()

            world = self.dump_sdf(self.world_name)
            print(f"DEBUG: before dump world {i}")
            world_file = f"{self.directory}/world_{i}.sdf" 
            with open(world_file, "w") as f:
                f.write(world)

            if self.diversity:
                flag, dist = self.diversity.add_and_check(world_file)
                self.diversity_rewards[i] = 1 if flag else 0


            with open(f"{self.directory}/id", "w") as f:
                f.write(f"{i}")

            # TODO: check gz liveness, if not, check f"{self.directory}/gz.err" for stack trace
            # check the status of the process
            if process_status.status() == psutil.STATUS_ZOMBIE:
                print("DEBUG: gz process not alive")
                break

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
            return None
        except:
            print("DEBUG: before coverage")
            self.cov_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
            self.cov_new.collect()
            diff = CoverageDiff()
            diff.compare(self.cov_new, self.cov_old)

            with open(f"{self.directory}/gz.out", "w") as f:
                # f.write(process.stdout.read().decode("utf-8"))
                f.write(out)
            with open(f"{self.directory}/gz.err", "w") as f:
                # f.write(process.stderr.read().decode("utf-8"))
                f.write(err)
            print(f"Diff new line: {diff.new_line}, new file: {diff.new_file}")

            if self.check_new_crash(f"{self.directory}/gz.err"):
                print(f"DEBUG: crash rewards: {i}")
                # TODO: dump crash to file
                for j in range(i):
                    self.crash_rewards[j] = 1

                print(f"DEBUG: {self.crash_rewards}")
            print("DEBUG: diff end")
            if self.bandits:
                ### for i in range(len(func_names)):
                ###     index = self.funcs.index(func_names[i])  
                ###     self.bandits[i].reward(index)
                if diff.new_line > 0:
                    rewards = [(1 if idx <= i else 0, self.diversity_rewards[idx], self.crash_rewards[idx]) for idx in range(self.num_seq)]
                else:
                    rewards = [(0, self.diversity_rewards[idx], self.crash_rewards[idx]) for idx in range(self.num_seq)]

                print(rewards)
                self.bandits.update(actions, rewards=rewards)

            return diff

    # 复制随机的sdf文件
    def copy_random_sdf(self):
        filenames = glob("./models/*.sdf")
        random_filename = random.choice(filenames)
        shutil.copyfile(random_filename, f"{self.directory}/a.sdf")

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
    def func_add_random_model(self, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(pose_min, pose_max, name, sdf_content, False)

    # 以true调用helper_func_add_random_model
    def func_add_mined_random_model(self, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(pose_min, pose_max, name, sdf_content, True)

    # 添加模型
    def helper_func_add_random_model(self, pose_min=-POSE, pose_max=POSE, name="model", sdf_content="", from_mined=False):
        # def create_model(world, name, x, y, z, sdf_content=None):

        scene, reserved_models = self.get_scene()
        if not reserved_models:
            return None
        if len(scene.model) > MAX_MODEL_NUM:
            return None
        service_name = f"/world/{self.world_name}/create"
        request = EntityFactory()
        if not sdf_content:
            model_gen = ModelGen(self.sdf_miner)
            if not from_mined:
                root = model_gen.generate_with_root_wrapper(name, from_mined)
                # TODO: check for exception
                sdf_content = root.to_string().encode("utf-8")
            else:
                sdf_content = self.plugin_miner.random_model_with_root()
        
        new_sdf = perturb_xml(str(sdf_content))
        if new_sdf is not None:
            request.sdf = new_sdf
            print("!!!DEBUG: add model request change success")
        else:
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


    def get_scene(self):
        # gz service -s /world/world_0/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 300 --req ''
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

    # 随机给模型添加组件
    def func_add_random_plugin_to_model(self):
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
        plugin = random.choice(self.plugin_miner.plugins_within_model)
        filename = plugin.get("filename")
        name = plugin.get("name")
        innerxml = "\n".join([tostring(c).decode("utf-8") for c in plugin.getchildren()])

        entity_plugin_pb = EntityPlugin_V()
        plugin_pb = Plugin()
        plugin_pb.filename = filename
        # print("!!!DEBUG: plugin_pb filename is %s" %(plugin_pb.filename))
        plugin_pb.name = name
        new_innerxml = perturb_xml(str(innerxml))
        if(new_innerxml is not None) :
            print("DEBUG: change plugin success")
            plugin_pb.innerxml = new_innerxml
        else:
            plugin_pb.innerxml = innerxml
        entity_plugin_pb.entity.id = model_id
        entity_plugin_pb.plugins.append(plugin_pb)
        # print("!!!DEBUG: plugin_pb filename is %s" %(plugin_pb.filename))
        # print("!!!DEBUG: plugin_pb name is %s" %(plugin_pb.name))
        # print("!!!DEBUG: old plugin_pb innerxml is \n%s" %(plugin_pb.innerxml))

        # print("!!!DEBUG: plugin_pb filename is %s" %(plugin_pb.filename))
        # print("!!!DEBUG entity_pulgin_pb begin")
        # print(type(entity_plugin_pb))
        # print("!!!DEBUG entity_pulgin_pb end")

        # new_request = perturb_protobuf_like_text(str(entity_plugin_pb))
        # if new_request is not None :
        #     entity_plugin_pb = new_request
        #     print("DEBUG: new_request success")
        
        # print("!!!DEBUG: new plugin_pb innerxml is \n%s" %(plugin_pb.innerxml))

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
            # print(f"DEBUG: type_name: {type_name} type_class: {type_class}")
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

def DEBUG_PRINT():
    print("BUILD DIR = " + BUILD_DIR)
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
    # set_pose("world_0", "model_0", 0, 0, 15)
    # with open("e.sdf") as f:
    #     sdf_content = f.read().encode("utf-8")

    # result, response = get_world()
    # world = response.data[0]
    # create_model(world, "model_0", 0, 0, 15)
    skipped = [
        # "/gazebo/resource_paths/resolve",
        # "/world/world_0/enable_collision",
        # "/world/world_0/disable_collision",
        # "/world/world_0/set_physics",
        # "/world/world_0/playback/control",
        "/server_control",
    ]
    # random_service_request(skipped=skipped)
    # random_topic_publish()
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
    BUILD_DIR = FIRST_DIR[DIR_FLAG] + "/build/"
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
    mab = create_smab_bernoulli_mo_cold_start(action_ids=[str(i) for i in range(NUM_ARM)], n_objectives=3)
    diversity = SdfDiversity("./models")
    crashes = set()
    i = 0
    ##### unit = SmithUnit(exp_dir, "a.sdf", options.num_seq, True, skipped, options.timeout, seed, None, mab, diversity, crashes) # bandits)
    ##### unit.create_sdf()
    ##### unit.pairwise_generate_and_test_commands()
    ##### unit.topic_fuzzing()
    # while i < options.iteration:
    while not stop:
        now = datetime.now().timestamp()
        if now - start >= 60 * 60 * 8:
            stop = True
        exp_dir = f"{options.directory}_{i}"
        unit = SmithUnit(exp_dir, "a.sdf", options.num_seq, True, skipped, options.timeout, seed, None, mab, diversity, crashes) # bandits)
        # unit.create_sdf()
        unit.copy_random_sdf()
        print("DEBUG: before generate_and_test_commands")
        print("id = " + str(i + 1))
        unit.cov_old.collect()
        print("DEBUG: now coverage is " + str(unit.cov_old.calculate_total_coverage()))
        diff = unit.generate_and_test_commands()
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
