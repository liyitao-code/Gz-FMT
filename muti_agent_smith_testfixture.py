#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

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
from gz.transport14 import Node
from modelsmith import RootGen, ModelGen, POSE, PLUGIN_DIR
from lxml.etree import tostring
import signal
import random
import re
import randomproto
import subprocess
from glob import glob
from os.path import basename
import os
import importlib
from optparse import OptionParser
from datetime import datetime, timedelta
from coverage_process import CoverageInfo, CoverageDiff, BUILD_DIR, GCOV_DIR
import copy
from enum import Enum
import psutil
import shutil
import time
from plugin_mining import PluginMiner, SdfMiner
from sdf_diversity import SdfDiversity
from crash_result import ErrorLog
import unittest
import xml.etree.ElementTree as ET
from gz_test_deps.sim import TestFixture, World, world_entity, K_NULL_ENTITY
### from mab.algs import ThompsomSampling, UCB1, UCBTuned

# from pybandits.smab import SmabBernoulli, create_smab_bernoulli_cold_start
# from pybandits.smab import SmabBernoulliMO, create_smab_bernoulli_mo_cold_start
# from pybandits.model import Beta
import logging
import logging.config
import func_timeout

from lxml import etree
import string

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
# from sklearn.metrics.pairwise import cosine_similarity
from collections import deque, defaultdict
from scipy.spatial.distance import cosine

# import tensorflow as tf
# from tensorflow.keras.models import Sequential
# from tensorflow.keras.layers import Dense
# from tensorflow.keras.optimizers import Adam

from search_plugin_in_model import retrieve_plugin_by_index
from search_plugin_in_world import retrieve_plugin_in_world_by_index
from search_model_with_plugin import retrieve_model_by_index

import csv
import json

FIRST_DIR = ['/home/liyitao/workspace/gz_lastest', '/home/liyitao/gazebo/800']
DIR_FLAG = 0

DIR = FIRST_DIR[DIR_FLAG] + '/install/lib/python/gz/msgs10'
MAX_MODEL_NUM = 20
NUM_ARM = 7 # dirty, should be calculated, not assigned
RANDPROTO_TIMEOUT = 10

actions = ['add_model', 'remove_model', 'modify_position', 'add_component']

import fcntl

sum_reward = 0

def kill_ruby_processes():
    # 遍历系统中的所有进程
    for proc in psutil.process_iter(attrs=['pid', 'name']):
        try:
            # 检查进程名称是否为 'ruby'
            if proc.info['name'] == 'ruby':
                pid = proc.info['pid']
                print(f"Killing ruby process with PID: {pid}")
                os.kill(pid, signal.SIGKILL)  # 使用 SIGKILL 强制终止进程
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            # 忽略在获取进程信息期间可能发生的异常
            pass

# 提取错误栈
class ErrorLog:
    def __init__(self, log_file="gz.err"):
        self.log_file = log_file
        self.trace = []
        if not os.path.exists(log_file):
            return

        with open(log_file) as f:
            self.content = f.read()
        self.get_stack_trace()
        self.trace = tuple(self.trace)

    def get_stack_trace(self):
        """用来从err文件里筛出错误栈"""
        for line in self.content.splitlines():
            if line.startswith("Stack trace"):
                continue
            elif line.startswith("Segmentation fault"):
                continue
            m = re.match(r'#\d\s+Object ".*?", at .*?, in (.*\(.*\))', line)
            if m:
                self.trace.append(m.group(1))

# 维护错误栈字典
class ErrorLogManager:
    def __init__(self):
        # 使用字典来保存不同错误栈的出现次数
        self.error_counts = defaultdict(int)

    def process_error_file(self, log_file):
        """
        处理一个新的错误文件，更新错误栈计数，并返回该错误栈的总出现次数。
        
        :param log_file: 要处理的错误日志文件路径。
        :return: 该错误栈的总出现次数。
        """
        error_log = ErrorLog(log_file)
        error_trace = error_log.trace

        if error_trace:
            # 更新错误栈计数
            self.error_counts[error_trace] += 1
            return self.error_counts[error_trace]
        else:
            # 如果没有有效的错误栈，返回0
            return 0

class OperatorSequenceManager:
    def __init__(self, param_limits, history_size=100):
        """
        初始化算子序列管理器。

        :param param_limits: 每个算子的参数上限值列表。
        :param history_size: 要保存的历史算子序列的最大数量。
        """
        self.param_limits = param_limits
        self.history_size = history_size
        self.history = deque(maxlen=history_size)

    def generate_random_operator_sequence(self, length=5):
        """
        随机生成符合规范的算子序列。

        :param length: 序列中元组的数量。
        :return: 生成的算子序列。
        """
        sequence = [(random.randint(0, 9), random.randint(0, self.param_limits[a]))
                    for a in (random.randint(0, 9) for _ in range(length))]
        return sequence

    def calculate_diversity(self, current_sequence, type):
        """
        计算当前算子序列的多样性。

        :param current_sequence: 当前算子序列。
        :param type: 如果为True，仅使用算子序列的开头数字；如果为False，使用完整的算子序列。
        :return: 当前算子序列的多样性值。
        """
        if len(self.history) < self.history_size / 10:
            return 0  # 若历史太少，则多样性为0

        diversity_sum = 0
        current_vector = self.sequence_to_vector(current_sequence, type)
        
        for past_sequence in self.history:
            past_vector = self.sequence_to_vector(past_sequence, type)
            similarity = 1 - cosine(current_vector, past_vector)
            diversity_sum += (1 - similarity)

        div_t = (1 / len(self.history)) * diversity_sum
        return div_t

    def sequence_to_vector(self, sequence, type=False):
        """
        将算子序列转换为向量表示，用于计算余弦相似度。
        
        :param sequence: 算子序列。
        :param type: 如果为True，仅考虑每个算子的第一个值；如果为False，考虑整个算子和参数。
        :return: 序列的向量表示。
        """
        if type:
            # 仅使用算子序列的开头数字
            vector = np.zeros(10)  # 由于算子的范围是0-9
            for a, _ in sequence:
                vector[a] += 1
        else:
            # 使用完整的算子序列（包括参数）
            max_param_value = max(self.param_limits)
            vector = np.zeros(10 * (max_param_value + 1))
            for a, b in sequence:
                index = a * (max_param_value + 1) + b
                vector[index] += 1
        return vector

    def add_to_history(self, sequence):
        """
        将算子序列添加到历史记录中。

        :param sequence: 要添加的算子序列。
        """
        self.history.append(sequence)

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

# 将带有参数的算子编号转为对应的网络的编号
action_agent_id = [-1, -1, 0, 1, -1, -1, -1, -1, 2, 3]

# list形式定义每个算子的参数数量
operator_parameter_counts = [1, 1, 117, 117, 1, 1, 1, 1, 123, 123]

action_history = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
action_para_history = [0, 0, 0, 0]

class SimulatorState:
    def __init__(self, sdf_file_path, cover=0, action_history = action_history):
        self.sdf_file_path = sdf_file_path
        self.model_with_plugin = 0
        self.model_with_none = 0
        self.plugin_in_model = 0
        self.plugin_in_world = 0
        self.cover_rate = cover
        self.action_his = action_history
        # self.sequence_history = sequence_history

        # 提取SDF文件的状态信息
        self.is_valid = self.extract_sdf_state()

    def extract_sdf_state(self):
        if not os.path.exists(self.sdf_file_path) or os.path.getsize(self.sdf_file_path) == 0:
            print(f"Warning: SDF file {self.sdf_file_path} does not exist or is empty.")
            return False

        try:
            tree = ET.parse(self.sdf_file_path)
            root = tree.getroot()
            self.model_with_plugin, self.model_with_none = self.extract_models(root)
            self.plugin_in_model, self.plugin_in_world = self.extract_plugin(root)
            return True
        except ET.ParseError:
            print(f"Error: Failed to parse SDF file {self.sdf_file_path}.")
            return False

    def extract_models(self, root):
        model_with_plugin = 0
        model_with_none = 0

        # for model in root.findall('.//model'):
        #     if model.find('plugin') is not None:  # Check if model contains any plugin
        #         models.append(format_plugin(ET.tostring(model, encoding='unicode')))

        for model in root.findall('.//model'):
            if model.find('plugin') is not None:
                model_with_plugin += 1
            else:
                model_with_none += 1

        return model_with_plugin, model_with_none
    
    def extract_plugin(self, root):
        plugin_in_model = 0
        plugin_in_world = 0

        # for model in root.findall('.//model'):
        #     if model.find('plugin') is not None:  # Check if model contains any plugin
        #         models.append(format_plugin(ET.tostring(model, encoding='unicode')))

        for model in root.findall('.//model'):
            for plugin in model.findall('plugin'):
                plugin_in_model += 1
        for world in root.findall('.//world'):
            for plugin in world.findall('plugin'):
                plugin_in_world += 1

        return plugin_in_model, plugin_in_world

    def extract_plugins(self, root):
        plugins = root.findall('.//plugin')
        return len(plugins)

    def to_vector(self):
        # 计算序列历史多样性度量
        # diversity_score = self.calculate_diversity_score()

        vector = [
            self.model_with_plugin,
            self.model_with_none,
            self.plugin_in_model,
            self.plugin_in_world,
            self.cover_rate,
            # self.action_his[0],
            # self.action_his[1],
            # self.action_his[2],
            # self.action_his[3],
            # self.action_his[4],
            # self.action_his[5],
            # self.action_his[6],
            # self.action_his[7],
            # self.action_his[8],
            # self.action_his[9],
            # diversity_score
        ]

        return vector


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
            return "func_add_random_model"  #
        elif action == SimulatorAction.RANDOM_LOAD_MODEL_XML:
            return "func_add_random_model_xml"  #
        elif action == SimulatorAction.RANDOM_ADD_PLUGIN:
            return "func_add_random_plugin_to_model"    #
        elif action == SimulatorAction.RANDOM_ADD_PLUGIN_XML:
            return "func_add_random_plugin_to_model_xml"    #
        elif action == SimulatorAction.RANDOM_REMOVE_MODEL:
            return "func_remove_random_model"
        elif action == SimulatorAction.RANDOM_EXEC_SERVICE:
            return "func_random_service"
        elif action == SimulatorAction.RANDOM_EXEC_TOPIC:
            return "func_random_topic"
        elif action == SimulatorAction.RANDOM_SET_POSE:
            return "func_random_pose"
        elif action == SimulatorAction.RANDOM_ADD_MODEL_WITH_PLUGIN:
            return "func_add_random_model_with_plugin"  #
        elif action == SimulatorAction.RANDOM_ADD_MODEL_WITH_PLUGIN_XML:
            return "func_add_random_model_with_plugin_xml"  #

def encode_action(action_type, parameter_index, action_param_counts):
    # 计算从0到当前action_type之前所有动作参数的和
    offset = sum(action_param_counts[i] for i in range(action_type))
    return offset + parameter_index

def decode_action(encoded_action):
    total = 0
    for action_type, count in action_param_counts.items():
        if total + count >= encoded_action:
            parameter_index = encoded_action - total
            return action_type, parameter_index

def calculate_reward(action_sequence, crash_occurred, coverage_increase, sequence_manager, crash_number, div_type = True, reward_type = True):
    '''奖励函数'''
    reward = 0
    # global sum_reward
    crash_reward = 0
    cov_reward = 0
    diversity_reward = 0

    # 崩溃激励
    if crash_occurred:
        # 如果发生崩溃，给予一个大的奖励
        # if crash_number < 12:
        #     crash_reward = 24 - 4 * crash_number
        # else:
        #     crash_reward = 0
        crash_reward = 4 * crash_number
    # 覆盖率激励
    if coverage_increase > 0:
        cov_reward = 0.2  # 根据覆盖率增加给予奖励

    # 多样性激励，使用论文里的公式
    # diversity_reward = calculate_diversity_reward(action_sequence, sequence_manager)
    diversity_reward = sequence_manager.calculate_diversity(action_sequence, div_type)
    reward = diversity_reward + crash_reward + cov_reward

    # sum_reward += reward
    print(f"DEBUG: crash reward is {crash_reward} , coverage reward is {cov_reward}, disversity reward is {diversity_reward}")
    return reward

# 双方的agent
# 定义高层策略（选择算子）和 Critic 网络的联合模型
class HighLevelActor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(HighLevelActor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        #self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(128, action_dim)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        #x = F.relu(self.fc2(x))
        return F.softmax(self.fc3(x), dim=-1)

class Critic(nn.Module):
    def __init__(self, state_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        #self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(128, 1)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        #x = F.relu(self.fc2(x))
        return self.fc3(x)

# 定义低层策略网络（选择参数）
class LowLevelPolicy(nn.Module):
    def __init__(self, state_size, parameter_count):
        super(LowLevelPolicy, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        #self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(128, parameter_count)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        #x = F.relu(self.fc2(x))
        return F.softmax(self.fc3(x), dim=-1)
    
# 生成整个算子操作序列
def generate_action_sequence(state, actor, low_models, num_actions):
    state_tensor = torch.FloatTensor(state.to_vector()).unsqueeze(0)
    high_policy = actor(state_tensor)  # 提取策略
    action_sequence = []
    action_log_probs = []  # 存储每个动作的对数概率

    for _ in range(num_actions):
        operator, operator_log_prob = select_action(high_policy)
        parameter = 0  # 默认为0
        parameter_log_prob = torch.tensor(0.0)  # 如果没有低层策略网络，log_prob为0

        # 如果需要选择参数
        if operator_parameter_counts[operator] > 1:
            low_policy = low_models[operator](state_tensor)
            parameter, parameter_log_prob = select_action(low_policy)

        action_sequence.append((operator, parameter))
        action_log_probs.append((operator_log_prob, parameter_log_prob))

    return action_sequence, action_log_probs

# 动作选择函数
def select_action(policy, epsilon=0.1):
    action_prob = policy.detach().numpy().flatten()
    action_prob = np.nan_to_num(action_prob, nan=1e-10)  # 用一个小正数替换 NaN
    action_prob /= np.sum(action_prob)  # 归一化
    if random.random() < epsilon:
        action = np.random.choice(len(action_prob))
    else:
        action = np.random.choice(len(action_prob), p=action_prob)
    log_prob = torch.log(policy.squeeze(0)[action])
    return action, log_prob


def find_largest_non_empty_world_file(tar_dir):
    """
    在指定目录下寻找所有符合 world_x.sdf 格式的文件，并返回 x 最大且文件不为空的那个文件的路径。
    如果所有 world_x.sdf 文件都为空，则返回当前目录下的 a.sdf。

    :param tar_dir: 目标目录。
    :return: 字符串类型，符合条件的 world_x.sdf 文件的路径或 a.sdf 的路径。
    """
    now_dir = os.getcwd()
    os.chdir(tar_dir)

    # 定义文件名的正则表达式模式
    pattern = re.compile(r'^world_(\d{1,2})\.sdf$')
    largest_file = None

    # 找到所有符合条件的文件
    world_files = [
        filename for filename in os.listdir('.')
        if pattern.match(filename) and os.path.getsize(filename) > 0
    ]

    # 按照 x 的值从大到小排序
    world_files.sort(key=lambda x: int(pattern.match(x).group(1)), reverse=True)

    # 找到第一个不为空的文件
    if world_files:
        largest_file = world_files[0]

    # 返回结果
    if largest_file is not None:
        ans = os.path.abspath(largest_file)
    else:
        ans = os.path.abspath('a.sdf')

    os.chdir(now_dir)
    return ans

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
    def __init__(self, gz_type, cmd, use_text=True, node=None, timeout=10000):
        self.gz_type = gz_type
        self.use_text = use_text
        self.cmd = cmd
        self.node = node or Node()
        self.timeout = timeout
        self.msg_type_convert = MessageTypeConvert()

    @staticmethod
    def dump_empty(filename):
        with open(filename, "w") as f:
            f.write("")

    def dump(self, filename):
        if self.use_text:
            with open(filename, "w") as f:
                if type(self.cmd) is list:
                    f.write("\n".join(self.cmd))
                else:
                    f.write(self.cmd)

    def _execute_service_text(self):
        """执行文本形式的服务调用"""
        if not self.cmd:
            return None
            
        try:
            # 从命令文本中解析服务信息
            cmd_parts = self.cmd.split()
            service_name = cmd_parts[cmd_parts.index('-s') + 1]
            
            # 获取服务信息
            info_list = self.node.service_info(service_name)
            if not info_list:
                print(f"服务不存在: {service_name}")
                return ""
                
            info = random.choice(info_list)
            rep_type = self.msg_type_convert.get_class_type(info.rep_type_name)
            req_type = self.msg_type_convert.get_class_type(info.req_type_name)
            
            if not req_type:
                print(f"无法获取请求类型: {info.req_type_name}")
                return ""
                
            # 生成随机请求
            try:
                random_req = func_timeout.func_timeout(
                    RANDPROTO_TIMEOUT,
                    randomproto.randproto,
                    args=[req_type]
                )
                
                # 发送请求
                result, response = self.node.request(
                    service_name,
                    random_req,
                    req_type,
                    rep_type,
                    self.timeout
                )
                
                return response if result else ""
                
            except func_timeout.FunctionTimedOut:
                print(f"生成请求超时: {service_name}")
                return ""
            except Exception as e:
                print(f"生成请求失败: {str(e)}")
                return ""
                
        except Exception as e:
            print(f"服务调用失败: {str(e)}")
            return ""

    def _execute_topic_text(self):
        """执行文本形式的话题发布"""
        if not self.cmd:
            return None
            
        results = []
        for topic_cmd in (self.cmd if isinstance(self.cmd, list) else [self.cmd]):
            try:
                # 从命令文本中解析话题信息
                cmd_parts = topic_cmd.split()
                topic_name = cmd_parts[cmd_parts.index('-t') + 1]
                
                # 获取话题信息
                info_list = self.node.topic_info(topic_name)
                if not info_list:
                    print(f"话题不存在: {topic_name}")
                    continue
                    
                info = random.choice(info_list)
                msg_type = self.msg_type_convert.get_class_type(info.msg_type)
                
                if not msg_type:
                    print(f"无法获取消息类型: {info.msg_type}")
                    continue
                    
                # 生成随机消息
                try:
                    random_msg = func_timeout.func_timeout(
                        RANDPROTO_TIMEOUT,
                        randomproto.randproto,
                        args=[msg_type]
                    )
                    
                    # 创建发布者并发布消息
                    publisher = self.node.advertise(topic_name, msg_type)
                    if publisher.publish(random_msg):
                        results.append("Success")
                    else:
                        results.append("Failed")
                    
                except func_timeout.FunctionTimedOut:
                    print(f"生成消息超时: {topic_name}")
                    results.append("Timeout")
                except Exception as e:
                    print(f"生成消息失败: {str(e)}")
                    results.append("Error")
                    
            except Exception as e:
                print(f"话题发布失败: {str(e)}")
                results.append("Error")
                
        return "\n".join(results)

    def execute(self):
        """执行命令"""
        if self.use_text:
            if self.gz_type == GzCommandType.SERVICE:
                return self._execute_service_text()
            elif self.gz_type == GzCommandType.TOPIC:
                return self._execute_topic_text()
        else:
            if self.gz_type == GzCommandType.SERVICE:
                # 直接使用 ServiceParam 对象
                service_param = self.cmd
                try:
                    result, response = self.node.request(
                        service_param.service_name,
                        service_param.request,
                        service_param.req_type,
                        service_param.rep_type,
                        service_param.timeout
                    )
                    return response if result else ""
                except Exception as e:
                    print(f"服务调用失败: {str(e)}")
                    return ""
                    
            elif self.gz_type == GzCommandType.TOPIC:
                # 直接使用 TopicParam 对象列表
                for topic_param in self.cmd:
                    try:
                        if not topic_param.publisher.publish(topic_param.message):
                            print(f"话题发布失败: {topic_param.topic_name}")
                    except Exception as e:
                        print(f"话题发布失败: {str(e)}")
                return "Done"

# 测试用的类
class SmithUnit(unittest.TestCase):
    """测试用的类"""
    _initialized = False  # 类级初始化标记

    def __init__(self, directory="exp", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0, sdf_miner=None, bandits=None, diversity=None, crashes=None, crash_manager = None):
        super().__init__()
        self.directory = directory
        self.world_path = None
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
        self.funcs = [member for member in dir(self) if member.startswith("func_")]
        self.cov_old = CoverageInfo(BUILD_DIR, GCOV_DIR)
        self.cov_new = None
        self.root_gen = RootGen(self.sdf_miner)
        self.sdf = None
        self.seed = seed
        self.bandits = bandits
        self.crashes = crashes
        self.crash_manager = crash_manager
        
        # TestFixture 相关的初始化
        self.fixture = None
        self.server = None
        self.model_names = []
        self.plugin_names = []
        
    def setUp(self):
        """初始化测试环境"""
        if self._initialized:
            return
        self._initialized = True
        self._load_world_config()
        self._init_transport()
        self._create_test_fixture()
        
    def _load_world_config(self):
        """增强的SDF解析器"""
        self.model_names = []
        self.plugin_names = []
        
        try:
            # 解析主世界文件
            tree = ET.parse(self.world_path)
            root = tree.getroot()
            
            # 解析模型名称
            world = root.find('world')
            if world is not None:
                self.world_name = world.get('name', 'default_world')
                for model in world.iter('model'):
                    if (name := model.get('name')) is not None:
                        self.model_names.append(name)
                        
                # 解析插件配置
                for plugin in world.iter('plugin'):
                    if (name := plugin.get('name')) is not None:
                        self.plugin_names.append(name)
            print("DEBUG: 建立world时，model列表: ")
            print(self.model_names)
            # 递归解析include的模型文件
            self._parse_included_models(root)
            
        except Exception as e:
            print(f"SDF解析失败: {str(e)}")
            
    def _init_transport(self):
        """初始化通信节点"""
        self.node = Node()
        
    def _create_test_fixture(self):
        """创建测试环境"""
        self.fixture = TestFixture(self.world_path)
        self.fixture.finalize()
        self.server = self.fixture.server()
        
    def _parse_included_models(self, element):
        """递归解析所有include的模型文件"""
        for include in element.iter('include'):
            uri = include.find('uri')
            if uri is None or not uri.text.startswith('model://'):
                continue
                
            model_path = os.path.join(now_path, 'models', uri.text[8:], 'model.sdf')
            if not os.path.exists(model_path):
                continue
                
            try:
                model_tree = ET.parse(model_path)
                model_root = model_tree.getroot()
                
                # 解析被包含模型的名称
                if (model_name := model_root.get('name')) is not None:
                    self.model_names.append(model_name)
                
                # 递归解析嵌套include
                self._parse_included_models(model_root)
                
            except Exception as e:
                print(f"包含模型解析警告: {str(e)}")

    def launch_gazebo(self, filename=""):
        """使用TestFixture启动Gazebo"""
        try:
            if filename:
                self.world_path = filename
            self.setUp()
            return True
        except Exception as e:
            print(f"DEBUG: TestFixture启动Gazebo失败: {str(e)}")
            return False

    def stop_gazebo(self, dump=False):
        """停止TestFixture的Gazebo实例"""
        try:
            if hasattr(self, 'fixture') and self.fixture and hasattr(self.fixture, 'stop'):
                self.fixture.stop()
            if hasattr(self, 'server') and self.server and hasattr(self.server, 'stop'):
                self.server.stop()
            if hasattr(self, 'process') and self.process:
                self.process.terminate()
                self.process.wait()
            return True
        except Exception as e:
            print(f"DEBUG: TestFixture停止Gazebo失败: {str(e)}")
            return False

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
        
    # def generate_and_test_commands_train(self, iteration=1000000):
    def generate_and_test_commands_train(self, state, sequence_manager, actor, critic, low_models, actor_optimizer, low_optimizers, critic_optimizer):
        state_dim = 5  # 状态共有5维
        action_dim = 10  # 10种不同的算子
        # 清除梯度
        actor_optimizer.zero_grad()  # 使用 actor_optimizer
        critic_optimizer.zero_grad()  # Critic 清除梯度
        for opt in low_optimizers:
            if opt is not None:
                opt.zero_grad()
        
        # 生成算子序列，长度为unit.num_seq
        action_sequence, action_log_probs = generate_action_sequence(state, actor, low_models, unit.num_seq)  # 修改：使用独立的 actor 模型
        
        print("DEBUG: action sequence is " + str(action_sequence))

        # 0. collect coverage info
        print("DEBUG: before cov")
        self.cov_old.collect()
        # 1. run gz sim a.sdf and sleep for a few seconds, dirty
        gz_sim = f"gz sim {self.directory}/{self.sdf_name} --seed {self.seed} -v 0 -r -s --headless-rendering"
        print("DEBUG: before subprocess gz sim")
        try:
            process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        except:
            print("DEBUG: subprocess launch gz error")
            return 0, 0

        process_status = psutil.Process(process.pid)
        time.sleep(5)

        try:
            result, response = self.get_world()
            self.world_name = response.data[0]
        except:
            print("DEBUG: gz process not alive")
            return 0, 0

        print("DEBUG: before loop gz commands")
        func_names = []

        print("DEBUG: before command range")
        i = 0
        for (operator, parameter) in action_sequence:
            
            # 1. 获取操作名
            # now_act, now_arg = decode_action(act + 1)
            func_name = SimulatorAction.perform_action(operator)
            func = getattr(self, func_name)
            action_history[operator] += 1
            # 2. apply the action
            print(func_name)

            if func_name in ["func_add_random_plugin_to_model", "func_add_random_plugin_to_model_xml", "func_add_random_model_with_plugin", "func_add_random_model_with_plugin_xml"]:
                command = func(parameter)
            else:
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

            # world = self.dump_sdf(self.world_name)
            # print(f"DEBUG: before dump world {i}")
            # world_file = f"{self.directory}/world_{i}.sdf" 
            # with open(world_file, "w") as f:
            #     f.write(world)

            # if self.diversity:
            #     flag, dist = self.diversity.add_and_check(world_file)
            #     self.diversity_rewards[i] = 1 if flag else 0


            with open(f"{self.directory}/id", "w") as f:
                f.write(f"{i}")
            i += 1
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

        crash_occurred = False  # 模拟检查是否发生崩溃
        crash_number = 1    # 崩溃发生的数量
        coverage_increase = 0  # 模拟增加的覆盖率
        # next_state = state  # 假设状态更新后为 next_state

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
            # return None
        except:
            print("DEBUG: before coverage")
            self.cov_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
            self.cov_new.collect()
            diff = CoverageDiff()
            diff.compare(self.cov_new, self.cov_old)

            with open(f"{self.directory}/gz.out", "w") as f:
                f.write(out)
            with open(f"{self.directory}/gz.err", "w") as f:
                f.write(err)
            print(f"Diff new line: {diff.new_line}, new file: {diff.new_file}")

            

            # if self.check_new_crash(f"{self.directory}/gz.err"):
            #     print(f"DEBUG: crash rewards: {i}")
                # TODO: dump crash to file
                # for j in range(i):
                #     self.crash_rewards[j] = 1
                # crash_occurred = True
                # print(f"DEBUG: {self.crash_rewards}")
            if self.check_crash(f"{self.directory}/gz.err"):
                crash_occurred = True
                crash_number = crash_manager.process_error_file(f"{self.directory}/gz.err")
                print(f"DEBUG: crash rewards: {i}")
                print(f"DEBUG: {self.crash_rewards}")

            # 计算增加的覆盖率
            # total_line = self.cov_new.get_total_line()
            # if total_line != 0:
            #     coverage_increase = diff.new_line / total_line 
            # else:
            #     coverage_increase = 0
            # print(f"Diff coverage increase: {coverage_increase}")

            
            # return diff
        print("DEBUG: now crash occurred is ", crash_occurred)
        # next_state = SimulatorState(find_largest_non_empty_world_file(self.directory), initial_coverage, sequence_history)
        # 计算激励
        reward = calculate_reward(action_sequence, crash_occurred, diff.new_line, sequence_manager, crash_number)
        print("DEBUG: train begin")
        # 反向传播和优化
        state_tensor = torch.FloatTensor(state.to_vector()).unsqueeze(0)
        high_policy = actor(state_tensor)  # 修改：使用独立的 actor 模型
        value = critic(state_tensor)  # 修改：使用独立的 critic 模型
        
        advantage = reward - value.item()  # 计算优势
        
        advantage_tensor = torch.tensor(advantage, requires_grad=True)
        # next_state_tensor = torch.FloatTensor(next_state.to_vector()).unsqueeze(0)    # 每次都是终止态，所以没有s'
        # next_value = critic(next_state_tensor)
        # gamma = 0.99
        # advantage = reward + gamma * next_value.item() - value.item() # 带有V(s')的优势计算

        actor_loss = -sum(op_log_prob + param_log_prob for op_log_prob, param_log_prob in action_log_probs) * advantage_tensor

        critic_loss = advantage_tensor ** 2
        # loss = actor_loss + critic_loss
        # loss.backward()

        # Actor 更新
        actor_optimizer.zero_grad()
        actor_loss.backward(retain_graph=True)  # 保留计算图以便后续反向传播
        actor_optimizer.step()

        # Critic 更新
        critic_optimizer.zero_grad()
        critic_loss.backward()
        critic_optimizer.step()
        # 记录损失
        self.log_training_metrics(self.directory, i, reward, actor_loss.item(), critic_loss.item())

        it = 0
        # 不在算子序列里的二级算子就不更新
        action_count = [0, 0, 0, 0]
        for (operator, parameter) in action_sequence:
            id = action_agent_id[operator]
            if id != -1:
                action_count[id] += 1
        for optimizer in low_optimizers:
            if optimizer is not None and action_count[id] > 0:
                optimizer.step()
        print("DEBUG: train end")
        # 将当前序列添加到历史
        sequence_manager.add_to_history(action_sequence)
        with open(f"{self.directory}/action.txt", 'a', encoding='utf-8') as file:
            file.write(str(action_sequence) + '\n')
        return reward, diff.new_line


    # 复制随机的sdf文件
    def copy_random_sdf(self):
        filenames = glob("./models/*.sdf")
        random_filename = random.choice(filenames)
        shutil.copyfile(random_filename, f"{self.directory}/a.sdf")

    # 创建sdf文件
    def create_sdf(self, dump=True):
        self.sdf = self.root_gen.generate()
        if dump:
            with open(f"{self.directory}/a.sdf", "w") as f:
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
    def func_add_random_model(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, False, False)

    # 添加带扰动的随机模型
    def func_add_random_model_xml(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, False, True)
    
    # 添加带有plugin的不带扰动的随机模型
    def func_add_random_model_with_plugin(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, True, False)
    
    # 添加带有plugin的带扰动的随机模型
    def func_add_random_model_with_plugin_xml(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content=""):
        return self.helper_func_add_random_model(model_id, pose_min, pose_max, name, sdf_content, True, True)

    # 添加模型
    def helper_func_add_random_model(self, model_id = -1, pose_min=-POSE, pose_max=POSE, name="model", sdf_content="", from_mined=False, xml_random = False):
        # def create_model(world, name, x, y, z, sdf_content=None):

        scene, reserved_models = self.get_scene()
        if scene is None or reserved_models is None:
            print("DEBUG: Failed to get scene info")
            return None
        if len(scene.model) > MAX_MODEL_NUM:
            print("DEBUG: Maximum model limit reached")
            return None
            
        service_name = f"/world/{self.world_name}/create"
        request = EntityFactory()

        try:
            if model_id == -1:
                model_gen = ModelGen(self.sdf_miner)
                if not from_mined:
                    root = model_gen.generate_with_root_wrapper(name, from_mined)
                    if root is None:
                        print("DEBUG: Failed to generate model root")
                        return None
                    sdf_content = root.to_string().encode("utf-8")
                else:
                    sdf_content = self.plugin_miner.random_model_with_root()
                    if not sdf_content:
                        print("DEBUG: Failed to get mined model with plugin")
                        return None
            else:
                sdf_content = retrieve_model_by_index(model_id)
                if not sdf_content:
                    print("DEBUG: Failed to retrieve model by index")
                    return None
            
            # 是否扰动
            if xml_random is True:
                new_sdf = perturb_xml(str(sdf_content))
                if new_sdf is not None:
                    request.sdf = new_sdf
                else:
                    print("DEBUG: XML perturbation failed, using original content")
                    request.sdf = sdf_content
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
        except Exception as e:
            print(f"DEBUG: Failed to create model: {str(e)}")
            return None
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
        print(f"DEBUG: available_models: {available_models}")
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
        # print("DEBUG: got here2")
        msg_type_convert = MessageTypeConvert()
        if not service_name:
            service_list = node.service_list()
            if not service_list:
                return None
            service_name = random.choice(service_list)
        # print("DEBUG: got here3")
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
        # print("DEBUG: got here4")
        info = random.choice(info_list)
        rep_type = msg_type_convert.get_class_type(info.rep_type_name)
        req_type = msg_type_convert.get_class_type(info.req_type_name)
        # print("DEBUG: got here5")
        if req_type:
            try:
                # print("DEBUG: got here6")
                random_req = randomproto.randproto(req_type)
                # print("DEBUG: got here7")
                req_text = str(random_req).strip()
                # print("DEBUG: got here8")
                cmd_txt = f"gz service --timeout {self.timeout} -s {service_name} --reptype {info.rep_type_name} --reqtype {info.req_type_name} --req '{req_text}'"
                gz_service = ServiceParam(service_name, random_req, req_type, rep_type, self.timeout)
                # print("DEBUG: got here9")

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
            
    def log_training_metrics(self, directory, epoch, reward, actor_loss, critic_loss, exploration_rate=0.1):
    	filename = f"{directory}/training_metrics.txt"
    	file_exists = os.path.isfile(filename)
    	with open(filename, mode='a', newline='') as file:
        	writer = csv.writer(file)
        	if not file_exists:
        		writer.writerow(['Epoch', 'Reward', 'Actor Loss', 'Critic Loss', 'Exploration Rate'])
        	writer.writerow([epoch, reward, actor_loss, critic_loss, exploration_rate])

class DynamicSmithTestRunner(SmithUnit):
    """动态测试执行器"""
    TEST_CONFIG = "test_config.json"  # 默认配置文件名

    def __init__(self, directory="exp", config_file=None, timeout=10000):
        """初始化方法"""
        super().__init__(directory=directory, timeout=timeout)
        self.config_file = config_file or self.TEST_CONFIG
        self.full_config = None
        self.operators = []
        
    def _load_config(self):
        """加载测试配置"""
        try:
            with open(self.config_file, 'r') as f:
                self.full_config = json.load(f)
        except Exception as e:
            print(f"配置加载失败: {str(e)}")
            return False
        return True
            
    def _map_operator(self, op_type):
        """映射算子类型到具体方法"""
        operator_map = {
            'AddModel': self.func_add_random_model,
            'AddModelXml': self.func_add_random_model_xml,
            'AddModelWithPlugin': self.func_add_random_model_with_plugin,
            'AddModelWithPluginXml': self.func_add_random_model_with_plugin_xml,
            'AddPlugin': self.func_add_random_plugin_to_model,
            'AddPluginXml': self.func_add_random_plugin_to_model_xml,
            'RemoveModel': self.func_remove_random_model,
            'RandomService': self.func_random_service,
            'RandomTopic': self.func_random_topic,
            'RandomPose': self.func_random_pose
        }
        return operator_map.get(op_type)
        
    def execute_test(self):
        """执行测试流程"""
        if not self._load_config():
            return False
            
        # 启动 Gazebo
        if not self.launch_gazebo(self.full_config["world_path"]):
            print("Gazebo 启动失败")
            return False
            
        try:
            # 执行测试步骤
            for step in self.full_config.get('steps', []):
                op_type = step.get('type')
                params = step.get('params', {})
                
                operator = self._map_operator(op_type)
                if operator is None:
                    print(f"未知的算子类型: {op_type}")
                    continue
                    
                try:
                    result = operator(**params)
                    print(f"执行算子 {op_type}: {'成功' if result else '失败'}")
                    time.sleep(1)  # 给 Gazebo 一些时间来处理操作
                except Exception as e:
                    print(f"算子执行失败 {op_type}: {str(e)}")
                    
        finally:
            # 停止 Gazebo
            self.stop_gazebo()
        
        return True

def cleanup():
    """清理环境"""
    subprocess.run("bash cleanup.sh", shell=True)

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-d", "--directory", dest="directory", default="exp")
    parser.add_option("-n", "--num-seq", dest="num_seq", type="int", default=10)
    parser.add_option("-t", "--timeout", dest="timeout", type="int", default=10000)
    parser.add_option("-c", "--config", dest="config_file", default="test_config.json")
    parser.add_option("-s", "--seed", dest="seed", type="int", default=0, help="seed for RNG")
    

    (options, args) = parser.parse_args()
    
    # 清理环境
    cleanup()
    
    # 创建输出目录
    if not os.path.exists(options.directory):
        os.makedirs(options.directory)
        
    # 创建并运行测试
    runner = DynamicSmithTestRunner(
        directory=options.directory,
        config_file=options.config_file,
        timeout=options.timeout
    )
    
    # 执行测试
    success = runner.execute_test()
    
    # 清理环境
    # cleanup()
    
    # 设置退出码
    sys.exit(0 if success else 1)
