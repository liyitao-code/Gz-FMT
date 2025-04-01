#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
# sys.path.append('/workspace/install/lib/python')

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.stringmsg_v_pb2 import StringMsg_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.sdf_generator_config_pb2 import SdfGeneratorConfig
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.plugin_pb2 import Plugin
from gz.transport13 import Node
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
# from mab.algs import ThompsomSampling, UCB1, UCBTuned

from pybandits.smab import SmabBernoulli, create_smab_bernoulli_cold_start
from pybandits.smab import SmabBernoulliMO, create_smab_bernoulli_mo_cold_start
from pybandits.model import Beta

import logging
import logging.config
import func_timeout

from lxml import etree
import string

import xml.etree.ElementTree as ET

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
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


FIRST_DIR = ['/home/liyitao/workspace', '/home/liyitao/gazebo/800']
DIR_FLAG = 0

DIR = FIRST_DIR[DIR_FLAG] + '/install/lib/python/gz/msgs10'
MAX_MODEL_NUM = 20
NUM_ARM = 7 # dirty, should be calculated, not assigned
RANDPROTO_TIMEOUT = 10

actions = ['add_model', 'remove_model', 'modify_position', 'add_component']

import fcntl

sum_reward = 0

def copy_sdf_file_by_index(sdf_index, destination_directory, source_directory=FIRST_DIR[DIR_FLAG]+'/rezilla-modelsmith-fb63e64b5fab/models'):
    
    index = int(sdf_index[0])
    # 获取source_directory中的所有.sdf文件
    sdf_files = [f for f in os.listdir(source_directory) if f.endswith('.sdf')]
    
    # 按字典顺序排序
    sdf_files.sort()
    
    # 检查索引是否在范围内
    if 0 <= index < len(sdf_files):
        # 获取对应索引的文件名
        sdf_filename = sdf_files[index]
        # 构建完整的源文件路径和目标文件路径
        source_file_path = os.path.join(source_directory, sdf_filename)
        destination_file_path = os.path.join(destination_directory, sdf_filename)
        
        # 如果目标目录不存在，创建它
        os.makedirs(destination_directory, exist_ok=True)
        
        # 将文件复制到目标目录
        shutil.copy(source_file_path, destination_file_path)
        
        # 返回文件名
        return sdf_filename
    else:
        # 如果索引超出范围，返回错误信息
        raise IndexError(f"Index {index} is out of range. There are only {len(sdf_files)} .sdf files.")


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

def save_training_metrics(filename, epoch, cumulative_reward, policy_change, loss_value, exploration_rate):
    """
    保存训练过程中的各种指标。

    :param filename: 要保存的 CSV 文件名。
    :param epoch: 当前的训练回合数。
    :param cumulative_reward: 当前回合的累积奖励。
    :param policy_change: 策略变化（例如动作选择分布的变化）。
    :param loss_value: 当前回合的损失值。
    :param exploration_rate: 当前回合的探索率。
    """
    # 检查文件是否存在，以决定是否需要写入表头
    file_exists = os.path.isfile(filename)

    # 打开文件进行写入
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)

        # 如果文件不存在，则写入表头
        if not file_exists:
            writer.writerow(['Epoch', 'Cumulative Reward', 'Policy Change', 'Loss Value', 'Exploration Rate'])

        # 写入一行数据
        writer.writerow([epoch, cumulative_reward, policy_change, loss_value, exploration_rate])

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

    # def calculate_reward(self, current_diversity, type=False, m=5):
    #     """
    #     计算当前算子序列的奖励值。

    #     :param current_diversity: 当前算子序列的多样性值。
    #     :param type: 如果为True，仅使用算子序列的开头数字；如果为False，使用完整的算子序列。
    #     :param m: 用于计算奖励的历史序列数量。
    #     :return: 当前算子序列的奖励值。
    #     """
    #     if len(self.history) < m:
    #         return 0.0  # 若历史不足m个，则奖励为0

    #     reward_sum = 0
    #     for i in range(1, m + 1):
    #         past_diversity = self.calculate_diversity(self.history[-i], type)
    #         reward_sum += (current_diversity - past_diversity)

    #     reward = (1 / m) * reward_sum
    #     return reward

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

class SimulatorState:
    def __init__(self, sdf_file_path, cover=0, ):
        self.sdf_file_path = sdf_file_path
        self.model_with_plugin = 0
        self.model_with_none = 0
        self.plugin_in_model = 0
        self.plugin_in_world = 0
        self.cover_rate = cover
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
            return "fund_add_random_model_with_plugin"  #
        elif action == SimulatorAction.RANDOM_ADD_MODEL_WITH_PLUGIN_XML:
            return "fund_add_random_model_with_plugin_xml"  #

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

def calculate_rewards(crash_flag, diff_new_line, similarity, actions, num_seq = 10):
    # 初始化奖励列表为三元组
    rewards = [(0, 0, 0) for _ in range(num_seq)]
    
    # 设置崩溃奖励
    if crash_flag == 1:
        for idx in range(num_seq):
            rewards[idx] = (1, rewards[idx][1], rewards[idx][2])
    
    # 设置新覆盖行奖励
    if diff_new_line > 0:
        for idx in range(num_seq):
            rewards[idx] = (rewards[idx][0], 1, rewards[idx][2])
    
    # 设置相似度奖励
    if similarity > 0.35:
        for idx in range(num_seq):
            rewards[idx] = (rewards[idx][0], rewards[idx][1], 1)

    # 计算 first_reward
    first_reward = rewards.copy()
    for i in range(num_seq):
        # 计算与之前相同的 action[i][0] 的次数
        same_operator_count = sum(1 for j in range(i) if actions[j][0] == actions[i][0])
        # 计算需要扣减的次数
        penalty_count = same_operator_count // (i // 3 + 1) if i > 0 else 0

        # 根据扣减次数调整奖励
        for _ in range(penalty_count):
            # 将奖励从后往前扣减
            if first_reward[i][2] == 1:
                first_reward[i] = (first_reward[i][0], first_reward[i][1], 0)
            elif first_reward[i][1] == 1:
                first_reward[i] = (first_reward[i][0], 0, first_reward[i][2])
            elif first_reward[i][0] == 1:
                first_reward[i] = (0, first_reward[i][1], first_reward[i][2])

    # 计算 second_reward
    second_reward = [sum(reward) for reward in rewards]  # 计算每个三元组的和
    for i in range(num_seq):
        # 计算与之前相同的 (action[i][0], action[i][1]) 的次数
        same_full_operator_count = sum(1 for j in range(i) if actions[j] == actions[i])
        # 计算与之前相同的 action[i][0] 的次数
        same_operator_count = sum(1 for j in range(i) if actions[j][0] == actions[i][0])
        
        # 确保分母不为零，避免除零错误
        if same_operator_count > 0:
            penalty_count = same_full_operator_count // same_operator_count
            # 根据扣减次数调整奖励
            for _ in range(penalty_count):
                if second_reward[i] > 0:
                    second_reward[i] -= 1

    return first_reward, second_reward

# 假设你有10个位置，每个位置可以选择10种算子
num_positions = 10
num_operators = 10
sdf_number = 309

# 创建动作ID集合，使用字符串类型
action_ids = {str(i) for i in range(num_operators)}  # 将整数转换为字符串

# 更新多臂老虎机
def update_bandit(bandit, secondary_bandits, sequence, reward):
    actions = []
    rewards = []
    
    for position, (operator, parameter) in enumerate(sequence):
        # 计算当前位置之前的相同算子数量
        same_operator_count = sum(1 for prev_position in range(position) if sequence[prev_position][0] == operator)
        
        # 减少奖励，根据相同算子的数量，每个减少10%
        adjusted_reward = reward * (0.9 ** same_operator_count)
        
        # 收集动作和对应的奖励
        actions.append(str(operator))
        
        # 假设 `bandit` 的策略是 `MultiObjectiveBandit`，并假设每个动作有相同数量的目标
        # rewards.append([adjusted_reward] * n_objectives)  # 如果你知道目标数量
        rewards.append([adjusted_reward])  # 如果不确定目标数量，或假设单目标

        # 更新算子的参数老虎机
        if action_param_counts[operator] > 1:
            secondary_bandits[operator].update(parameter, adjusted_reward)
    
    print(f"DEBUG: action is {actions}, rewards is {rewards}")

    # 更新多目标老虎机
    bandit.update(actions=actions, rewards=rewards)

def generate_operator_sequence(bandit, secondary_bandits, action_param_counts):
    sequence = []
    # 使用 predict 方法来选择动作
    selected_actions, _ = bandit.predict(n_samples=num_positions)
    
    for position, operator_str in enumerate(selected_actions):
        operator = int(operator_str)  # 将选择的字符串转换回整数
        
        if action_param_counts[operator] > 1:
            parameter_bandit = secondary_bandits[operator]
            parameter = parameter_bandit.select_arm()
        else:
            parameter = 0
        
        sequence.append((operator, parameter))
    return sequence

# 多臂老虎机，UCB1策略以及乐观初始值策略，动态调整epsilon
class MultiArmedBandit:
    def __init__(self, num_arms, initial_value=1.0, initial_epsilon=1.0, min_epsilon=0.1, total_iterations=1000):
        self.num_arms = num_arms
        self.initial_epsilon = initial_epsilon
        self.min_epsilon = min_epsilon
        self.total_iterations = total_iterations
        self.current_iteration = 0
        self.counts = np.zeros(num_arms)
        self.values = np.full(num_arms, initial_value)
        self.total_counts = 0

    def select_arm(self):
        # 动态调整epsilon值
        epsilon = max(self.min_epsilon, self.initial_epsilon * (1 - self.current_iteration / self.total_iterations))
        self.current_iteration += 1

        if random.random() < epsilon:
            # Explore: 随机选择一个动作
            return random.randint(0, self.num_arms - 1)
        else:
            # Use UCB1 strategy
            if 0 in self.counts:
                return np.argmin(self.counts)  # 优先选择未被选择过的动作
            else:
                ucb_values = self.values + np.sqrt((2 * np.log(self.total_counts + 1)) / self.counts)
                return np.argmax(ucb_values)

    def update(self, chosen_arm, reward):
        self.counts[chosen_arm] += 1
        self.total_counts += 1
        n = self.counts[chosen_arm]
        value = self.values[chosen_arm]
        self.values[chosen_arm] = ((n - 1) / float(n)) * value + (1 / float(n)) * reward


# 生成整个算子操作序列，二级随机版本
# def generate_operator_sequence(bandit, action_param_counts, sequence_length=10):
#     sequence = []
#     for _ in range(sequence_length):
#         operator = bandit.select_arm()
#         param_max = action_param_counts[operator] - 1
#         parameter = random.randint(0, param_max)
#         sequence.append((operator, parameter))
#     return sequence


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
                self.pb2_modules.append(importlib.import_module(f"gz.msgs10.{file}"))
            except:
                print(f"error processing gz.msgs10.{file}")

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
    def __init__(self, directory="exp", sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0, sdf_miner=None, bandits=None, diversity=None, crashes=None, crash_manager = None):
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
        self.crash_manager = crash_manager

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
    
    # 检测是否有新崩溃
    def check_crash(self, err_file):
        """
        检查错误日志文件中是否包含崩溃的关键字 "Aborted" 或 "Segmentation fault"。

        :param err_file: 错误日志文件路径。
        :return: 如果包含崩溃关键字返回 True，否则返回 False。
        """
        if not os.path.exists(err_file):
            return False

        try:
            with open(err_file, 'r') as file:
                for line in file:
                    # 检查每一行是否包含关键字
                    if "Aborted" in line or "Segmentation fault" in line:
                        return True
        except Exception as e:
            print(f"Error reading {err_file}: {e}")
        
        return False


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
        
    def generate_and_test_commands_train(self, primary_bandit, secondary_bandits, sequence_manager, now_coverage):
        # 生成算子序列，长度为unit.num_seq
        # action_sequence, action_log_probs = generate_action_sequence(state, actor, low_models, unit.num_seq)  # 修改：使用独立的 actor 模型
        action_sequence = generate_operator_sequence(primary_bandit, secondary_bandits, action_param_counts)
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
            # 2. apply the action
            print(func_name)

            if func_name in ["func_add_random_plugin_to_model", "func_add_random_plugin_to_model_xml", "fund_add_random_model_with_plugin", "fund_add_random_model_with_plugin_xml"]:
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

            with open(f"{self.directory}/id", "w") as f:
                f.write(f"{i}")
            
            # TODO: check gz liveness, if not, check f"{self.directory}/gz.err" for stack trace
            # check the status of the process
            if process_status.status() == psutil.STATUS_ZOMBIE:
                print("DEBUG: gz process not alive")
                break
            i += 1

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
        new_cover_line = 0  # 新增覆盖行数
        similarity = 0  # 序列相似度
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
            new_cover_line = diff.new_line
            print(f"Diff new line: {diff.new_line}, new file: {diff.new_file}")

            if self.check_crash(f"{self.directory}/gz.err"):
                crash_occurred = True
                crash_number = crash_manager.process_error_file(f"{self.directory}/gz.err")
                print(f"DEBUG: crash rewards: {i}")
                print(f"DEBUG: {self.crash_rewards}")

        print("DEBUG: now crash occurred is ", crash_occurred)
        # now_coverage += diff.new_line
        # 计算激励

        first_action = [str(action[0]) for action in action_sequence]

        similarity = sequence_manager.calculate_diversity(action_sequence, True)
        
        first_rewards, second_rewards = calculate_rewards(crash_occurred, new_cover_line, similarity, action_sequence)

        print("DEBUG: reward is :")
        print(f"DEBUG: first action is {first_action}")
        print(f"DEBUG: crash is {crash_occurred}, cover line is {new_cover_line}, similarity is  {similarity}")
        print(f"DEBUG: first rewards is {first_rewards}")
        print(f"DEBUG: second rewards is {second_rewards}")
        
        print("DEBUG: train begin")

        # 更新主老虎机
        bandit.update(first_action, first_rewards)
        # 更新算子的参数老虎机
        for position, (operator, parameter) in enumerate(action_sequence):
            if action_param_counts[operator] > 1:
                secondary_bandits[operator].update(parameter, second_rewards[position])        

        print("DEBUG: train end")
        # 将当前序列添加到历史
        sequence_manager.add_to_history(action_sequence)
        with open(f"{self.directory}/action.txt", 'a', encoding='utf-8') as file:
            file.write(str(action_sequence) + '\n')
        return crash_occurred, diff.new_line


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
        if xml_random is True:
            new_sdf = perturb_xml(str(sdf_content))
            if new_sdf is not None:
                request.sdf = new_sdf
                # print("!!!DEBUG: add model request change success")
            else:
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

def DEBUG_PRINT():
    print("BUILD DIR = " + BUILD_DIR)
    print(type(StringMsg))


if __name__ == "__main__":
    exp_dir = "/tmp/exp"
    if not os.path.exists(exp_dir):
        os.mkdir(exp_dir)
    
    print("DEBUG: clean cov")
    os.system(f"{FIRST_DIR[DIR_FLAG]}/rezilla-modelsmith-fb63e64b5fab/cleanup.sh")

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
    parser.add_option("-n", "--num-seq", dest="num_seq", type="int", default=20, help="number of gz commands")
    parser.add_option("-p", "--plugin", dest="plugin", action="store_true", help="enable mined plugin")
    parser.add_option("-t", "--timeout", dest="timeout", type="int", default=10000, help="timeout")

    (options, args) = parser.parse_args()

    # print(os.chdir("/home/liyitao/workspace/exp/muti_1/_5"))
    # print(os.getcwd())
    # largest_world_file = find_largest_world_file()
    # print(largest_world_file)
    # exit()

    if options.seed:
        seed = options.seed
    else:
        seed = int(datetime.now().timestamp())

    print(seed)
    BUILD_DIR = FIRST_DIR[DIR_FLAG] + "/build/"
    
    with open("seed", "w") as f:
        f.write(f"seed: {seed}")

    start = datetime.now().timestamp()
    turn = -1
    stop = False

    cov_line_time = 0
    cov_line_turn = 0
    
    num_operators = 10  # 10种不同的算子
    initial_value = 1.0  # 乐观初始值
    initial_epsilon = 1.0  # 初始的探索概率
    min_epsilon = 0.1  # 最小的探索概率
    total_iterations = 700  # 总共进行700次迭代
    sequence_length = 10  # 算子序列长度

    # 创建多目标老虎机
    bandit = create_smab_bernoulli_mo_cold_start(action_ids, n_objectives=3)

    # 为每个序列位置创建一个独立的多臂老虎机
    primary_bandits = [MultiArmedBandit(num_operators, initial_value, initial_epsilon, min_epsilon, total_iterations) for _ in range(sequence_length)]
    # 为每个有多个参数的算子创建一个对应的多臂老虎机
    secondary_bandits = [None] * num_operators
    for operator, param_count in action_param_counts.items():
        if param_count > 1:
            secondary_bandits[operator] = MultiArmedBandit(param_count, initial_value, initial_epsilon, min_epsilon, total_iterations)

    # 创建sdf老虎机
    # sdf_bandits = MultiArmedBandit(param_count, initial_value, initial_epsilon, min_epsilon, total_iterations)
    sdf_ids = {str(i) for i in range(sdf_number)} 
    sdf_bandits = create_smab_bernoulli_mo_cold_start(sdf_ids, n_objectives=2)

    # 创建序列管理器
    sequence_manager = OperatorSequenceManager(operator_parameter_counts)
    # 错误栈字典
    crash_manager = ErrorLogManager()
    # mab = create_smab_bernoulli_mo_cold_start(action_ids=[str(i) for i in range(NUM_ARM)], n_objectives=3)
    initial_coverage = 0.0
    # diversity = SdfDiversity("./models")
    crashes = set()
    i = 0
    
    total_reward = 0

    # for i in range(0,10):
    #     que = generate_operator_sequence(primary_bandit, secondary_bandits, action_param_counts, sequence_length=10)

    #     for (operator, parameter) in que:
    #         print(f"{operator},{parameter}")

    

    while not stop:
        now = datetime.now().timestamp()
        # 检查是否有.gcda文件，没有就退出
        # if now - start > 200 and gcda_flag is False:
        #     search_path = '../build/*.gcda'
        #     gcda_files = glob(search_path)
        #     if not gcda_files:
        #         print("no gcda file, need execute . ./install/setup.bash")
        #         sys.exit(1)  # 没有gcda文件就退出
        #     gcda_flag = True

        if (now - start ) > 600 * turn:
            with open(f"{options.directory}cov_time.txt", "a") as file:
                file.write(f"time is {now - start}, cover line is {cov_line_time}\n")
            turn = (now - start) / 600
        if now - start >= 60 * 60 * 12:
            stop = True
        
        exp_dir = f"{options.directory}_{i}"
        # print("!!!DEBUG: now reward is " + str(total_reward))
        

        # sdf_state = get_sdf_initial_state()
        # selected_actions, _ = bandit.predict(n_samples=num_positions)
        selected_sdf, _ = sdf_bandits.predict(n_samples=1)
        print(f"DEBUG: sdf id is {selected_sdf}")
        # unit.create_sdf()
        sdf_name = copy_sdf_file_by_index(selected_sdf, exp_dir)
        unit = SmithUnit(exp_dir, sdf_name, options.num_seq, True, skipped, options.timeout, seed, None, None, None, crashes, crash_manager) # bandits)
        # unit.copy_random_sdf()
        # state = SimulatorState(os.path.join(exp_dir, "a.sdf"), initial_coverage)

        
        print("DEBUG: before generate_and_test_commands")
        print("id = " + str(i))
        unit.cov_old.collect()
        # print("DEBUG: now coverage is " + str(unit.cov_old.calculate_total_coverage()))
        # def generate_and_test_commands_train(self, state, sequence_manager, high_model, low_models, high_optimizer, low_optimizers, critic, critic_optimizer):

        crash_occurred, cov_line = unit.generate_and_test_commands_train(bandit, secondary_bandits, sequence_manager, initial_coverage)
        # def generate_and_test_commands_train(self, state, sequence_history, model, optimizer):
        # total_reward += reward
        initial_coverage += cov_line
        cov_line_time += cov_line
        cov_line_turn += cov_line

        a = 1 if crash_occurred else 0
        b = 1 if cov_line > 0 else 0
        sdf_bandits.update(selected_sdf, [(a, b)])
        with open(f"{options.directory}cov_turn.txt", "a") as file:
            file.write(f"turn is {i}, now time is {now - start}, scover line is {cov_line_turn}\n")
        # print("DEBUG: now total reward is ", total_reward)
        i += 1
        
        time.sleep(5)
        # very dirty, just try it for now
        kill_ruby_processes()
        subprocess.run("pkill -9 ruby", shell=True)
        
    print("End of servicesmith.py")

