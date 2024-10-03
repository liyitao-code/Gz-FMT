import random
import numpy as np
from collections import deque
from scipy.spatial.distance import cosine

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

    def calculate_diversity(self, current_sequence, type=True):
        """
        计算当前算子序列的多样性。

        :param current_sequence: 当前算子序列。
        :param type: 如果为True，仅使用算子序列的开头数字；如果为False，使用完整的算子序列。
        :return: 当前算子序列的多样性值。
        """
        if not self.history:
            return 1.0  # 若无历史，则多样性最大

        diversity_sum = 0
        current_vector = self.sequence_to_vector(current_sequence, type)
        
        for past_sequence in self.history:
            past_vector = self.sequence_to_vector(past_sequence, type)
            similarity = 1 - cosine(current_vector, past_vector)
            diversity_sum += (1 - similarity)

        div_t = (1 / len(self.history)) * diversity_sum
        return div_t

    def calculate_reward(self, current_diversity, type=True, m=5):
        """
        计算当前算子序列的奖励值。

        :param current_diversity: 当前算子序列的多样性值。
        :param type: 如果为True，仅使用算子序列的开头数字；如果为False，使用完整的算子序列。
        :param m: 用于计算奖励的历史序列数量。
        :return: 当前算子序列的奖励值。
        """
        if len(self.history) < m:
            return 0.0  # 若历史不足m个，则奖励为0

        reward_sum = 0
        for i in range(1, m + 1):
            past_diversity = self.calculate_diversity(self.history[-i], type)
            reward_sum += (current_diversity - past_diversity)

        reward = (1 / m) * reward_sum
        return reward

    def sequence_to_vector(self, sequence, type=True):
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

# 使用示例
param_limits = [1, 1, 117, 117, 1, 1, 1, 1, 123, 123]  # 这里是每个算子的参数上限
manager = OperatorSequenceManager(param_limits)


for i in range(0, 20):
    # 生成一个随机算子序列
    current_sequence = manager.generate_random_operator_sequence()

    # 计算当前序列的多样性
    current_diversity = manager.calculate_diversity(current_sequence, False)

    # 计算当前序列的奖励值
    reward = manager.calculate_reward(current_diversity, False)

    # 将当前序列添加到历史记录
    manager.add_to_history(current_sequence)

    print(f"当前序列: {current_sequence}")
    print(f"多样性: {current_diversity}")
    print(f"奖励: {reward}")
