'''
动态算子测试框架
通过JSON配置定义测试步骤
整个链路为 
'''

import os
import json
import unittest
import sys
import random
import xml.etree.ElementTree as ET
import time

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

from gz_test_deps.sim import TestFixture, World, world_entity, K_NULL_ENTITY
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs.entity_pb2 import Entity
from gz.transport14 import Node
from sdformat15 import Root as SDFRoot
from gz.msgs11.boolean_pb2 import Boolean
from gz_test_deps.common import set_verbosity
from gz.sim9 import World  # 从正确路径导入基础类
from gz.msgs.model_pb2 import Model
# from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.entity_pb2 import Entity
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.pose_pb2 import Pose
from gz.transport14 import Node
from sdformat15 import Root as SDFRoot
import random
import os
from gz.msgs11.scene_pb2 import Scene
from gz.msgs11.empty_pb2 import Empty

now_path = "/home/liyitao/workspace/rezilla-modelsmith-sim9"

# class GazeboOperator:
#     def __init__(self, world_name="default", node=None, timeout=10000):
#         self.world_name = world_name
#         self.node = node or Node()
#         self.timeout = timeout
#         self._initialize_dependencies()

#     def _initialize_dependencies(self):
#         """初始化依赖组件"""
#         from .model_generator import ModelGen  # 假设有模型生成组件
#         from .plugin_miner import PluginMiner  # 假设有插件挖掘组件
        
#         self.model_gen = ModelGen()
#         self.plugin_miner = PluginMiner()

#     # 核心功能方法 -------------------------------------------------
#     def get_world_info(self):
#         """获取当前Gazebo世界信息"""
#         service_name = "/gazebo/worlds"
#         request = Empty()
#         return self.node.request(service_name, request, Empty, StringMsg_V, self.timeout)

#     def generate_service_topic(self, st_name):
#         """生成服务/话题命令"""
#         service_list = self.node.service_list()
#         topic_list = self.node.topic_list()
        
#         if st_name in service_list:
#             return self._generate_service_command(st_name)
#         elif st_name in topic_list:
#             return self._generate_topic_command(st_name)
#         else:
#             return None

#     def dump_world_sdf(self):
#         """导出当前世界SDF"""
#         service_name = f"/world/{self.world_name}/generate_world_sdf"
#         request = SdfGeneratorConfig()
#         return self.node.request(service_name, request, SdfGeneratorConfig, StringMsg, self.timeout)

#     def add_random_model(self, pose_range=(-5,5)):
#         """添加随机模型"""
#         model_sdf = self.model_gen.generate()
#         return self._add_model(
#             sdf_content=model_sdf,
#             pose_range=pose_range
#         )

#     def add_model_with_plugin(self, plugin_name, pose_range=(-5,5)):
#         """添加带插件的模型"""
#         model_sdf = self.model_gen.generate()
#         plugin_config = self.plugin_miner.get_plugin(plugin_name)
#         return self._add_model(
#             sdf_content=model_sdf,
#             plugin_config=plugin_config,
#             pose_range=pose_range
#         )

#     def remove_model(self, model_name):
#         """删除指定模型（增强版）"""
#         # 1. 构建删除请求
#         req = Entity()
#         req.name = model_name
#         req.type = Entity.MODEL
        
#         # 2. 调用删除服务
#         service_path = f"/world/{self.world_name}/remove/entity"
#         success, response = self.node.request(
#             service_path,
#             req,
#             Entity,
#             Boolean,
#             self.timeout
#         )
        
#         # 3. 验证删除结果
#         if success and response.data:
#             print(f"模型 {model_name} 删除成功")
#             return True
#         print(f"删除失败: {response.result}")
#         return False

#     def random_pose(self, model_name):
#         """设置随机位姿"""
#         service_name = f"/world/{self.world_name}/set_pose"
#         request = Pose()
#         request.name = model_name
#         request.position.x = random.uniform(-5,5)
#         request.position.y = random.uniform(-5,5)
#         request.position.z = random.uniform(0,5)
#         return self.node.request(service_name, request, Pose, Boolean, self.timeout)

#     # 私有辅助方法 -------------------------------------------------
#     def _add_model(self, sdf_content, pose_range, plugin_config=None):
#         """添加模型底层实现"""
#         service_name = f"/world/{self.world_name}/create"
#         request = EntityFactory()
        
#         # 构建SDF内容
#         if plugin_config:
#             sdf_content = self._inject_plugin(sdf_content, plugin_config)
        
#         request.sdf = sdf_content
#         request.pose.position.x = random.uniform(*pose_range)
#         request.pose.position.y = random.uniform(*pose_range)
#         request.pose.position.z = random.uniform(0, pose_range[1])
        
#         return self.node.request(
#             service_name,
#             request,
#             EntityFactory,
#             Boolean,
#             self.timeout
#         )

#     def _inject_plugin(self, sdf_content, plugin_config):
#         """注入插件到模型SDF"""
#         # 这里需要实现XML解析和插件注入逻辑
#         # 示例代码可能需要根据实际SDF结构调整
#         from xml.etree.ElementTree import fromstring, SubElement
        
#         root = fromstring(sdf_content)
#         plugin_elem = SubElement(root, 'plugin')
#         plugin_elem.set('name', plugin_config['name'])
#         plugin_elem.set('filename', plugin_config['filename'])
        
#         return tostring(root).decode()

#     def _generate_service_command(self, service_name):
#         """生成服务命令"""
#         info = self.node.service_info(service_name)[0]
#         request = randomproto.randproto(info.req_type)
#         return {
#             'type': 'service',
#             'service': service_name,
#             'request': request,
#             'req_type': info.req_type,
#             'rep_type': info.rep_type
#         }

#     def _generate_topic_command(self, topic_name):
#         """生成话题命令""" 
#         info = self.node.topic_info(topic_name)[0]
#         msg = randomproto.randproto(info.msg_type)
#         return {
#             'type': 'topic',
#             'topic': topic_name,
#             'message': msg,
#             'msg_type': info.msg_type
#         }

class TestConfigError(Exception):
    """测试配置异常"""
    pass

class OperatorExecutionError(Exception):
    """算子执行异常"""
    pass

class BaseTestFixture(unittest.TestCase):
    """基础测试夹具（支持动态world路径）"""

    _initialized = False  # 类级初始化标记

    def __init__(self, methodName='runTest', world_path=None):
        super().__init__(methodName)
        self.world_path = world_path
        self.world_name = ""  # 新增字段
        self.model_names = []
        self.plugin_names = []
    
    def setUp(self):
        if not self.world_path:
            self.fail("World路径未指定")
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
            self.fail(f"SDF解析失败: {str(e)}")
    
    def _init_transport(self):
        """初始化通信节点"""
        self.node = Node()
    
    def _create_test_fixture(self):
        """创建测试环境"""
        self.fixture = TestFixture(self.world_path)
        self.fixture.finalize()
        self.server = self.fixture.server()

    def _parse_included_models(self, element):
        """递归解析所有include的模型文件[7](@ref)"""
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

class ModelOperator:
    """算子基类"""
    def __init__(self, node, world_name):
        self.node = node
        self.world_name = world_name
    
    def get_scene_info(self):
        """获取当前场景信息"""
        try:
            # 先尝试获取一次场景信息
            service_name = f"/world/{self.world_name}/scene/info"
            request = Empty()
            result, response = self.node.request(
                service=service_name,
                request=request,
                request_type=Empty,
                response_type=Scene,
                timeout=5000
            )
            
            if result and response:
                # 过滤掉保留的模型名称
                reserved_models = {"ground_model", "ceiling_model", "west_model", 
                                 "east_model", "north_model", "south_model"}
                available_models = [model for model in response.model 
                                 if model.name not in reserved_models]
                return available_models
            return []
        except Exception as e:
            print(f"[DEBUG] 获取场景信息失败: {str(e)}")
            return []
            
    def execute(self, ecm, info):
        """执行操作，返回是否成功"""
        raise NotImplementedError
    
    @classmethod
    def from_config(cls, config, node, world_name):
        """从配置创建算子"""
        raise NotImplementedError

class AddModelOperator(ModelOperator):
    """模型添加算子"""
    def __init__(self, node, world_name, model_path):
        super().__init__(node, world_name)
        with open(model_path, 'r') as f:
            self.model_sdf = f.read()
        self.model_path = model_path
    
    def execute(self, ecm, info):
        # 在执行前解析新增模型的SDF
        try:
            model_tree = ET.parse(self.model_path)
            model_root = model_tree.getroot()
            new_name = model_root.get('name')
            
            if new_name in self.test_fixture.model_names:
                raise OperatorExecutionError(f"模型名称冲突: {new_name}")
            world = World(world_entity(ecm))
            print("DEBUG: 当前world中的model量为 " + str(world.model_count(ecm)))
            req = EntityFactory()
            req.sdf = self.model_sdf
            
            try:
                success, response = self.node.request(
                    f"/world/{self.world_name}/create",
                    req,
                    EntityFactory,
                    Boolean,
                    5000
                )

                print(f"[DEBUG] 添加算子执行成功，响应状态: {success}, 数据: {response.data}")

                return success and response.data
            except Exception as e:
                raise OperatorExecutionError(f"添加模型失败: {str(e)}")
            
        except ET.ParseError as e:
            raise OperatorExecutionError(f"模型文件解析失败: {str(e)}")
    
    @classmethod
    def from_config(cls, config, node, world_name):
        if 'model_path' not in config:
            raise TestConfigError("缺少model_path参数")
        return cls(node, world_name, config['model_path'])

class RemoveModelOperator(ModelOperator):
    """指定模型删除算子"""
    def __init__(self, node, world_name, model_name):
        super().__init__(node, world_name)
        self.model_name = model_name
    
    def execute(self, ecm, info):
        try:
            # 验证模型存在性
            world = World(world_entity(ecm))
            if world.model_by_name(ecm, self.model_name) == K_NULL_ENTITY:
                raise OperatorExecutionError(f"模型不存在: {self.model_name}")

            # 构建删除请求
            request = Entity()
            request.name = self.model_name
            request.type = Entity.MODEL

            # 调用删除服务
            service = f"/world/{self.world_name}/remove"
            success, response = self.node.request(
                service=service,
                request=request,
                request_type=Entity,
                response_type=Boolean,
                timeout=5000
            )

            if success and response and response.data:
                print(f"[DEBUG] 删除模型成功: {self.model_name}")
                world = World(world_entity(ecm))
                print("DEBUG: 当前world中的model量为 " + str(world.model_count(ecm)))
                return True
            else:
                print(f"[DEBUG] 删除模型失败: {self.model_name}")
                return False
            
        except Exception as e:
            raise OperatorExecutionError(f"删除失败: {str(e)}")
    
    @classmethod
    def from_config(cls, config, node, world_name):
        if 'model_name' not in config:
            raise TestConfigError("缺少model_name参数")
        return cls(node, world_name, config['model_name'])

class RandomRemoveModelOperator(ModelOperator):
    """随机删除算子（动态名称选择）"""
    def __init__(self, node, world_name):
        super().__init__(node, world_name)
        
    def execute(self, ecm, info):
        try:
            # 获取当前场景中的所有可用模型
            available_models = self.get_scene_info()
            
            if not available_models:
                print("[DEBUG] 当前场景中没有可删除的模型")
                return False
                
            # 随机选择一个模型
            model_to_remove = random.choice(available_models)
            current_models = self.get_scene_info()
            print("DEBUG: 删除前，当前world中的model量为 " + str(len(current_models)))
            
            # 构建删除请求
            request = Entity()
            request.name = model_to_remove.name
            request.id = model_to_remove.id
            request.type = Entity.MODEL
            
            # 调用删除服务
            service = f"/world/{self.world_name}/remove"
            success, response = self.node.request(
                service=service,
                request=request,
                request_type=Entity,
                response_type=Boolean,
                timeout=5000
            )
            # while(1):
            #     time.sleep(1)
            if success and response and response.data:
                print(f"[DEBUG] 随机删除模型成功: {model_to_remove.name} (ID: {model_to_remove.id})")
                # 等待一段时间让 Gazebo 更新场景状态
                time.sleep(0.5)  # 等待500毫秒
                # 重新获取场景信息以显示当前模型数量
                current_models = self.get_scene_info()
                print("DEBUG: 删除后，当前world中的model量为 " + str(len(current_models)))
                return True
            else:
                print(f"[DEBUG] 随机删除模型失败: {model_to_remove.name} (ID: {model_to_remove.id})")
                return False
                
        except Exception as e:
            raise OperatorExecutionError(f"随机删除失败: {str(e)}")
    
    @classmethod
    def from_config(cls, config, node, world_name):
        return cls(node, world_name)

OPERATOR_CLASSES = {
    'AddModel': AddModelOperator,
    'RemoveModel': RemoveModelOperator,
    'RandomRemoveModel': RandomRemoveModelOperator
}

class DynamicTestRunner(BaseTestFixture):
    """动态测试执行器v3（修复重复初始化）"""
    TEST_CONFIG = "test_config.json"

    def __init__(self, methodName='runTest'):
        """重写初始化方法"""
        # 延迟到setUp阶段初始化
        super().__init__(methodName=methodName, world_path=None)
        self.full_config = None
        self.operators = []

    def _load_config(self):
        """优化的配置加载方法"""
        try:
            # 加载JSON配置
            with open(self.TEST_CONFIG, 'r') as f:
                self.full_config = json.load(f)
            
            # 路径标准化处理
            config_dir = os.path.dirname(os.path.abspath(self.TEST_CONFIG))
            self.full_config['world_path'] = os.path.normpath(
                os.path.join(config_dir, self.full_config['world_path'])
            )
            
            # 直接解析SDF获取world名称
            if not os.path.isfile(self.full_config['world_path']):
                raise TestConfigError(f"World文件不存在: {self.full_config['world_path']}")
            
            # 解析world名称（避免创建临时夹具）
            tree = ET.parse(self.full_config['world_path'])
            root = tree.getroot()
            world_elem = root.find('world')
            if world_elem is not None:
                self.world_name = world_elem.get('name', 'default_world')
            else:
                raise TestConfigError("SDF文件中未找到world元素")

            # 预处理模型路径
            for step in self.full_config.get('steps', []):
                if step['type'] == 'AddModel':
                    model_path = os.path.normpath(
                        os.path.join(config_dir, step['params']['model_path'])
                    )
                    if not os.path.isfile(model_path):
                        raise TestConfigError(f"模型文件不存在: {model_path}")
                    step['params']['model_path'] = model_path

        except json.JSONDecodeError as e:
            raise TestConfigError(f"JSON解析失败: {str(e)}")
        except ET.ParseError as e:
            raise TestConfigError(f"SDF解析失败: {str(e)}")
        except Exception as e:
            raise TestConfigError(f"配置加载异常: {str(e)}")

    def _create_operators(self):
        """优化的算子创建方法"""
        required_params = {
            'AddModel': ['model_path'],
            'RemoveModel': ['model_name'],
            'RandomRemoveModel': []
        }
        
        for step_idx, step in enumerate(self.full_config.get('steps', [])):
            # 参数校验
            if (op_type := step.get('type')) is None:
                raise TestConfigError(f"步骤{step_idx+1}缺少type字段")
            
            if op_type not in OPERATOR_CLASSES:
                raise TestConfigError(f"步骤{step_idx+1}未知算子类型: {op_type}")

            # 参数必要性检查
            missing = [p for p in required_params.get(op_type, []) 
                      if p not in step.get('params', {})]
            if missing:
                raise TestConfigError(f"步骤{step_idx+1} ({op_type}) 缺少参数: {', '.join(missing)}")

            try:
                operator = OPERATOR_CLASSES[op_type].from_config(
                    step.get('params', {}),
                    self.node,
                    self.world_name
                )
                operator.test_fixture = self
                self.operators.append(operator)
            except Exception as e:
                raise TestConfigError(f"步骤{step_idx+1}创建失败: {str(e)}")

    def setUp(self):
        """重构的初始化流程"""
        try:
            # 阶段1: 加载配置
            self._load_config()
            
            # 阶段2: 初始化父类
            super().__init__(
                methodName='runTest', 
                world_path=self.full_config['world_path']
            )
            super().setUp()  # 调用BaseTestFixture的setUp
            
            # 阶段3: 创建算子
            self._create_operators()
            
        except TestConfigError as e:
            self.fail(f"配置错误: {str(e)}")
        except Exception as e:
            self.fail(f"初始化异常: {str(e)}")

    def test_dynamic_workflow(self):
        """修复参数传递方式"""
        execution_count = 0
        success_flag = True
        error_log = []
        
        def _step_executor(info, ecm):
            nonlocal execution_count, success_flag
            if execution_count >= len(self.operators):
                return
                
            try:
                op = self.operators[execution_count]
                if not op.execute(ecm, info):
                    error_log.append(f"步骤{execution_count+1}执行失败")
                    success_flag = False
                execution_count += 1
            except OperatorExecutionError as e:
                error_log.append(f"步骤{execution_count+1}错误: {str(e)}")
                success_flag = False
                execution_count += 1
        
        self.fixture.on_pre_update(_step_executor)
        
        try:
            # 关键修改点：使用位置参数
            while execution_count < len(self.operators):
                self.server.run(True, 1, False)  # 参数顺序：blocking, iterations, paused
                
            self.fixture.on_pre_update(None)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        except Exception as e:
            self.fail(f"运行时异常: {str(e)}")
        
        if not success_flag:
            self.fail("\n".join([
                f"共{len(self.operators)}个步骤，失败{len(error_log)}处:",
                *error_log
            ]))

if __name__ == '__main__':
    unittest.main()

    