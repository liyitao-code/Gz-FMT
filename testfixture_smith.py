'''
本脚本用来在单元测试版本里执行
'''

import os
import unittest
import sys

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

from gz_test_deps.sim import TestFixture, World, world_entity, K_NULL_ENTITY
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.transport14 import Node
from sdformat15 import Root as SDFRoot
from gz.msgs11.boolean_pb2 import Boolean

class BaseTestFixture(unittest.TestCase):
    """基础测试夹具，负责World的创建和管理"""
    def setUp(self):
        # 加载基础World
        self.world_path = "/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/shapes.sdf"
        self._load_base_world()
        self._setup_transport()
        self._create_test_fixture()
        
    def _load_base_world(self):
        """加载基础SDF并解析世界名称"""
        root = SDFRoot()
        try:
            root.load(self.world_path)
            world = root.world_by_index(0)
            self.world_name = world.name()
            self.assertTrue(len(self.world_name) > 0, "World name is empty")
        except Exception as e:
            self.fail(f"Failed to load world SDF: {str(e)}")
            
    def _setup_transport(self):
        """初始化传输节点"""
        self.node = Node()
        
    def _create_test_fixture(self):
        """创建测试夹具"""
        self.fixture = TestFixture(self.world_path)
        self.fixture.finalize()
        self.server = self.fixture.server()

    def _get_world_models(self, ecm):
        """获取当前world中所有非保留模型名称列表"""
        world_e = world_entity(ecm)
        world = World(world_e)
        reserved_models = {"ground_plane", "sun", "walls"}  # 根据实际情况调整保留模型列表
        
        return [
            Model(world.model_by_index(ecm, i)).name(ecm)
            for i in range(world.model_count(ecm))
            if Model(world.model_by_index(ecm, i)).name(ecm) not in reserved_models
        ]

class ModelOperator:
    """模型操作算子基类"""
    def __init__(self, node, world_name):
        self.node = node
        self.world_name = world_name
        
    def execute(self, ecm, info):
        raise NotImplementedError

class AddModelOperator(ModelOperator):
    """添加模型算子"""
    def __init__(self, node, world_name, model_path):
        super().__init__(node, world_name)
        with open(model_path, 'r') as f:
            self.model_sdf = f.read()
            
    def execute(self, ecm, info):
        req = EntityFactory()
        req.sdf = self.model_sdf
        
        service_name = f"/world/{self.world_name}/create"
        success, response = self.node.request(
            service_name,
            req,
            EntityFactory,
            Boolean,
            5000
        )
        return success and response.data

class RemoveModelOperator(ModelOperator):
    """删除指定模型的算子"""
    def __init__(self, node, world_name, model_name):
        super().__init__(node, world_name)
        self.model_name = model_name
        
    def execute(self, ecm, info):
        req = EntityFactory()
        req.remove_name = self.model_name
        
        service_name = f"/world/{self.world_name}/remove"
        success, response = self.node.request(
            service_name,
            req,
            EntityFactory,
            Boolean,
            5000
        )
        print(f"删除模型 {self.model_name} {'成功' if success and response.data else '失败'}")
        return success and response.data

class RandomRemoveModelOperator(ModelOperator):
    """随机删除模型的算子"""
    def __init__(self, node, world_name, base_fixture):
        super().__init__(node, world_name)
        self.base_fixture = base_fixture
        
    def execute(self, ecm, info):
        # 获取可删除模型列表
        available_models = self.base_fixture._get_world_models(ecm)
        if not available_models:
            print("没有可删除的模型")
            return False
        else:
            print("DEBUG: " + available_models)
        # 随机选择模型
        selected_model = random.choice(available_models)
        print(f"随机选择要删除的模型: {selected_model}")
        
        # 执行删除
        return RemoveModelOperator(self.node, self.world_name, selected_model).execute(ecm, info)

class RemoveModelTest(TestSequenceRunner):
    """删除模型测试"""
    def _setup_operators(self):
        # 先添加一个模型以便删除
        model_path = "/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/model.sdf"
        self.operators = [
            AddModelOperator(self.node, self.world_name, model_path),
            RandomRemoveModelOperator(self.node, self.world_name, self)
        ]
        
    def test_workflow(self):
        super().test_workflow()
        self.assertEqual(self.final_model_count, self.initial_model_count, "模型数量应恢复初始值")

class ComplexTest(TestSequenceRunner):
    """复杂流程测试：添加->随机删除->再添加"""
    def _setup_operators(self):
        model_path = "/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/model.sdf"
        self.operators = [
            AddModelOperator(self.node, self.world_name, model_path),
            RandomRemoveModelOperator(self.node, self.world_name, self),
            AddModelOperator(self.node, self.world_name, model_path)
        ]
        
    def test_workflow(self):
        super().test_workflow()
        self.assertEqual(self.final_model_count, self.initial_model_count + 1, "最终应多一个模型")

class TestSequenceRunner(BaseTestFixture):
    """测试流程执行器"""
    def _setup_operators(self):
        """初始化算子序列（子类需重写）"""
        self.operators = []
        
    def _setup_callbacks(self):
        """设置回调函数"""
        self.initial_model_count = 0
        self.final_model_count = 0
        
        def pre_update(info, ecm):
            world_e = world_entity(ecm)
            world = World(world_e)
            if info.iterations == 0:
                self.initial_model_count = world.model_count(ecm)
                
            for op in self.operators:
                op.execute(ecm, info)
                
        def post_update(info, ecm):
            world_e = world_entity(ecm)
            world = World(world_e)
            self.final_model_count = world.model_count(ecm)
            
        self.fixture.on_pre_update(pre_update)
        self.fixture.on_post_update(post_update)
        
    def test_workflow(self):
        """执行测试流程"""
        self._setup_operators()
        self._setup_callbacks()
        
        # 运行3个迭代周期（可根据需要调整）
        self.server.run(True, 3, False)
        
        # 基础验证
        # self.assertGreater(self.initial_model_count, 7, "初始模型数量异常")
        print(f"测试结果: 初始模型 {self.initial_model_count} -> 最终模型 {self.final_model_count}")

class DefaultWorldTest(TestSequenceRunner):
    """默认测试：仅加载基础World"""
    def test_workflow(self):
        super().test_workflow()
        # self.assertEqual(self.final_model_count, self.initial_model_count, "模型数量不应变化")

class AddModelTest(TestSequenceRunner):
    """添加模型测试"""
    def _setup_operators(self):
        model_path = "/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/model.sdf"
        self.operators = [
            AddModelOperator(self.node, self.world_name, model_path)
        ]
        
    def test_workflow(self):
        super().test_workflow()
        # self.assertEqual(self.final_model_count, self.initial_model_count + 1, "模型数量应增加1")

if __name__ == '__main__':
    unittest.main()
