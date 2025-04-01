import os
import unittest
import sys

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')

from gz_test_deps.sim import TestFixture, World
from gz_test_deps.math import Pose3d
from gz_test_deps.common import components
from gz_test_deps.sim import EntityComponentManager



class GazeboTestFramework(unittest.TestCase):
    _world_path = None
    _model_path = None
    _fixture = None
    _server = None
    _added_entities = []

    @classmethod
    def configure(cls, world_path, model_path):
        """配置方法（需要在运行测试前调用）"""
        def validate_path(path, is_file=True):
            abs_path = os.path.abspath(path)
            if not os.path.exists(abs_path):
                raise FileNotFoundError(f"路径不存在: {abs_path}")
            if is_file and not os.path.isfile(abs_path):
                raise ValueError(f"不是文件: {abs_path}")
            return abs_path

        cls._world_path = validate_path(world_path)
        cls._model_path = validate_path(model_path)
        
        if not cls._model_path.lower().endswith('.sdf'):
            raise ValueError("仅支持SDF格式模型文件")

    @classmethod
    def setUpClass(cls):
        if not cls._world_path or not cls._model_path:
            raise RuntimeError("需要先调用configure()方法配置路径")

        try:
            print(f"\n[初始化] 加载世界文件: {cls._world_path}")
            cls._fixture = TestFixture(cls._world_path)
            cls._fixture.finalize()
            cls._server = cls._fixture.server()
        except Exception as e:
            raise RuntimeError(f"初始化失败: {str(e)}")

    def test_add_single_model(self):
        """测试添加单个模型"""
        # 读取模型文件内容
        with open(self._model_path, 'r') as f:
            model_sdf = f.read()
        
        # 生成唯一名称
        model_name = os.path.splitext(os.path.basename(self._model_path))[0]
        unique_name = f"{model_name}_test_{id(self)}"

        # 定义模型添加回调
        def add_model_callback(info, ecm):
            try:
                # 使用EntityComponentManager创建实体
                entity = ecm.createEntity()
                
                # 添加必要组件
                ecm.createComponent(entity, components.Name(unique_name))
                ecm.createComponent(entity, components.Pose(Pose3d(0, 0, 0, 0, 0, 0)))
                ecm.createComponent(entity, components.ModelSdf(model_sdf))
                
                self._added_entities.append(entity)
                print(f"\n[操作] 已添加实体: {entity}")

            except Exception as e:
                self.fail(f"添加模型失败: {str(e)}")

        # 执行添加操作
        self._fixture.on_pre_update(add_model_callback)
        self._server.run(True, 1, False)

        # 验证结果
        def verify_model(info, ecm):
            for entity in self._added_entities:
                if not ecm.hasComponent(entity, components.ModelSdf):
                    self.fail(f"实体 {entity} 未找到ModelSdf组件")
                print(f"[验证] 实体 {entity} 存在")

        self._fixture.on_post_update(verify_model)
        self._server.run(True, 1, False)

    @classmethod
    def tearDownClass(cls):
        if cls._server and cls._fixture:
            print("\n[清理] 移除测试实体...")
            def cleanup(info, ecm):
                for entity in cls._added_entities:
                    if ecm.hasComponent(entity, components.ModelSdf):
                        ecm.destroyEntity(entity)
            cls._fixture.on_pre_update(cleanup)
            cls._server.run(True, 1, False)

if __name__ == '__main__':
    # 在此配置路径（示例路径，请替换为实际路径）
    GazeboTestFramework.configure(
        world_path="/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/shapes.sdf",
        model_path="/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/model.sdf"
    )
    unittest.main()
