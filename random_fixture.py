import os
import unittest
import sys

sys.path.append('/home/liyitao/workspace/gz_lastest/install/lib/python')
sys.path.append('/home/liyitao/workspace/gz_lastest/src/gz-sim/python/test')


from gz_test_deps.sim import TestFixture, World, world_entity
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.transport14 import Node
from sdformat15 import Root as SDFRoot
from gz.msgs11.boolean_pb2 import Boolean  # 添加必要的消息类型

class FuzzTest(unittest.TestCase):
    def test_add_model(self):
        world_path="/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/shapes.sdf"
        model_path="/home/liyitao/workspace/rezilla-modelsmith-sim9/test_model/model.sdf"
        # 基础世界SDF路径
        base_world_path = world_path
        # 模型SDF路径
        model_sdf_path = model_path

        # 解析基础世界SDF以获取世界名称
        root = SDFRoot()
        root.load(base_world_path)
        world_name = root.world_by_index(0).name()

        # 创建TestFixture
        fixture = TestFixture(base_world_path)

        # 读取模型SDF内容
        with open(model_sdf_path, 'r') as f:
            model_sdf = f.read()

        # 创建EntityFactory请求
        req = EntityFactory()
        req.sdf = model_sdf

        # 初始化传输节点
        node = Node()

        # 用于跟踪是否已添加模型和模型数量
        model_added = False
        initial_model_count = 0
        final_model_count = 0
        
        def on_pre_update(info, ecm):
            nonlocal model_added, initial_model_count
            world_e = world_entity(ecm)
            world = World(world_e)
            
            # 确保在第一次pre_update时获取初始模型数量
            if info.iterations == 0:
                # 添加延迟获取逻辑
                initial_model_count = world.model_count(ecm)
                if initial_model_count == 0:
                    # 可能模型尚未加载完成，添加调试输出
                    print(f"Iteration {info.iterations}: 初始模型数量={initial_model_count}")
                    return
                
            # 在第二次迭代时发送添加请求
            if not model_added and info.iterations == 1:
                service_name = f"/world/{world_name}/create"
                
                # 修正后的参数顺序
                success, response = node.request(
                    service_name,
                    req,            # 请求实例（必须实例化后的对象）
                    EntityFactory,  # 请求类型（消息类型类）
                    Boolean,        # 响应类型（消息类型类）
                    5000            # 超时时间(ms)
                )

                model_added = True

        def on_post_update(info, ecm):
            nonlocal final_model_count
            world_e = world_entity(ecm)
            world = World(world_e)
            final_model_count = world.model_count(ecm)
            print(f"当前模型数量: {final_model_count}")  # 添加调试输出

        fixture.on_pre_update(on_pre_update)
        fixture.on_post_update(on_post_update)
        fixture.finalize()

        server = fixture.server()
        # 运行足够次数以确保添加完成
        server.run(True, 3, False)

        # 验证模型数量增加
        print("\nfinal model count is %d, initial model count is %d" %(final_model_count, initial_model_count))

if __name__ == '__main__':
    unittest.main()
