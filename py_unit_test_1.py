import sys
sys.path.append('/home/liyitao/workspace/gazebo/install/lib/python')
sys.path.append('/home/liyitao/workspace/gazebo/src/gz-sim/python/test/')

import unittest
from gz.sim9 import TestFixture  # 请根据具体的API版本调整导入路径

class TestGazeboLoading(unittest.TestCase):
    def setUp(self):
        # 从命令行参数获取SDF文件路径
        if len(sys.argv) < 2:
            raise ValueError("请提供一个SDF文件路径作为命令行参数")
        self.sdf_path = sys.argv[1]

        # 初始化TestFixture
        self.fixture = TestFixture(self.sdf_path)

    def test_load_sdf_and_start_gazebo(self):
        # 加载SDF文件并创建Gazebo进程
        # TestFixture会启动一个Gazebo实例，加载指定的SDF文件
        try:
            self.fixture.LoadSdf()
            print(f"SDF文件 {self.sdf_path} 已成功加载")
            
            # 在这里可以进行其他的测试，例如检查Gazebo进程是否在运行
            self.assertTrue(self.fixture.is_running(), "Gazebo进程未能启动")
        
        except Exception as e:
            self.fail(f"加载SDF文件或启动Gazebo进程时出现异常: {e}")

    def tearDown(self):
        # 结束Gazebo进程
        self.fixture.Close()
        print("Gazebo进程已结束")

if __name__ == "__main__":
    # 移除第一个参数，因为unittest默认会把第一个参数当作测试模块
    unittest.main(argv=sys.argv[:1])
