"""Required R00-R05 source and test registration checks."""
##
# @file test_r00_r05_deliverables.py
# @brief R00–R05交付路径与测试注册文本检查。
#
# 仅检查文件存在与CMake/xacro字符串，没有展开模型、编译、执行launch或验证被注册测试实际运行。

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R00R05Deliverables(unittest.TestCase):
    ##
    # @brief 只核两个Python验收入口存在，不执行。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_r01_and_r02_acceptance_entrypoints_are_delivered(self):
        for name in ("test_r01_acceptance.py", "test_r02_acceptance.py"):
            path = ROOT / "scripts/dummy/tests" / name
            self.assertTrue(path.is_file(), str(path))

    ##
    # @brief 核参考链文件存在及macro/Joint6文本，不比较FK。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_reference_chain_xacro_is_delivered(self):
        path = ROOT / "src/dummy-ros2_description/urdf/dummy_reference_chain.xacro"
        self.assertTrue(path.is_file(), str(path))
        text = path.read_text()
        self.assertIn("<xacro:macro", text)
        self.assertIn("Joint6", text)

    ##
    # @brief 核launch测试文件与CMake add_launch_test文本，不启动Gazebo。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_gazebo_runtime_launch_test_is_registered(self):
        path = ROOT / "src/dummy_bringup/test/test_gazebo_runtime.launch.py"
        self.assertTrue(path.is_file(), str(path))
        cmake = (ROOT / "src/dummy_bringup/CMakeLists.txt").read_text()
        self.assertIn("test_gazebo_runtime.launch.py", cmake)
        self.assertIn("add_launch_test", cmake)

    ##
    # @brief 核规划launch测试路径在CMake中，不证明运行结果。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_moveit_plan_execute_launch_test_is_registered(self):
        path = ROOT / "src/dummy_bringup/test/test_plan_execute.launch.py"
        self.assertTrue(path.is_file(), str(path))
        cmake = (ROOT / "src/dummy_bringup/CMakeLists.txt").read_text()
        self.assertIn("test_plan_execute.launch.py", cmake)


if __name__ == "__main__":
    unittest.main()
