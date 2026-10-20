"""离线视觉工具必须保留材质分组，且不覆写源/碰撞资源。"""
##
# @file test_optimize_step_visuals.py
# @brief 临时OBJ/MTL视觉优化子进程与源/碰撞资源保护回归。
#
# 实际运行离线optimize_step_visuals CLI处理最小两材质OBJ，未加载CAD真模型/ROS或执行机械动作。核源字节/材质组及碰撞文件不复制，不等于视觉网格几何误差或性能已全面验证。
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'optimize_step_visuals.py'


##
# @brief 仅对临时最小视觉资源运行离线工具的unittest集合。
class VisualOptimizerTest(unittest.TestCase):
    ##
    # @brief 两材质red/gray保留，源OBJ与collision原字节不变、MTL复制，输出无collision副本且有验证JSON。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_distinct_output_preserves_materials_and_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'source'
            source.mkdir()
            original = ('mtllib full_step_test-colors.mtl\nusemtl red\n'
                        'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n'
                        'usemtl gray\nv 0 0 1\nf 1 2 4\n')
            obj = source / 'full_step_test-colors.obj'
            obj.write_text(original)
            (source / obj.with_suffix('.mtl').name).write_text('newmtl red\nKd 1 0 0\nnewmtl gray\nKd .6 .6 .6\n')
            collision = source / 'full_step_test-collision-local.stl'
            collision.write_bytes(b'collision must remain untouched')
            output = Path(tmp) / 'output'
            result = subprocess.run([sys.executable, str(SCRIPT), str(source), str(output)], capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertEqual(original, obj.read_text())
            self.assertEqual(b'collision must remain untouched', collision.read_bytes())
            self.assertEqual({'red', 'gray'}, {line.split()[1] for line in (output / obj.name).read_text().splitlines() if line.startswith('usemtl ')})
            self.assertEqual((source / obj.with_suffix('.mtl').name).read_bytes(), (output / obj.with_suffix('.mtl').name).read_bytes())
            self.assertFalse((output / collision.name).exists())
            self.assertTrue((output / 'visual-mesh-verification.json').is_file())


if __name__ == '__main__':
    unittest.main()
