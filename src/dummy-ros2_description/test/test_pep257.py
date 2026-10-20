# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

##
# @file test_pep257.py
# @brief 描述包docstring风格检查的ament入口。
#
# ament main扫描相对当前工作目录的点与test，依赖调用方工作目录；Doxygen注释不改变原__doc__，本测试不构建/运行ROS。
from ament_pep257.main import main
import pytest


##
# @brief 调用原PEP257入口并要求rc0，失败报告原风格错误。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
