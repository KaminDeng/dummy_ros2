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
# @file test_copyright.py
# @brief 描述包版权头检查的ament入口。
#
# 实际调用ament_copyright扫描指定描述包/test范围，仅代码版权格式，不构建或运行ROS。原Apache许可文本完整保留。
from pathlib import Path

from ament_copyright.main import main
import pytest


##
# @brief 向ament main传包根与test两个路径参数，返回非零即断言失败。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[str(Path(__file__).resolve().parents[1]), 'test'])
    assert rc == 0, 'Found errors'
