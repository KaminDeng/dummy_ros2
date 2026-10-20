# 包布局、资源标记和历史缓存阅读说明

本页补充模型资源指南之外的包元数据与六个保持原字节的标记/二进制文件。
包名、安装目录、插件名、规划组、设备profile和线协议版本各有自己的用途，不能因名称相似混用。

## 从源码到安装区

描述包使用 `ament_python`：[setup.py](../../src/dummy-ros2_description/setup.py)
把Python包、资源marker、launch、URDF、mesh和RViz配置安装到相应目录。
[package.xml](../../src/dummy-ros2_description/package.xml) 提供ROS依赖元数据，
[setup.cfg](../../src/dummy-ros2_description/setup.cfg) 约定脚本的lib目录。
Python包的 [__init__.py](../../src/dummy-ros2_description/dummy-ros2_description/__init__.py)
只作包标记，没有设备连接或控制初始化。`console_scripts`为空，不能把资源包当成控制驱动节点。

MoveIt配置包使用 `ament_cmake`：[CMakeLists.txt](../../src/dummy_moveit_config/CMakeLists.txt)
实际构建键盘输入示例，并安装launch/config/worlds；目录中另有历史Servo C++示例源码，
源码存在不代表已建对应可执行文件。两个包的构建成功也不代表Gazebo、RViz或实机已启动。

[.setup_assistant](../../src/dummy_moveit_config/.setup_assistant) 是历史生成元数据，
记录URDF来源、SRDF路径、作者/生成时间与控制接口。原件有两段同名且同值的`control_xacro`；
本轮保留结构，当前YAML加载器对同名映射取最后一段。它不能代替运行期设备能力或许可。

## 保持原字节的六个文件

| 文件 | 用途与阅读边界 |
| --- | --- |
| [src/dummy-ros2_description/resource/dummy-ros2_description](../../src/dummy-ros2_description/resource/dummy-ros2_description) | 零字节ament资源索引marker，安装到share/ament_index/resource_index/packages后使包可被索引。这里靠文件名/存在性标记，不向其中插入注释。 |
| [src/dummy-ros2_description/launch/__pycache__/display.launch.cpython-314.pyc](../../src/dummy-ros2_description/launch/__pycache__/display.launch.cpython-314.pyc) | 历史CPython 3.14字节码缓存；当前Jazzy测试使用系统Python 3.12，不能把此缓存当作当前Python源码或验证产物。对应可读实现以同目录外的.launch.py源文件为准，launch安装/发现按源码文件名处理。 |
| [src/dummy-ros2_description/launch/__pycache__/gazebo.launch.cpython-314.pyc](../../src/dummy-ros2_description/launch/__pycache__/gazebo.launch.cpython-314.pyc) | 历史CPython 3.14字节码缓存；当前Jazzy测试使用系统Python 3.12，不能把此缓存当作当前Python源码或验证产物。对应的gazebo.launch.py源文件当前不在该受控launch目录，保留的pyc不会因此自动成为标准launch入口。 |
| [src/dummy_moveit_config/launch/__pycache__/demo.launch.cpython-314.pyc](../../src/dummy_moveit_config/launch/__pycache__/demo.launch.cpython-314.pyc) | 历史CPython 3.14字节码缓存；当前Jazzy测试使用系统Python 3.12，不能把此缓存当作当前Python源码或验证产物。对应可读实现以同目录外的.launch.py源文件为准，launch安装/发现按源码文件名处理。 |
| [src/dummy_moveit_config/launch/__pycache__/gz_moveit.launch.cpython-314.pyc](../../src/dummy_moveit_config/launch/__pycache__/gz_moveit.launch.cpython-314.pyc) | 历史CPython 3.14字节码缓存；当前Jazzy测试使用系统Python 3.12，不能把此缓存当作当前Python源码或验证产物。对应可读实现以同目录外的.launch.py源文件为准，launch安装/发现按源码文件名处理。 |
| [src/dummy_moveit_config/launch/__pycache__/gz_sim.launch.cpython-314.pyc](../../src/dummy_moveit_config/launch/__pycache__/gz_sim.launch.cpython-314.pyc) | 历史CPython 3.14字节码缓存；当前Jazzy测试使用系统Python 3.12，不能把此缓存当作当前Python源码或验证产物。对应可读实现以同目录外的.launch.py源文件为准，launch安装/发现按源码文件名处理。 |

`.pyc`包含解释器版本相关的字节码/头信息，不能插入Doxygen文本，也不应为了注释任务重生成它们。
[.gitignore](../../.gitignore) 忽略新缓存不等于取消追踪既有缓存；上述五个文件仍是当前git已追踪原件，
没有把“忽略匹配”误记为“文件不存在”。本页的保留策略也不是建议在部署中加载旧解释器缓存。

## RViz保存的内容

[display.rviz](../../src/dummy-ros2_description/config/display.rviz) 是纯显示配置；
其中旧wheel/motor的Links/Frames列表只是历史属性缓存，不会向共享URDF增加那些连杆或TF。
[moveit.rviz](../../src/dummy_moveit_config/config/moveit.rviz) 使用MotionPlanning显示，
[moveit_empty.rviz](../../src/dummy_moveit_config/config/moveit_empty.rviz) 监听历史Servo规划场景。

`Fixed Frame`负责渲染坐标系，Orbit的距离/焦点/角度负责相机；这些数值不是关节目标。
`Collision Enabled`只切换RobotModel碰撞几何的显示，不关闭MoveIt碰撞检查。
`QMainWindow State`是Qt窗口布局序列化字符串，不能按机械臂状态或控制命令解读。
若点击配置了话题的交互工具，它可以发布相应ROS消息，但该消息本身不建立DPSDK会话/permit。

## 注释格式与验证

Python使用独立的`##`行开启Doxygen块，后续采用单`#`写`@file/@brief/@param/@return`等内容。
这样不引入运行期docstring，AST保持原样，也兼容现有flake8对带文字多重井号的检查。
该写法依据 [Doxygen的Python注释说明](https://www.doxygen.nl/manual/docblocks.html#pythonblocks)，
并已用Doxygen 1.9.8实际提取验证。嵌套回调若被解析器合并入外层说明，应在文档中用内部回调分节，
不要把内函数参数错误声明成外函数参数，也不要通过关闭参数检查隐藏问题。

XML/Xacro/SRDF采用原生XML注释，YAML/RViz/CMake等采用原生井号注释；
二进制、严格JSON及此类空marker用配套逐文件说明，保留数据和产物身份。
注释检查比较Python AST、C/C++标记、XML属性/文本和YAML数据；
已有构建与回归仍需实际运行，不能只凭“看起来只是注释”宣称通过。
