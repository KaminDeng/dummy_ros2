# 当前协议与组件输入

ROS生产、SIL与离线工具只消费明确选择的DPSDK安装发布。C++公共目标为`DPSDKHostSDK::dummy_protocol`；Python布局由同版`dp_protocol_layout_reader.py`读取。

旧V1、wire2.4 Codec、镜像IDL和146页回放专用文档已随废弃协议删除。历史采集原件未重写；当前故障收尾测试使用`src/dummy_motion_client/test/current_fault_fixture.hpp`合成输入，保留收据、当前Fault观测和合法SHA坏尾拒绝覆盖。

业务模型、profile和未资格策略继续保留；部署安装模型、SIL资格与动态解码以显式安装SDK为准，不能从历史合同目录获得运行许可。
