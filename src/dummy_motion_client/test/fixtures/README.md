# 业务数据保留范围

`trusted_sil_bundle`和`application_wire_source`只保留机器人model/profile业务材料，已移除独立Codec、IDL、枚举、wire布局与旧manifest。它们不再作为运行合同或SIL信任来源。

当前SIL/profile/候选资格链测试从显式`DPSDK_HOST_SDK_PREFIX/share/dummy_ref/robot_contract`读取。测试的外来bundle、未资格化候选与故障页由当前安装来源在临时目录生成；故障数据入口为同级`current_fault_fixture.hpp`，不重放旧wire2.4页。
