# 机器人业务数据

当前wire字段、枚举与发布身份统一读取选定DPSDK安装包中的`lib/cmake/DpProtocol`，不再维护ROS镜像布局。机器人运行合同位于该安装包的`share/dummy_ref/robot_contract`。

本目录保留[模型和profile](30-models-profiles.dox)、[策略和元数据](40-policy-metadata.dox)的业务说明。旧wire2.4字段表、146页fixture和冻结Codec逐符号页面已删除。业务模型/策略原文件保留，不能凭文件摘要取得实机资格。
