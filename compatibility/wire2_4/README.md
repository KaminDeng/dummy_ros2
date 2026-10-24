# wire 2.4 历史兼容资源

这里保留原 `contracts/dummy_robot_v2_legacy_2_4/codec/` 中的原始字节，仅供历史原件解码和软件回归使用。当前控制合同位于 `contracts/dummy_robot_v2/`，本目录不构成可选择的控制 bundle，不包含运行或运动授权。

- `wire_layout.json`：供 `scripts/dummy/join_real_6dof_evidence.py` 读取 wire 2.4 证据；面板安装和部署会携带同一文件。
- `codec/`：三个冻结生成头，仅供 `fault_cleanup_receipt_tests` 重放历史故障载荷，保持与当前 wire 2.5 编译输入隔离。

不要重新生成或修改这些原件以匹配当前协议；当前数据继续使用 wire 2.5 布局。
