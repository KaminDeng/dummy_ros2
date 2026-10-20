# 冻结合同数据阅读

这里提供外部中文说明，共119物理源、80种内容。首次归档基线为ROS HEAD `7c9285f6c63cc162245df1299a0b6b8522044d38`；公开版对其中6份来源元数据JSON做路径脱敏，不再声称这6份当前字节等于旧HEAD。当前SHA/bytes与归档旧SHA分别记录在机器映射中，原页锚保留为稳定导航。

脱敏仅涉及来源路径与公开说明，正式合同、原payload、数值及历史资格/运行结论不变。两份candidate manifest的原文SHA已改变，旧activation原文绑定失效，未重签旧activation或转移旧资格。

先读 [总览](00-overview.dox)，再按问题查：

- [全IDL字段](10-wire-fields.dox)：消息/服务分段、单位、有效位、回执与终态。
- [布局和枚举](20-layouts-enums.dox)：v1、当前2.5、legacy2.4，State350/301B。
- [模型与profile](30-models-profiles.dox)：缩放、方向、Home/Rest、限位和历史差异。
- [策略和metadata](40-policy-metadata.dox)：默认拒绝、候选参数、no-load与权限边界。
- [manifest及来源](50-manifests-origins.dox)：各SHA角色、application/trusted重构输入。
- [报告和病例](60-evidence-fixtures.dox)：R00–R08、NOT_RUN、FK/CAN数学及只读规划证据。
- [逐路径SHA表](70-source-map.dox) / [机器映射](source-map.json) / [内容覆盖账](reading-coverage.json)。

`.dox`用Doxygen页面与Markdown表格书写，可直接读源码或由Doxygen提取。这里不宣称IDL/proto原生符号绑定，也不把页提取、静态SHA或软件历史PASS当实机执行/机械安全资格。

## 生成可阅读HTML

需要本机可调用的 Doxygen（验证版本1.9.8），无需 ROS、SDK 构建或设备。从仓库根目录执行：

```sh
cd docs/dummy/contracts-data-reading
doxygen Doxyfile
```

打开 `/tmp/ros-contracts-data-reading/html/index.html`。输出位于 `/tmp` 的单层目录，Doxygen可直接创建；警告文件为 `/tmp/ros-contracts-data-reading-warnings.log`。配置仅以本目录 `*.dox` 为输入，XML同时输出供表项复核，不解析IDL原生符号，也不执行生成器。原件与逐路径SHA见 [source-map.json](source-map.json)，覆盖账见 [reading-coverage.json](reading-coverage.json)。
