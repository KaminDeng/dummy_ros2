# 历史J1激活验证器与wire-v1报告说明

[逐函数/参数/流程说明](policy-functions.dox)解释J1激活验证器的8个顶层函数、1个局部JSON回调、
固定字段/单位、停止包络、Host时间和证据读取，以及CLI消费记录检查边界；末节解释wire-v1历史报告。
两个V1原件已退出当前合同目录，仅可从Git历史 `c76b1eb06924010daa735288066f9c43a26ed6d1` 查阅。[source-map](source-map.json)明确记录Git对象与历史SHA，不表示当前文件存在；本页保留参数、数值及历史边界说明，不是当前V2激活入口。

结构验证成功仍固定返回`motion_allowed=false`和`STRUCTURALLY_VALID_NOT_AUTHORIZED`。
本工具不连接设备、不消费/更新许可账本，不能把`--check-json`成功当作完整激活通过。
Doxygen页面使用实际源码签名和参数表；未把文档页当作Python原符号的原生绑定。

从本仓库根运行下面配置，仅提取文档，不导入或执行validator：

```bash
doxygen docs/dummy/activation-identity-reading/Doxyfile
```

下文只解释已归档原件；当前V2操作从 [运行手册](../ros-runbook.md) 和 [当前codec说明](../generated-codecs/README.md) 进入。
