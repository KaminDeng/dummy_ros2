# 冻结激活验证器与wire-v1报告阅读入口

[逐函数/参数/流程说明](policy-functions.dox)解释J1激活验证器的8个顶层函数、1个局部JSON回调、
固定字段/单位、停止包络、Host时间和证据读取，以及CLI消费记录检查边界；末节解释wire-v1历史报告。
这两个原件参与manifest的payload SHA，保持每字节不变。[source-map](source-map.json)记录当前原件身份。

结构验证成功仍固定返回`motion_allowed=false`和`STRUCTURALLY_VALID_NOT_AUTHORIZED`。
本工具不连接设备、不消费/更新许可账本，不能把`--check-json`成功当作完整激活通过。
Doxygen页面使用实际源码签名和参数表；未把文档页当作Python原符号的原生绑定。

从本仓库根运行下面配置，仅提取文档，不导入或执行validator：

```bash
doxygen docs/dummy/activation-identity-reading/Doxyfile
```

源码、manifest及原报告摘要保持；修改验证器行为必须另走合同/发布流程，不能只刷新SHA放行。
