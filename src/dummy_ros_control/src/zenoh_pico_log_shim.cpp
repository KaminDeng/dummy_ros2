/**
 * @file zenoh_pico_log_shim.cpp
 * @brief 提供链接所需的zenoh日志printf符号并抑制其输出。
 * 本实现不读取格式/可变参数，不写stdio或诊断journal；返回0只表示此shim固定行为，不是传输成功或日志保存成功。
 */
#include <cstdarg>

/**
 * @brief 以C链接导出空日志适配。
 * @return 始终0；匿名const char*格式和省略参数均未使用。
 */
extern "C" int dp_zenoh_log_printf(const char*, ...) {
  return 0;
}
