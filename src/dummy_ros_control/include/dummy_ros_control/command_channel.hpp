/**
 * @file command_channel.hpp
 * @brief 声明控制器到硬件worker共用的进程内命令快照通道。
 * 实现中身份writer由mutex串行，控制update只读取原子身份及授权世代；更新不发送网络。清身份通过抬高世代和失效代水位阻断旧记录，不能据此宣称设备已停止。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {
/**
 * @brief 发布或刷新当前控制授权身份。
 * 同一授权只更新ticket；身份变化递增authority世代。此接口含writer mutex，不能泛称所有命令通道操作均无锁。
 * @param identity boot/session/reference/motion及非零设备票据。
 */
void set_command_identity(const CommandIdentity &identity) noexcept;
/**
 * @brief 无条件撤销当前本地授权世代。
 * 保留底层身份数值但使其世代不再匹配；当前已发布记录的代号成为失效水位，不发送Stop。
 */
void clear_command_identity() noexcept;
/**
 * @brief 仅在被退役身份仍为当前身份时撤销。
 * 防止迟到旧会话清理撤销新授权。
 * @param expected 预期退役的boot/session/reference/motion；ticket不参与比较。
 * @return true已撤销，false当前已是另一身份。
 */
bool clear_command_identity_if(const CommandIdentity &expected) noexcept;
/**
 * @brief 检查记录绑定的授权世代仍被发布且有效。
 * 不替代worker的ticket时效、设备身份、限位及commit检查。
 * @param record 待转交或待发送控制记录。
 * @return bool；要求非零authority_generation并同时等于当前/已发布世代。
 */
bool command_record_current(const CommandGenerationRecord &record) noexcept;
/**
 * @brief 保存1轴或6轴有限目标的最近控制快照。
 * 前后核对authority世代，实时路径不等待网络；返回非零不意味着worker已发送。
 * @param positions 调用期间有效的位置数组，rad。
 * @param count 只能1或6；未用轴填NaN。
 * @param update_time_ns 控制器ROS更新时间ns；当前实现不使用它判断新鲜度。
 * @param context 回调兼容上下文；当前实现未使用。
 * @return 宿主generation；非法输入或授权更新竞争返回0。
 */
std::uint64_t publish_control_update(const double *positions, std::size_t count,
                                     std::int64_t update_time_ns, void *context) noexcept;
/**
 * @brief 适配JTC的void控制更新回调。
 * 忽略publish_control_update的返回值；失效记录由后续消费者水位处理。
 * @param positions 有限rad位置数组。
 * @param count 1或6轴。
 * @param update_time_ns ROS控制更新时间ns，原样转交。
 * @param context 用户回调上下文，原样转交。
 */
void controller_update_sink(const double *positions, std::size_t count,
                            std::int64_t update_time_ns, void *context) noexcept;
/**
 * @brief 读取当前授权下高于消费/失效水位的最新更新。
 * 读取最新快照不消耗网络带宽；硬件层须再尝试有界队列。
 * @param last_generation 上次成功转交的宿主generation。
 * @return optional记录，旧授权或没有可见更新时为空。
 */
std::optional<CommandGenerationRecord> consume_control_update(
    std::uint64_t last_generation) noexcept;
// Reserve device-observed admitted/committed sequence numbers, never an ACK.
bool advance_control_sequence_floor(std::uint32_t floor) noexcept;
}  // namespace dummy_ros_control
