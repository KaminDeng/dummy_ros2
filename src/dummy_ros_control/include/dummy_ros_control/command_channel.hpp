#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {
void set_command_identity(const CommandIdentity &identity) noexcept;
void clear_command_identity() noexcept;
bool clear_command_identity_if(const CommandIdentity &expected) noexcept;
bool command_record_current(const CommandGenerationRecord &record) noexcept;
std::uint64_t publish_control_update(const double *positions, std::size_t count,
                                     std::int64_t update_time_ns, void *context) noexcept;
void controller_update_sink(const double *positions, std::size_t count,
                            std::int64_t update_time_ns, void *context) noexcept;
std::optional<CommandGenerationRecord> consume_control_update(
    std::uint64_t last_generation) noexcept;
}  // namespace dummy_ros_control
