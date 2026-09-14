// Generated from robot_profiles/dummy/model.yaml. Do not edit.
#pragma once
#include <cstdint>
namespace dummy::model {
struct Joint { const char* name; uint8_t can_node; int8_t direction; double gear_ratio; double lower; double upper; };
inline constexpr Joint kJoints[6] = {
    {"Joint1", 1U, -1, 30.0, -2.9670597283903604, 2.9670597283903604},
    {"Joint2", 2U, 1, 30.0, -1.2740903539558606, 1.5707963267948966},
    {"Joint3", 3U, -1, 30.0, 0.6108652381980153, 3.1415926535897931},
    {"Joint4", 4U, 1, 24.0, -3.1415926535897931, 3.1415926535897931},
    {"Joint5", 5U, -1, 30.0, -2.0943951023931953, 2.0943951023931953},
    {"Joint6", 6U, -1, 50.0, -12.566370614359172, 12.566370614359172}
};
constexpr bool within_limits(const Joint& joint, double position_rad) {
    return position_rad >= joint.lower && position_rad <= joint.upper;
}
constexpr double preserve_multiturn_position(const Joint& joint, double position_rad) {
    return within_limits(joint, position_rad) ? position_rad : 0.0;
}
}  // namespace dummy::model
