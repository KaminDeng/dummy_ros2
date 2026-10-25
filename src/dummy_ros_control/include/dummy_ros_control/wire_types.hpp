/** @brief 当前类型来自安装协议；冻结2.4仅供显式历史回归翻译单元。 */
#pragma once
#if defined(DUMMY_LEGACY_WIRE_2_4)
#include "dp_dummy_robot_v2.hpp"
namespace dummy_wire = dp::protocol::dummy_robot_v2;
namespace dummy_wire_schema = dp::protocol::dummy_robot_v2;
#else
#include "dummy_ref/wire_validation.hpp"
namespace dummy_wire = dp::iface;
namespace dummy_wire_schema = dummy_ref::protocol;
#endif
