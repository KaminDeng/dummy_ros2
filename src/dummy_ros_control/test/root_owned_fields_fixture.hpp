/** @file root_owned_fields_fixture.hpp
 * @brief 测试直接操作真实关节records的几何/质量辅助，不定义业务消息或布局。 */
#pragma once
#include "dummy_ros_control/message_presence.hpp"
#include <array>
#include <cassert>
#include <vector>
#include <algorithm>
#include "canonical_records.h"
#include "dummy_ros_control/capture_record.hpp"
namespace root_fixture {
inline std::uint8_t configured(const dummy_ref::messages::Command& command){std::uint8_t mask=0;assert(command.joints());for(const auto& j:*command.joints())if(j.joint_id()&&j.configured()&&*j.configured())mask|=std::uint8_t(1U<<*j.joint_id());return mask;}
template<class T> bool canonical_equal(const std::uint8_t* bytes,std::size_t size,const T& value){using Traits=dp::protocol::canonical::Traits<T>;std::array<std::uint8_t,Traits::kMaxEncodedSize> encoded{};std::size_t written=0;return Traits::encode(value,encoded.data(),encoded.size(),&written)&&size==written&&std::equal(encoded.begin(),encoded.begin()+written,bytes);}
inline dp::comm::SourceId capture_source(const std::uint8_t* bytes){dummy_ros_control::CaptureRecordHeader header;const bool valid=dummy_ros_control::decode_capture_header(bytes,dummy_ros_control::kCaptureHeaderSize,&header);assert(valid);return header.source;}

template<class State,class Setter> void set_joint_values(State& state,Setter setter){
 assert(state.joints());
 for(auto& joint:*state.joints()){assert(joint.joint_id()&&*joint.joint_id()<6);setter(joint,*joint.joint_id());}
}
inline void set_configured(dummy_ref::messages::State& state,std::uint8_t mask){set_joint_values(state,[&](auto& j,unsigned axis){j.configured_quality()=bool(mask&(1U<<axis));});}
inline void set_position_quality(dummy_ref::messages::State& state,std::uint8_t mask){set_joint_values(state,[&](auto& j,unsigned axis){j.position_valid_quality()=bool(mask&(1U<<axis));});}
inline void set_velocity_quality(dummy_ref::messages::State& state,std::uint8_t mask){set_joint_values(state,[&](auto& j,unsigned axis){j.velocity_valid_quality()=bool(mask&(1U<<axis));});}
inline void set_raw_quality(dummy_ref::messages::State& state,std::uint8_t mask){set_joint_values(state,[&](auto& j,unsigned axis){j.raw_position_valid_quality()=bool(mask&(1U<<axis));});}
inline void set_hold_quality(dummy_ref::messages::State& state,std::uint8_t mask){set_joint_values(state,[&](auto& j,unsigned axis){j.hold_valid_quality()=bool(mask&(1U<<axis));});}
inline std::uint8_t position_quality(const dummy_ref::messages::State& state){return dummy_ros_control::position_valid_joint_mask(state);}
inline std::uint8_t velocity_quality(const dummy_ref::messages::State& state){return dummy_ros_control::velocity_valid_joint_mask(state);}
inline std::uint8_t raw_quality(const dummy_ref::messages::State& state){return dummy_ros_control::raw_position_joint_mask(state);}
inline std::uint8_t hold_quality(const dummy_ref::messages::State& state){std::uint8_t mask=0;assert(state.joints());for(const auto& j:*state.joints())if(j.joint_id()&&j.hold_valid_quality()&&*j.hold_valid_quality())mask|=std::uint8_t(1U<<*j.joint_id());return mask;}
template<class State,class Getter> auto joint_values(const State& state,Getter getter){
 using Value=std::decay_t<decltype(getter(*state.joints()->begin()))>;
 std::array<Value,6> values{};assert(state.joints());
 for(const auto& joint:*state.joints()){assert(joint.joint_id()&&*joint.joint_id()<6);values[*joint.joint_id()]=getter(joint);}return values;
}
#define ROOT_FIXTURE_JOINT_VALUES(NAME,FIELD) \
 template<class State,class Values> void set_##NAME(State& state,const Values& values){set_joint_values(state,[&](auto& j,unsigned axis){j.FIELD()=values[axis];});} \
 template<class State> auto NAME(const State& state){return joint_values(state,[](const auto& j){assert(j.FIELD());return *j.FIELD();});}
ROOT_FIXTURE_JOINT_VALUES(position,position_rad)
ROOT_FIXTURE_JOINT_VALUES(velocity,estimated_velocity_rad_s)
ROOT_FIXTURE_JOINT_VALUES(raw_position,raw_motor_position_rev)
ROOT_FIXTURE_JOINT_VALUES(hold_position,hold_position_rad)
ROOT_FIXTURE_JOINT_VALUES(feedback_sequence,feedback_seq)
ROOT_FIXTURE_JOINT_VALUES(feedback_age,can_rx_age_us)
#undef ROOT_FIXTURE_JOINT_VALUES
}
