/** @file root_component_stimulus.hpp
 * @brief 虚拟PTY测试刺激：业务只用真实Owned，管道正文复用中央canonical记录。
 * @details 控制头是同一Host ABI的测试容器，不是协议业务DTO、网络Root字节或正式capture格式。 */
#pragma once
#include "owned_message_fixtures.hpp"
#include "dp_comm/receive_info.hpp"
#include "canonical_records.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <type_traits>
#include <unistd.h>
namespace root_fixture {
/** @brief 场景控制及独立Source元数据；不重复业务字段。 */
struct Control {
 dp::comm::SourceId status_source{},state_source{},evidence_source{};
 bool done=false,send_status=true,send_state=true,send_evidence=false;
 unsigned omit_state_field=0;
 std::uint64_t root_generation=0,invalid_root_source_boot=0;
 bool invalid_root_file_id=false;
};
static_assert(std::is_trivially_copyable_v<Control>,"同宿主控制头须可平凡复制");
struct Stimulus : Control {
 dummy_ref::messages::Status status=owned_fixture::status();
 dummy_ref::messages::State state=owned_fixture::state();
 dummy_ref::messages::Evidence evidence=owned_fixture::evidence();
};
/** @brief 只搬测试管道字节；处理短读写及EINTR，不解释业务。 */
inline bool write_bytes(int fd,const void* input,std::size_t size){
 const auto* bytes=static_cast<const std::uint8_t*>(input);
 for(std::size_t done=0;done<size;){const auto count=::write(fd,bytes+done,size-done);if(count>0)done+=std::size_t(count);else if(count<0&&errno==EINTR)continue;else return false;}return true;
}
inline bool read_bytes(int fd,void* output,std::size_t size){
 auto* bytes=static_cast<std::uint8_t*>(output);
 for(std::size_t done=0;done<size;){const auto count=::read(fd,bytes+done,size-done);if(count>0)done+=std::size_t(count);else if(count<0&&errno==EINTR)continue;else return false;}return true;
}
/** @brief Owned不是可平凡复制对象；测试IPC也必须用同一中央生成记录。 */
template<class T> bool write_value(int fd,const T& value){
 using Codec=dp::protocol::canonical::Traits<T>;
 std::array<std::uint8_t,Codec::kMaxEncodedSize> bytes{};std::size_t size=0;
 if(!Codec::encode(value,bytes.data(),bytes.size(),&size)||size>UINT32_MAX)return false;
 const auto count=static_cast<std::uint32_t>(size);
 return write_bytes(fd,&count,sizeof(count))&&write_bytes(fd,bytes.data(),size);
}
template<class T> bool read_value(int fd,T& value){
 using Codec=dp::protocol::canonical::Traits<T>;
 std::uint32_t count=0;std::array<std::uint8_t,Codec::kMaxEncodedSize> bytes{};
 return read_bytes(fd,&count,sizeof(count))&&count<=bytes.size()&&read_bytes(fd,bytes.data(),count)&&Codec::decode(bytes.data(),count,&value);
}
inline bool write_stimulus(int fd,const Stimulus& value){
 const auto& control=static_cast<const Control&>(value);
 return write_bytes(fd,&control,sizeof(control))&&(value.done||
  (write_value(fd,value.status)&&write_value(fd,value.state)&&write_value(fd,value.evidence)));
}
inline bool read_stimulus(int fd,Stimulus& value){
 auto& control=static_cast<Control&>(value);
 return read_bytes(fd,&control,sizeof(control))&&(value.done||
  (read_value(fd,value.status)&&read_value(fd,value.state)&&read_value(fd,value.evidence)));
}
}
