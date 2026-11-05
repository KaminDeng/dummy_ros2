/** @file capture_record.hpp
 * @brief 独立DPRAW02容器头；业务载荷仅由中央canonical Traits编解码。
 */
#pragma once
#include "dp_comm/receive_info.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
namespace dummy_ros_control {
enum class CaptureRecordKind:std::uint8_t {kPage=1U,kState=2U,kStatus=3U,kManagementRequest=4U,kManagementResponse=5U};
/** @brief 来源为实际SDK身份；session/run/序号仅为捕获定位，不替代业务载荷。 */
struct CaptureRecordHeader {
  CaptureRecordKind kind=CaptureRecordKind::kStatus;
  std::uint16_t size=0U;
  std::uint64_t received_at_us=0U;
  dp::comm::SourceId source{};
  std::uint64_t session_id=0U,run_id=0U;
  std::uint32_t evidence_seq=0U;
};
inline constexpr std::array<std::uint8_t,8> kCaptureMagic{'D','P','R','A','W','0','2',0};
inline constexpr std::size_t kCaptureHeaderSize=59U;
namespace capture_detail {
inline void put(std::uint8_t* out,std::uint64_t value,std::size_t width) noexcept {for(std::size_t i=0;i<width;++i)out[i]=static_cast<std::uint8_t>(value>>(i*8U));}
inline std::uint64_t get(const std::uint8_t* in,std::size_t width) noexcept {std::uint64_t value=0U;for(std::size_t i=0;i<width;++i)value|=std::uint64_t(in[i])<<(i*8U);return value;}
}
/** @brief 容器字段使用明确小端位宽，不依赖C++struct或FlatBuffer布局。 */
inline std::array<std::uint8_t,kCaptureHeaderSize> encode_capture_header(const CaptureRecordHeader& header) noexcept {
  std::array<std::uint8_t,kCaptureHeaderSize> out{};
  out[0]=static_cast<std::uint8_t>(header.kind);capture_detail::put(out.data()+1U,header.size,2U);
  capture_detail::put(out.data()+3U,header.received_at_us,8U);capture_detail::put(out.data()+11U,header.source.boot_id,8U);
  capture_detail::put(out.data()+19U,header.session_id,8U);capture_detail::put(out.data()+27U,header.run_id,8U);
  capture_detail::put(out.data()+35U,header.evidence_seq,4U);std::copy_n(header.source.device.bytes,16U,out.begin()+39U);
  capture_detail::put(out.data()+55U,header.source.context_id,4U);return out;
}
/** @brief 完整验证后再替换输出；拒绝截断、未知kind和空载荷。 */
inline bool decode_capture_header(const std::uint8_t* bytes,std::size_t size,CaptureRecordHeader* out) noexcept {
  if(!bytes || !out || size<kCaptureHeaderSize || bytes[0]<1U || bytes[0]>5U)return false;
  CaptureRecordHeader header;
  header.kind=static_cast<CaptureRecordKind>(bytes[0]);header.size=static_cast<std::uint16_t>(capture_detail::get(bytes+1U,2U));
  if(!header.size)return false;
  header.received_at_us=capture_detail::get(bytes+3U,8U);header.source.boot_id=capture_detail::get(bytes+11U,8U);
  header.session_id=capture_detail::get(bytes+19U,8U);header.run_id=capture_detail::get(bytes+27U,8U);
  header.evidence_seq=static_cast<std::uint32_t>(capture_detail::get(bytes+35U,4U));std::copy_n(bytes+39U,16U,header.source.device.bytes);
  header.source.context_id=static_cast<std::uint32_t>(capture_detail::get(bytes+55U,4U));*out=header;return true;
}
}
