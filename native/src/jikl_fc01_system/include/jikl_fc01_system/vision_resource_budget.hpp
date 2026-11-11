#pragma once
// 视觉身份的应用层预算；ROS 消息 schema 不变。DDS 反序列化可能已分配原始字符串，
// 此门仅保证本工程复制、缓存和序列化的每个身份最多 256 UTF-8 字节，不声称限制线上报文。
#include <cstddef>
#include <string_view>

namespace fc01::vision {
// 与既有速度预览的 256 字节来源预算一致；每个字段独立计数，不截断后冒充同一身份。
inline constexpr std::size_t identity_max_bytes = 256;

// 先查字节长度，再扫描最多 256 字节；空字段留给原协议有效位判断，不在这里补全身份。
// 拒绝 NUL、过长和非法 UTF-8，避免 C 字符串截断或生成无效 JSON；不分配、不修改输入。
inline bool identity_within_budget(std::string_view value) noexcept {
  if (value.size() > identity_max_bytes)
    return false;
  for (std::size_t offset = 0; offset < value.size();) {
    const auto lead = static_cast<unsigned char>(value[offset++]);
    if (!lead)
      return false;
    if (lead < 0x80)
      continue;
    unsigned count{};
    unsigned code{};
    unsigned minimum{};
    if (lead >= 0xc2 && lead <= 0xdf) {
      count = 1;
      code = lead & 0x1f;
      minimum = 0x80;
    } else if (lead >= 0xe0 && lead <= 0xef) {
      count = 2;
      code = lead & 0x0f;
      minimum = 0x800;
    } else if (lead >= 0xf0 && lead <= 0xf4) {
      count = 3;
      code = lead & 0x07;
      minimum = 0x10000;
    } else {
      return false;
    }
    if (offset + count > value.size())
      return false;
    while (count--) {
      const auto next = static_cast<unsigned char>(value[offset++]);
      if ((next & 0xc0) != 0x80)
        return false;
      code = (code << 6) | (next & 0x3f);
    }
    if (code < minimum || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff))
      return false;
  }
  return true;
}
} // namespace fc01::vision
