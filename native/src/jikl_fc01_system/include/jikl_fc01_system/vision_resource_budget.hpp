#pragma once
// 视觉身份的应用层预算；ROS 消息 schema 不变。DDS 反序列化可能已分配原始字符串，
// 此门仅保证本工程复制、缓存和序列化的每个身份最多 256 UTF-8 字节，不声称限制线上报文。
#include <cstddef>
#include <string_view>

namespace fc01::vision {
// 与既有速度预览的 256 字节来源预算一致；每个字段独立计数，不截断后冒充同一身份。
// UTF-8长度按字节，不按汉字数：例如一个常见汉字通常占3字节，256字节不等于256个汉字。
// constexpr使此上限在编译期可用；inline允许同一头被多个.cpp包含而不产生重复定义。
inline constexpr std::size_t identity_max_bytes = 256;

// 先查字节长度，再扫描最多 256 字节；空字段留给原协议有效位判断，不在这里补全身份。
// 拒绝 NUL、过长和非法 UTF-8，避免 C 字符串截断或生成无效 JSON；不分配、不修改输入。
// 输入string_view只借用原字符串的字节，不复制/持有它；本函数也不会把该视图保存起来。
// 返回true仅表示文本在资源/编码预算内；空串会通过，是否必填、是否获授权由调用者另查。
// noexcept表示异常不能逃出本函数；这里仅做有界扫描，不分配字符串、不写日志。
inline bool identity_within_budget(std::string_view value) noexcept {
  if (value.size() > identity_max_bytes)
    return false;
  for (std::size_t offset = 0; offset < value.size();) {
    // char在不同平台可能带符号；先转unsigned char，才能稳定按0..255识别UTF-8字节。
    // offset++先取当前下标再前进；lead为本次字符的首字节。
    const auto lead = static_cast<unsigned char>(value[offset++]);
    if (!lead)
      return false;
    if (lead < 0x80)
      continue;
    // ASCII在上方直接通过；下面按首字节确定还需1/2/3个续字节及最短合法码点。
    // count/code/minimum都是本次字符的局部工作量，不能跨字符借用上次解析结果。
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
    // 续字节必须以二进制10开头；每次将已积累的码点左移6位，再拼接本字节低6位。
    // 位运算&是取掩码、|是拼接；它们不是逻辑短路&&/||。
    while (count--) {
      const auto next = static_cast<unsigned char>(value[offset++]);
      if ((next & 0xc0) != 0x80)
        return false;
      code = (code << 6) | (next & 0x3f);
    }
    // 排除过长编码、Unicode上界以外的码点和UTF-16代理区，防止不同字节表示冒充同一身份。
    if (code < minimum || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff))
      return false;
  }
  return true;
}
} // namespace fc01::vision
