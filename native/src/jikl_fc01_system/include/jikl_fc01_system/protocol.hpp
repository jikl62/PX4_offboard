#pragma once
// 本机控制协议：解析控制台的一行序号/操作/最多四个数值参数，并转义 JSON 文本。
// 这里只做语法和有限数检查；每个命令在 Runtime 中仍要经过状态及权限检查。
#include <charconv>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace fc01 {
// seq 为非零单调请求号；op 保留原协议命令字；values 仅存有限 double，不携带执行授权。
struct Request {
  std::uint64_t seq{};
  std::string op;
  std::vector<double> values;
};
inline Request parse_request(const std::string& line) {
  // 输入是已分帧的一行文本，输出拥有自己的字符串/数值副本；不借用line内部指针到返回后。
  // 语法错误抛invalid_argument，stod还可能抛out_of_range；分配错误也原样传播，上层决定断连。
  // 限制单行 1024 字节，避免未分帧的大输入无界占用资源。
  if (line.size() > 1024)
    throw std::invalid_argument("LINE_TOO_LONG");
  std::istringstream input(line);
  std::string token;
  Request r;
  if (!(input >> token >> r.op))
    throw std::invalid_argument("REQUEST_FORMAT");
  // 完整解析无符号请求号，禁止尾随垃圾和 0；防重放的严格递增由 UnixServer 维护。
  auto result = std::from_chars(token.data(), token.data() + token.size(), r.seq);
  // 两个指针标记半开区间[起点,终点)，result.ptr必须恰好到终点才算完整解析，避免“12abc”被当12。
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size() || r.seq == 0)
    throw std::invalid_argument("SEQUENCE_INVALID");
  // 每个数值必须被 stod 完整消费、为有限值且总数不超过四个，NaN/Inf 不进入控制器。
  while (input >> token) {
    std::size_t used{};
    double n = std::stod(token, &used);
    // &used让stod写回已消费字符数；n是局部数值，先完整/有限校验再入vector，不接受NaN/Inf。
    if (used != token.size() || !std::isfinite(n) || r.values.size() >= 4)
      throw std::invalid_argument("ARGUMENT_INVALID");
    r.values.push_back(n);
  }
  return r;
}
// 输出 JSON 字符串时转义引号、反斜杠及常见控制符；其它低控制字节按原策略丢弃。
inline std::string quote(const std::string& value) {
  // 只构造JSON字符串副本，不修改value；这里的转义不等于校验外部来源身份或给命令授权。
  std::string out = "\"";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += char(c);
    } else if (c == '\n')
      out += "\\n";
    else if (c == '\r')
      out += "\\r";
    else if (c == '\t')
      out += "\\t";
    else if (c >= 32)
      out += char(c);
  }
  return out + '"';
}
} // namespace fc01
