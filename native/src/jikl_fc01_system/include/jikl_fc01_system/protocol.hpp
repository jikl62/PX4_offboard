#pragma once
#include <charconv>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace fc01 {
struct Request {std::uint64_t seq{};std::string op;std::vector<double> values;};
inline Request parse_request(const std::string& line){
  if(line.size()>1024)throw std::invalid_argument("LINE_TOO_LONG");
  std::istringstream input(line);std::string token;Request r;
  if(!(input>>token>>r.op))throw std::invalid_argument("REQUEST_FORMAT");
  auto result=std::from_chars(token.data(),token.data()+token.size(),r.seq);
  if(result.ec!=std::errc{}||result.ptr!=token.data()+token.size()||r.seq==0)
    throw std::invalid_argument("SEQUENCE_INVALID");
  while(input>>token){
    std::size_t used{};double n=std::stod(token,&used);
    if(used!=token.size()||!std::isfinite(n)||r.values.size()>=4)
      throw std::invalid_argument("ARGUMENT_INVALID");
    r.values.push_back(n);
  }
  return r;
}
inline std::string quote(const std::string& value){
  std::string out="\"";
  for(unsigned char c:value){
    if(c=='"'||c=='\\'){out+='\\';out+=char(c);}
    else if(c=='\n')out+="\\n";
    else if(c=='\r')out+="\\r";
    else if(c=='\t')out+="\\t";
    else if(c>=32)out+=char(c);
  }
  return out+'"';
}
} // namespace fc01
