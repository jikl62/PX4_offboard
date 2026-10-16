#include "authority_test_support.hpp"
#include <iostream>
using namespace authority_test;
int main(){
  auto c=config();f::Runtime r(c);auto s=sample();r.request("INTERACTIVE",{},s,0);
  s.stationary_seconds=.75;s.attitude.vz=.12;
  assert(r.request("AUTHORITY_RC",{},s,0).message=="AUTHORITY_SAFE_GROUND_REQUIRED");
  const auto failed=r.authority_json(s);
  assert(failed.find("\"authority_ground_failed_mask\":24")!=std::string::npos);
  assert(failed.find("\"stationary_seconds\":0.75")!=std::string::npos);
  assert(failed.find("\"required_ground_seconds\":2")!=std::string::npos);
  assert(failed.find("\"ground_vz_tolerance\":0.1")!=std::string::npos);
  s.stationary_seconds=3;s.attitude.vz=0;
  const auto recovered=r.authority_json(s);
  assert(recovered.find("\"authority_ground_failed_mask\":0")!=std::string::npos);
  assert(recovered.find("\"last_authority_ground_failed_mask\":24")!=std::string::npos);
  assert(r.request("AUTHORITY_RC",{},s,0).accepted);
  assert(r.authority_json(s).find("\"last_authority_ground_failed_mask\":0")!=std::string::npos);
  std::cout<<"REVIEW_EXACT_FAILED_GROUND_SAMPLE_AND_CURRENT_DIAGNOSTICS PASS\n";
}
