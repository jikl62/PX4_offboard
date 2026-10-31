#include "jikl_fc01_system/attitude_diagnostics.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
namespace a=fc01::attitude;
constexpr double pi=3.1415926535897932384626433832795;
static a::Quaternion roll(double deg){const double h=deg*pi/360;return {std::cos(h),std::sin(h),0,0};}
static a::Quaternion pitch(double deg){const double h=deg*pi/360;return {std::cos(h),0,std::sin(h),0};}
int main(){
 auto x=a::attitude_diagnostics(roll(10.15),{1,0,0,0});
 assert(x.valid&&std::abs(x.roll_deg-10.15)<1e-8);
 assert(std::abs(x.pitch_deg)<1e-8&&std::abs(x.tilt_deg-10.15)<1e-8);
 assert(std::abs(x.error_deg-10.15)<1e-8);
 auto y=a::attitude_diagnostics(pitch(-10.04),{1,0,0,0});
 assert(y.valid&&std::abs(y.pitch_deg+10.04)<1e-8&&std::abs(y.tilt_deg-10.04)<1e-8);
 auto same=a::attitude_diagnostics(roll(10.15),roll(10.15));
 assert(same.valid&&same.error_deg<1e-8);
 auto antipodal=a::attitude_diagnostics({-1,0,0,0},{1,0,0,0});
 assert(antipodal.valid&&antipodal.error_deg<1e-8);
 assert(!a::attitude_diagnostics({NAN,0,0,0},{1,0,0,0}).valid);
 assert(!a::attitude_diagnostics({1,0,0,0},{0,0,0,0}).valid);
 std::cout<<"FC01_P0_ATTITUDE_OBSERVATION_PASS\n";
}
