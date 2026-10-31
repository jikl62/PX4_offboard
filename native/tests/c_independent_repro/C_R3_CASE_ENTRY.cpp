// Separate entry points into C's byte-identical source. Keep assertions enabled.
// The original main is renamed solely to select both cases after one aborts.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main c_r3_original_main
#include "C_R3_C11_C12_ORIGINAL.cpp"
#undef main
#pragma GCC diagnostic pop
#include <string>
int main(int argc,char** argv){
  if(argc==2&&std::string(argv[1])=="c11")late_arm_accept_labels_disarm_ack();
  else if(argc==2&&std::string(argv[1])=="c12")late_arm_rejected_triggers_new_disarm();
  else return 2;
}
