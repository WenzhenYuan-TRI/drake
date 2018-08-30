#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h" 
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"

#include <iostream>

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";

class ConstantPositionInput{
  public:
    ConstantPositionInput(){
        lcm_.subscribe(kLcmStatusChannel,
                    &ConstantPositionInput::HandleStatus, this);
    }

    void Run() {
        lcmt_allegro_command allegro_command;
        allegro_command.num_joints = kAllegroNumJoints;
        allegro_command.joint_position.resize(kAllegroNumJoints, 0.);
        allegro_command.num_torques = 0;
        allegro_command.joint_torque.resize(kAllegroNumJoints, 0.);

        allegro_command.joint_position = { 1.396,     0,  0.4, 1., 
                                       -0.1,   1.6,  1.7, 1.,
                                          0,   1.6,  1.7, 1., 
                                        0.1,   1.6,  1.7, 1.};

        while (true){
          while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }

          allegro_command.utime = allegro_status_.utime;
          lcm_.publish(kLcmCommandChannel, &allegro_command);
          sleep(0.1);
        }
    }

  private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;    
    std::cout<<allegro_status_.joint_position_measured[0]<<std::endl;
    std::cout<<allegro_status_.joint_torque_commanded[0]<<std::endl<<std::endl;
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
};




int do_main() {
  ConstantPositionInput runner;
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::allegro_hand::do_main();
}
