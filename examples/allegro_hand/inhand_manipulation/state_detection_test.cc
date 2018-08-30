/// To see whether the program can detect the state of the fingers

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h" 
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"

#include <iostream>
#include <Eigen/Dense>

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

        // while (true){
        for(int i=0; i<1000;i++){
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
    Eigen::VectorXd velocity_state(kAllegroNumJoints);
    Eigen::VectorXi stopped_motion(kAllegroNumJoints);
    Eigen::VectorXd dis_to_desired_pose(kAllegroNumJoints);
    Eigen::VectorXi reach_desired_pose(kAllegroNumJoints);
    Eigen::VectorXd exert_torque(kAllegroNumJoints);
    Eigen::VectorXd torque_by_velocity(kAllegroNumJoints);
    double velocity_thresh = 0.5;
    double position_thresh = 0.05; 

    for(int i = 0; i < kAllegroNumJoints; i++){
      velocity_state(i) = allegro_status_.joint_velocity_estimated[i];
      if (abs(velocity_state(i)) < velocity_thresh) stopped_motion(i) = 1;
      else stopped_motion(i) = 0;

      dis_to_desired_pose(i) = allegro_status_.joint_position_commanded[i] - 
                                allegro_status_.joint_position_measured[i];
      if (abs(dis_to_desired_pose(i) < position_thresh)) 
          reach_desired_pose(i) = 1;
      else reach_desired_pose(i) = 0;

      exert_torque(i) = allegro_status_.joint_torque_commanded[i];
      torque_by_velocity(i) = exert_torque(i) / (abs(velocity_state(i))+0.001);
    }

    // Test display
    std::cout<<
    torque_by_velocity.segment(0, 4).transpose()<<"    "<<
    torque_by_velocity.segment(4, 4).transpose()<<"    "<<
    torque_by_velocity.segment(8, 4).transpose()<<"    "<<
    torque_by_velocity.segment(12, 4).transpose()<<"    "<<
    std::endl;  
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
