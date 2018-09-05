#include "drake/examples/allegro_hand/allegro_hand_state.h"

namespace drake {
namespace examples {
namespace allegro_hand {


void AllegroHandState::Update(const lcmt_allegro_status* allegro_state_msg){

  lcmt_allegro_status status = *allegro_state_msg;

  double* ptr = &(status.joint_velocity_estimated[0]);
  Eigen::ArrayXd joint_velocity = Eigen::Map<Eigen::ArrayXd>(
                                            ptr, AllegroNumJoints);
  Eigen::ArrayXd torque_command = Eigen::Map<Eigen::ArrayXd>( 
                    &(status.joint_torque_commanded[0]),  AllegroNumJoints);

  is_joint_stuck = joint_velocity.abs() < velocity_thresh; 

  Eigen::Array<bool, Eigen::Dynamic, 1> reverse = (joint_velocity * torque_command)
      < -0.001;
  is_joint_stuck += reverse; 

  is_finger_stuck.setZero();
  if (is_joint_stuck.segment(0,  4).all()) is_finger_stuck(0) = true;
  if (is_joint_stuck.segment(5,  3).all()) is_finger_stuck(1) = true;
  if (is_joint_stuck.segment(9,  3).all()) is_finger_stuck(2) = true;
  if (is_joint_stuck.segment(13, 3).all()) is_finger_stuck(3) = true;

  // // if (reverse.segment(2,  2).any()) is_finger_stuck(0) = true;
  if (reverse.segment(5,  3).any()) is_finger_stuck(1) = true;
  if (reverse.segment(9,  3).any()) is_finger_stuck(2) = true;
  if (reverse.segment(13, 3).any()) is_finger_stuck(3) = true;

}

Eigen::Vector4d AllegroHandState::FingerClosePose(int finger_index){
  Eigen::Vector4d pose;
  if (finger_index == 0)
    pose << 1.396,   0.85,  0., 1.3 ;
  else if (finger_index == 1)
    pose << 0.08,   0.9,  0.75, 1.5 ; 
  else if (finger_index == 2)
    pose << 0.1,   0.9,  .75, 1.5 ;
  else 
    pose <<  0.12,   0.9,  .75, 1.5 ; 
  return pose;
}

Eigen::Vector4d AllegroHandState::FingerOpenPose(int finger_index){
  Eigen::Vector4d pose;
  pose.setZero();
  if (finger_index == 0)
    pose << 0.263,   1.1,  0,0. ;
  return pose;
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake