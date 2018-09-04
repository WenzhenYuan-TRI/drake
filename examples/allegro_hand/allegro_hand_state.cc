#include "drake/examples/allegro_hand/allegro_hand_state.h"

namespace drake {
namespace examples {
namespace allegro_hand {


void AllegroHandState::Update(const lcmt_allegro_status* allegro_state_msg){

  lcmt_allegro_status status = *allegro_state_msg;

  double* ptr = &(status.joint_velocity_estimated[0]);
  Eigen::ArrayXd joint_velocity = Eigen::Map<Eigen::ArrayXd>(
                                  ptr,
                                  AllegroNumJoints);

  is_joint_stuck = joint_velocity.abs() < velocity_thresh; 


  Eigen::Array<bool, Eigen::Dynamic, 1> reverse = joint_velocity * Eigen::Map<Eigen::ArrayXd>(
                            &(status.joint_torque_commanded[0]), AllegroNumJoints) < 0;
  is_joint_stuck += reverse; 

  is_finger_stuck.setZero();
  if (is_joint_stuck.segment(0,  4).all()) is_finger_stuck(0) = true;
  if (is_joint_stuck.segment(5,  3).all()) is_finger_stuck(1) = true;
  if (is_joint_stuck.segment(9,  3).all()) is_finger_stuck(2) = true;
  if (is_joint_stuck.segment(13, 3).all()) is_finger_stuck(3) = true;

  std::cout<<is_joint_stuck.segment(0,  4).transpose()<<std::endl;
}

Eigen::Vector4d AllegroHandState::FingerClosePose(int finger_index){
  Eigen::Vector4d pose;
  if (finger_index == 0)
    pose << 1.396,   0,  0.4, 1. ;
  else if (finger_index == 1)
    pose << -0.1,   1.6,  1.7, 1. ; 
  else if (finger_index == 2)
    pose <<  0,   1.6,  1.7, 1. ;
  else 
    pose <<  0.1,   1.6,  1.7, 1. ; 
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