#include "drake/examples/allegro_hand/allegro_common.h"
#include <iostream>

namespace drake {
namespace examples {
namespace allegro_hand {

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(kAllegroNumJoints);
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.5;
  // (*Kp)[0] /= 5;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    // (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
    (*Kd)[i] = 1e-2;
  }
  // (*Kd)[0] /= 5;
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);


  Kp->segment<4>(4).setConstant(5);
  // Kd->segment<4>(0).setZero();
}

const MatrixX<double> GetControllerInputStateProjectionMat()
{
  MatrixX<double> state_projection = 
  // auto state_projection
  MatrixX<double>::Identity(
        kAllegroNumJoints * 2, kAllegroNumJoints * 2);
      // Eigen::MatrixXd::Identity(kAllegroNumJoints * 2, kAllegroNumJoints *Identity(kAllegroNumJoints * 2, kAllegroNumJoints * 2); 2);

  if(kAllegroNumJoints == 8){
    state_projection = Eigen::MatrixXd::Zero(kAllegroNumJoints * 2, kAllegroNumJoints * 2); 
    state_projection(0, 0) = 1;
    state_projection(1, 2) = 1;
    state_projection(2, 4) = 1;
    state_projection(3, 6) = 1;

    state_projection(4, 1) = 1;
    state_projection(5, 3) = 1;
    state_projection(6, 5) = 1;
    state_projection(7, 7) = 1;

    state_projection(8, 8) = 1;
    state_projection(9, 10) = 1;
    state_projection(10, 12) = 1;
    state_projection(11, 14) = 1;

    state_projection(12, 9) = 1;
    state_projection(13, 11) = 1;
    state_projection(14, 13) = 1;
    state_projection(15, 15) = 1;

  }
  return state_projection;
}

const Eigen::VectorXd SetTargetJointPose(){
  Eigen::VectorXd const_pos = Eigen::VectorXd::Zero(kAllegroNumJoints * 2) ;
  const_pos(1)=0.3;
  const_pos(2)=0.4;

  const_pos(4) = 0.8;
  const_pos(6) = 0.5;


  return const_pos;
}






}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake