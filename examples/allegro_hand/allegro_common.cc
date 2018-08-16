#include "drake/examples/allegro_hand/allegro_common.h"
#include <iostream>

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


namespace drake {
namespace examples {
namespace allegro_hand {

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(kAllegroNumJoints);
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.5;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    // (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
    (*Kd)[i] = 5e-2;
  }
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);
}


void GetControlPortMapping(multibody::multibody_plant::MultibodyPlant<double>& plant, 
                           MatrixX<double>& Px, MatrixX<double>& Py) {
  // Create a map from fingers dofs in a "desired order" into the order these
  // same dofs are arranged in the state vector.
  // This is the projection matrix Px for the PID controller.

  // This map defines in what order we want the PID controller to "see" the
  // states.
  std::vector<std::string> joint_names_map;

  // This maps from a JointIndex in the MBP into our "user-defined" ordering.
  std::vector<int> joint_user_index_map(plant.num_joints(), 1e4);

  // Thumb
  joint_names_map.push_back("joint_12");
  joint_names_map.push_back("joint_13");
  joint_names_map.push_back("joint_14");
  joint_names_map.push_back("joint_15");

  // Index
  joint_names_map.push_back("joint_0");
  joint_names_map.push_back("joint_1");
  joint_names_map.push_back("joint_2");
  joint_names_map.push_back("joint_3");

    // Middle
  joint_names_map.push_back("joint_4");
  joint_names_map.push_back("joint_5");
  joint_names_map.push_back("joint_6");
  joint_names_map.push_back("joint_7");

    // End
  joint_names_map.push_back("joint_8");
  joint_names_map.push_back("joint_9");
  joint_names_map.push_back("joint_10");
  joint_names_map.push_back("joint_11");

  const int num_plant_positions = plant.num_positions();

  // Projection matrix. We include "all" dofs in the hand.
  // x_tilde = Px * x;
  // where:
  //  x is the state in the MBP.
  //  x_tilde is the state in the order we want it for our better understanding.
  Px.resize(kAllegroNumJoints * 2 , plant.num_multibody_states());
  Px.setZero();
  int joint_user_index = 0;  
  for (const auto& joint_name : joint_names_map) {
    const auto& joint = plant.GetJointByName(joint_name);

    // JointIndex to user index map.
    joint_user_index_map[joint.index()] = joint_user_index;

    const int q_index = joint.position_start();
    const int v_index = joint.velocity_start();
    Px(joint_user_index, q_index) = 1.0;
    Px(kAllegroNumJoints + joint_user_index, num_plant_positions + v_index) = 1.0;
    ++joint_user_index;
  }

    PRINT_VARn(Px);

  // Verify the mapping (or "projection") matrix Px only has a single 1.0 entry
  // per row/column.
  for (int i=0;i<plant.num_multibody_states();++i) {
    DRAKE_DEMAND(Px.row(i).sum() == 1.0);
  }

  // Build the projection matrix Py for the PID controller. Maps u_c from
  // the controller into u for the MBP, that is, u = Py * u_c where:
  //  u_c is the output from the PID controller in our prefered order.
  //  u is the output as require by the MBP.
  Py.resize(plant.num_actuated_dofs(), kAllegroNumJoints);
  Py.setZero();
  for (multibody::JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuated_dofs(); ++actuator_index) {
    const auto& actuator = plant.model().get_joint_actuator(actuator_index);
    const auto& joint = actuator.joint();
    Py(actuator_index, joint_user_index_map[joint.index()]) = 1.0;
  }

    PRINT_VARn(Py);

  // Verify the mapping (or "projection") matrix Py only has a single 1.0 entry
  // per row/column.
  // for (int i=0;i<plant.num_multibody_states();++i) {
  //   std::cout<<(Py.col(i).sum())<<"    ";
  // }
}

const Eigen::VectorXd SetTargetJointPose(){
  Eigen::VectorXd const_pos = Eigen::VectorXd::Zero(kAllegroNumJoints * 2) ;
  const_pos(0)=0.8;
  const_pos(1)=1.1;
  const_pos(2)=1;

  const_pos(4) = 0.5;
  const_pos(6) = 0.5;

  const_pos(12) = 0.2;


  return const_pos;
}






}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake