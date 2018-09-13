#include "drake/examples/allegro_hand/point_grasp/point_grasp_common.h"

#include <iostream>

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::geometry::Sphere;
using multibody::Frame;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;
using systems::DiscreteUpdateEvent;

AllegroFingerIKMoving::AllegroFingerIKMoving(MultibodyPlant<double>& plant,
     MatrixX<double> Px) : plant_(&plant){
  Px_half = Px.block(0, 0, kAllegroNumJoints, plant_->num_positions());

  allegro_command.num_joints = kAllegroNumJoints;
  allegro_command.joint_position.resize(kAllegroNumJoints, 0.);
  allegro_command.num_torques = 0;
  allegro_command.joint_torque.resize(kAllegroNumJoints, 0.);

  Isometry3<double> ini_transform;
  ini_transform.matrix().setIdentity();
  saved_target.push_back(ini_transform);
  saved_target.push_back(ini_transform);
  saved_target.push_back(ini_transform);
  saved_target.push_back(ini_transform);
}

void AllegroFingerIKMoving::CommandFingerMotion(
      std::vector<Isometry3<double>> finger_target, 
      std::vector<Isometry3<double>> frame_transfer,
      std::vector<int> finger_id, double target_tor = 5e-3){

  DRAKE_DEMAND(finger_id.size() == finger_target.size());
  DRAKE_DEMAND(finger_id.size() == frame_transfer.size());

  multibody::InverseKinematics ik_(*plant_);
  const Frame<double>& WorldFrame= plant_->world_frame();

  // add position constraints for each fingers
  Eigen::Vector3d p_W_tor = Eigen::Vector3d::Constant(target_tor);
  Eigen::Vector3d p_W;
  int cur_finger_id;
  bool target_updated_flag = false;
  for(unsigned int i=0; i < finger_id.size(); i++){
    cur_finger_id = finger_id[i];
    DRAKE_DEMAND(cur_finger_id >= 0 && cur_finger_id < 4);
    Eigen::Vector3d p_TipFinger(0, 0, 0.0267 + 0.012);    
    const Frame<double>* fingertip_frame{nullptr};
    if(cur_finger_id == 0){ // if it's the thumb
        p_TipFinger(2) = 0.0423/*+ 0.012*/;
        fingertip_frame =&( plant_->GetFrameByName("link_15"));

        frame_transfer[i].translation()-=Eigen::Vector3d(0,0,0.012);
    }
    else{
        fingertip_frame = &(plant_->GetFrameByName("link_"+std::to_string(
                                                  cur_finger_id * 4 - 1)));
    }

    Isometry3<double> finger_target_transfer = frame_transfer[i] * finger_target[i];
    p_W = finger_target_transfer.translation();
    if(! finger_target_transfer.isApprox(saved_target[cur_finger_id])){
        target_updated_flag = true;
        saved_target[cur_finger_id] = finger_target_transfer;
    }
    
    std::cout<<p_W.transpose()<<std::endl;

    ik_.AddPositionConstraint(*fingertip_frame, p_TipFinger, 
                            WorldFrame, p_W-p_W_tor, p_W+p_W_tor);
  }
  if(!target_updated_flag) return;

  // ------------- For test -----------
  Eigen::VectorXd initial_guess(16);
  initial_guess.setZero();
  initial_guess.segment<3>(5) << 0.5, 0.5, 0.5;
  initial_guess.segment<3>(9) << 0.5, 0.5, 0.5;
  initial_guess.segment<3>(13) << 0.5, 0.5, 0.5;
  initial_guess.segment<4>(0) << 1,0, 1, 0.5;

  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), Px_half.transpose() * initial_guess);
  //-------------------------

  const auto result = ik_.get_mutable_prog()->Solve();
  std::cout<<"Did IK find result? "<<result<<"  "<< solvers::SolutionResult::kSolutionFound<<std::endl;
  const auto q_sol = ik_.prog().GetSolution(ik_.q());
  Eigen::VectorXd q_sol_only_hand = Px_half * q_sol;
  std::cout<<q_sol_only_hand.transpose()<<std::endl;

  // Send info to hand
  // just for test---
  // q_sol_only_hand.segment<4>(0)<<0.27,0,0,0;

  Eigen::VectorXd::Map(&allegro_command.joint_position[0], kAllegroNumJoints)
                             = q_sol_only_hand;
  // allegro_command.joint_position[4] = 0.1;
  lcm_.publish("ALLEGRO_COMMAND", &allegro_command);
}



MugSetting::MugSetting(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, 
              const Body<double>& mug_body) : 
    plant_(plant), scene_graph_(scene_graph), mug_body_(mug_body)
{
    IniRotAngles<<M_PI/2, 0 , 0;
    IniTransPosition<< 0.095,0.082, 0.095;
    AddGrippingPoint();
}


std::vector<Isometry3<double>> MugSetting::GenerateTargetFrame(){
  std::vector<Isometry3<double>> frame_poses; 
  const double central_point = MugHeight / 2;
  constexpr double index_finger_interval = 0.045;
  constexpr double thumb_partial = 0.005;
  
  TargetGraspPos.resize(4,3);
  TargetGraspPos.row(2) << 0, MugRadius, central_point;
  TargetGraspPos.row(1) << 0, MugRadius, central_point - index_finger_interval;
  TargetGraspPos.row(3) << 0, MugRadius, central_point + index_finger_interval;
  TargetGraspPos.row(0) << 0, -MugRadius, central_point - thumb_partial;
  Eigen::Vector4d TargetRotAngle(M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2);

  Eigen::Isometry3d X_BF; /* pose of cup upper frame F in mug body frame B */
  X_BF.makeAffine();
  for (int i=0; i < 4; i++){
    X_BF.translation() = TargetGraspPos.row(i);
    X_BF.linear() = math::RotationMatrix<double>(math::RollPitchYaw<double>(
                    Eigen::Vector3d(TargetRotAngle(i), 0, 0))).matrix();
    frame_poses.push_back(X_BF);
  }
  return frame_poses;
}




// This function is just for test
void MugSetting::AddGrippingPoint(){
  const double central_point = MugHeight / 2;
  constexpr double index_finger_interval = 0.045;
  constexpr double thumb_partial = 0.005;
  
  TargetGraspPos.resize(4,3);
  TargetGraspPos.row(2) << 0, MugRadius, central_point;
  TargetGraspPos.row(1) << 0, MugRadius, central_point - index_finger_interval;
  TargetGraspPos.row(3) << 0, MugRadius, central_point + index_finger_interval;
  TargetGraspPos.row(0) << 0, -MugRadius, central_point - thumb_partial;


  // display for test
  const geometry::VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 1.0));
  constexpr double display_radius = 0.005;
  Eigen::Isometry3d X_FS;
  for (int i=0; i < 4; i++){
    X_FS.translation() = TargetGraspPos.row(i);
    plant_->RegisterVisualGeometry(mug_body_, X_FS, Sphere(display_radius),
                                "point_2", red, scene_graph_);
  }

  // X_FS.translation() =Eigen::Vector3d(0.095, 0.057, 0.135);
  //   plant_->RegisterVisualGeometry(plant_->world_body(), X_FS, Sphere(display_radius*2),
  //                               "point_2", red, scene_graph_);
}


// Just for test
//template <typename T> 
void MugSetting::CalcPointPosition(/*const systems::Context<T>& context*/){
  // calculate frame
  // auto Frame_Mug = mug_body_.body_frame();
  // Isometry3<T> X_OW =
  //     CalcRelativeTransform(context, Frame_Mug, plant_->GetFrameByName("world"));
  // p_AQi->template topRows<3>() = X_AB * p_BQi.template topRows<3>();
  Eigen::Isometry3d X_OW;
  Eigen::Isometry3d X_PO;
  X_OW.translation() = IniTransPosition;
  X_OW.linear() =  math::RotationMatrix<double>
                  (math::RollPitchYaw<double>(IniRotAngles)).matrix();
  X_PO.linear() = Eigen::Matrix3d::Identity();
  X_OW.makeAffine();
  X_PO.makeAffine();

  Eigen::Vector3d pose;
  pose(3) = 1;
  for (int i=0; i < 4; i++){
      pose = TargetGraspPos.row(i).transpose();
      X_PO.translation() = pose;
      std::cout<< "Target Point Position: " <<X_PO.translation().transpose()<<std::endl;
      std::cout<<(X_OW * X_PO).translation().transpose()<<std::endl;
  }
}

void MugSetting::TestReachingPosition(MatrixX<double> Px){

  multibody::InverseKinematics ik_(*plant_);
  const Frame<double>& world_frame = plant_->world_frame();
  const Frame<double>& finger_frame = plant_->GetFrameByName("link_3");
  std::cout<<world_frame.body().name()<<finger_frame.body().name()<<std::endl;

  // Position constraint
  Eigen::Vector3d p_W(0.095, 0.057,  0.135);
  Eigen::Vector3d p_W_tor = Eigen::Vector3d::Ones() * 2e-3;
  Eigen::Vector3d p_TipFinger(0, 0, 0.0267+0.012);
  // p_W = Eigen::Vector3d(0.06, 0.058, 0.17);
  // p_TipFinger.setZero();
  p_W =  TargetGraspPos.row(1);

  Eigen::MatrixXd Px_half = Px.block(0,0,16,23);
  Eigen::VectorXd initial_guess(16);
  initial_guess.setZero();
  initial_guess.segment<3>(5) << 0.5, 0.5, 0.5;

  // ik_.AddPositionConstraint(finger_frame, p_TipFinger, world_frame, p_W-p_W_tor,
  //                           p_W+p_W_tor);
  ik_.AddPositionConstraint(finger_frame, p_TipFinger, plant_->GetFrameByName("main_body"), p_W-p_W_tor,
                            p_W+p_W_tor);
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), Px_half.transpose() * initial_guess);
  const auto result = ik_.get_mutable_prog()->Solve();
  std::cout<<"Did IK find result? "<<result<<"  "<< solvers::SolutionResult::kSolutionFound<<std::endl;
  const auto q_sol = ik_.prog().GetSolution(ik_.q());
  std::cout<<q_sol<<std::endl;

  Eigen::VectorXd q_sol_input = Px_half * q_sol;


  // Send info to hand
  ::lcm::LCM lcm_;
  lcmt_allegro_command allegro_command;
  allegro_command.num_joints = 16;
  allegro_command.joint_position.resize(16, 0.);
  allegro_command.num_torques = 0;
  allegro_command.joint_torque.resize(16, 0.);
  Eigen::VectorXd::Map(&allegro_command.joint_position[0], 16)
                             = q_sol_input;
  // allegro_command.joint_position[4] = 0.1;
   lcm_.publish("ALLEGRO_COMMAND", &allegro_command);

}

/*

ObjectStateHandler::ObjectStateHandler(){
  // Object position - 7 numbers; velocity - 6 numbers
  // position numbers: first 4 are quaternions, then 3 are positions
  this->DeclareInputPort(systems::kVectorValued, 13);

  this->DeclarePeriodicPublish(1);
}


void ObjectStateHandler::DoPublish(const Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>&) const {

  const auto& ObjectPos = this->EvalVectorInput(context, 0)->get_value();

  std::cout<<ObjectPos.transpose()<<std::endl;
}
*/

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake