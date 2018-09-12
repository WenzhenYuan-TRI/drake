#include "drake/examples/allegro_hand/point_grasp/point_grasp_common.h"

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/lcm/drake_lcm.h"

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
    // X_BF.rotate(Eigen::AngleAxis<double>(TargetRotAngle(i), 
                                         // Eigen::Vector3d::UnitX()));
    std::cout<<X_BF.matrix()<<std::endl<<std::endl;
    frame_poses.push_back(X_BF);
  }
  return frame_poses;
}





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

  X_FS.translation()= Eigen::Vector3d(0.06, 0.058, 0.17);
  X_FS.translation()= Eigen::Vector3d(0.095, 0.017,  0.135);
  plant_->RegisterVisualGeometry(plant_->world_body(), X_FS, Sphere(display_radius),
                                "point_0", red, scene_graph_);

  // end display test
}

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
  // ik_.get_mutable_prog()->AddConstraint(
  //   solvers::internal::ParseQuadraticConstraint(
  //     ik_.q().segment<16>(7).cast<symbolic::Expression>().squaredNorm(), 1, 1));
  // ik_.get_mutable_prog()->AddConstraint(
  //   solvers::internal::ParseQuadraticConstraint(
  //     ik_.q().segment<4>().cast<symbolic::Expression>().squaredNorm(), 1, 1));

  // Position constraint
  Eigen::Vector3d p_W(0.095, 0.057,  0.135);
  Eigen::Vector3d p_W_tor = Eigen::Vector3d::Ones() * 2e-3;
  Eigen::Vector3d p_TipFinger(0, 0, 0.0267+0.012);
  // p_W = Eigen::Vector3d(0.06, 0.058, 0.17);
  // p_TipFinger.setZero();

  Eigen::MatrixXd Px_half = Px.block(0,0,16,23);
  Eigen::VectorXd initial_guess(16);
  initial_guess.setZero();
  initial_guess.segment<3>(5) << 0.5, 0.5, 0.5;

  ik_.AddPositionConstraint(finger_frame, p_TipFinger, world_frame, p_W-p_W_tor,
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

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake