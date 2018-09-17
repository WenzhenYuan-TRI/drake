/// To see whether the program can detect the state of the fingers

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_hand_state.h" 
#include "drake/examples/allegro_hand/allegro_lcm.h" 
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"

#include "robotlocomotion/pose_t.hpp"

#include <iostream>
#include <iomanip>  
#include <Eigen/Dense>



namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsParameters;
using drake::manipulation::planner::DoDifferentialInverseKinematics;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Frame;

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";
const char* const kLcmObjTargetChannel = "TARGET_POS_STATUS";

class ConstantPositionInput{
  public:
    ConstantPositionInput(){
      lcm_.subscribe(kLcmStatusChannel,
                    &ConstantPositionInput::HandleStatus, this);
      lcm_.subscribe(kLcmObjTargetChannel,
                    &ConstantPositionInput::TargetFrameInput, this);

      // load plant of the hand; only for right hand in this case
      const std::string HandPath = FindResourceOrThrow("drake/manipulation/models"
                  "/allegro_hand_description/sdf/allegro_hand_description_right.sdf");
      multibody::parsing::AddModelFromSdfFile(HandPath, plant_);
      const auto& joint_hand_root = plant_->GetBodyByName("hand_root");
      plant_->AddJoint<multibody::WeldJoint>( "weld_hand", plant_->world_body(),
         {}, joint_hand_root, {}, Isometry3<double>::Identity());
      plant_->Finalize();
      plant_context_ = plant_->CreateDefaultContext();
      //todo -- ini Px
      MatrixX<double> Px, Py;
      GetControlPortMapping(*plant_, Px, Py);
      Px_half = Px.block(0,0,16,16);

    }

    void Run() {
        allegro_command.num_joints = kAllegroNumJoints;
        allegro_command.joint_position.resize(kAllegroNumJoints, 0.);
        allegro_command.num_torques = 0;
        allegro_command.joint_torque.resize(kAllegroNumJoints, 0.);

        flag_moving = true;
        Eigen::VectorXd target_joint_pose(kAllegroNumJoints);
        target_joint_pose.setZero();
        target_joint_pose(0) = 1.396;
        target_joint_pose(1) = 0.3;
        // move to the position of opening the hand
        MovetoPositionUntilStuck(target_joint_pose);

        // -------------
        while(true) {
            while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }
        }
    }

  private:

  inline void PublishPositionCommand(const Eigen::VectorXd target_joint_pose){
      Eigen::VectorXd::Map(&allegro_command.joint_position[0], 
                             kAllegroNumJoints) = target_joint_pose;
      lcm_.publish(kLcmCommandChannel, &allegro_command);
  }

  inline void MovetoPositionUntilStuck(const Eigen::VectorXd target_joint_pose){
      PublishPositionCommand(target_joint_pose);
      for (int i = 0; i<40; i++){
          while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }
      }
      while (flag_moving) {
          while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }
      }
  }

  void iniIKtarget(std::vector<Isometry3<double>> finger_target, /* finger position in the tip frame*/
      std::vector<Isometry3<double>> frame_transfer,    /* transform of the fingertip in the world frame*/
      std::vector<int> finger_id, double target_tor = 5e-3){

    DRAKE_DEMAND(finger_id.size() == finger_target.size());
    DRAKE_DEMAND(finger_id.size() == frame_transfer.size());

    multibody::InverseKinematics ik_(*plant_);
    const Frame<double>& WorldFrame= plant_->world_frame();

    Eigen::Vector3d p_W_tor = Eigen::Vector3d::Constant(target_tor);
    Eigen::Vector3d p_W;
    int cur_finger_id;
    for(unsigned int i=0; i < finger_id.size(); i++){
      cur_finger_id = finger_id[i];
      DRAKE_DEMAND(cur_finger_id >= 0 && cur_finger_id < 4);
      Eigen::Vector3d p_TipFinger(0, 0, 0.0267/* + 0.012*/);    
      const Frame<double>* fingertip_frame{nullptr};
      if(cur_finger_id == 0){ // if it's the thumb
          p_TipFinger(2) = 0.0423/*+ 0.012*/;
          fingertip_frame =&( plant_->GetFrameByName("link_15"));

          finger_target[i].translation()+=Eigen::Vector3d(0,0,0.012);
      }
      else{
          fingertip_frame = &(plant_->GetFrameByName("link_"+std::to_string(
                                                    cur_finger_id * 4 - 1)));

          finger_target[i].translation()+=Eigen::Vector3d(0,0,0.012);
      }

      Isometry3<double> finger_target_transfer = frame_transfer[i] * finger_target[i];
      p_W = finger_target_transfer.translation();
      saved_target[cur_finger_id] = finger_target_transfer;
      ik_.AddPositionConstraint(*fingertip_frame, p_TipFinger, 
                            WorldFrame, p_W-p_W_tor, p_W+p_W_tor);
    }

    // ini guess
    // ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), saved_joint_command);

    const auto result = ik_.get_mutable_prog()->Solve();
    std::cout<<"Did IK find result? "<<result<<"  "<<
         solvers::SolutionResult::kSolutionFound<<std::endl;
    const auto q_sol = ik_.prog().GetSolution(ik_.q());
    Eigen::VectorXd q_sol_only_hand = Px_half * q_sol; /* the joint position in the pre-set order*/
    saved_joint_command = q_sol;  // this saved command is for 

    SendJointCommand();
  }

  // using differential IK to update the target
  void updateIKtarget(std::vector<Isometry3<double>> finger_target, /* finger position in the tip frame*/
      std::vector<Isometry3<double>> frame_transfer,    /* transform of the fingertip in the world frame*/
      std::vector<int> finger_id){
  
    DRAKE_DEMAND(finger_id.size() == finger_target.size());
    DRAKE_DEMAND(finger_id.size() == frame_transfer.size());

    // update context of the plant
    plant_->tree().get_mutable_multibody_state_vector(plant_context_.get()).head(16) = 
        saved_joint_command;

    // set parameters
    std::unique_ptr<DifferentialInverseKinematicsParameters> params_ =
       std::make_unique<DifferentialInverseKinematicsParameters>(16, 16);
    params_->set_nominal_joint_position(saved_joint_command);
    params_->set_unconstrained_degrees_of_freedom_velocity_limit(0.6);
    params_->set_timestep(1e-3);
    std::pair<VectorX<double>, VectorX<double>> q_bounds = {
        VectorX<double>::Constant(16, -0.5), VectorX<double>::Constant(16, 1.6)};
    std::pair<VectorX<double>, VectorX<double>> v_bounds = {
        VectorX<double>::Constant(16, -0.7), VectorX<double>::Constant(16, 0.7)};
    params_->set_joint_position_limits(q_bounds);
    params_->set_joint_velocity_limits(v_bounds);

    Eigen::Vector3d p_W;
    int cur_finger_id;
    bool target_updated_flag = false;
    for(unsigned int i=0; i < finger_id.size(); i++){
      cur_finger_id = finger_id[i];
      DRAKE_DEMAND(cur_finger_id >= 0 && cur_finger_id < 4);
      Eigen::Vector3d p_TipFinger(0, 0, 0.0267);    
      const Frame<double>* fingertip_frame{nullptr};
      if(cur_finger_id == 0){ // if it's the thumb
          p_TipFinger(2) = 0.0423/*+ 0.012*/;
          fingertip_frame =&( plant_->GetFrameByName("link_15"));

          finger_target[i].translation()+=Eigen::Vector3d(0,0,0.012);
      }
      else{
          fingertip_frame = &(plant_->GetFrameByName("link_"+std::to_string(
                                                    cur_finger_id * 4 - 1)));

          finger_target[i].translation()+=Eigen::Vector3d(0,0,0.012);
      }

      // todo : see whether to update

      Isometry3<double> finger_target_transfer = frame_transfer[i] * finger_target[i];
      if(! saved_target[cur_finger_id].isApprox(finger_target_transfer)){
        target_updated_flag = true;
        saved_target[cur_finger_id] = finger_target_transfer;
      }
      else continue;
      saved_target[cur_finger_id] = finger_target_transfer;

      // ----------- differentical IK -----------
      // desired

      DifferentialInverseKinematicsResult mbt_result = 
          DoDifferentialInverseKinematics(plant_->tree(), *plant_context_, finger_target_transfer,
                                         *fingertip_frame, *params_);
      saved_joint_command += mbt_result.joint_velocities.value() * params_->get_timestep();
      std::cout<<mbt_result.joint_velocities.value().transpose()<<std::endl;
    }

    if (target_updated_flag) SendJointCommand();
  }

  void SendJointCommand(){
      Eigen::VectorXd::Map(&allegro_command.joint_position[0], kAllegroNumJoints)
                             = saved_joint_command;
      lcm_.publish(kLcmCommandChannel, &allegro_command);
  }


  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;    
    hand_state.Update(status);
    flag_moving=!hand_state.IsAllFingersStuck();
  }

  // return the 4 frames of the preset positions of the target of the
  // fingertips, based on the frame of the object. Currently the parameters
  // are designed for the mug
  std::vector<Isometry3<double>>* CalcFingerTargetFrame(Isometry3<double> obj_frame) {

    std::vector<Isometry3<double>>* frame_poses; 
    const double MugHeight = 0.14;
    const double MugRadius = 0.04;
    const double central_point = MugHeight / 2;
    const double index_finger_interval = 0.045;
    const double thumb_partial = 0.005;

    Eigen::MatrixXd TargetGraspPos(4,3);
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
      frame_poses->push_back(X_BF * obj_frame);
    }

    return frame_poses;
  }

  void TargetFrameInput(const ::lcm::ReceiveBuffer*, const std::string&,
                        const robotlocomotion::pose_t* msg_mug_position) {

    // todo -- update the target finger from mug positionposition 

    Isometry3<double> mug_position;
    mug_position.matrix().setIdentity();
    mug_position.translation() << msg_mug_position->position.x,
                                  msg_mug_position->position.y, 
                                  msg_mug_position->position.z;
    mug_position.rotate(Eigen::Quaternion<double>(msg_mug_position->orientation.w,
                                          msg_mug_position->orientation.x,
                                          msg_mug_position->orientation.y,
                                          msg_mug_position->orientation.z));
    std::vector<Isometry3<double>>* track_position = CalcFingerTargetFrame(mug_position);


    std::vector<int> finger_id{0,1,2,3};
  
    std::vector<Isometry3<double>>* finger_target_pose{nullptr};
    Isometry3<double> P_F;
    P_F.matrix().setIdentity();
    for(int i=0; i<4; i++){
      P_F.translation() = Eigen::Vector3d(0,0,0);
      finger_target_pose->push_back(P_F);
    }

    if (!IK_inied) {
      iniIKtarget(*finger_target_pose, *track_position, finger_id, 5e-3);
    } 
    else {
      updateIKtarget(*finger_target_pose, *track_position, finger_id);
    }  
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
  lcmt_allegro_command allegro_command;
  AllegroHandState hand_state;

  bool flag_moving = true;

  bool IK_inied = false; 

  // params for hand plant
  MultibodyPlant<double>* plant_{nullptr};
  std::unique_ptr<systems::Context<double>> plant_context_;
  std::vector<Isometry3<double>> saved_target;
  // Eigen::VectorXd saved_joint_position;
  Eigen::VectorXd saved_joint_command;
  MatrixX<double> Px_half;
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
