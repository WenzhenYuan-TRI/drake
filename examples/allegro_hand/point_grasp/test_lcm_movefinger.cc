/// To see whether the program can detect the state of the fingers

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_hand_state.h" 
#include "drake/examples/allegro_hand/allegro_lcm.h" 
#include "drake/examples/allegro_hand/point_grasp/mug_state_set.h" 
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
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Frame;

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";
const char* const kLcmObjTargetChannel = "TARGET_POS_STATUS";

class ConstantPositionInput{
  public:
    ConstantPositionInput(){
      std::cout<<"program started\n";

      lcm_.subscribe(kLcmStatusChannel,
                    &ConstantPositionInput::HandleStatus, this);
      lcm_.subscribe(kLcmObjTargetChannel,
                    &ConstantPositionInput::TargetFrameInput, this);

      // load plant of the hand; only for right hand in this case
      plant_ = std::make_unique<MultibodyPlant<double>>(1e-3); 
      const std::string HandPath = FindResourceOrThrow("drake/manipulation/models"
                  "/allegro_hand_description/sdf/allegro_hand_description_right.sdf");
      std::cout<<"found path of model \n";
      multibody::parsing::AddModelFromSdfFile(HandPath, plant_.get());
      std::cout<<"parsing model to plant \n";
      const auto& joint_hand_root = plant_->GetBodyByName("hand_root");
      plant_->AddJoint<multibody::WeldJoint>( "weld_hand", plant_->world_body(),
         {}, joint_hand_root, {}, Isometry3<double>::Identity());
      plant_->Finalize();
      plant_context_ = plant_->CreateDefaultContext();
      //ini Px
      MatrixX<double> Px, Py;
      GetControlPortMapping(*plant_, Px, Py);
      Px_half = Px.block(0,0,16,16);

      following_target_frame = false;
      mug_pose_.matrix().setIdentity();
      saved_target.resize(4);
      track_position_.resize(4);

      // ini command for allegro hand
      allegro_command.num_joints = kAllegroNumJoints;
      allegro_command.joint_position.resize(kAllegroNumJoints, 0.);
      allegro_command.num_torques = 0;
      allegro_command.joint_torque.resize(kAllegroNumJoints, 0.);
    }

    void Run() {
        Eigen::VectorXd target_joint_pose(kAllegroNumJoints);
        target_joint_pose.setZero();
        target_joint_pose(0) = 1.396;
        target_joint_pose(1) = 0.3;
        // move to the position of opening the hand
        MovetoPositionUntilStuck(target_joint_pose);

        // grasp on points
        mug_state_.GetGraspTargetFrames(mug_pose_, &track_position_,
                                        &relative_finger_pose_);        
        iniIKtarget();
        following_target_frame = true;
        KeepMovingUntilStuck(200);
        following_target_frame = false;

        mug_state_.UpdateMugPose(mug_pose_);
        while(true) {
        // std::cout<<mug_pose_.matrix()<<std::endl;
        mug_state_.GetXRotatedTargetFrame(-10/180.0*M_PI, &track_position_);
        // std::cout<<track_position_[1].matrix()<<std::endl;
        iniIKtarget();
        std::cout<<"moving in one direction \n";
        // std::cout<<mug_pose_.matrix()<<std::endl;

        // following_target_frame = true;
        sleep(10);
        KeepMovingUntilStuck(2000);      
        std::cout<<"moving done \n";


        mug_state_.GetXRotatedTargetFrame(10/180.0*M_PI, &track_position_);
        // std::cout<<track_position_[1].matrix()<<std::endl;
        iniIKtarget(); 
        std::cout<<"moving in one direction \n";
        sleep(10);
        KeepMovingUntilStuck(2000);   
        std::cout<<"moving done \n";
      }
    }

  private:

  inline void PublishPositionCommand(const Eigen::VectorXd target_joint_pose){
      Eigen::VectorXd::Map(&allegro_command.joint_position[0], 
                             kAllegroNumJoints) = target_joint_pose;
      lcm_.publish(kLcmCommandChannel, &allegro_command);
  }

  inline void KeepMovingUntilStuck(int count = 40)
  {  
      for (int i = 0; i<count; i++){
          while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }
      }
      flag_moving = true;
      while (flag_moving) {
          while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) { }
      }
  }

  inline void MovetoPositionUntilStuck(const Eigen::VectorXd target_joint_pose){
      PublishPositionCommand(target_joint_pose);
      KeepMovingUntilStuck(100);
  }

  void iniIKtarget(double target_tor = 5e-3){

    multibody::InverseKinematics ik_(*plant_);
    const Frame<double>& WorldFrame= plant_->world_frame();

    Eigen::Vector3d p_W_tor = Eigen::Vector3d::Constant(target_tor);
    Eigen::Vector3d p_W;
    Isometry3<double> fingertip_offset;
    fingertip_offset.matrix().setIdentity();
    Isometry3<double> finger_target_transfer;
    fingertip_offset.translation() = Eigen::Vector3d(0, 0, 0.012);          
    for(unsigned int cur_finger=0; cur_finger < 4; cur_finger++){
      Eigen::Vector3d p_TipFinger(0, 0, 0.0267/* + 0.012*/);    
      const Frame<double>* fingertip_frame{nullptr};
      if(cur_finger == 0){ // if it's the thumb
          p_TipFinger(2) = 0.0423;
          fingertip_frame =&( plant_->GetFrameByName("link_15"));
          // fingertip_offset.translation() = Eigen::Vector3d(0, 0, 0.03);
          // finger_target_transfer = track_position_[cur_finger] * fingertip_offset;
      }
      else{
          fingertip_frame = &(plant_->GetFrameByName("link_"+std::to_string(
                                                    cur_finger * 4 - 1)));          
      }

      finger_target_transfer = track_position_[cur_finger] * fingertip_offset;
      p_W = finger_target_transfer.translation();
      saved_target[cur_finger] = finger_target_transfer;
      ik_.AddPositionConstraint(*fingertip_frame, p_TipFinger, 
                            WorldFrame, p_W-p_W_tor, p_W+p_W_tor);
    }

    const auto result = ik_.get_mutable_prog()->Solve();
    std::cout<<"Did IK find result? "<<result<<"  "<<
         solvers::SolutionResult::kSolutionFound<<std::endl;
    const auto q_sol = ik_.prog().GetSolution(ik_.q());
    saved_joint_command = q_sol;  // this saved command is for 
    SendJointCommand();
  }

  // using differential IK to update the target
  void updateIKtarget() {
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
    bool target_updated_flag = false;
    Isometry3<double> fingertip_offset;
    fingertip_offset.matrix().setIdentity();
    fingertip_offset.translation() += Eigen::Vector3d(0, 0, 0.012);
    for(unsigned int cur_finger=0; cur_finger < 4; cur_finger++){
      // if the new position is similar to the saved one, don't need to update
      Isometry3<double> finger_target_transfer = mug_pose_ * 
          relative_finger_pose_[cur_finger] * fingertip_offset;
      if(saved_target[cur_finger].isApprox(finger_target_transfer)) continue;

      std::cout<<"differentical IK \n";

      Eigen::Vector3d p_TipFinger(0, 0, 0.0267);    
      const Frame<double>* fingertip_frame{nullptr};
      if(cur_finger == 0){ // if it's the thumb
          p_TipFinger(2) = 0.0423;
          fingertip_frame =&( plant_->GetFrameByName("link_15"));
      }
      else{
          fingertip_frame = &(plant_->GetFrameByName("link_"+std::to_string(
                                                    cur_finger * 4 - 1)));
      }

      // differentical IK
      Vector6<double> V_WE_desired =
          manipulation::planner::ComputePoseDiffInCommonFrame(
          saved_target[cur_finger], finger_target_transfer);
      MatrixX<double> J_WE(6, 16);
      plant_->tree().CalcFrameGeometricJacobianExpressedInWorld(
          *plant_context_, *fingertip_frame, Vector3<double>::Zero(), &J_WE);

      DifferentialInverseKinematicsResult mbt_result = 
          DoDifferentialInverseKinematics(saved_joint_command, Eigen::VectorXd::Zero(16), 
                                          V_WE_desired, J_WE, *params_);
      std::cout<<mbt_result.status<<std::endl;
      if (mbt_result.status == DifferentialInverseKinematicsStatus::kNoSolutionFound) {
        saved_joint_command += mbt_result.joint_velocities.value();
        saved_target[cur_finger] = finger_target_transfer;
        target_updated_flag = true;
      }
    }

    if (target_updated_flag) SendJointCommand();
  }

  void SendJointCommand(){
      Eigen::VectorXd::Map(&allegro_command.joint_position[0], kAllegroNumJoints)
                             = Px_half * saved_joint_command;
      lcm_.publish(kLcmCommandChannel, &allegro_command);
  }


  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;    
    hand_state.Update(status);
    flag_moving=!hand_state.IsAllFingersStuck();
  }

  void TargetFrameInput(const ::lcm::ReceiveBuffer*, const std::string&,
                        const robotlocomotion::pose_t* msg_mug_pose_) {

    // update the target finger from mug positionposition     
    mug_pose_.matrix().setIdentity();
    mug_pose_.translation() << msg_mug_pose_->position.x,
                               msg_mug_pose_->position.y, 
                               msg_mug_pose_->position.z;
    mug_pose_.rotate(Eigen::Quaternion<double>(msg_mug_pose_->orientation.w,
                                               msg_mug_pose_->orientation.x,
                                               msg_mug_pose_->orientation.y,
                                               msg_mug_pose_->orientation.z));
    if (!following_target_frame) return;

    updateIKtarget(); 
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
  lcmt_allegro_command allegro_command;
  AllegroHandState hand_state;

  bool flag_moving = true;
  bool following_target_frame = false; 

  bool IK_inied = false; 

  // params for hand plant
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  std::vector<Isometry3<double>> saved_target;
  Eigen::VectorXd saved_joint_command;
  MatrixX<double> Px_half;

  // control part for the target finger positions
  SetMugStateControl mug_state_;
  Isometry3<double> mug_pose_;
  std::vector<Isometry3<double>> track_position_;
  std::vector<Isometry3<double>> relative_finger_pose_;
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
