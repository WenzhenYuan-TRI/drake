#include "drake/examples/allegro_hand/point_grasp/object_state_handler.h"

// for publish frame to lcm
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcm/drake_lcm.h"


namespace drake {
namespace examples {
namespace allegro_hand {

// Test: publish frame to lcm 
void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm) {
  DRAKE_DEMAND(poses.size() == names.size());
  lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Isometry3f pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[i] = names[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }
  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm->Publish(
      "DRAKE_DRAW_FRAMES_" + channel_name, bytes.data(), num_bytes, {});
}



ObjectFrameTracker::ObjectFrameTracker(const MultibodyPlant<double>& plant, 
     const std::vector<multibody::FrameIndex>& frames, 
     const std::string& obj_body_name) : 
          plant_(&plant), obj_body_name_(obj_body_name), frames_(frames){
    // ObjectFrameTracker input port with BasicVector for the sate x of the plant..

  state_input_port_ = this->DeclareInputPort(systems::kVectorValued, 
      plant_->num_multibody_states()).get_index();
  frame_poses_port_ = this->DeclareAbstractOutputPort(std::vector<Eigen::Isometry3d>(),
      &ObjectFrameTracker::CalcFramePoses).get_index();

  plant_context_ = plant_->CreateDefaultContext();

  this->DeclarePeriodicPublish(kObjectStatePublishPeriod);
}


void ObjectFrameTracker::CalcFramePoses(const systems::Context<double>& context,
        std::vector<Isometry3<double>>* frame_poses) const {
    const systems::BasicVector<double>* state_vector = this->EvalVectorInput(
                                            context, state_input_port_);
    plant_->tree().get_mutable_multibody_state_vector(plant_context_.get()) = 
        state_vector->get_value();
    frame_poses->clear();
    for (auto frame_index : frames_) {
      const auto X_WF = plant_->tree().CalcRelativeTransform(*plant_context_, 
          plant_->world_frame(), plant_->tree().get_frame(frame_index));
      frame_poses->push_back(X_WF);
    }
}

// This function is just for test: display the frames of the tracked points
void ObjectFrameTracker::DoPublish(const systems::Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>& ) const{
   const systems::BasicVector<double>* state_vector = this->EvalVectorInput(
                                            context, state_input_port_);
    plant_->tree().get_mutable_multibody_state_vector(plant_context_.get()) = 
        state_vector->get_value();
    std::vector<Isometry3<double>> frame_poses;
    for (auto frame_index : frames_) {
      const auto X_WF = plant_->tree().CalcRelativeTransform(*plant_context_, 
          plant_->world_frame(), plant_->tree().get_frame(frame_index));
      frame_poses.push_back(X_WF);
    }

  // ------------ test for display frames ---------------
  std::vector<std::string> frame_names;
  for (int i=0; i<4;i++){
    frame_names.push_back("ObjTargetFrame" + std::to_string(i));
  }
  lcm::DrakeLcm lcm;
  PublishFramesToLcm("TargetPos", frame_poses, frame_names, &lcm);
}



ObjectStateHandler::ObjectStateHandler(AllegroFingerIKMoving* FingerMotionCommander) :
        FingerMotionCommander_(FingerMotionCommander){
  object_poses_input_port_ = this->DeclareAbstractInputPort().get_index();
  this->DeclarePeriodicPublish(100);
}

// Experiment with moving the fingers
void ObjectStateHandler::DoPublish(const systems::Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>& ) const{

  const systems::AbstractValue* state_vector = this->EvalAbstractInput(
                                            context, object_poses_input_port_);
  const auto frame_poses = state_vector->GetValue<std::vector<Isometry3<double>>>();

  std::vector<int> finger_id{0,1,2,3};
  
  std::vector<Isometry3<double>> finger_target_pose;
  Isometry3<double> P_F;
  P_F.matrix().setIdentity();
  for(int i=0; i<4; i++){
    P_F.translation() = Eigen::Vector3d(0,0,0);
    finger_target_pose.push_back(P_F);
  }

  FingerMotionCommander_->CommandFingerMotion_indipendentfingers(
          finger_target_pose, frame_poses, finger_id, 2e-3);


}




}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

