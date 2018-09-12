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






ObjectFrameConverter::ObjectFrameConverter(MultibodyPlant<double>& plant, 
    const std::string ObjectBodyName) : plant_(&plant), 
                                        obj_body_name_(ObjectBodyName) {
    // ObjectFrameConverter input port with BasicVector for the sate x of the plant..

  state_input_port_ = this->DeclareInputPort(systems::kVectorValued, 
      plant_->num_multibody_states()).get_index();
  frame_poses_port_ = this->DeclareAbstractOutputPort(std::vector<Eigen::Isometry3d>(),
      &ObjectFrameConverter::CalcFramePoses).get_index();

  ini_mug_target_frames();
  plant_context_ = plant_->CreateDefaultContext();
}


void ObjectFrameConverter::ini_mug_target_frames(){
  // add frame to the plant


/*
  // display for test
  const geometry::VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 1.0));
  constexpr double display_radius = 0.005;
  Eigen::Isometry3d X_FS;
  for (int i=0; i < 4; i++){
    X_FS.translation() = TargetTransformation.row(i);
    plant_->RegisterVisualGeometry(mug_body_, X_FS, Sphere(display_radius),
                                "point_2", red, scene_graph_);
  }
*/

  // --------------------
  const multibody::Body<double>& mug_body = plant_->GetBodyByName(obj_body_name_);

  const double MugHeight = 0.14;
  const double MugRadius = 0.04;
  const double central_point = MugHeight / 2;
  const double up_interval = 0.045;
  const double thumb_partial = 0.005;

  Eigen::MatrixXd TargetTransformation(4,3);
  TargetTransformation.row(2) << 0, MugRadius, central_point;
  TargetTransformation.row(1) << 0, MugRadius, central_point - up_interval;
  TargetTransformation.row(3) << 0, MugRadius, central_point + up_interval;
  TargetTransformation.row(0) << 0, -MugRadius, central_point - thumb_partial;
  Eigen::Vector4d TargetRotAngle(-M_PI/2, M_PI/2, M_PI/2, M_PI/2);

  Eigen::Isometry3d X_BF; /* pose of cup upper frame F in mug body frame B */
  for (int i=0; i < 4; i++){
    X_BF.translation() = TargetTransformation.row(i);
    X_BF.rotate(Eigen::AngleAxis<double>(TargetRotAngle(i), 
                                         Eigen::Vector3d::UnitX()));
    const auto& mug_target_frame = plant_->AddFrame(//<multibody::FixedOffsetFrame>(
      std::make_unique<multibody::FixedOffsetFrame<double>>("ObjTargetFrame" + std::to_string(i),
      mug_body, X_BF));
    frames_.push_back(mug_target_frame);
  }

  // -----test: add visualization for test using visualizing frames through lcm

  std::vector<Eigen::Isometry3d> pose_frames;// = {X_BF};
  std::vector<std::string> frame_names;// = {"name"};
  lcm::DrakeLcm lcm;
  for (int i=0; i < 4; i++){
    X_BF.translation() = TargetTransformation.row(i);
    X_BF.rotate(Eigen::AngleAxis<double>(TargetRotAngle(i), 
                                         Eigen::Vector3d::UnitX()));
    pose_frames.push_back(X_BF);
    frame_names.push_back("ObjTargetFrame" + std::to_string(i));
  }
  PublishFramesToLcm("TargetPos", pose_frames, frame_names, &lcm);

  // TODO: link to main program

}


void ObjectFrameConverter::CalcFramePoses(const systems::Context<double>& context,
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





}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

