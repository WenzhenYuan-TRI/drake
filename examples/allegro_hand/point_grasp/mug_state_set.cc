#include "drake/examples/allegro_hand/point_grasp/mug_state_set.h"

namespace drake {
namespace examples {
namespace allegro_hand {

SetMugStateControl::SetMugStateControl() {

    // ini the contact frames on the mug
    Eigen::MatrixXd TargetGraspPos(4,3);
    TargetGraspPos.row(2) << 0, MugRadius, central_point;
    TargetGraspPos.row(1) << 0, MugRadius, central_point - index_finger_interval;
    TargetGraspPos.row(3) << 0, MugRadius, central_point + index_finger_interval;
    TargetGraspPos.row(0) << 0, -MugRadius, central_point - thumb_partial;
    Eigen::Vector4d TargetRotAngle(M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2);

    Eigen::Isometry3d X_BF; /* pose of cup upper frame F in mug body frame B */
    X_BF.matrix().setIdentity();
    for (int i=0; i < 4; i++){
      X_BF.translation() = TargetGraspPos.row(i);
      X_BF.linear() = math::RotationMatrix<double>(math::RollPitchYaw<double>(
                      Eigen::Vector3d(TargetRotAngle(i), 0, 0))).matrix();
      contact_mug_frames_.push_back(X_BF);
    }

}


void SetMugStateControl::GetGraspTargetFrames(const Isometry3<double>& obj_frame, 
                                    std::vector<Isometry3<double>>* frame_poses,
                                    std::vector<Isometry3<double>>* relative_finger_pose) {
  if (frame_poses->size() < 4) 
      *frame_poses = std::vector<drake::Isometry3<double>>(4);
  if (relative_finger_pose->size() < 4) 
      *relative_finger_pose = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.01);

  for (int i=0; i < 4; i++) {
      (*relative_finger_pose)[i] = contact_mug_frames_[i] * grasp_offset;
      (*frame_poses)[i] = obj_frame * contact_mug_frames_[i] * grasp_offset;
  }
  grasp_offset.translation() = Eigen::Vector3d(0, 0, 0.02);
  (*frame_poses)[0] = (*frame_poses)[0] * grasp_offset;
}

void SetMugStateControl::GetXRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* frame_poses) {

  if (frame_poses->size() < 4) 
      *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> temp; 
  temp.matrix().setIdentity();
  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.translation()<<0,0,-MugHeight/2;
  temp.rotate(Eigen::AngleAxis<double>(rotation_angle, Eigen::Vector3d::UnitX()));
  tar_mug_frame = temp * tar_mug_frame;
  temp.matrix().setIdentity();
  temp.translation() << 0,0,MugHeight/2;
  tar_mug_frame = temp * tar_mug_frame;
  tar_mug_frame = tar_mug_frame * ref_mug_pose_;
  // tar_mug_frame.rotate(Eigen::AngleAxis<double>(rotation_angle, Eigen::Vector3d::UnitX()));

  for (int i=0; i < 4; i++) {
      (*frame_poses)[i] = tar_mug_frame * contact_mug_frames_[i] * grasp_offset;
  }
}

void SetMugStateControl::GetYRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* frame_poses) {

  if (frame_poses->size() < 4) 
      *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> temp; 
  temp.matrix().setIdentity();
  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.translation()<<0,0,-MugHeight/2;
  temp.rotate(Eigen::AngleAxis<double>(rotation_angle, Eigen::Vector3d::UnitY()));
  tar_mug_frame = temp * tar_mug_frame;
  temp.matrix().setIdentity();
  temp.translation() << 0,0,MugHeight/2;
  tar_mug_frame = temp * tar_mug_frame;
  tar_mug_frame = tar_mug_frame * ref_mug_pose_;
  // tar_mug_frame.rotate(Eigen::AngleAxis<double>(rotation_angle, Eigen::Vector3d::UnitY()));

  for (int i=0; i < 4; i++) {
      (*frame_poses)[i] = tar_mug_frame * contact_mug_frames_[i] * grasp_offset;
  }
}

void SetMugStateControl::GetZRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* frame_poses) {

  if (frame_poses->size() < 4) 
      *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.rotate(Eigen::AngleAxis<double>(rotation_angle, Eigen::Vector3d::UnitZ()));
  tar_mug_frame = ref_mug_pose_ * tar_mug_frame;
 
  for (int i=0; i < 4; i++) {
      (*frame_poses)[i] = tar_mug_frame * contact_mug_frames_[i] * grasp_offset;
  }
}

void SetMugStateControl::GetTransTargetFrame(const Vector3<double> translation_vector,
                              std::vector<Isometry3<double>>* frame_poses) {

  Isometry3<double> tar_mug_frame = ref_mug_pose_;
  tar_mug_frame.translation() += translation_vector;
  // std::cout<<ref_mug_pose_.matrix()<<std::endl;
  // std::cout<<tar_mug_frame.matrix()<<std::endl;

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);
  for (int i=0; i < 4; i++) {
      (*frame_poses)[i] = tar_mug_frame * contact_mug_frames_[i] * grasp_offset;
  }
}



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake