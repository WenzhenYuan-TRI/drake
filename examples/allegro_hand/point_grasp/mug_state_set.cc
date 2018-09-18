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


void SetMugStateControl::GetGraspTargetFrames(Isometry3<double> obj_frame, 
                                    std::vector<Isometry3<double>>* frame_poses) {

  if (frame_poses->size() < 4) 
      *frame_poses = std::vector<drake::Isometry3<double>>(4);

  for (int i=0; i < 4; i++) {
      (*frame_poses)[i] = obj_frame * contact_mug_frames_[i];
  }
}



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake