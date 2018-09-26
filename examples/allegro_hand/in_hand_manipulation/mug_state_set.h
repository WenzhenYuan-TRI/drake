#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace examples {
namespace allegro_hand {

class SetMugStateControl{
  public:
    SetMugStateControl();

    // return the 4 frames of the preset positions of the target of the
    // fingertips when only grasping on the mug based on the mug pose.
    void GetGraspTargetFrames(const Isometry3<double>& obj_frame, 
                        std::vector<Isometry3<double>>* finger_frames,
                        std::vector<Isometry3<double>>* relative_finger_pose);

    void GetXRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
    void GetYRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
    void GetZRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
    void GetTransTargetFrame(const Vector3<double> translation_vector,
                              std::vector<Isometry3<double>>* finger_frames);

    void UpdateMugPose(const Isometry3<double>& mug_frame) {
        ref_mug_pose_ = mug_frame;
    }

    void PublishTargetFrametoLcm(const Isometry3<double>& mug_frame);

  private:

    // frames of the selected fingertip frame on the cup
    std::vector<Isometry3<double>> contact_mug_frames_;

    // the saved reference of mug position
    Isometry3<double> ref_mug_pose_;

    // dimension of the mug
    const double MugHeight = 0.14;
    const double MugRadius = 0.04;
    // parameters about the grasp point
    const double central_point = MugHeight / 2;
    const double index_finger_interval = 0.045;
    const double thumb_partial = 0.012;

};

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm);

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake