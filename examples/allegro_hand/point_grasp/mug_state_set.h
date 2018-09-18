#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace allegro_hand {

class SetMugStateControl{
  public:
    SetMugStateControl();

    // return the 4 frames of the preset positions of the target of the
    // fingertips when only grasping on the mug, based on the frame of the object. 
    void GetGraspTargetFrames(Isometry3<double> obj_frame, 
                              std::vector<Isometry3<double>>* finger_frames);

    void UpdateMugPose(Isometry3<double> mug_frame) {
        ref_mug_pose_ = mug_frame;
    }

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
    const double thumb_partial = 0.007;

};





}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake