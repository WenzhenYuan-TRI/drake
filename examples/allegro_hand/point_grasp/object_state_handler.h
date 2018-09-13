#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/allegro_hand/point_grasp/point_grasp_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::multibody::multibody_plant::MultibodyPlant;

const double kObjectStatePublishPeriod = 0.05;

// template <typename T>
class ObjectFrameTracker : public systems::LeafSystem<double> {
public:
  ObjectFrameTracker(const MultibodyPlant<double>& plant, 
                     const std::vector<multibody::FrameIndex>& frames,
                     const std::string& obj_body_name);

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_frame_output_port() const {
    return this->get_output_port(frame_poses_port_);
  }



private:

  void CalcFramePoses(const systems::Context<double>& context, 
                      std::vector<Isometry3<double>>* frame_poses) const;

  void DoPublish(const systems::Context<double>& context,
                 const std::vector<const systems::PublishEvent<double>*>& events)
                 const override;


  const MultibodyPlant<double>* plant_{nullptr};
  const std::string obj_body_name_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  const std::vector<multibody::FrameIndex> frames_;
  int frame_poses_port_{-1};
  int state_input_port_;

};


class ObjectStateHandler : public systems::LeafSystem<double> {
public:
  ObjectStateHandler(AllegroFingerIKMoving* FingerMotionCommander);

  const systems::InputPort<double>& get_frame_input_port() const {
    return this->get_input_port(object_poses_input_port_);
  }

private:
  void DoPublish(const systems::Context<double>& context,
                 const std::vector<const systems::PublishEvent<double>*>& events)
                 const override;

  const std::vector<Isometry3<double>> object_frames_;
  int object_poses_input_port_;
  AllegroFingerIKMoving* FingerMotionCommander_;

};




void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm);



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

