#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::multibody::multibody_plant::MultibodyPlant;

class ObjectFrameConverter : public systems::LeafSystem<double> {
public:
  ObjectFrameConverter(const MultibodyPlant<double>& plant, 
                       const std::string ObjectBodyName);

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_frame_output_port() const {
    return this->get_output_port(frame_poses_port_);
  }

private:
  void ini_mug_target_frames();

  void CalcFramePoses(const systems::Context<double>& context, 
                      std::vector<Isometry3<double>>* frame_poses) const;


  MultibodyPlant<double>* plant_{nullptr};
  const std::string obj_body_name_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  const std::vector<multibody::FrameIndex> frames_;
  int frame_poses_port_{-1};
  int state_input_port_;

};



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

