#pragma once

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::geometry::SceneGraph;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::Body;


// This is a temporary class that defines the inital position of the mug
class MugSetting{
public:
  MugSetting(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, 
              const Body<double>& mug_body);

  Eigen::Vector3d IniRotAngles;
  Eigen::Vector3d IniTransPosition;

  Eigen::MatrixXd TargetGraspPos; 

private:
  void AddGrippingPoint();

  const double MugHeight = 0.14;
  const double MugRadius = 0.04;

  MultibodyPlant<double>* plant_;
  SceneGraph<double>* scene_graph_;
  const Body<double>& mug_body_;

};



class ObjectStateHandler : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ObjectStateHandler)
  ObjectStateHandler();

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(0);
  }

 private:

  void DoPublish(const systems::Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>& events)
    const override;

};






}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
