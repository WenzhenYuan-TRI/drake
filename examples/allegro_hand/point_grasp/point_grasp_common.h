#pragma once

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/systems/framework/leaf_system.h"

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/examples/allegro_hand/allegro_common.h"




namespace drake {
namespace examples {
namespace allegro_hand {

using drake::geometry::SceneGraph;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::Body;


class AllegroFingerIKMoving{
public:
  AllegroFingerIKMoving(MultibodyPlant<double>& plant, MatrixX<double> Px);

  void CommandFingerMotion(
      std::vector<Isometry3<double>> finger_target, 
      std::vector<Isometry3<double>> frame_transfer,
      std::vector<int> finger_id, double target_tor);

  void CommandFingerMotion_indipendentfingers(
      std::vector<Isometry3<double>> finger_target, 
      std::vector<Isometry3<double>> frame_transfer,
      std::vector<int> finger_id, double target_tor);

private:

  void IniFingerPlant();

  MultibodyPlant<double>* plant_;
  MatrixX<double> Px_half;
  ::lcm::LCM lcm_;
  lcmt_allegro_command allegro_command;

  // this is used for test -- reduce repeated position
  std::vector<Isometry3<double>> saved_target;
  Eigen::VectorXd saved_joint_position;
  Eigen::VectorXd saved_joint_command;

  std::vector<std::unique_ptr<MultibodyPlant<double>>> finger_plant;

};

// This is a temporary class that defines the inital position of the mug
class MugSetting{
public:
  MugSetting(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, 
              const Body<double>& mug_body);

  Eigen::Vector3d IniRotAngles;
  Eigen::Vector3d IniTransPosition;

  Eigen::MatrixXd TargetGraspPos; 

  std::vector<Isometry3<double>> GenerateTargetFrame();

// template <typename T> 
  void CalcPointPosition(/*const systems::Context<T>& context*/);
  void TestReachingPosition(MatrixX<double> Px);

private:
  void AddGrippingPoint();

  const double MugHeight = 0.14;
  const double MugRadius = 0.04;

  MultibodyPlant<double>* plant_;
  SceneGraph<double>* scene_graph_;
  const Body<double>& mug_body_;
};


/*
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
*/

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
