#include "drake/examples/allegro_hand/point_grasp/point_grasp_common.h"

#include <iostream>

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::geometry::Sphere;

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;
using systems::DiscreteUpdateEvent;

MugSetting::MugSetting(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, 
              const Body<double>& mug_body) : 
    plant_(plant), scene_graph_(scene_graph), mug_body_(mug_body)
{
    IniRotAngles<<M_PI/2, 0 , 0;
    IniTransPosition<< 0.095,0.082, 0.095;
    AddGrippingPoint();

}


void MugSetting::AddGrippingPoint(){
  const double central_point = MugHeight / 2;
  constexpr double index_finger_interval = 0.045;
  constexpr double thumb_partial = 0.005;
  
  TargetGraspPos.resize(4,3);
  TargetGraspPos.row(2) << 0, MugRadius, central_point;
  TargetGraspPos.row(1) << 0, MugRadius, central_point - index_finger_interval;
  TargetGraspPos.row(3) << 0, MugRadius, central_point + index_finger_interval;
  TargetGraspPos.row(0) << 0, -MugRadius, central_point - thumb_partial;

  // display for test
  const geometry::VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 1.0));
  constexpr double display_radius = 0.005;
  Eigen::Isometry3d X_FS;
  for (int i=0; i < 4; i++){
    X_FS.translation() = TargetGraspPos.row(i);
    plant_->RegisterVisualGeometry(mug_body_, X_FS, Sphere(display_radius),
                                "point_2", red, scene_graph_);
  }

  // end display test
}


ObjectStateHandler::ObjectStateHandler(){
  // Object position - 7 numbers; velocity - 6 numbers
  // position numbers: first 4 are quaternions, then 3 are positions
  this->DeclareInputPort(systems::kVectorValued, 13);

  this->DeclarePeriodicPublish(1);
}


void ObjectStateHandler::DoPublish(const Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>&) const {

  const auto& ObjectPos = this->EvalVectorInput(context, 0)->get_value();

  std::cout<<ObjectPos.transpose()<<std::endl;



}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake