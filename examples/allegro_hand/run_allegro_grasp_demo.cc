/// @file
///
/// This demo sets up a position controlled and gravity compensated Allegro 
/// hand within a simulation, to reach and hold a given joint space pose.
/// Position controlled using PID controller. 
/// The hand is initialized at 0 joint space pose, and is controlled to track 
/// and hold a final joint space pose. The preset final joint space pose is 
/// set in the class AllegroConstantJointValue. The states of the hand plant and
/// the controller are logged and printed in the simulation.

#include <gflags/gflags.h>

#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_set_joint_pose.h"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"  
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <iostream>

namespace drake {


namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;

DEFINE_double(simulation_time, 1, "Number of seconds to simulate");

DEFINE_string(test_hand, "right", "Which hand to model: 'left' or 'right'");

DEFINE_double(max_time_step, 1.0e-4,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");

DEFINE_bool(add_gravity, false, "Whether adding gravity in the simulation");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  const std::string HandSdfPath = "drake/manipulation/models/allegro_hand_"
      "description/sdf/allegro_hand_description_" + FLAGS_test_hand + ".sdf";
  const std::string ObjectModelPath = "drake/examples/allegro_hand/grasp/"
      "models/objects/simple_mug.sdf"

  // build the scene and the hand model
  geometry::SceneGraph<double>& scene_graph = 
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  std::string full_name = FindResourceOrThrow(HandSdfPath);
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);
  std::string full_name = FindResourceOrThrow(ObjectModelPath);
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);
  if (FLAGS_add_gravity)  plant.AddForceElement<
      multibody::UniformGravityFieldElement>(-9.81 * Eigen::Vector3d::UnitZ());


    // Weld the hand to the world frame
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>( "weld_hand", plant.world_body(), {}, 
      joint_hand_root, {}, Isometry3<double>::Identity());

  plant.Finalize(&scene_graph);
  plant.set_penetration_allowance(2e-2);  /* in [m] */
  /* The maximum slipping speed allowed during stiction. [m/s]*/
  plant.set_stiction_tolerance(1e-2);      

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  // visualization of the hand model
  const systems::rendering::PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<systems::rendering::PoseBundleToDrawMessage>();
  systems::lcm::LcmPublisherSystem& publisher = *builder.template 
          AddSystem<systems::lcm::LcmPublisherSystem>("DRAKE_VIEWER_DRAW",
          std::make_unique<systems::lcm::Serializer<lcmt_viewer_draw>>(), &lcm);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);




  // object position
  const Body<double>& mug = plant.GetBodyByName("main_body");
  const Body<double>& hand = plant.GetBodyByName("hand_root");
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize the mug pose to be right in the middle between the fingers.
  std::vector<Isometry3d> X_WB_all;
  plant.model().CalcAllBodyPosesInWorld(plant_context, &X_WB_all);
  const Eigen::Vector3d& p_WHand = X_WB_all[hand.index()].translation();
  Eigen::Isometry3d X_WM;
  Eigen::Vector3d rpy( M_PI /2, 0, 0);
  X_WM.linear() = RotationMatrix<double>(RollPitchYaw<double>(rpy)).matrix();
  X_WM.translation() = p_WHand + Vector3d(15, 0, 30);
  plant.model().SetFreeBodyPoseOrThrow(mug, X_WM, &plant_context);









  // set target joint position object
  AllegroConstantJointValue hand_joint_target(plant);

  VectorX<double> kp, kd, ki;
  MatrixX<double> Px, Py;
  GetControlPortMapping(plant, Px, Py);
  SetPositionControlledGains(&kp, &ki, &kd);
  auto controller = builder.AddSystem<
      systems::controllers::PidController>(hand_joint_target.get_Px(), 
        hand_joint_target.get_Py(), kp, ki, kd);
  builder.Connect(plant.get_continuous_state_output_port(),
                 controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port_control(),
                 plant.get_actuation_input_port());









  // Set target pose

  target_pose = hand_joint_target.set_close_hand();
  const_src->set_name("constant_source");
  builder.Connect(const_src->get_output_port(),
                 controller->get_input_port_desired_state());




  // Publish contact results for visualization.
  const auto& contact_results_to_lcm =
      *builder.AddSystem<multibody::multibody_plant::ContactResultsToLcmSystem>
      (plant);
  const auto& contact_results_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>
      ("CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_results_to_lcm.get_input_port(0));
  builder.Connect(contact_results_to_lcm.get_output_port(0),
                  contact_results_publisher.get_input_port());


  // Dispatch the message to load geometry.
  geometry::DispatchLoadMessage(scene_graph);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Set up simulator. Using semi_explicit_euler for now.
  const double max_time_step = FLAGS_max_time_step > 0 ? 
      FLAGS_max_time_step : plant.get_contact_penalty_method_time_scale() / 30;
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator{nullptr};
  integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(1e-2);

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 1;
}  // int main

}  // namespace
}  // namespace allegro
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::allegro_hand::DoMain();
}
