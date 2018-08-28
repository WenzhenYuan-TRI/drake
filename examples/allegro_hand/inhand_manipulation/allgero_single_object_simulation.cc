/// @file
///
/// This file set up a simulation environment of an allegro hand an object. It
/// is intended to be a be a direct replacement for the Allegro Hand driver and
/// the actual robot hardware. The only controllable interface in this instance
/// is the joints on the hand.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_string(test_hand, "right", "Which hand to model: 'left' or 'right'");
DEFINE_double(max_time_step, 1.5e-4, "Simulation time step used for intergrator.");
DEFINE_bool(add_gravity, true,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  geometry::SceneGraph<double>& scene_graph = 
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  const std::string full_name = FindResourceOrThrow("drake/manipulation/models"
                  "/allegro_hand_description/sdf/allegro_hand_description_"
                  + FLAGS_test_hand + ".sdf");
  const std::string ObjectModelPath = "drake/examples/allegro_hand/grasp/"
                                      "models/objects/simple_mug.sdf";
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);

  // Weld the hand to the world frame
  // TODO(WenzhenYuan-TRI): adding the DOF to enable the hand to move free in
  // the 3D space
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>( "weld_hand", plant.world_body(), {},
      joint_hand_root, {}, Isometry3<double>::Identity());

  // Add gravity, if needed
  if (FLAGS_add_gravity)
    plant.AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Eigen::Vector3d::UnitZ());

  // Visualization
  geometry::ConnectVisualization(scene_graph, &builder, &lcm);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // Publish contact results for visualization.
  const auto& contact_results_to_lcm = *builder.AddSystem<
      multibody::multibody_plant::ContactResultsToLcmSystem>(plant);
  const auto& contact_results_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>
      ("CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_results_to_lcm.get_input_port(0));
  builder.Connect(contact_results_to_lcm.get_output_port(0),
                  contact_results_publisher.get_input_port());

  // Controller
  VectorX<double> kp, kd, ki;
  MatrixX<double> Px, Py;
  GetControlPortMapping(plant, Px, Py);
  SetPositionControlledGains(&kp, &ki, &kd);
  auto controller = builder.AddSystem<
      systems::controllers::PidController>(Px, Py, kp, ki, kd); 
  builder.Connect(plant.get_continuous_state_output_port(),
                 controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port_control(),
                 plant.get_actuation_input_port());  

std::cout<<"set controller"<<std::endl;

    // Create the command subscriber and status publisher.
  auto command_sub = builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",
                                                                 &lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      base_builder->AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  std::vector<int> iiwa_instances =
      {RigidBodyTreeConstants::kFirstNonWorldModelInstanceId};
  auto external_torque_converter =
      base_builder->AddSystem<IiwaContactResultsToExternalTorque>(
          tree, iiwa_instances);
  auto status_pub = base_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS",
                                                               &lcm));
  status_pub->set_name("status_publisher");
  status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto status_sender = base_builder->AddSystem<IiwaStatusSender>(num_joints);
  status_sender->set_name("status_sender");













  // Now the model is complete.
  plant.Finalize(&scene_graph);
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  geometry::DispatchLoadMessage(scene_graph, &lcm);

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);
}  // main

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::allegro_hand::DoMain();
}
