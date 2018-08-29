#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the allegro hand.

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace allegro_hand {

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
const double kLcmStatusPeriod = 0.005;

/// Handles lcmt_allegro_command messages from a LcmSubscriberSystem.
/// Has two output ports: one for the commanded position for each joint along
/// with an estimate of the commanded velocity for each joint, and another for
/// commanded additional feedforward joint torque.
class AllegroCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AllegroCommandReceiver)

  explicit AllegroCommandReceiver(int num_joints = kAllegroNumJoints);

  /// Sets the initial position of the controlled hand prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the open hand pose will be the zero
  /// configuration.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>> x) const;

  const systems::OutputPort<double>& get_commanded_state_output_port()
      const {
    return this->get_output_port(0);
  }

  const systems::OutputPort<double>& get_commanded_torque_output_port()
      const {
    return this->get_output_port(1);
  }

 private:
  void CopyStateToOutput(const systems::Context<double>& context, int start_idx,
                         int length,
                         systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const int num_joints_;
};


/// Creates and outputs lcmt_allegro_status messages.
///
/// This system has five vector-valued input ports, one for the plant's
/// current state, one for the most recently received position command, one
/// for the most recently received joint torque command, one for the plant's
/// measured joint torque, and one for the plant's external joint torque. The
/// last two inputs are optional. If left unconnected, the measured joint torque
/// field in the output message will be identical to the commanded joint torque,
/// and external torque will be filled with zeros.
/// The state and command ports contain a position and velocity for each joint
/// (velocity is unused, this is done to be more readily compatible with the
/// outputs from IiwaCommandReceiver and RigidBodyPlant). The torque related
/// ports contain a single torque for each joint.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_iiwa_status`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// systems::Value object templated on type `lcmt_iiwa_status`. For an example
/// of this, see iiwa_wsg_simulation.cc.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
class AllegroStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AllegroStatusSender)

  explicit AllegroStatusSender(int num_joints = kAllegroNumJoints);

  const systems::InputPort<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  const systems::InputPort<double>& get_commanded_torque_input_port()
      const {
    return this->get_input_port(2);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_allegro_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_allegro_status* output) const;

  const int num_joints_;
};










}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
