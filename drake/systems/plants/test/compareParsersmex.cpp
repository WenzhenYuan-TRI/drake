#include <mex.h>

#include <cmath>
#include <iostream>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/drakeMexUtil.h"
#include "drake/util/testUtil.h"

using namespace Eigen;
using namespace std;

using drake::CompareMatrices;
using drake::MatrixCompareType;

/*
 * compares C++ robots generated via the matlab constructModelmex with the same
 * robot generated via the c++ parser
 */

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 1) {
    mexErrMsgIdAndTxt(
        "Drake:compareParsersmex:NotEnoughInputs",
        "Usage compareParsersmex(model_ptr, urdf_file, floating_base_type)");
  }

  // first get the model_ptr back from matlab
  RigidBodyTree* matlab_model = (RigidBodyTree*)getDrakeMexPointer(prhs[0]);

  char urdf_file[1000];
  mxGetString(prhs[1], urdf_file, 1000);
  char floating_base_type_str[100] = "rpy";
  if (nrhs > 2) mxGetString(prhs[2], floating_base_type_str, 100);
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  if (strcmp(floating_base_type_str, "fixed") == 0)
    floating_base_type = DrakeJoint::FIXED;
  else if (strcmp(floating_base_type_str, "rpy") == 0)
    floating_base_type = DrakeJoint::ROLLPITCHYAW;
  else if (strcmp(floating_base_type_str, "quat") == 0)
    floating_base_type = DrakeJoint::QUATERNION;
  else
    mexErrMsgIdAndTxt(
        "Drake:compareParsersmex:BadInputs",
        "Unknown floating base type.  must be 'fixed', 'rpy', or 'quat'");

  RigidBodyTree* cpp_model = new RigidBodyTree(urdf_file, floating_base_type);

  // Compute coordinate transform between the two models (in case they are not
  // identical)
  MatrixXd P =
      MatrixXd::Zero(cpp_model->number_of_positions(),
                     matlab_model->number_of_positions());  // projection from
                                                            // the coordinates
                                                            // of matlab_model
                                                            // to cpp_model
  for (int i = 0; i < cpp_model->bodies.size(); i++) {
    if (cpp_model->bodies[i]->has_mobilizer_joint() &&
        cpp_model->bodies[i]->getJoint().getNumPositions() > 0) {
      RigidBody* b = matlab_model->FindChildBodyOfJoint(
          cpp_model->bodies[i]->getJoint().getName());
      if (b == nullptr) continue;
      for (int j = 0; j < b->getJoint().getNumPositions(); j++) {
        P(cpp_model->bodies[i]->get_position_start_index() + j,
          b->get_position_start_index() + j) = 1.0;
      }
    }
  }
  if (!P.isApprox(MatrixXd::Identity(matlab_model->number_of_positions(),
                                     matlab_model->number_of_positions()))) {
    cout << "P = \n" << P << endl;
    mexErrMsgTxt("ERROR: coordinates don't match");
  }

  std::default_random_engine generator;  // note: gets the same seed every time,
                                         // would have to seed it manually
  std::normal_distribution<double> distribution(0.0, 1.0);
  for (int trial = 0; trial < 1; trial++) {
    // generate random q
    VectorXd matlab_q = matlab_model->getRandomConfiguration(generator);
    VectorXd cpp_q(cpp_model->number_of_positions());
    cpp_q.noalias() = P * matlab_q;

    if ((matlab_model->number_of_positions() !=
         matlab_model->number_of_velocities()) ||
        (cpp_model->number_of_positions() !=
         cpp_model->number_of_velocities())) {
      mexErrMsgTxt(
          "ERROR: num_positions != num_velocities have to generate another P "
          "for "
          "this case");
    }

    // generate random v
    VectorXd matlab_v(matlab_model->number_of_velocities()),
        cpp_v(cpp_model->number_of_velocities());
    for (int i = 0; i < matlab_model->number_of_velocities(); i++)
      matlab_v[i] = distribution(generator);
    cpp_v.noalias() = P * matlab_v;

    // run kinematics
    KinematicsCache<double> matlab_cache =
        matlab_model->doKinematics(matlab_q, matlab_v, true);
    KinematicsCache<double> cpp_cache =
        cpp_model->doKinematics(cpp_q, cpp_v, true);

    {  // compare H, C, and B
      eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
          f_ext;

      auto matlab_H = matlab_model->massMatrix(matlab_cache);
      auto cpp_H = cpp_model->massMatrix(cpp_cache);

      std::string explanation;
      if (!CompareMatrices(matlab_H, cpp_H, 1e-8, MatrixCompareType::absolute,
                           &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: H doesn't match: " + explanation);
      }

      auto matlab_C = matlab_model->dynamicsBiasTerm(matlab_cache, f_ext);
      auto cpp_C = cpp_model->dynamicsBiasTerm(cpp_cache, f_ext);

      if (!CompareMatrices(matlab_C, cpp_C, 1e-8, MatrixCompareType::absolute,
                           &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: C doesn't match: " + explanation);
      }

      if (!CompareMatrices(matlab_model->B, cpp_model->B, 1e-8,
                           MatrixCompareType::absolute, &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: B doesn't match: " + explanation);
      }
    }

    {  // compare joint limits
      std::string explanation;
      if (!CompareMatrices(matlab_model->joint_limit_min,
                           cpp_model->joint_limit_min, 1e-8,
                           MatrixCompareType::absolute, &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: joint_limit_min doesn't match: " +
            explanation);
      }

      if (!CompareMatrices(matlab_model->joint_limit_max,
                           cpp_model->joint_limit_max, 1e-8,
                           MatrixCompareType::absolute, &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: joint_limit_max doesn't match: " +
            explanation);
      }
    }

    {  // compare position constraints
      auto matlab_phi = matlab_model->positionConstraints(matlab_cache);
      auto cpp_phi = cpp_model->positionConstraints(cpp_cache);

      std::string explanation;
      if (!CompareMatrices(matlab_phi, cpp_phi, 1e-8,
                           MatrixCompareType::absolute, &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: phi doesn't match doesn't "
            "match: " +
            explanation);
      }

      auto matlab_dphi =
          matlab_model->positionConstraintsJacobian(matlab_cache);
      auto cpp_dphi = cpp_model->positionConstraintsJacobian(cpp_cache);
      if (!CompareMatrices(matlab_dphi, cpp_dphi, 1e-8,
                           MatrixCompareType::absolute, &explanation)) {
        throw std::runtime_error(
            "Drake: CompareParserMex: ERROR: dphi doesn't match: " +
            explanation);
      }
    }
  }

  delete cpp_model;
}
