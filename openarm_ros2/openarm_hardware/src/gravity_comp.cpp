// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/gravity_comp.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <sstream>

#include "rclcpp/logging.hpp"

namespace openarm_hardware {

bool GravityComp::init(const std::string& urdf_path, const std::string& root_link,
                       const std::string& leaf_link) {
  auto logger = rclcpp::get_logger("OpenArm_v10HW.gravity");

  std::ifstream file(urdf_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open URDF: %s", urdf_path.c_str());
    return false;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();

  auto urdf_model = urdf::parseURDF(buffer.str());
  if (!urdf_model) {
    RCLCPP_ERROR(logger, "Failed to parse URDF: %s", urdf_path.c_str());
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree)) {
    RCLCPP_ERROR(logger, "Failed to build KDL tree from %s", urdf_path.c_str());
    return false;
  }

  if (!tree.getChain(root_link, leaf_link, chain_)) {
    RCLCPP_ERROR(logger, "Failed to get KDL chain %s -> %s", root_link.c_str(),
                 leaf_link.c_str());
    return false;
  }

  ndof_ = chain_.getNrOfJoints();
  // 루트가 body_link0이므로 중력은 그 프레임 기준 -Z (레퍼런스와 동일).
  solver_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0, 0, -9.81));

  RCLCPP_INFO(logger, "Gravity model ready: %s -> %s, %zu joints", root_link.c_str(),
              leaf_link.c_str(), ndof_);
  return true;
}

bool GravityComp::getGravity(const double* q, double* tau) {
  if (!solver_) return false;

  KDL::JntArray q_kdl(ndof_);
  for (size_t i = 0; i < ndof_; ++i) q_kdl(i) = q[i];

  KDL::JntArray g(ndof_);
  if (solver_->JntToGravity(q_kdl, g) < 0) return false;

  for (size_t i = 0; i < ndof_; ++i) tau[i] = g(i);
  return true;
}

}  // namespace openarm_hardware
