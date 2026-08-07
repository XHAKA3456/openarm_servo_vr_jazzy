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

#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <memory>
#include <string>
#include <vector>

namespace openarm_hardware {

// KDL 기반 중력 토크 계산 (references/openarm_teleop의 Dynamics 클래스 이식, 중력만).
// 체인 루트는 openarm_body_link0: 팔이 몸체에 rpy=±90도로 장착돼 있어도
// 체인이 그 고정조인트를 포함하므로 중력 벡터 회전을 KDL이 자동 처리한다.
class GravityComp {
 public:
  // urdf_path: 평문 URDF 파일 (launch가 xacro를 덤프해 전달)
  bool init(const std::string& urdf_path, const std::string& root_link,
            const std::string& leaf_link);

  // q[ndof] (관절각, 체인 순서 = joint1..7) -> tau[ndof] = G(q) [Nm]
  // G(q)는 M(q)q'' + C + G = tau 의 중력항: 그대로 tau_ff에 넣으면 정적 유지 토크.
  bool getGravity(const double* q, double* tau);

  size_t ndof() const { return ndof_; }
  bool ok() const { return static_cast<bool>(solver_); }

 private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  size_t ndof_ = 0;
};

}  // namespace openarm_hardware
