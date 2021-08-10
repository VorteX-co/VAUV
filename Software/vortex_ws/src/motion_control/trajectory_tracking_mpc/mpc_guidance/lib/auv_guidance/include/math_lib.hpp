// Copyright 2021 VorteX-co
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
#ifndef MATH_LIB_HPP_
#define MATH_LIB_HPP_
#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// Useful math tools
namespace auv_core
{
namespace math_lib
{
int sign(double x);

int sign(float x);

int sign(int x);

Eigen::MatrixXf sign(const Eigen::Ref<const Eigen::MatrixXf> & mat);

Eigen::MatrixXd sign(const Eigen::Ref<const Eigen::MatrixXd> & mat);
}  // namespace math_lib
}  // namespace auv_core

#endif  // MATH_LIB_HPP_
