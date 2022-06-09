// Copyright 2022 VorteX-co
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
#include "thrust_allocation.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>


// =========================================================================
void Allocator::set_params(
  const double & k, const MatrixXd & thrusters,
  const Vector4d & coeff_left, const Vector4d & coeff_right)
{
  k_ = k;
  thrusters_ = thrusters;
  coeff_left_ = coeff_left;
  coeff_right_ = coeff_right;
  // Number of thrusters
  size_t r = thrusters_.cols();
  // Number of controls
  size_t n = thrusters_.rows();
  // Thruster configuration matrix
  Eigen::MatrixXd Tconfig(n, r);
  for (int i = 0; i < r; i++) {
    // Calculating the Thrust Configuration of thruster i
    Tconfig.col(i) = thrust_configuration(thrusters_.col(i));
    // Force Coefficient Matrix for the r thrusters
    Eigen::MatrixXd K = k_ * Eigen::MatrixXd::Identity(r, r);
    // τ = T K u    -->   τ = B u    ..   B(n,r)
    Eigen::MatrixXd B = Tconfig * K;
    // u = B† τ    --> B† = Bpinv(r,n)  Moore–Penrose inverse of B
    Bpinv_ = B.transpose() * (B * B.transpose()).inverse();
  }
}
// =========================================================================
Eigen::VectorXd Allocator::wrench_to_pwm_thrusters(const Eigen::VectorXd & wrench)
{
  /**
   * @brief  rpm_to_pwm_thrusters compute commanded pwm for each thruster
   * @param wrench , control forces and moments
   * @return pwm vector for 8 thrusters
   */
  size_t r = thrusters_.cols();
  Eigen::VectorXd u_alloc(r);
  //  u = B† τ
  u_alloc = Bpinv_ * wrench;
  // Commanded r propellers speed vector n (RPM)
  Eigen::VectorXd n(r);
  for (size_t i = 0; i < r; i++) {
    // u = n |n|
    // n = sign(u) * sqrt(|u|)
    n(i) = (u_alloc(i) < 0) ?
      -1 :
      (u_alloc(i) > 0) * std::sqrt(std::abs(u_alloc(i)));
    /* Given n vector (rpm) compute the equivalent pwm
     * There are 3 operation regions [left,neutral,right]
     * [left,right] for -ve and +ve rpm respectively
     * The relationship between [pwm and rpm] for each region
     *  is approximated to 3rd polynomial ,, y = a + b * x + c * x^2 + d * x^3
     * x is the rpm value and y is the output pwm value
     * The coefficients [a,b,c,d] are computed by fitting 3rd polynomial
     * between Thw PWM to RPM values The coefficients are calculated offline
     * and passed as parameter vector
     */
    const int min_rpm = 36;
    const int neutral_pwm = 1500;
    Eigen::VectorXd pwm(r);
    for (int i = 0; i < r; i++) {
      if (abs(n(i)) < min_rpm) {
        pwm(i) = neutral_pwm;
      } else if (abs(n(i)) > min_rpm && n(i) > 0) {
        pwm(i) = coeff_right_(3) + coeff_right_(2) * n(i) +
          coeff_right_(1) * n(i) * n(i) +
          coeff_right_(0) * n(i) * n(i) * n(i);
      } else if (abs(n(i)) > min_rpm && n(i) < 0) {
        pwm(i) = coeff_left_(3) + coeff_left_(2) * abs(n(i)) +
          coeff_left_(1) * abs(n(i)) * abs(n(i)) +
          coeff_left_(0) * abs(n(i)) * abs(n(i)) * abs(n(i));
      }

      const int max_pwm = 1896;
      if (pwm(i) > max_pwm) {
        pwm(i) = max_pwm;
      }
      const int min_pwm = 1100;
      if (pwm(i) < min_pwm) {
        pwm(i) = min_pwm;
      }
    }
    return pwm;
  }
}
// =========================================================================

/**
 * @brief thrust_configuration compute thruster contribution for every DOF
 * @param Tpose thruster position and orientation w.r.t COG [x,y,z,r,p,y]
 * @return 6DOF thrust configuration vector
 */
Vector6d Allocator::thrust_configuration(const Vector6d & Tpose)
{
  // Given  Thruster pose [X,Y,Z,φ,θ,ψ]
  /* Compute the contribution to each DOF
   *
   *           Surge                  cos θ cos φ
   *           Sway                   sin θ cos φ
   *           Heave                      sin φ
   *           Roll              =    −Z sin θ + Y sin φ
   * Ti =    Pitch                  −Z cos θ + X sin φ
   *           Yaw                   −Y cos θ + X sin θ
   * Reference: Viktor Berg (2012) equation(2.38)
   */
  double stheta = sin(Tpose(4));
  double ctheta = cos(Tpose(4));
  double spsi = sin(Tpose(5));
  double cpsi = cos(Tpose(5));
  Vector6d T;
  T << ctheta * cpsi, ctheta * spsi, stheta,
    Tpose(1) * stheta - Tpose(2) * spsi,
    Tpose(0) * stheta - Tpose(2) * cpsi, Tpose(0) * spsi - Tpose(1) * cpsi;
  return T;
}
