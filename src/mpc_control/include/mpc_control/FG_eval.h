#ifndef MPC_CONTROL_FG_EVAL_H
#define MPC_CONTROL_FG_EVAL_H

#include <vector>
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <Eigen/Dense>
using CppAD::AD;
static constexpr double Lf = 1.53;

class FG_eval {
public:
  // Polynomial coefficients fitted to the waypoints
  Eigen::VectorXd coeffs;

  // Constructor: store the coefficients
  FG_eval(const Eigen::VectorXd& coeffs_) : coeffs(coeffs_) {}

  // This is the vector type IPOPT & CppAD expect
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // fg[0] holds the cost; fg[1..] holds the constraints
  void operator()(ADvector& fg, const ADvector& vars) {
    // The very first element of fg is reserved for the cost
    fg[0] = 0;

    // --- 1) Cost function ---
    // Reference values (tweak these in MPC.cpp or MPC.h)
    const double ref_v   =  25.0;    // target speed
    const int N         =  10;      // horizon length
    const double dt     =  0.1;     // timestep duration
    const double w_cte  = 2000.0;   // weight for cte
    const double w_epsi = 2000.0;   // weight for orientation error
    const double w_v    =   1.0;    // weight for velocity error
    const double w_delta=   5.0;    // weight for steering use
    const double w_a    =   5.0;    // weight for throttle use
    const double w_ddelta=200.0;    // weight for steering change rate
    const double w_da   =  10.0;    // weight for throttle change rate

    // Starting indices in vars vector (must match MPC.cpp)
    const int x_start     = 0;
    const int y_start     = x_start + N;
    const int psi_start   = y_start + N;
    const int v_start     = psi_start + N;
    const int cte_start   = v_start + N;
    const int epsi_start  = cte_start + N;
    const int delta_start = epsi_start + N;
    const int a_start     = delta_start + N - 1;

    // 1a) Cost for state errors over the horizon
    for (int t = 0; t < N; t++) {
      fg[0] += w_cte   * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_epsi  * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w_v     * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // 1b) Cost for use of actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += w_delta  * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a      * CppAD::pow(vars[a_start + t],     2);
    }

    // 1c) Cost for the gap between sequential actuations (smoothness)
    for (int t = 0; t < N - 2; t++) {
      fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_da     * CppAD::pow(vars[a_start     + t + 1] - vars[a_start     + t], 2);
    }

    // --- 2) Model constraints ---
    // Initial constraints: set to the current state
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints follow the vehicle model kinematics
    for (int t = 1; t < N; t++) {
      // State at time t
      AD<double> x1    = vars[x_start    + t];
      AD<double> y1    = vars[y_start    + t];
      AD<double> psi1  = vars[psi_start  + t];
      AD<double> v1    = vars[v_start    + t];
      AD<double> cte1  = vars[cte_start  + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // State at time t-1
      AD<double> x0    = vars[x_start    + t - 1];
      AD<double> y0    = vars[y_start    + t - 1];
      AD<double> psi0  = vars[psi_start  + t - 1];
      AD<double> v0    = vars[v_start    + t - 1];
      AD<double> cte0  = vars[cte_start  + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuations at time t-1
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start     + t - 1];

      // Desired y based on the polynomial fit
      AD<double> f0     = coeffs[0]
                        + coeffs[1] * x0
                        + coeffs[2] * x0 * x0
                        + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0= CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

      // Kinematic model equations
      fg[1 + x_start   + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start   + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start   + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start+ t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

#endif  // MPC_CONTROL_FG_EVAL_H
