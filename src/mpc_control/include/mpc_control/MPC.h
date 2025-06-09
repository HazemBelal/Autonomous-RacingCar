#ifndef MPC_H
#define MPC_H

#include <vector>

// Use the system Eigen install location:
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class MPC {
 public:
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &coeffs);
};

#endif  // MPC_H
