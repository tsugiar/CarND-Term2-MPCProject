#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
                       double prev_delta, double prev_accel);
};

#endif /* MPC_H */
