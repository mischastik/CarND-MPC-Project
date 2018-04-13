#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Set the timestep length and duration


class MPC {
public:
	static constexpr size_t N = 10;
	static constexpr double dt = 0.1;
	static constexpr double ref_v = 75;
	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double latencySecs);
};

#endif /* MPC_H */
