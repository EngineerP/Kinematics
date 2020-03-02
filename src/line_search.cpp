#include "line_search.h"
#include <iostream>
#include "dos.h"
#include "windows.h"

// Given a function handle that computes an objective value to minimize, a
// function handle that projects a guess onto a set of constraints and a
// direction to search along, use exponential search to find a decreasing step
// distance.
//
// Inputs:
//   f  function that computes scalar objective value at a given z
//   z  #z vector of initial z values
//   dz  #z vector of changes to z
//   max_step  maximum step to take in dir
// Returns optimal step distance

double line_search(
	const std::function<double(const Eigen::VectorXd&)>& f,
	const std::function<void(Eigen::VectorXd&)>& proj_z,
	const Eigen::VectorXd& z,
	const Eigen::VectorXd& dz,
	const double max_step)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	double step = max_step;

	double E = f(z);
	Eigen::VectorXd Zdescent = z - step * dz;
	proj_z(Zdescent);
	double Edescent = f(Zdescent);
	int count = 0;
	while (Edescent > E)
	{
		step /= 2.0;
		Zdescent = z - step * dz;
		proj_z(Zdescent);
		Edescent = f(Zdescent);
		if (step == 0) return 0;
	}

	return step;
  /////////////////////////////////////////////////////////////////////////////
}
