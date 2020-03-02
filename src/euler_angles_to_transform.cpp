#include "euler_angles_to_transform.h"

// Construct a rotation matrix (as a 4x4 transformation) given a set of Euler
// angles.
//
// Inputs:
//   xzx  3-vector of extrinsic Euler angles rotating about the x-, z-, and
//     x-axes.
// Returns 3d Eigen Affine transformation.
Eigen::Affine3d euler_angles_to_transform(
	const Eigen::Vector3d& xzx)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	Eigen::Vector3d input = xzx * (M_PI / 180.0);
	Eigen::Affine3d Rx0;
	Rx0.matrix() <<
		1, 0, 0, 0,
		0, cos(input[0]), -sin(input[0]), 0,
		0, sin(input[0]), cos(input[0]), 0,
		0, 0, 0, 1;

	Eigen::Affine3d Rz1;
	Rz1.matrix() <<
		cos(input[1]), -sin(input[1]), 0, 0,
		sin(input[1]), cos(input[1]), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::Affine3d Rx2;
	Rx2.matrix() <<
		1, 0, 0, 0,
		0, cos(input[2]), -sin(input[2]), 0,
		0, sin(input[2]), cos(input[2]), 0,
		0, 0, 0, 1;

	Eigen::Affine3d A;
	A = Rx2 * Rz1 * Rx0;
	return A;
	/////////////////////////////////////////////////////////////////////////////
}
