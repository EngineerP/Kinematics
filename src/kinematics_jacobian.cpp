#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>


void kinematics_jacobian(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b,
	Eigen::MatrixXd& J)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	J = Eigen::MatrixXd::Zero(b.size() * 3, skeleton.size() * 3);
	Eigen::VectorXd tips = transformed_tips(skeleton, b);

	for (int j = 0; j < skeleton.size(); j++) {
		for (int k = 0; k < 3; k++) {
			Skeleton copy = skeleton;
			copy[j].xzx[k] += 0.0000001;
			Eigen::VectorXd Htips = transformed_tips(copy, b);
			for (int i = 0; i < 3 * b.size(); i++) {
				J(i, 3 * j + k) = (Htips[i] - tips[i]) / 0.0000001;
			}
		}
	}
	/////////////////////////////////////////////////////////////////////////////
}
