#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

// Given a skeleton and a list of end-effector constraints, construct function
// handles that will compute:
//    the least-squares objective value,
//    the least-squares objective gradient, and
//    project a potential solution onto a feasible solution (within joint limit
//    bounds).
//
// Inputs:
//   skeleton  #bones list of bones
//   b  #b list of indices into skeleton of end-effector constraints
//   xb0  3*#b list of end-effector positions, ordered as [x0 y0 z0 x1 y1 z1 …]
// Outputs:
//   f  function handle that computes the least-squares objective value given a
//     #bones list of Euler angles 
//   grad_f  function handle that computes the least-squares objective gradient
//     given a #bones list of Euler angles 
//   proj_z  function handle that projects a given set of Euler angles onto the
//     input skeleton's joint angles
//   

void end_effectors_objective_and_gradient(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b,
	const Eigen::VectorXd& xb0,
	std::function<double(const Eigen::VectorXd&)>& f,
	std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& grad_f,
	std::function<void(Eigen::VectorXd&)>& proj_z)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	f = [&](const Eigen::VectorXd& A)->double
	{
		Skeleton copy = copy_skeleton_at(skeleton, A);
		Eigen::VectorXd tips = transformed_tips(copy, b);
		double sumLS = 0.0;
		for (int i = 0; i < b.size(); i++) {
			Eigen::Vector3d xb(tips[3 * i], tips[3 * i + 1], tips[3 * i + 2]);
			Eigen::Vector3d q(xb0[3 * i], xb0[3 * i + 1], xb0[3 * i + 2]);

			sumLS += (xb - q).squaredNorm();
		}
		return sumLS;
	};

	grad_f = [&](const Eigen::VectorXd& A)->Eigen::VectorXd
	{
		Skeleton copy = copy_skeleton_at(skeleton, A);
		Eigen::VectorXd tips = transformed_tips(copy, b);
		Eigen::MatrixXd J;
		kinematics_jacobian(copy, b, J);
		Eigen::VectorXd dEdx = 2.0 * (tips - xb0);
		Eigen::VectorXd grad = Eigen::VectorXd::Zero(dEdx.size());
		grad = J.transpose() * dEdx;
		return grad;
	};

	proj_z = [&](Eigen::VectorXd& A)
	{
		assert(skeleton.size() * 3 == A.size());
		for (int i = 0; i < skeleton.size(); i++) {
			Bone bone = skeleton[i];
			A[3 * i] = std::max(bone.xzx_min[0], std::min(bone.xzx_max[0], A[3 * i]));
			A[3 * i + 1] = std::max(bone.xzx_min[1], std::min(bone.xzx_max[1], A[3 * i + 1]));
			A[3 * i + 2] = std::max(bone.xzx_min[2], std::min(bone.xzx_max[2], A[3 * i + 2]));
		}

	};
	/////////////////////////////////////////////////////////////////////////////
}
