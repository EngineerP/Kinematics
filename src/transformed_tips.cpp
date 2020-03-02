#include "transformed_tips.h"
#include "forward_kinematics.h"

// Compute the positions of specified bone "tips" (e.g., where the bone
// connects to its children; as opposed to "tails", where the bone connects to
// its parent) _after_ evaluating the forward kinematics chain in the given
// skeleton.
//
// Inputs:
//   skeleton  #bones list of bones with relative transformations stored in .xzx
//   b  #b list of indices into skelton of endpoints to compute
// Returns  #b*3 vector of transformed tip positions 

Eigen::VectorXd transformed_tips(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> T;
	forward_kinematics(skeleton, T);
	Eigen::VectorXd tipPoints(b.size() * 3);
	for (int i = 0; i < b.size(); i++) {
		Eigen::Vector4d tipPoint = T[b[i]] * skeleton[b[i]].rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0.0, 0.0, 1.0);
		//tipPoint /= tipPoint[3];
		tipPoints[3 * i] = tipPoint[0];
		tipPoints[3 * i + 1] = tipPoint[1];
		tipPoints[3 * i + 2] = tipPoint[2];
	}
	return tipPoints;
	/////////////////////////////////////////////////////////////////////////////
}
