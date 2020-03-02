#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function
#include <stack>

// Compute the deformations (tranformations from rest pose) for each node in a
// forward kinematics hiearchy (reading each node's relative transformation from
// rest).
//
// Inputs:
//   skeleton  transformation hierarchy of #T bones
// Outputs:
//   T  #T list of affine transformations

void forward_kinematics(
	const Skeleton& skeleton,
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	T.resize(skeleton.size(), Eigen::Affine3d::Identity());

	//Build Spaghetti stack
	std::stack<std::pair<int, Bone>> boneStack;
	for (int i = 0; i < skeleton.size(); i++) {
		Bone bone = skeleton[i];
		int index = i;
		while (index != -1) {
			auto bone = skeleton[index];
			boneStack.push(std::make_pair(index, bone));
			index = bone.parent_index;
		}
		while (!boneStack.empty()) {
			std::pair<int, Bone> bonePair = boneStack.top();
			Bone curBone = bonePair.second;
			Eigen::Matrix4d transform = T[bonePair.first].matrix();
			if (transform.isIdentity()){//T[bonePair.first].isApprox(Eigen::Affine3d::Identity())) {
				if (curBone.parent_index == -1) {
					T[bonePair.first] = curBone.rest_T * euler_angles_to_transform(curBone.xzx) * curBone.rest_T.inverse();
				}
				else {
					T[bonePair.first] = T[curBone.parent_index] * curBone.rest_T * euler_angles_to_transform(curBone.xzx) * curBone.rest_T.inverse();
				}
			}
			boneStack.pop();
		}
	}
	/////////////////////////////////////////////////////////////////////////////
}
