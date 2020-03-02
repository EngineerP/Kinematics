#include "linear_blend_skinning.h"

// Given a mesh with vertices in their rest positions, a skeleton, bone
// transformations, and per-vertex weights, compute the linear blend skinning
// deformation.
//
// Inputs:
//   V  #V by 3 list of rest pose mesh vertex positions
//   skeleton  #T list of skeleton bones 
//   T  #T list of affine transformations
//   W  #V by #W list of weights (so that W(v,skeleton[i].weight_index) is the
//     weight at vertex v corresponding to transformation T[i] (Recall:
//     weight_index=-1 indicates no associated weights).
// Outputs:
//   U  #V by 3 list of deformed mesh vertex positions

void linear_blend_skinning(
	const Eigen::MatrixXd& V,
	const Skeleton& skeleton,
	const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T,
	const Eigen::MatrixXd& W,
	Eigen::MatrixXd& U)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	U.resize(V.rows(), V.cols());
	U = Eigen::MatrixXd::Zero(U.rows(), U.cols());
	//Eigen::MatrixXd hU(V.rows(), V.cols() + 1);
	//for (int i = 0; i < skeleton.size(); i++) {
	//	Bone bone = skeleton[i];
	//	if (bone.weight_index == -1) continue;
	//	Eigen::Vector4d hWeightSum(0, 0, 0, 0);
	//	for (int j = 0; j < W.rows(); j++) {
	//		Eigen::Vector4d vertex(V(j, 0), V(j, 1), V(j, 2), 1.0);
	//		hU.row(i) += W(j, bone.weight_index) * (T[i] * vertex);
	//	}
	//	hWeightSum /= hWeightSum[3];
	//	U.row(i) = Eigen::RowVector3d(hWeightSum[0], hWeightSum[1], hWeightSum[2]);
	//}

	//Loop Through Vertices
	for (int i = 0; i < W.rows(); i++) {
		Eigen::Vector4d hWeightSum(0, 0, 0, 0);
		Eigen::Vector4d vertex(V(i, 0), V(i, 1), V(i, 2), 1.0);
		//Loop through the bones
		for (int j = 0; j < skeleton.size(); j++) {
			//Add to the weighted sum
			Bone bone = skeleton[j];
			if (bone.weight_index == -1) continue;	
			hWeightSum += W(i, bone.weight_index) * (T[j] * vertex);
		}
		//hWeightSum /= hWeightSum[3];
		U.row(i) = Eigen::RowVector3d(hWeightSum[0], hWeightSum[1], hWeightSum[2]);
	}

	/////////////////////////////////////////////////////////////////////////////
}
