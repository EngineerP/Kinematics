#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

// Interpolate a given set of 3d vector keyframes (e.g., of Euler angles) at a
// time t using a Catmull-Rom spline.
//
// Inputs:
//   keyframes  #keyframes list of pairs of key-times and key-values sorted in
//     ascending order of key-time
//   t  query time
// Returns inteprolated value at time t

Eigen::Vector3d catmull_rom_interpolation(
	const std::vector<std::pair<double, Eigen::Vector3d> >& keyframes,
	double t)
{
	/////////////////////////////////////////////////////////////////////////////
	// Replace with your code
	// Algorithm Based on Centripetal Catmull Rom Spline as shown by in the definition on https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
	if (keyframes.size() == 0)
		return Eigen::Vector3d(0, 0, 0);
	//Mod the Time to loop the animation
	t = fmod(t, keyframes.back().first - keyframes.front().first);
	t += keyframes.front().first;

	//Find the Frames t lies between
	int index = 0;
	for (int i = 0; i < keyframes.size(); i++) {
		index = i;
		if (t < keyframes[i].first) {
			break;
		}
	}

	//Get the keyframes and values

	double t1 = keyframes[index - 1].first;
	double t2 = keyframes[index].first;


	Eigen::Vector3d P1 = keyframes[index - 1].second;
	Eigen::Vector3d P2 = keyframes[index].second;

	double t0;
	double t3;
	Eigen::Vector3d P0;
	Eigen::Vector3d P3;

	//Edge Cases for when there are not two points left or right of the query time
	// if the time is before any keyframe is given, return 000
	if (index == 1) {
		t0 = t1 - (t2 - t1);
		P0 = P1 - (P2 - P1);
	}
	else {
		t0 = keyframes[index - 2].first;
		P0 = keyframes[index - 2].second;
	}

	if (index == keyframes.size() - 1) {
		t3 = t2 + (t2 - t1);
		P3 = P2 + (P2 - P1);
	}
	else {
		t3 = keyframes[index + 1].first;
		P3 = keyframes[index + 1].second;
	}

	double u = (t - t1) / (t2 - t1);

	Eigen::RowVector4d uVec(1.0, u, pow(u, 2.0), pow(u, 3.0));
	Eigen::MatrixXd CRMatrix(4, 4);
	CRMatrix << 0, 1.0, 0, 0,
		-0.5, 0, 0.5, 0,
		1, -2.5, 2, -0.5,
		-0.5, 1.5, -1.5, 0.5;

	Eigen::MatrixXd P(4, 3);
	P.row(0) = P0.transpose();
	P.row(1) = P1.transpose();
	P.row(2) = P2.transpose();
	P.row(3) = P3.transpose();

	return uVec*CRMatrix*P;
	/////////////////////////////////////////////////////////////////////////////
}
