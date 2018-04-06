#include "common.h"

std::vector<float> computeEularAngles(Eigen::Matrix4f& R, bool israd){
	std::vector<float> result(3, 0);
	const float pi = 3.14159265397932384626433;

	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = pi - theta1;
		float psi1 = atan2(R(2, 1) / cos(theta1), R(2, 2) / cos(theta1));
		float psi2 = atan2(R(2, 0) / cos(theta2), R(2, 2) / cos(theta2));
		float pfi1 = atan2(R(1, 0) / cos(theta1), R(0, 0) / cos(theta1));
		float pfi2 = atan2(R(1, 0) / cos(theta2), R(0, 0) / cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	}
	else{
		float phi = 0;
		float delta = atan2(R(0, 1), R(0, 2));
		if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN){ // R(2,0) == -1
			theta = pi / 2;
			psi = phi + delta;
		}
		else{
			theta = -pi / 2;
			psi = -phi + delta;
		}
	}

	// psi is along x-axis, theta is along y-axis, pfi is along z axis
	if (israd){ // for rad 
		result[0] = psi;
		result[1] = theta;
		result[2] = pfi;
	}
	else{ // for deg
		result[0] = psi * 180 / pi;
		result[1] = theta * 180 / pi;
		result[2] = pfi * 180 / pi;
	}
	return result;
}