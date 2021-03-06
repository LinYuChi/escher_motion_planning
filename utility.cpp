#include "utility.h"

#include <cmath>

using OpenRAVE::dReal; using OpenRAVE::RaveTransformMatrix; using OpenRAVE::Vector;
using std::vector;
using std::cos; using std::sin;

const OpenRAVE::dReal foot_height_c = 0.25;
const OpenRAVE::dReal foot_width_c = 0.135;
const OpenRAVE::dReal hand_height_c = 0.20;
const OpenRAVE::dReal hand_width_c = 0.14;

// euclidean distance btwn two points in a 2D coordinate system
dReal euclidean_distance_2d(const Vector & q, const Vector & p) {
	return sqrt(pow(q.x - p.x, 2) + pow(q.y - p.y, 2));
}

// euclidean distance btwn two points in a 3D coordinate system
dReal euclidean_distance_3d(const Vector & q, const Vector & p) {
	return sqrt(pow(q.x - p.x, 2) + pow(q.y - p.y, 2) + pow(q.z - p.z, 2));
}

RaveTransformMatrix<dReal> get_SO3(const RPY_tf & e) {
	double roll_in_rad = e.roll * (M_PI / 180);
	double pitch_in_rad = e.pitch * (M_PI / 180);
	double yaw_in_rad = e.yaw * (M_PI / 180);

	double roll_x = cos(roll_in_rad);
	double roll_y = sin(roll_in_rad);

	double pitch_x = cos(pitch_in_rad);
	double pitch_y = sin(pitch_in_rad);

	double yaw_x = cos(yaw_in_rad);
	double yaw_y = sin(yaw_in_rad);

	RaveTransformMatrix<dReal> ret_mat;
	ret_mat.rotfrommat(pitch_x * yaw_x, -pitch_x * yaw_y, pitch_y,
							  roll_x * yaw_y + yaw_x * roll_y * pitch_y, roll_x * yaw_x - roll_y * pitch_y * yaw_y, -pitch_x * roll_y,
							  roll_y * yaw_y - roll_x * yaw_x * pitch_y, yaw_x * roll_y + roll_x * pitch_y * yaw_y, roll_x * pitch_x);
	return ret_mat;
}

// SE(3) is similar