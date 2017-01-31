#include "structures.h"
#include <cmath>

using namespace OpenRAVE;
using std::vector;
using std::abs; using std::min; using std::sqrt;

const dReal ground_box_x_c = 0;
const dReal ground_box_y_c = 0;
const dReal ground_box_z_c = -0.005;
const dReal ground_box_theta_c = 0;
const dReal ground_box_ex_c = 5.5;
const dReal ground_box_ey_c = 3.5;
const dReal ground_box_ez_c = 0.005;
const Vector ground_box_color_c = Vector(120.0/255, 120.0/255, 120.0/255);

const dReal box_granularity_c = .01;

int Structure::num_structures = 0;

Box::Box(KinBodyPtr _kinbody, Vector _color, dReal _x, dReal _y, dReal _z, dReal _theta, 
	dReal _ex, dReal _ey, dReal _ez) : Structure(_kinbody), color(_color),
	x(_x), y(_y), z(_z), theta(_theta), ex(_ex), ey(_ey), ez(_ez) {

		// See: https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
		rot_mat.rotfrommat(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1);
		rot_mat.trans = Vector(x, y, z);
}

Transform Box::get_transform() const {
	return rot_mat;
}

Transform Box::get_inverse_transform() const {
	return rot_mat.inverse();
}

vector<AABB> Box::get_parameter() const {
	vector<AABB> ret_vec(1);
	ret_vec[0] = {RaveVector<dReal>(x, y, z), RaveVector<dReal>(ex, ey, ez)};
	return ret_vec;
}


/*** BOUNDARIES ***/
bool Box::within_x_bounds(const Vector & projected) const {
	return abs(projected[0]) <= ex;
}

bool Box::within_y_bounds(const Vector & projected) const {
	return abs(projected[1]) <= ey;
}


/*** PERPENDICULAR DISTANCES ***/
dReal Box::dist_from_pos_y_bound(const Vector & projected) const {
	return abs(ey - projected[1]);
}

dReal Box::dist_from_neg_y_bound(const Vector & projected) const {
	return abs(-ey - projected[1]);
}

dReal Box::dist_from_pos_x_bound(const Vector & projected) const {
	return abs(ex - projected[0]);
}

dReal Box::dist_from_neg_x_bound(const Vector & projected) const {
	return abs(-ex - projected[0]);
}


/*** CORNER DISTANCES ***/
dReal Box::dist_from_quadrant_one_corner(const Vector & projected) const {
	return sqrt(pow(dist_from_pos_x_bound(projected), 2) + pow(dist_from_pos_y_bound(projected), 2));
}

dReal Box::dist_from_quadrant_two_corner(const Vector & projected) const {
	return sqrt(pow(dist_from_neg_x_bound(projected), 2) + pow(dist_from_pos_y_bound(projected), 2));
}

dReal Box::dist_from_quadrant_three_corner(const Vector & projected) const {
	return sqrt(pow(dist_from_neg_x_bound(projected), 2) + pow(dist_from_neg_y_bound(projected), 2));
}

dReal Box::dist_from_quadrant_four_corner(const Vector & projected) const {
	return sqrt(pow(dist_from_pos_x_bound(projected), 2) + pow(dist_from_neg_y_bound(projected), 2));
}


/*** BOUNDARY POINTS ***/
Vector Box::over_pos_y_bound(const Vector & projected) const {
	Vector over_boundary_point{projected[0], ey + box_granularity_c, 0, 1};
	return get_transform() * over_boundary_point;
}

Vector Box::over_neg_y_bound(const Vector & projected) const {
	Vector over_boundary_point{projected[0], -1 * (ey + box_granularity_c), 0, 1};
	return get_transform() * over_boundary_point;
}

Vector Box::over_pos_x_bound(const Vector & projected) const {
	Vector over_boundary_point{ex + box_granularity_c, projected[1], 0, 1};
	return get_transform() * over_boundary_point;
}

Vector Box::over_neg_x_bound(const Vector & projected) const {
	Vector over_boundary_point{-1 * (ex + box_granularity_c), projected[1], 0, 1};
	return get_transform() * over_boundary_point;
}


Ground_box::Ground_box(KinBodyPtr _kinbody) : Box(_kinbody, ground_box_color_c,
					   ground_box_x_c, ground_box_y_c, ground_box_z_c, ground_box_theta_c,
					   ground_box_ex_c, ground_box_ey_c, ground_box_ez_c) {}

General_box::General_box(KinBodyPtr _kinbody, dReal _x, dReal _y, dReal _z, dReal _theta, 
						 dReal _ex, dReal _ey, dReal _ez) : Box(_kinbody, Vector(210/255, 210/255, 210/255),
					   	 _x, _y, _z, _theta, _ex, _ey, _ez) {}
