#include "structures.h"
#include <cmath>

using namespace OpenRAVE;

const dReal init_ground_box_x_c = 2.5;
const dReal init_ground_box_y_c = 0.;
const dReal init_ground_box_ex_c = 3.5;
const dReal init_ground_box_ey_c = 3.5;
const dReal init_ground_box_ez_c = 0.005;
const dReal init_ground_box_theta_c = 0.;
const dReal init_ground_box_color_c = 120.0/255;

Box::Box(int _id, dReal _z) : Structure(_id), x(init_ground_box_x_c), y(init_ground_box_y_c), theta(init_ground_box_theta_c), ex(init_ground_box_ex_c), ey(init_ground_box_ey_c), ez(init_ground_box_ez_c) {
	z = _z - ez;
	color = Vector(init_ground_box_color_c, init_ground_box_color_c, init_ground_box_color_c);
}

Transform Box::get_transform() const {
	RaveTransformMatrix<dReal> rot_mat;
	rot_mat.rotfrommat(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1);
	rot_mat.trans = Vector(x, y, z);
	return rot_mat;
}

Transform Box::get_inverse_transform() const {
	RaveTransformMatrix<dReal> rot_mat;
	rot_mat.rotfrommat(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1);
	rot_mat.trans = Vector(x, y, z);
	return rot_mat.inverse();
}
