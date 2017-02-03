#include "structures.h"
#include <cmath>

using namespace OpenRAVE;
using std::vector;
using std::abs; using std::min; using std::sqrt;

const dReal ground_box_x_c = 2.5;
const dReal ground_box_y_c = 0;
const dReal ground_box_thickness_c = .01;
const dReal ground_box_theta_c = 0;
const dReal ground_box_ex_c = 3.5;
const dReal ground_box_ey_c = 3.5;
const Vector ground_box_color_c = Vector(120.0/255, 120.0/255, 120.0/255);

const dReal box_granularity_c = .01;
const dReal error_tolerance_c = .005;


int Structure::num_structures = 0;

/*** PRIVATE MEM FNS ***/
// given leg lengths, get hypotenuse length
dReal Box::hypotenuse(dReal q, dReal p) const {
	return sqrt(pow(q, 2) + pow(p, 2));
}

Box::Box(KinBodyPtr _kinbody, Vector _color, dReal _x, dReal _y, dReal _z, dReal _theta, 
	dReal _ex, dReal _ey, dReal _ez) : Structure(_kinbody), color(_color),
	x(_x), y(_y), z(_z), theta(_theta), ex(_ex), ey(_ey), ez(_ez) {

		// See: https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
		transform_matrix.rotfrommat(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1);
		transform_matrix.trans = Vector(x, y, z);
}

Transform Box::get_transform() const {
	return transform_matrix;
}

Transform Box::get_inverse_transform() const {
	return transform_matrix.inverse();
}

vector<AABB> Box::get_parameter() const {
	vector<AABB> ret_vec(1);
	ret_vec[0] = {Vector(0, 0, 0), Vector(ex, ey, ez)};
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
	return hypotenuse(dist_from_pos_x_bound(projected), dist_from_pos_y_bound(projected));
}

dReal Box::dist_from_quadrant_two_corner(const Vector & projected) const {
	return hypotenuse(dist_from_neg_x_bound(projected), dist_from_pos_y_bound(projected));
}

dReal Box::dist_from_quadrant_three_corner(const Vector & projected) const {
	return hypotenuse(dist_from_neg_x_bound(projected), dist_from_neg_y_bound(projected));
}

dReal Box::dist_from_quadrant_four_corner(const Vector & projected) const {
	return hypotenuse(dist_from_pos_x_bound(projected), dist_from_neg_y_bound(projected));
}


/*** BOUNDARY POINTS ***/
Vector Box::over_pos_y_bound(const Vector & projected) const {
	Vector over_boundary_point{projected[0], 0, 0, 1};
	if(projected[1] > ey) {
		over_boundary_point[1] = ey - box_granularity_c;
	} else {
		over_boundary_point[1] = ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_neg_y_bound(const Vector & projected) const {
	Vector over_boundary_point{projected[0], 0, 0, 1};
	if(projected[1] > -ey) {
		over_boundary_point[1] = -ey - box_granularity_c;
	} else {
		over_boundary_point[1] = -ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_pos_x_bound(const Vector & projected) const {
	Vector over_boundary_point{0, projected[1], 0, 1};
	if(projected[0] > ex) {
		over_boundary_point[0] = ex - box_granularity_c;
	} else {
		over_boundary_point[0] = ex + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_neg_x_bound(const Vector & projected) const {
	Vector over_boundary_point{0, projected[1], 0, 1};
	if(projected[0] > -ex) {
		over_boundary_point[0] = -ex - box_granularity_c;
	} else {
		over_boundary_point[0] = -ex + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

/*** CORNER POINTS ***/
Vector Box::over_quadrant_one_corner(const Vector & projected) const {
	Vector over_boundary_point{0, 0, 0, 1};
	if(projected[0] > ex) {
		over_boundary_point[0] = ex - box_granularity_c;
	} else {
		over_boundary_point[0] = ex + box_granularity_c;
	}
	if(projected[1] > ey) {
		over_boundary_point[1] = ey - box_granularity_c;
	} else {
		over_boundary_point[1] = ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_quadrant_two_corner(const Vector & projected) const {
	Vector over_boundary_point{0, 0, 0, 1};
	if(projected[0] > -ex) {
		over_boundary_point[0] = -ex - box_granularity_c;
	} else {
		over_boundary_point[0] = -ex + box_granularity_c;
	}
	if(projected[1] > ey) {
		over_boundary_point[1] = ey - box_granularity_c;
	} else {
		over_boundary_point[1] = ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_quadrant_three_corner(const Vector & projected) const {
	Vector over_boundary_point{0, 0, 0, 1};
	if(projected[0] > -ex) {
		over_boundary_point[0] = -ex - box_granularity_c;
	} else {
		over_boundary_point[0] = -ex + box_granularity_c;
	}
	if(projected[1] > -ey) {
		over_boundary_point[1] = -ey - box_granularity_c;
	} else {
		over_boundary_point[1] = -ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}

Vector Box::over_quadrant_four_corner(const Vector & projected) const {
	Vector over_boundary_point{0, 0, 0, 1};
	if(projected[0] > ex) {
		over_boundary_point[0] = ex - box_granularity_c;
	} else {
		over_boundary_point[0] = ex + box_granularity_c;
	}
	if(projected[1] > -ey) {
		over_boundary_point[1] = -ey - box_granularity_c;
	} else {
		over_boundary_point[1] = -ey + box_granularity_c;
	}
	return get_transform() * over_boundary_point;
}


Ground_box::Ground_box(KinBodyPtr _kinbody) : Box(_kinbody, ground_box_color_c,
					   ground_box_x_c, ground_box_y_c, -ground_box_thickness_c / 2, ground_box_theta_c,
					   ground_box_ex_c, ground_box_ey_c, ground_box_thickness_c / 2) {}

General_box::General_box(KinBodyPtr _kinbody, dReal _x, dReal _y, dReal height, dReal _theta, 
						 dReal _ex, dReal _ey) : Box(_kinbody, Vector(220/255, 220/255, 220/255),
					   	 _x, _y, height / 2, _theta, _ex, _ey, height / 2) {}



/*** TRIANGULAR MESH ***/

/*** PRIVATE MEM FNS ***/

OpenRAVE::dReal Tri_mesh::distance(OpenRAVE::Vector q, OpenRAVE::Vector p) const {
	return sqrt(pow(q[0] - p[0], 2) + pow(q[1] - p[1], 2) + pow(q[2] - p[2], 2));
}

/*** PUBLIC MEM FNS ***/

void Tri_mesh::transform_data(OpenRAVE::Transform transform) {
	Vector transformed_normal = transform * get_normal();
	nx = transformed_normal[0];
	ny = transformed_normal[1];
	nz = transformed_normal[2];

	// vertices transformation here

	// c assignment
}

Vector Tri_mesh::get_normal() const {
	return Vector{nx, ny, nz};
}

Vector Tri_mesh::get_center() const {
	return Vector{xo, yo, zo};
}

Transform Tri_mesh::get_transform() const {
	return transform_matrix;
}

Transform Tri_mesh::get_inverse_transform() const {
	return transform_matrix.inverse();
}

Transform Tri_mesh::projection_plan_frame(Vector point, Vector ray) const {
	return ;
}


bool Tri_mesh::inside_polygon(Vector point) const {
	dReal x = point[0]; dReal y = point[1]; dReal z = point[2];

	if(abs(nx * x + ny * y + nz * z + c) > error_tolerance_c) {
		return false;
	}

	if (distance() >= circumscribed_radius) {
		return false;
	}

	projected_point = projection_plane_frame(point);
	return inside_polygon_plane_frame(projected_point);
}
