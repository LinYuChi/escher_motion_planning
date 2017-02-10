#include "structures.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iterator>

using namespace OpenRAVE;
using std::vector; using std::pair; using std::map;
using std::abs; using std::sqrt;
using std::numeric_limits;
using std::swap; using std::min; using std::max;
using std::distance;

const dReal ground_box_x_c = 2.5;
const dReal ground_box_y_c = 0;
const dReal ground_box_thickness_c = .01;
const dReal ground_box_theta_c = 0;
const dReal ground_box_ex_c = 3.5;
const dReal ground_box_ey_c = 3.5;
const Vector ground_box_color_c = Vector(120.0/255, 120.0/255, 120.0/255);

const dReal box_granularity_c = .01;
const dReal error_tolerance_c = .005;
const dReal surface_slice_resolution_c = .005;


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
	return transform_matrix * over_boundary_point;
}

Vector Box::over_neg_y_bound(const Vector & projected) const {
	Vector over_boundary_point{projected[0], 0, 0, 1};
	if(projected[1] > -ey) {
		over_boundary_point[1] = -ey - box_granularity_c;
	} else {
		over_boundary_point[1] = -ey + box_granularity_c;
	}
	return transform_matrix * over_boundary_point;
}

Vector Box::over_pos_x_bound(const Vector & projected) const {
	Vector over_boundary_point{0, projected[1], 0, 1};
	if(projected[0] > ex) {
		over_boundary_point[0] = ex - box_granularity_c;
	} else {
		over_boundary_point[0] = ex + box_granularity_c;
	}
	return transform_matrix * over_boundary_point;
}

Vector Box::over_neg_x_bound(const Vector & projected) const {
	Vector over_boundary_point{0, projected[1], 0, 1};
	if(projected[0] > -ex) {
		over_boundary_point[0] = -ex - box_granularity_c;
	} else {
		over_boundary_point[0] = -ex + box_granularity_c;
	}
	return transform_matrix * over_boundary_point;
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
	return transform_matrix * over_boundary_point;
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
	return transform_matrix * over_boundary_point;
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
	return transform_matrix * over_boundary_point;
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
	return transform_matrix * over_boundary_point;
}


Ground_box::Ground_box(KinBodyPtr _kinbody) : Box(_kinbody, ground_box_color_c,
					   ground_box_x_c, ground_box_y_c, -ground_box_thickness_c / 2, ground_box_theta_c,
					   ground_box_ex_c, ground_box_ey_c, ground_box_thickness_c / 2) {}

General_box::General_box(KinBodyPtr _kinbody, dReal _x, dReal _y, dReal height, dReal _theta, 
						 dReal _ex, dReal _ey) : Box(_kinbody, Vector(),
					   	 _x, _y, height / 2, _theta, _ex, _ey, height / 2) {}


/*** TRIANGULAR MESH ***/

/*** PRIVATE MEM FNS ***/

dReal Tri_mesh::euclidean_distance(Vector q, Vector p) const {
	return sqrt(pow(q[0] - p[0], 2) + pow(q[1] - p[1], 2) + pow(q[2] - p[2], 2));
}

void Tri_mesh::update_center() {
	circumradius = 0;
	for(int i = 0; i < vertices.size() - 1; ++i) {
		for(int j = i + 1; j < vertices.size(); ++j) {
			dReal dist = euclidean_distance(vertices[i], vertices[j]);
			if(dist / 2 > circumradius) {
				circumradius = dist / 2;
				xo = (vertices[i][0] + vertices[j][0]) / 2;
				yo = (vertices[i][1] + vertices[j][1]) / 2;
				zo = (vertices[i][2] + vertices[j][2]) / 2;
			}
		}
	}
}

void Tri_mesh::update_proj_vertices() {
	min_proj_x = numeric_limits<dReal>::max();
	max_proj_x = numeric_limits<dReal>::min();

	min_proj_y = numeric_limits<dReal>::max();
	max_proj_y = numeric_limits<dReal>::min();

	for(Vector const & vertex : vertices) {
		Vector proj_vertex = projection_plane_frame(vertex);
		proj_vertices.push_back(proj_vertex);
		
		min_proj_x = min(min_proj_x, proj_vertex.x);
		max_proj_x = max(max_proj_x, proj_vertex.x);

		min_proj_y = min(min_proj_y, proj_vertex.y);
		max_proj_y = max(max_proj_y, proj_vertex.y);
	}

	if(proj_vertices.size()) { 
		proj_vertices.push_back(proj_vertices[0]); // "close the loop"
	}
}

void Tri_mesh::update_approx_boundary() {
	for(pair<int, int> edge : edges) {
		dReal start_x = proj_vertices[edge.first].x;
		dReal start_y = proj_vertices[edge.first].y;
		dReal end_x = proj_vertices[edge.second].x;
		dReal end_y = proj_vertices[edge.second].y;

		if(start_x == end_x) continue;

		if(start_x > end_x) {
			swap(start_x, end_x);
			swap(start_y, end_y);
		}

		int start_x_key = ceil((start_x - min_proj_x) / surface_slice_resolution_c);
		int end_x_key = (end_x - min_proj_x) / surface_slice_resolution_c;

		for(int i = start_x_key; i <= end_x_key; ++i) {
			dReal x_coord = i * surface_slice_resolution_c + min_proj_x;
			dReal y_coord = start_y + (x_coord - start_x) / (end_x - start_x) * (end_y - start_y);
			if(boundaries.find(i) != boundaries.end()) {
				boundaries[i].push_back(y_coord);
			} else {
				boundaries[i] = {y_coord};
			}
		}

		// sort all y boundary points
		for(auto & boundary_pair : boundaries) {
			sort(boundary_pair.second.begin(), boundary_pair.second.end());
		}
	}
}

/*** PUBLIC MEM FNS ***/

Tri_mesh::Tri_mesh(KinBodyPtr _kinbody, Vector plane_parameters,
				   vector<pair<int, int> > _edges, vector<Vector> _vertices) :
				   Structure(_kinbody), edges(_edges), vertices(_vertices) {

	plane_parameters.normalize();
	nx = plane_parameters[0];
	ny = plane_parameters[1];
	nz = plane_parameters[2];
	c = plane_parameters[3];

	update_center();

	Vector xv{vertices[0][0] - xo, vertices[0][1] - yo, vertices[0][2] - zo};
	xv.normalize();
	Vector yv = get_normal().cross(xv); // double check this?
	Vector zv = get_normal();

	transform_matrix.rotfrommat(xv[0], xv[1], xv[2], yv[0], yv[1], yv[2], zv[0], zv[1], zv[2]);
	transform_matrix.trans = get_center();
	inverse_transform_matrix = transform_matrix.inverse();

	update_proj_vertices();
}


void Tri_mesh::transform_data(OpenRAVE::Transform transform) {
	transform_matrix = transform_matrix * transform;
	inverse_transform_matrix = inverse_transform_matrix * transform.inverse();

	Vector transformed_normal = transform * get_normal();
	nx = transformed_normal.x;
	ny = transformed_normal.y;
	nz = transformed_normal.z;

	Vector transformed_center = transform * get_center();
	xo = transformed_center.x;
	yo = transformed_center.y;
	zo = transformed_center.z;

	c = -(nx * vertices[0].x + ny * vertices[0].y + nz * vertices[0].z);

	for(int i = 0; i < vertices.size(); ++i) {
		vertices[i] = transform * vertices[i];
	}

	update_proj_vertices();
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
	return inverse_transform_matrix;
}

// returns 2D point projected in plane frame. This assumes the "ray" is the surface normal.
Vector Tri_mesh::projection_plane_frame(const Vector & point) const {
	Vector proj_point = inverse_transform_matrix * point;
	proj_point.z = 0; // flatten point
	return proj_point;
}


bool Tri_mesh::inside_polygon(const Vector & point) const {
	dReal x = point[0]; dReal y = point[1]; dReal z = point[2];

	if(abs(nx * x + ny * y + nz * z + c) > error_tolerance_c) {
		return false;
	}

	if (euclidean_distance(point, get_center()) >= circumradius) {
		return false;
	}

	Vector projected_point = projection_plane_frame(point);
	return inside_polygon_plane_frame(projected_point);
}

bool Tri_mesh::inside_polygon_plane_frame(const Vector & projected_point) const {
	if(projected_point.x > max_proj_x || projected_point.x < min_proj_x || 
	   projected_point.y > max_proj_y || projected_point.y < min_proj_y) {
		return false;
	}

	int query_x = (projected_point.x - min_proj_x) / surface_slice_resolution_c;
	auto y_bounds_it = boundaries.find(query_x);

	if(y_bounds_it == boundaries.end()) {
		return false;
	}

	int pass_boundary_count = 0;

	// considers points "on border" to be within the frame
	auto bound_it = lower_bound(y_bounds_it->second.begin(), y_bounds_it->second.end(), projected_point.y);
	return distance(bound_it, y_bounds_it->second.begin()) % 2 == 1;
}
