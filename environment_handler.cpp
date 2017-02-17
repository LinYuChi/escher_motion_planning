#include "environment_handler.h"

#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <iostream>
#include <set>

using namespace OpenRAVE;

using std::vector; using std::pair; using std::set;
using std::cout; using std::endl;
using std::max;
using std::numeric_limits;
using std::unique_ptr; using std::move;

const double error_c = .001;
const double clearance_error_c = .01;

/*** PRIVATE MEM FNS ***/

// get height of tallest surface encapsulating (x,y) point
dReal Environment_handler::highest_z(dReal x, dReal y) {
	Vector point{x, y, 0, 1}; // reduce to 2 dimensions
	double max_height = 0;
	for(unique_ptr<Box> & box : boxes) {
		Vector projected_foot_point = box->get_inverse_transform() * point;
		if(box->within_x_bounds(projected_foot_point) &&
		   box->within_y_bounds(projected_foot_point)) {
			max_height = max(max_height, box->get_height());
		}
	}
	return max_height;
}

// returns true if over_boundary_point is a "fake" boundary (e.g. adjacent box is same height)
bool Environment_handler::even_boundary_surface_height(const Vector& over_boundary_point, dReal z) {
	return z == highest_z(over_boundary_point[0], over_boundary_point[1]);
}

void Environment_handler::add_tri_mesh_cylinder(dReal z_range, dReal r) {
	RaveCreateKinBody(penv);

	// TODO: complete fn
}

/*** PUBLIC MEM FNS ***/

Environment_handler::Environment_handler(InterfaceType i_type, EnvironmentBasePtr _penv) : penv(_penv) {
	update_environment(i_type);
}

void Environment_handler::update_environment(InterfaceType i_type) {
	for(unique_ptr<Box> & box : boxes) {
		penv->Remove(box->get_kinbody());
	}

	for(unique_ptr<Tri_mesh> & tri_mesh : tri_meshes) {
		penv->Remove(tri_mesh->get_kinbody());
	}

	unique_ptr<Box> ground (new Ground_box{RaveCreateKinBody(penv)});
	unique_ptr<Box> b1 (new General_box{RaveCreateKinBody(penv), -2, -3, .1, 0, .5, .5});
	unique_ptr<Box> b2 (new General_box{RaveCreateKinBody(penv), -2, -3, .1, 0, .5, .5});

	vector<Vector> tri_vertices {
		{0.5, 0.5, -.005},
		{-0.5, 0.5, -.005},
		{-0.5, -0.5, -.005},
		{0.5, -0.5, -.005}
	};

	vector<pair<int, int> > tri_edges {
		std::make_pair(0, 1),
		std::make_pair(1, 2),
		std::make_pair(2, 3)
	};

	unique_ptr<Tri_mesh> tri (new Tri_mesh{RaveCreateKinBody(penv), {0, 0, 1, 0}, tri_edges, tri_vertices});

	// boxes.push_back(move(b1));
	// boxes.push_back(move(b2));
	// boxes.push_back(move(ground));

	tri_meshes.push_back(move(tri));

	for(unique_ptr<Box> & box : boxes) {
		box->get_kinbody()->InitFromBoxes(box->get_parameter(), true);
		penv->Add(box->get_kinbody());
		box->get_kinbody()->SetTransform(box->get_transform());
		box->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(box->get_color());
	}

	for(unique_ptr<Tri_mesh> & tri_mesh : tri_meshes) {
		TriMesh tm;
		tm.vertices = tri_vertices;
		tm.indices = {0, 1, 2, 0, 2, 3};
		tri_mesh->get_kinbody()->InitFromTrimesh(tm, true);
		penv->Add(tri_mesh->get_kinbody());
		tri_mesh->get_kinbody()->SetTransform(tri_mesh->get_transform());
	}
}

// box world
// this fn can be less LOC by creating vector of "target points" (e.g. courners, 
// perpendiculars, etc) and defining distance/"over boundary point" functions for
// separate cases (corners, perpendiculars)
double Environment_handler::dist_to_boundary(dReal x, dReal y, dReal z) {
	double nearest_boundary_dist = numeric_limits<double>::max();
	Vector point{x, y, z};
	for(unique_ptr<Box> & box : boxes) {
		if(z > box->get_height() + error_c) {
			// ignore surfaces below point as their boundaries may be overriden
			continue;
		}

		Vector projected_point = box->get_inverse_transform() * point;

		if(box->within_x_bounds(projected_point)) {
			if(box->dist_from_pos_y_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = box->over_pos_y_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = box->dist_from_pos_y_bound(projected_point);
				}
			}

			if(box->dist_from_neg_y_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = box->over_neg_y_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = box->dist_from_neg_y_bound(projected_point);
				}
			}

		}

		if(box->within_y_bounds(projected_point)) {
			if(box->dist_from_pos_x_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = box->over_pos_x_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = box->dist_from_pos_x_bound(projected_point);
				}
			}

			if(box->dist_from_neg_x_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = box->over_neg_x_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = box->dist_from_neg_x_bound(projected_point);
				}
			}

		}

		// check distance to corners of surface
		// possible use case: point is not perpendicular to any of surface's boundaries

		if(box->dist_from_quadrant_one_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = box->over_quadrant_one_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = box->dist_from_quadrant_one_corner(projected_point);
			}
		}

		if(box->dist_from_quadrant_two_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = box->over_quadrant_two_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = box->dist_from_quadrant_two_corner(projected_point);
			}
		}

		if(box->dist_from_quadrant_three_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = box->over_quadrant_three_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = box->dist_from_quadrant_three_corner(projected_point);
			}
		}

		if(box->dist_from_quadrant_four_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = box->over_quadrant_four_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = box->dist_from_quadrant_four_corner(projected_point);
			}
		}

	}
	
	return nearest_boundary_dist;
}

// returns set of circular regions
vector<Vector> Environment_handler::sample_points(const Tri_mesh & tri_mesh, double resolution, double boundary_clearance) {
	vector<Vector> contact_samples;
	set<pair<dReal, dReal> > checked_samples; // use pairs to utilize c++ pair's less than operator in set ordering

	for(dReal proj_x = tri_mesh.get_min_proj_x(); proj_x < tri_mesh.get_max_proj_x(); proj_x += resolution) {
		for(dReal proj_y = tri_mesh.get_min_proj_y(); proj_y < tri_mesh.get_max_proj_y(); proj_y += resolution) {

			pair<dReal, dReal> curr_proj{proj_x, proj_y};
			Vector curr_proj_point{curr_proj.first, curr_proj.second, 0}; // flatten to two dimensions

			if(checked_samples.find(curr_proj) != checked_samples.end()) {
				continue;
			}

			checked_samples.insert(curr_proj);

			if(!tri_mesh.inside_polygon_plane_frame(curr_proj_point)) {
				continue;
			}

			dReal r = tri_mesh.dist_to_boundary(curr_proj_point);

			if(r <= boundary_clearance + clearance_error_c) {
				continue;
			}

			// check collision
			Vector point = tri_mesh.get_transform() * curr_proj_point;

			// call point_free_space

			curr_proj_point.z = r - boundary_clearance;
			contact_samples.push_back({curr_proj_point});

			// TODO: add proj_xx and proj_yy
		}
	}
	return contact_samples;
}

// Checks if there are obstacles inside radius r of sampled point above the surface
bool Environment_handler::point_free_space(const Tri_mesh & tri_mesh, dReal r,
										   RaveTransformMatrix<dReal> tf) {
	return true; 
	//TODO : complete this fn
}