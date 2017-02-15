#include "environment_handler.h"

#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <iostream>

using namespace OpenRAVE;

using std::vector; using std::pair;
using std::cout; using std::endl;
using std::max;
using std::numeric_limits;
using std::unique_ptr; using std::move;

const double error_c = .001;

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
		{0.5, 0.5, 0},
		{-0.5, 0.5, 0},
		{-0.5, -0.5, 0},
		{0.5, -0.5, 0}
	};

	vector<pair<int, int> > tri_edges {
		std::make_pair(0, 1),
		std::make_pair(1, 2),
		std::make_pair(2, 3),
		std::make_pair(3, 0)
	};

	unique_ptr<Tri_mesh> tri (new Tri_mesh{RaveCreateKinBody(penv), {0, 0, 1, 0}, tri_edges, tri_vertices});

	// boxes.push_back(move(b1));
	// boxes.push_back(move(b2));
	// boxes.push_back(move(ground));

	// tri_meshes.push_back(move(tri));

	// for(unique_ptr<Box> & box : boxes) {
	// 	box->get_kinbody()->InitFromBoxes(box->get_parameter(), true);
	// 	penv->Add(box->get_kinbody());
	// 	box->get_kinbody()->SetTransform(box->get_transform());
	// 	box->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(box->get_color());
	// }
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
			cout << over_boundary_point << endl;
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