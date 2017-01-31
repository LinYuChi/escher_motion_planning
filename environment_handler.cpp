#include "environment_handler.h"

#include <string>
#include <limits>

using namespace OpenRAVE;

using std::max;
using std::numeric_limits;

const double error_c = .001;

/*** PRIVATE MEM FNS ***/

// get height of tallest surface encapsuling (x,y) point
dReal Environment_handler::highest_z(dReal x, dReal y) {
	Vector point{x, y, 0, 1}; // reduce to 2 dimensions
	double max_height = 0;
	for(std::unique_ptr<Structure> & structure : structures) {
		Vector projected_foot_point = structure->get_inverse_transform() * point;
		if(structure->within_x_bounds(projected_foot_point) &&
		   structure->within_y_bounds(projected_foot_point)) {
			max_height = max(max_height, structure->get_height());
		}
	}
	return max_height;
}

// returns true if over_boundary_point is a "fake" boundary (e.g. adjacent structure is same height)
bool Environment_handler::even_boundary_surface_height(const Vector& over_boundary_point, dReal z) {
	return z == highest_z(over_boundary_point[0], over_boundary_point[0]);
}


/*** PUBLIC MEM FNS ***/

Environment_handler::Environment_handler(InterfaceType i_type, EnvironmentBasePtr _penv) : penv(_penv) {
	update_environment(i_type);
}

void Environment_handler::update_environment(InterfaceType i_type) {
	for(std::unique_ptr<Structure> & structure : structures) {
		penv->Remove(structure->get_kinbody());
	}

	std::unique_ptr<Structure> wall (new General_box{RaveCreateKinBody(penv), 0.5, -.8, 2.4, 0, 0.1, .1, .1});
	std::unique_ptr<Structure> ground (new Ground_box{RaveCreateKinBody(penv)});
	structures.push_back(std::move(wall));
	structures.push_back(std::move(ground));

	for(std::unique_ptr<Structure> & structure : structures) {
		structure->get_kinbody()->InitFromBoxes(structure->get_parameter(), true);
		penv->Add(structure->get_kinbody());
		structure->get_kinbody()->SetTransform(structure->get_transform());
		structure->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(structure->get_color());
	}

}

// box world
// this fn can be less LOC by creating vector of "target points" (e.g. courners, 
// perpendiculars, etc) and defining distance/"over boundary point" functions for
// separate cases (corners, perpendiculars)
double Environment_handler::dist_to_boundary(dReal x, dReal y, dReal z) {
	double nearest_boundary_dist = numeric_limits<double>::max();
	Vector point{x, y, z};

	for(std::unique_ptr<Structure> & structure : structures) {
		if(z > structure->get_height() + error_c) {
			// ignore surfaces below point as their boundaries may be overriden
			continue;
		}

		Vector projected_point = structure->get_inverse_transform() * point;

		if(structure->within_x_bounds(projected_point)) {
			if(structure->dist_from_pos_y_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = structure->over_pos_y_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = structure->dist_from_pos_y_bound(projected_point);
				}
			}

			if(structure->dist_from_neg_y_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = structure->over_neg_y_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = structure->dist_from_neg_y_bound(projected_point);
				}
			}

		}

		if(structure->within_y_bounds(projected_point)) {
			if(structure->dist_from_pos_x_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = structure->over_pos_x_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = structure->dist_from_pos_x_bound(projected_point);
				}
			}

			if(structure->dist_from_neg_x_bound(projected_point) < nearest_boundary_dist) {
				Vector over_boundary_point = structure->over_neg_x_bound(projected_point);

				if(!even_boundary_surface_height(over_boundary_point, z)) {
					nearest_boundary_dist = structure->dist_from_neg_x_bound(projected_point);
				}
			}

		}

		// check distance to corners of surface
		// possible use case: point is not perpendicular to any of surface's boundaries

		if(structure->dist_from_quadrant_one_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = structure->over_quadrant_one_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = structure->dist_from_quadrant_one_corner(projected_point);
			}
		}

		if(structure->dist_from_quadrant_two_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = structure->over_quadrant_two_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = structure->dist_from_quadrant_two_corner(projected_point);
			}
		}

		if(structure->dist_from_quadrant_three_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = structure->over_quadrant_three_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = structure->dist_from_quadrant_three_corner(projected_point);
			}
		}

		if(structure->dist_from_quadrant_four_corner(projected_point) < nearest_boundary_dist) {
			Vector over_boundary_point = structure->over_quadrant_four_corner(projected_point);

			if(!even_boundary_surface_height(over_boundary_point, z)) {
				nearest_boundary_dist = structure->dist_from_quadrant_four_corner(projected_point);
			}
		}

	}
	
	return nearest_boundary_dist;
}