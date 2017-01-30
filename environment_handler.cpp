#include "environment_handler.h"

#include <string>
#include <limits>

using namespace OpenRAVE;

using std::max;
using std::numeric_limits;

const double error_c = .001;

/*** PRIVATE MEM FNS ***/

// get height of tallest surface encapsuling (x,y) point
dReal Environment_handler::get_z(dReal x, dReal y) {
	Vector point{x, y, 0, 1}; // reduce to 2 dimensions
	double max_height = 0;
	for(Structure* structure : structures) {
		Vector projected_foot_point = structure->get_inverse_transform() * point;
		if(structure->within_x_boundary(projected_foot_point) &&
		   structure->within_y_boundary(projected_foot_point)) {
			max_height = max(max_height, structure->get_height());
		}
	}
	return max_height;
}

/*** PUBLIC MEM FNS ***/

Environment_handler::Environment_handler(InterfaceType i_type, EnvironmentBasePtr _penv) : penv(_penv) {
	update_environment(i_type);
}

void Environment_handler::update_environment(InterfaceType i_type) {
	for(Structure* structure : structures) {
		penv->Remove(structure->get_kinbody());
	}

	General_box wall{RaveCreateKinBody(penv), 0.5, -.8, 2.4, 0, 0.1, .1, .1};

	Ground_box ground{RaveCreateKinBody(penv)};
	structures.push_back(&ground);


	structures.push_back(&wall);

	for(Structure* structure : structures) {
		structure->get_kinbody()->InitFromBoxes(structure->get_parameter(), true);
		penv->Add(structure->get_kinbody());
		structure->get_kinbody()->SetTransform(structure->get_transform());
		structure->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(structure->get_color());
	}

}

double Environment_handler::dist_to_boundary(dReal x, dReal y, dReal z) {
	double nearest_boundary_dist = numeric_limits<double>::max();
	
	Vector point{x, y, z, 1};

	for(Structure* structure : structures) {
		if(z <= structure->get_height() + error_c) { // ignore surfaces below point as their boundaries may be "overruled"
			Vector projected_point = structure->get_inverse_transform() * point;

			if(structure->within_x_boundary(projected_point)) {
				if(dist_to_left_bound(projected_point) < nearest_boundary_dist) {
					if()
				}
			}
		}
	}
	return 0;
}