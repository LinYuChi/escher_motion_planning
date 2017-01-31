#ifndef ENVIRONMENT_HANDLER_H
#define ENVIRONMENT_HANDLER_H

#include "structures.h"

#include <vector>

class Environment_handler {
	std::vector<Structure*> structures;
	OpenRAVE::EnvironmentBasePtr penv;
	// double start_dist_to_boundary;
	// double goal_dist_to_boundary;
	// double goal_x;
	// double goal_y;
	// double goal_z;

	// get height of tallest surface encapsuling (x,y) point
	OpenRAVE::dReal highest_z(OpenRAVE::dReal x, OpenRAVE::dReal y);
	// returns true if over_boundary_point is a "fake" boundary (e.g. adjacent structure is same height)
	bool even_boundary_surface_height(const OpenRAVE::Vector& over_boundary_point, OpenRAVE::dReal z);
public:
	Environment_handler(OpenRAVE::InterfaceType i_type, OpenRAVE::EnvironmentBasePtr _penv);
	void update_environment(OpenRAVE::InterfaceType i_type);
	// box world
	double dist_to_boundary(OpenRAVE::dReal x, OpenRAVE::dReal y, OpenRAVE::dReal z);
};

#endif
