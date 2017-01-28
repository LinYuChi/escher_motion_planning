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
	OpenRAVE::dReal get_z(OpenRAVE::dReal x, OpenRAVE::dReal y);
public:
	Environment_handler(OpenRAVE::InterfaceType i_type, OpenRAVE::EnvironmentBasePtr _penv);
	void update_environment(OpenRAVE::InterfaceType i_type);
	double dist_to_boundary(OpenRAVE::dReal x, OpenRAVE::dReal y, OpenRAVE::dReal z);
};

#endif
