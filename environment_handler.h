#ifndef ENVIRONMENT_HANDLER_H
#define ENVIRONMENT_HANDLER_H

#include "structures.h"

#include <vector>

class Environment_handler {
	std::vector<Structure*> structures;
	OpenRAVE::EnvironmentBasePtr penv;
	double start_dist_to_boundary;
	double goal_dist_to_boundary;
	double goal_x;
	double goal_y;
	double goal_z;
public:
	Environment_handler(OpenRAVE::InterfaceType i_type, OpenRAVE::EnvironmentBasePtr _penv);
	void update_environment(OpenRAVE::InterfaceType i_type);
	double dist_to_boundary(dReal x, dReal y, dReal z) {}
};

#endif
