#ifndef ENVIRONMENT_HANDLER_H
#define ENVIRONMENT_HANDLER_H

#include "structures.h"

#include <vector>

class Environment_handler {
	std::vector<Structure> structures;
	OpenRAVE::EnvironmentBasePtr penv;
public:
	Environment_handler(OpenRAVE::EnvironmentBasePtr penv_);
	void update_environment();
	double dist_to_boundary();
};

#endif
