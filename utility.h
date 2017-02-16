#ifndef UTILIT_H
#define UTILITY_H

#include <openrave/plugin.h>
#include <vector>

std::vector<OpenRAVE::dReal> get_SO3(OpenRAVE::dReal roll_in_deg,
									 OpenRAVE::dReal pitch_in_deg, OpenRAVE::dReal yaw_in_deg);

#endif
