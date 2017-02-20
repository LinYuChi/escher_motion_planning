#ifndef UTILITY_H
#define UTILITY_H

#include <openrave/plugin.h>

const OpenRAVE::dReal foot_height_c = 0.25;
const OpenRAVE::dReal foot_width_c = 0.135;
const OpenRAVE::dReal hand_height_c = 0.20;
const OpenRAVE::dReal hand_width_c = 0.14;

struct RPY_tf {
	OpenRAVE::dReal x;
	OpenRAVE::dReal y;
	OpenRAVE::dReal z;
	OpenRAVE::dReal roll; // degrees
	OpenRAVE::dReal pitch; // degrees
	OpenRAVE::dReal yaw; // degrees
};

OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> get_SO3(const RPY_tf & extremity);

#endif
