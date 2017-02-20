#ifndef UTILITY_H
#define UTILITY_H

#include <openrave/plugin.h>

const extern OpenRAVE::dReal foot_height_c;
const extern OpenRAVE::dReal foot_width_c;
const extern OpenRAVE::dReal hand_height_c;
const extern OpenRAVE::dReal hand_width_c;

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
