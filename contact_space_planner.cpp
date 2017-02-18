#include "contact_space_planner.h"
#include "utility.h"

#include "cmath"

using OpenRAVE::dReal; using OpenRAVE::RaveTransformMatrix;

// foot orientation projected to flat gruond
dReal Node::get_left_horizontal_yaw() const {
	RaveTransformMatrix<dReal> left_foot_rotation = get_SO3(left_foot);
	return atan2(left_foot_rotation.m[3], left_foot_rotation.m[0]) * 180 / M_PI;
}

// foot orientation projected to flat gruond
dReal Node::get_right_horizontal_yaw() const {
	RaveTransformMatrix<dReal> right_foot_rotation = get_SO3(right_foot);
	return atan2(right_foot_rotation.m[3], right_foot_rotation.m[0]) * 180 / M_PI;
}

