#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include "utility.h"
#include "contact_space_planner.h"

#include <vector>
#include <string>

struct Contact_region {
	OpenRAVE::Vector position; // region center
	OpenRAVE::Vector normal; // unit normal vector
	OpenRAVE::dReal radius;
	int group; // the robot can slide between contacts of the same group
};

enum class Manip {
	L_foot,
	R_foot,
	L_hand,
	R_hand
};

struct Contact {
	RPY_tf transform;
	Manip manip;
};

struct Motion_plan {
	std::vector<std::vector<std::pair<Manip, int> > > waypoint_contact_manips;
	std::vector<RPY_tf> contact_sequence;
	std::vector<Node> contact_plan; // each node differs from previous node by one contact pose
	std::string trajectory_filepath; // path to openrave file
	OpenRAVE::dReal plan_travel_dist; // dist between motion plan start and goal
	// planning_time, use chrono library?
};

#endif
