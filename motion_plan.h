#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include "utility.h"
#include "contact_space_planner.h"

#include <vector>
#include <string>
#include <map>

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
	RPY_tf tf;
	Manip manip;
};

struct Motion_plan {
	std::vector<std::vector<std::pair<Manip, int> > > waypoint_contact_manips;
	std::vector<Contact> contact_sequence;
	std::vector<Node> contact_plan; // each node differs from previous node by one contact pose
	std::string trajectory_filepath; // path to openrave file
	OpenRAVE::dReal plan_travel_dist; // dist between motion plan start and goal
	// planning_time, use chrono library?
};

struct Motion_plan_cluster {
	int representative_index; // indexes into plan_indices vector
	std::vector<int> plan_indices; // each value in vector indexes into motion_plans index in Motion_plan_library
};

class Motion_plan_library {
	std::vector<Motion_plan> motion_plans;



	// motion plan distance range -> motion plan clusters mapping
	// map keys represent range [key, key + distance_delta)
	std::map<OpenRAVE::dReal, std::vector<Motion_plan_cluster>> partitioned_clusters;
public:
	std::vector<Contact> transform_plan(const std::vector<Contact> & c_seq, OpenRAVE::dReal x_mp,
										OpenRAVE::dReal y_mp, OpenRAVE::dReal z_mp, OpenRAVE::dReal theta_mp,
										const std::vector<OpenRAVE::dReal> & s);
	// append motion plan to library, updating clusters as necessary
	void append_plan(const Motion_plan & plan);

	// returns motion plans in order of most likely to fit environment
	std::vector<Motion_plan_cluster> query(const std::vector<Contact_region> & contact_regions,
								   const OpenRAVE::Vector & start, const OpenRAVE::Vector & goal) const;
	void learn(std::vector<Contact> contact_sequence);
};

#endif
