#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include "utility.h"
#include "contact_space_planner.h"

#include <vector>
#include <string>
#include <map>

class Drawing_handler;
struct Mp_optimization_vars;

struct Contact_region {
	OpenRAVE::Vector position; // region center
	OpenRAVE::Vector normal; // unit normal vector
	OpenRAVE::dReal radius;
	int group; // the robot can slide between contacts of the same group
};

enum Manip {
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
	OpenRAVE::dReal plan_travel_dist; // start-goal dist.
};

struct Motion_plan_cluster {
	int representative_index; // indexes into plan_indices vector
	std::vector<int> plan_indices; // each value in vector indexes into motion_plans index in Motion_plan_library
};

class Motion_plan_library {
	std::vector<Motion_plan> motion_plans;

	Motion_plan cluster_rep = {
		{},
		{
	        {{0, 0.15, 0}, Manip::L_foot},
	        {{0, -0.15, 0}, Manip::R_foot},
	        {{.15, 0.15, 0}, Manip::L_foot},
	        {{.3, -0.15, 0}, Manip::R_foot},
	        {{.45, 0.15, 0}, Manip::L_foot},
	        {{.6, -0.15, 0}, Manip::R_foot},
	        {{.75, 0.15, 0}, Manip::L_foot},
	        {{.9, -0.15, 0}, Manip::R_foot},
	        {{1.05, 0.15, 0}, Manip::L_foot},
	        {{1.2, -0.15, 0}, Manip::R_foot},
	        {{1.35, 0.15, 0}, Manip::L_foot},
	        {{1.5, -0.15, 0}, Manip::R_foot},
	        {{1.65, 0.15, 0}, Manip::L_foot},
	        {{1.8, -0.15, 0}, Manip::R_foot},
	        {{1.95, 0.15, 0}, Manip::L_foot},
	        {{2.1, -0.15, 0}, Manip::R_foot},
	        {{2.1, 0.15, 0}, Manip::L_foot},
		},
		{},
		"",
		0
	};

	// motion plan distance range -> motion plan clusters mapping
	// map keys represent range [key, key + distance_delta)
	std::map<OpenRAVE::dReal, std::vector<Motion_plan_cluster>> partitioned_clusters;
public:
	std::vector<Contact> transform_plan(const std::vector<Contact> & c_seq, const Mp_optimization_vars & mp_vars) const;

	// returns motion plans in order of most likely to fit environment
	// start/goal are packed as [x,y,z,radius]
	void query(Drawing_handler & dh, const std::vector<Contact_region> & contact_regions,
								   const OpenRAVE::Vector & start, const OpenRAVE::Vector & goal) const;
	void learn(std::vector<Contact> contact_sequence);
};

#endif
