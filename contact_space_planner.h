#ifndef CONTACT_SPACE_PLANNER_H
#define CONTACT_SPACE_PLANNER_H

#include <openrave/plugin.h>
#include <vector>

class Node {
	// [x, y, z, roll, pitch, yaw] transforms
	std::vector<OpenRAVE::dReal> left_foot;
	std::vector<OpenRAVE::dReal> right_foot;
	std::vector<OpenRAVE::dReal> left_arm;
	std::vector<OpenRAVE::dReal> right_arm;

	Node* parent;
	double edge_cost;
	double accumulated_cost;
	double estimated_future_cost; // A* heuristic
	double dr_ability; // disturbance rejection ability

	// prev_move_manip; // enum
public:
	Node(std::vector<OpenRAVE::dReal> _left_foot, std::vector<OpenRAVE::dReal> _right_foot,
		 std::vector<OpenRAVE::dReal> _left_arm, std::vector<OpenRAVE::dReal> _right_arm,
		 Node* _parent, double _accumulated_cost, double _estimated_future_cost) : 
		 left_foot(_left_foot), right_foot(_right_foot), left_arm(_left_arm), right_arm(_right_arm),
		 parent(_parent), accumulated_cost(_accumulated_cost),
		 estimated_future_cost(_estimated_future_cost) {}

	// foot orientation projected to flat gruond
	OpenRAVE::dReal get_left_horizontal_yaw() const;
};

#endif
