#ifndef CONTACT_SPACE_PLANNER_H
#define CONTACT_SPACE_PLANNER_H

#include "utility.h"

#include <openrave/plugin.h>
#include <vector>

class Node {
	RPY_tf left_foot;
	RPY_tf right_foot;
	RPY_tf left_hand;
	RPY_tf right_hand;

	Node* parent;
	double edge_cost;
	double accumulated_cost;
	double estimated_future_cost; // A* heuristic
	double dr_ability; // disturbance rejection ability

	// prev_move_manip; // enum
public:
	Node(RPY_tf _left_foot, RPY_tf _right_foot, RPY_tf _left_hand,
		 RPY_tf _right_hand, Node* _parent, double _accumulated_cost,
		 double _estimated_future_cost) : left_foot(_left_foot), right_foot(_right_foot),
		 left_hand(_left_hand), right_hand(_right_hand), parent(_parent),
		 accumulated_cost(_accumulated_cost), estimated_future_cost(_estimated_future_cost) {}

	// foot orientation projected to flat gruond
	OpenRAVE::dReal get_left_horizontal_yaw() const;
	OpenRAVE::dReal get_right_horizontal_yaw() const;

	inline RPY_tf get_left_foot(){return left_foot;}
	inline RPY_tf get_right_foot(){return right_foot;}
	inline RPY_tf get_left_hand(){return left_hand;}
	inline RPY_tf get_right_hand(){return right_hand;}
	inline Node*  get_parent(){return parent;}
};

#endif
