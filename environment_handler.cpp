#include "environment_handler.h"

#include <string>

using namespace OpenRAVE;

Environment_handler::Environment_handler(InterfaceType i_type, EnvironmentBasePtr penv_) : penv(penv_) {
	update_environment(i_type);
}

void Environment_handler::update_environment(InterfaceType i_type) {
	// for(Box & structure : structures) {
	// 	penv->Remove(structure.get_kinbody());
	// }

	// Box b1{0};
	structures.push_back({0});
	// goal_x = 4;
	// goal_y = 0;

	// for(Box & structure : structures) {
	// 	structure.get_kinbody()->SetName(std::to_string(structure.get_id()));
	// 	std::vector<AABB> box_parameter;
	// 	box_parameter.push_back({RaveVector<dReal>(0, 0, 0), RaveVector<dReal>(5, 5, 5)});
	// 	structure.get_kinbody()->InitFromBoxes(box_parameter, true);
	// 	penv->AddKinBody(structure.get_kinbody());
	// 	structure.get_kinbody()->SetTransform(structure.get_transform());
	// }

}
