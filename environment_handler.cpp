#include "environment_handler.h"

#include <string>

using namespace OpenRAVE;

Environment_handler::Environment_handler(InterfaceType i_type, EnvironmentBasePtr _penv) : penv(_penv) {
	update_environment(i_type);
}

void Environment_handler::update_environment(InterfaceType i_type) {
	for(Structure* structure : structures) {
		penv->Remove(structure->get_kinbody());
	}

	Ground_box ground{RaveCreateKinBody(penv)};
	General_box wall{RaveCreateKinBody(penv), 0.2, 0.8, 2.4, 0, .9, .1, .1};
	structures.push_back(&ground);
	structures.push_back(&wall);


	for(Structure* structure : structures) {
		structure->get_kinbody()->InitFromBoxes(structure->get_parameter(), true);
		penv->Add(structure->get_kinbody());
		structure->get_kinbody()->SetTransform(structure->get_transform());
		structure->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(structure->get_color());
	}

}
