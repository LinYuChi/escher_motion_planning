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
	structures.push_back(&ground);

	for(Structure* structure : structures) {
		structure->get_kinbody()->InitFromBoxes(structure->get_parameter(), true);
		penv->Add(structure->get_kinbody());
		structure->get_kinbody()->SetTransform(structure->get_transform());
		structure->get_kinbody()->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor(structure->get_color());
	}

}
