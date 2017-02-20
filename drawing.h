#ifndef DRAWING_H
#define DRAWING_H

#include "contact_space_planner.h"
#include "utility.h"
#include "structures.h"

#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>

using namespace OpenRAVE;

class Drawing_handler{
	std::vector<boost::shared_ptr<void> > graphptrs;
	EnvironmentBasePtr penv;
	std::vector< RaveVector<dReal> > foot_corners;
	std::vector< RaveVector<dReal> > hand_corners;
public:
	Drawing_handler(EnvironmentBasePtr _penv);
	void clear_handler();
	void DrawBodyPath(Node* current); // Draw the upperbody path in thr door planning, postpone this implementation.(DrawPaths)
	void DrawGridPath(); // Draw the Dijkstra grid path, postpone implementation.
	void DrawContactPath(Node* current); // Draw the contact path given the final state(DrawStances)
	void DrawContacts(Node* node); // Draw the contacts of one node(DrawStance)
	// void DrawContact(enum contact_type,contact_transform); // Draw one contact.(DrawContact)
	void DrawLocation(RaveTransformMatrix<dReal> transform, RaveVector<float> color); // Draw a point at the location(DrawLocation)
	void DrawLocation(RaveVector<dReal> location, RaveVector<float> color); // Draw a point at the location(DrawLocation)
	void DrawTransform(RaveTransformMatrix<dReal> transform); // Draw the transform in 3 axes(DrawOrientation)
	void DrawManipulatorPoses(RobotBasePtr robot); // Draw the manipulator poses given robot object(DrawManipulatorPoses)
	void DrawGoalRegion(RaveTransformMatrix<dReal> transform, double radius); // Draw the region with given transform and radius.(DrawRegion)
	void DrawRegion(RaveVector<dReal> center, RaveVector<dReal> normal, double radius, float line_width); // Draw the region with given center, normal and radius.(DrawContactRegion)
	void DrawLineSegment(RaveVector<dReal> from_vec, RaveVector<dReal> to_vec); // Draw a line segment given two ends(DrawLineStrips)
	void DrawSurface(Tri_mesh trimesh); // Draw the trimesh surface.(DrawSurface)
	void DrawObjectPath(Node* current); // Draw the manipulated object path, postpone implementation.(DrawObjectPath)
	
};

#endif
