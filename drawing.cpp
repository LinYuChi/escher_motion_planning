#include "drawing.h"
#include "contact_space_planner.h"
#include "utility.h"
#include "structures.h"

#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
#include <cmath>
#include <stdlib.h>

using namespace OpenRAVE;

Drawing_handler::Drawing_handler(EnvironmentBasePtr _penv):penv(_penv)
{
	foot_corners.resize(4);
	foot_corners[0] = RaveVector<dReal>(foot_height_c/2,foot_width_c/2,0.01);
	foot_corners[1] = RaveVector<dReal>(-foot_height_c/2,foot_width_c/2,0.01);
	foot_corners[2] = RaveVector<dReal>(-foot_height_c/2,-foot_width_c/2,0.01);
	foot_corners[3] = RaveVector<dReal>(foot_height_c/2,-foot_width_c/2,0.01);

	hand_corners.resize(4);
	hand_corners[0] = RaveVector<dReal>(-0.01,hand_height_c/2,hand_width_c/2);
	hand_corners[1] = RaveVector<dReal>(-0.01,-hand_height_c/2,hand_width_c/2);
	hand_corners[2] = RaveVector<dReal>(-0.01,-hand_height_c/2,-hand_width_c/2);
	hand_corners[3] = RaveVector<dReal>(-0.01,hand_height_c/2,-hand_width_c/2);
}

void Drawing_handler::clear_handler()
{
	graphptrs.clear();
}

void Drawing_handler::DrawBodyPath(Node* current) // Draw the upperbody path in thr door planning, postpone this implementation.(DrawPaths)
{

}

void Drawing_handler::DrawGridPath() // Draw the Dijkstra grid path, postpone implementation.
{

}

void Drawing_handler::DrawContactPath(Node* current) // Draw the contact path given the final state(DrawStances)
{
    Node* c = current;
    while(c != NULL)
    {
    	DrawContacts(c);    	
        c = c->get_parent();
    }
}
void Drawing_handler::DrawContacts(Node* node) // Draw the contacts of one node(DrawStance)
{
	// draw left foot pose
    RaveTransformMatrix<dReal> left_foot_transform = get_SO3(node->get_left_foot());
    std::vector< RaveVector<dReal> > transformed_left_foot_corners(4);

    for(unsigned int i = 0; i < transformed_left_foot_corners.size(); i++)
    {
    	transformed_left_foot_corners[i] = left_foot_transform*foot_corners[i];
    }

    float left_foot_corners0_x_float = (float)transformed_left_foot_corners[0].x;
    graphptrs.push_back(penv->drawlinestrip(&(left_foot_corners0_x_float ),transformed_left_foot_corners.size(),sizeof(transformed_left_foot_corners[0]),5,RaveVector<float>(1,0,0,0)));

    // draw right foot pose
    RaveTransformMatrix<dReal> right_foot_transform = get_SO3(node->get_right_foot());
    std::vector< RaveVector<dReal> > transformed_right_foot_corners(4);

    for(unsigned int i = 0; i < transformed_right_foot_corners.size(); i++)
    {
    	transformed_right_foot_corners[i] = right_foot_transform*foot_corners[i];
    }

    float right_foot_corners0_x_float = (float)transformed_right_foot_corners[0].x;
    graphptrs.push_back(penv->drawlinestrip(&(right_foot_corners0_x_float),transformed_right_foot_corners.size(),sizeof(transformed_right_foot_corners[0]),5,RaveVector<float>(0,1,0,0)));

    // draw left hand pose
    if(node->get_left_hand().x != -99.0)
    {
        RaveTransformMatrix<dReal> left_hand_transform = get_SO3(node->get_left_hand());
        std::vector< RaveVector<dReal> > transformed_left_hand_corners(4);

        for(unsigned int i = 0; i < transformed_left_hand_corners.size(); i++)
        {
        	transformed_left_hand_corners[i] = left_hand_transform*hand_corners[i];
        }

        float left_hand_corners0_x_float = (float)transformed_left_hand_corners[0].x;
        graphptrs.push_back(penv->drawlinestrip(&(left_hand_corners0_x_float),transformed_left_hand_corners.size(),sizeof(transformed_left_hand_corners[0]),5,RaveVector<float>(0,0,1,0)));
    }

    // draw right hand pose
    if(node->get_right_hand().x != -99.0)
    {
        RaveTransformMatrix<dReal> right_hand_transform = get_SO3(node->get_right_hand());
        std::vector< RaveVector<dReal> > transformed_right_hand_corners(4);

        for(unsigned int i = 0; i < transformed_right_hand_corners.size(); i++)
        {
        	transformed_right_hand_corners[i] = right_hand_transform*hand_corners[i];
        }

        float right_hand_corners0_x_float = (float)transformed_right_hand_corners[0].x;
        graphptrs.push_back(penv->drawlinestrip(&(right_hand_corners0_x_float),transformed_right_hand_corners.size(),sizeof(transformed_right_hand_corners[0]),5,RaveVector<float>(1,1,0,0)));
    }
}
// void Drawing_handler::DrawContact(enum contact_type,contact_transform); // Draw one contact.(DrawContact)

void Drawing_handler::DrawLocation(RaveTransformMatrix<dReal> transform, RaveVector<float> color) // Draw a point at the location(DrawLocation)
{
	float trans_x_float = (float)transform.trans[0];
	graphptrs.push_back(penv->plot3(&(trans_x_float), 1, 0, 15, color));
}

void Drawing_handler::DrawLocation(RaveVector<dReal> location, RaveVector<float> color) // Draw a point at the location(DrawLocation)
{
	float trans_x_float = (float)location[0];
	graphptrs.push_back(penv->plot3(&(trans_x_float), 1, 0, 15, color));
}

void Drawing_handler::DrawTransform(RaveTransformMatrix<dReal> transform) // Draw the transform in 3 axes(DrawOrientation)
{
	RaveVector<dReal> from_vec = transform.trans;
	RaveVector<dReal> to_vec_x = from_vec + 0.2 * RaveVector<dReal>(transform.m[0],transform.m[4],transform.m[8]);
	RaveVector<dReal> to_vec_y = from_vec + 0.2 * RaveVector<dReal>(transform.m[1],transform.m[5],transform.m[9]);
	RaveVector<dReal> to_vec_z = from_vec + 0.2 * RaveVector<dReal>(transform.m[2],transform.m[6],transform.m[10]);

	graphptrs.push_back(penv->drawarrow(from_vec, to_vec_x, 0.005, RaveVector<float>(1, 0, 0)));
    graphptrs.push_back(penv->drawarrow(from_vec, to_vec_y, 0.005, RaveVector<float>(0, 1, 0)));
    graphptrs.push_back(penv->drawarrow(from_vec, to_vec_z, 0.005, RaveVector<float>(0, 0, 1)));
}

void Drawing_handler::DrawManipulatorPoses(RobotBasePtr robot) // Draw the manipulator poses given robot object(DrawManipulatorPoses)
{
	std::vector< boost::shared_ptr<RobotBase::Manipulator> > manipulators = robot->GetManipulators();

	for(unsigned int i = 0; i < manipulators.size(); i++)
	{
		boost::shared_ptr<RobotBase::Manipulator> manipulator = manipulators[i];
		DrawTransform(RaveTransformMatrix<dReal>(manipulator->GetTransform()));
	}
}

void Drawing_handler::DrawGoalRegion(RaveTransformMatrix<dReal> transform, double radius) // Draw the region with given transform and radius.(DrawRegion)
{
	RaveVector<dReal> center = transform.trans;
	RaveVector<dReal> x_vector = RaveVector<dReal>(transform.m[0],transform.m[4],transform.m[8]);
	RaveVector<dReal> y_vector = RaveVector<dReal>(transform.m[1],transform.m[5],transform.m[9]);

	DrawRegion(center, RaveVector<dReal>(0,0,1), radius, 5.0); // Draw the region with given center, normal and radius.(DrawContactRegion)

	std::vector< RaveVector<dReal> > arrow_points(5);

	arrow_points[0] = center - radius*2.0/3.0 * x_vector;
	arrow_points[1] = center + radius*2.0/3.0 * x_vector;
	arrow_points[2] = center + radius/2.0 * y_vector;
	arrow_points[3] = center + radius*2.0/3.0 * x_vector;
	arrow_points[4] = center - radius/2.0 * y_vector;

	float arrow_point_x_float = (float)arrow_points[0].x;

	graphptrs.push_back(penv->drawlinestrip(&(arrow_point_x_float),arrow_points.size(),sizeof(arrow_points[0]),5.0,RaveVector<float>(0,0,0,0)));
}

void Drawing_handler::DrawRegion(RaveVector<dReal> center, RaveVector<dReal> normal, double radius, float line_width) // Draw the region with given center, normal and radius.(DrawContactRegion)
{
	RaveVector<dReal> x_vector;
	if(normal.x == 0 && normal.y == 0)
	{
        x_vector = RaveVector<dReal>(1,0,0);
	}
    else
    {
    	x_vector = RaveVector<dReal>(normal.y,-normal.x,0);
        x_vector = x_vector.normalize3();
    }
        
    RaveVector<dReal> y_vector = normal.cross(x_vector);

    std::vector< RaveVector<dReal> > region_boundary_points;
    region_boundary_points.resize(37);
    RaveVector<dReal> region_boundary_point;

	for(unsigned int i = 0; i < 37; i++)
	{
		region_boundary_point = center + std::cos(i*10*(M_PI / 180))*radius*x_vector + std::sin(i*10*(M_PI / 180))*radius*y_vector;
		region_boundary_points[i] = region_boundary_point;
	}

	float region_boundary_point_x_float = (float)region_boundary_points[0].x;
	graphptrs.push_back(penv->drawlinestrip(&(region_boundary_point_x_float),region_boundary_points.size(),sizeof(region_boundary_points[0]),line_width,RaveVector<float>(0,0,0,0)));

	graphptrs.push_back(penv->drawarrow(center, center + 0.1 * normal, 0.005, RaveVector<float>(1,0,0)));
}

void Drawing_handler::DrawLineSegment(RaveVector<dReal> from_vec, RaveVector<dReal> to_vec) // Draw a line segment given two ends(DrawLineStrips)
{
	std::vector< RaveVector<dReal> > line_endpoints = {from_vec,to_vec};
	float line_endpoint_x_float = (float)line_endpoints[0].x;
	graphptrs.push_back(penv->drawlinestrip(&(line_endpoint_x_float),line_endpoints.size(),sizeof(line_endpoints[0]),3.0,RaveVector<float>(0,0,0,0)));
}

void Drawing_handler::DrawSurface(Tri_mesh trimesh) // Draw the trimesh surface.(DrawSurface)
{
	float r = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);
	float g = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);
	float b = static_cast<float> (std::rand()) / static_cast<float> (RAND_MAX);

	float total_rgb = std::sqrt(r*r+g*g+b*b);
	r = r/total_rgb;
	g = g/total_rgb;
	b = b/total_rgb;

	// DrawOrientation(trimesh.transform_matrix);

	// for boundary in struct.boundaries:

	// 	boundaries_point = np.zeros((2,3),dtype=float)
	// 	boundaries_point[0:1,:] = np.atleast_2d(np.array(struct.vertices[boundary[0]]))
	// 	boundaries_point[1:2,:] = np.atleast_2d(np.array(struct.vertices[boundary[1]]))

	// 	draw_handles.append(env.drawlinestrip(points = boundaries_point,linewidth = 5.0,colors = np.array((r,g,b))))

	// graphptrs.push_back(penv->drawtrimesh())
	// _penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,RaveVector<float>(1,0.5,0.5,1))
	// draw_handles.append(env.drawtrimesh(trimesh.kinbody->GetLinks()[0].GetCollisionData().vertices,struct.kinbody.GetLinks()[0].GetCollisionData().indices,RaveVector<float>(r,g,b,1.0)))

}

void Drawing_handler::DrawObjectPath(Node* current) // Draw the manipulated object path, postpone implementation.(DrawObjectPath)
{

}