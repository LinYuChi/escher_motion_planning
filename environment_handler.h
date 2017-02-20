#ifndef ENVIRONMENT_HANDLER_H
#define ENVIRONMENT_HANDLER_H

#include "structures.h"
#include "motion_plan.h"
#include "drawing.h"

#include <vector>
#include <memory>

class Environment_handler {
	std::vector<std::unique_ptr<Box>> boxes;
	std::vector<std::unique_ptr<Tri_mesh>> tri_meshes;
	OpenRAVE::EnvironmentBasePtr penv;
	std::vector<OpenRAVE::GraphHandlePtr > graphptrs;
	Drawing_handler dh;

	// double start_dist_to_boundary;
	// double goal_dist_to_boundary;
	// double goal_x;
	// double goal_y;
	// double goal_z;

	// get height of tallest surface encapsulating (x,y) point
	OpenRAVE::dReal highest_z(OpenRAVE::dReal x, OpenRAVE::dReal y);
	// returns true if over_boundary_point is a "fake" boundary (e.g. adjacent structure is same height)
	bool even_boundary_surface_height(const OpenRAVE::Vector& over_boundary_point, OpenRAVE::dReal z);
	void add_tri_mesh_cylinder(OpenRAVE::dReal z_range, OpenRAVE::dReal r);
public:
	Environment_handler(OpenRAVE::EnvironmentBasePtr _penv);
	void update_environment();
	// box world
	double dist_to_boundary(OpenRAVE::dReal x, OpenRAVE::dReal y, OpenRAVE::dReal z);

	// returns set of circular regions
	std::vector<OpenRAVE::Vector> sample_points(const Tri_mesh & tri_mesh, double resolution, double boundary_clearance) const;

	// Checks if there are obstacles inside radius r of sampled point above the surface
	bool point_free_space(const Tri_mesh & tri_mesh, OpenRAVE::dReal r,
						  OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> tf);

	std::vector<Contact_region> get_contact_regions() const;
};

#endif
