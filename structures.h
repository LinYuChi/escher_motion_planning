#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <openrave/plugin.h>
#include <string>
#include <vector>
#include <utility>
#include <map>

class Structure {
	static int num_structures;
	OpenRAVE::KinBodyPtr kinbody;
	int id;

	void set_name() { kinbody->SetName(std::to_string(id)); }
public:
	Structure(OpenRAVE::KinBodyPtr _kinbody) : kinbody(_kinbody), id(num_structures++) { set_name(); }
	OpenRAVE::KinBodyPtr get_kinbody() const { return kinbody; }
};

class Box : public Structure {
	OpenRAVE::Vector color;

	// Transformation matrix
	OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> transform_matrix;

	/// Box position
	OpenRAVE::dReal x;
	OpenRAVE::dReal y;
	OpenRAVE::dReal z;

	// Box orientation
	OpenRAVE::dReal theta;

	// Box thickness
	OpenRAVE::dReal ex;
	OpenRAVE::dReal ey;
	OpenRAVE::dReal ez;

	// given leg lengths, get hypotenuse length
	OpenRAVE::dReal hypotenuse(OpenRAVE::dReal q, OpenRAVE::dReal p) const;	
public:
	Box(OpenRAVE::KinBodyPtr _kinbody,
		OpenRAVE::Vector _color, OpenRAVE::dReal _x, OpenRAVE::dReal _y, 
		OpenRAVE::dReal _z, OpenRAVE::dReal _theta, OpenRAVE::dReal _ex,
		OpenRAVE::dReal _ey, OpenRAVE::dReal _ez);

	OpenRAVE::dReal get_height() const { return z + ez; }
	OpenRAVE::Vector get_color() const { return color; }

	OpenRAVE::Transform get_transform() const;
	OpenRAVE::Transform get_inverse_transform() const;
	std::vector<OpenRAVE::AABB> get_parameter() const;

	bool within_x_bounds(const OpenRAVE::Vector & projected) const;
	bool within_y_bounds(const OpenRAVE::Vector & projected) const;

	OpenRAVE::dReal dist_from_pos_y_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_neg_y_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_pos_x_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_neg_x_bound(const OpenRAVE::Vector & projected) const;

	OpenRAVE::dReal dist_from_quadrant_one_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_quadrant_two_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_quadrant_three_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::dReal dist_from_quadrant_four_corner(const OpenRAVE::Vector & projected) const;

	OpenRAVE::Vector over_pos_y_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_neg_y_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_pos_x_bound(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_neg_x_bound(const OpenRAVE::Vector & projected) const;

	OpenRAVE::Vector over_quadrant_one_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_quadrant_two_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_quadrant_three_corner(const OpenRAVE::Vector & projected) const;
	OpenRAVE::Vector over_quadrant_four_corner(const OpenRAVE::Vector & projected) const;
};

class Ground_box : public Box {
public:
	Ground_box(OpenRAVE::KinBodyPtr _kinbody);
};

class General_box : public Box {
public:
	General_box(OpenRAVE::KinBodyPtr _kinbody, OpenRAVE::dReal _x, OpenRAVE::dReal _y,
				OpenRAVE::dReal height, OpenRAVE::dReal _theta, OpenRAVE::dReal _ex,
				OpenRAVE::dReal _ey);
};

class Tri_mesh : public Structure {
	std::map<OpenRAVE::dReal, std::vector<OpenRAVE::dReal> > boundaries; // discretized mapping from x coord to y bounds
	OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> transform_matrix;
	OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> inverse_transform_matrix;
	std::vector<std::pair<int, int> > edges; // contains indices into vertices vector
	std::vector<OpenRAVE::Vector> vertices;
	std::vector<OpenRAVE::Vector> proj_vertices; // last vertex is same as first vertex, i.e. "closed loop"

	OpenRAVE::dReal min_proj_x;
	OpenRAVE::dReal max_proj_x;

	OpenRAVE::dReal min_proj_y;
	OpenRAVE::dReal max_proj_y;

	// nx * xo + ny * yo + nz * zo + c = 0
	OpenRAVE::dReal nx;
	OpenRAVE::dReal ny;
	OpenRAVE::dReal nz;
	OpenRAVE::dReal c;

	// center coordinates
	OpenRAVE::dReal xo;
	OpenRAVE::dReal yo;
	OpenRAVE::dReal zo;

	OpenRAVE::dReal circumradius;

	// euclidean distance btwn two points in a 2D coordinate system
	OpenRAVE::dReal euclidean_distance_2d(const OpenRAVE::Vector & q, const OpenRAVE::Vector & p) const;
	// euclidean distance btwn two points in a 3D coordinate system
	OpenRAVE::dReal euclidean_distance_3d(const OpenRAVE::Vector & q, const OpenRAVE::Vector & p) const;
	void update_center();
	void update_proj_vertices();
	void update_approx_boundary();
public:
	Tri_mesh(OpenRAVE::KinBodyPtr _kinbody, OpenRAVE::Vector plane_parameters,
			 std::vector<std::pair<int, int> > _edges,
			 std::vector<OpenRAVE::Vector> _vertices);
	void transform_data(OpenRAVE::Transform transform);
	OpenRAVE::Vector get_normal() const;
	OpenRAVE::Vector get_center() const;
	OpenRAVE::Transform get_transform() const;
	OpenRAVE::Transform get_inverse_transform() const;

	// returns 2D point projected in plane frame. This assumes the "ray" is the surface normal.
	OpenRAVE::Vector projection_plane_frame(const OpenRAVE::Vector & point) const;
	// returns 3D point projected in plane frame. Ray is a 3D unit vector.
	OpenRAVE::Vector projection_global_frame(const OpenRAVE::Vector & point, const OpenRAVE::Vector & ray) const;
	bool inside_polygon(const OpenRAVE::Vector & point) const;
	bool inside_polygon_plane_frame(const OpenRAVE::Vector & projected_point) const;

	// polygon must be convex. contacts are rectangles.
	// TODO: split this fn up instead of switching on type
	bool contact_inside_polygon(const OpenRAVE::Transform & tf, const std::string & contact_type) const;

	// roll is the rotation of the contact about ray
	OpenRAVE::Transform projection(const OpenRAVE::Vector & origin, const OpenRAVE::Vector & ray, OpenRAVE::dReal roll,
								   const std::string & end_effector_type, bool valid_contact) const;

	// extract binary checking into another fn
	OpenRAVE::dReal dist_to_boundary(OpenRAVE::Vector point, OpenRAVE::dReal search_radius = 999
									 /*, bool binary_checking = false*/) const;

};

#endif
