#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <openrave/plugin.h>
#include <string>
#include <vector>

class Structure {
	static int num_structures;
	OpenRAVE::KinBodyPtr kinbody;
	int id;

	void set_name() { kinbody->SetName(std::to_string(id)); }
public:
	Structure(OpenRAVE::KinBodyPtr _kinbody) : kinbody(_kinbody), id(num_structures++) { set_name(); }
	OpenRAVE::KinBodyPtr get_kinbody() const { return kinbody; }

	virtual OpenRAVE::dReal get_height() const = 0;
	virtual OpenRAVE::Vector get_color() const = 0;

	virtual OpenRAVE::Transform get_transform() const = 0;
	virtual OpenRAVE::Transform get_inverse_transform() const = 0;
	virtual std::vector<OpenRAVE::AABB> get_parameter() const = 0;

	/*** BOX-SPECIFIC FNS BELOW ***/
	virtual bool within_x_bounds(const OpenRAVE::Vector & projected) const = 0;
	virtual bool within_y_bounds(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_pos_y_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_neg_y_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_pos_x_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_neg_x_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_quadrant_one_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_quadrant_two_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_quadrant_three_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::dReal dist_from_quadrant_four_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_pos_y_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_neg_y_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_pos_x_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_neg_x_bound(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_quadrant_one_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_quadrant_two_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_quadrant_three_corner(const OpenRAVE::Vector & projected) const = 0;
	virtual OpenRAVE::Vector over_quadrant_four_corner(const OpenRAVE::Vector & projected) const = 0;

};

class Box : public Structure {
	OpenRAVE::Vector color;

	// Transformation matrix
	OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> rot_mat;


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

	// euclidean distance
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

// class Trimesh_surface : public Structure {
// public:
// 	Trimesh_surface(std::vector<OpenRAVE::dReal> plane_parameters, );
// };

#endif
