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

	virtual OpenRAVE::Vector get_color() const = 0;

	virtual OpenRAVE::Transform get_transform() const = 0;
	virtual OpenRAVE::Transform get_inverse_transform() const = 0;
	virtual std::vector<OpenRAVE::AABB> get_parameter() const = 0;
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

	OpenRAVE::dReal ex;
	OpenRAVE::dReal ey;
	OpenRAVE::dReal ez;
public:
	Box(OpenRAVE::KinBodyPtr _kinbody,
		OpenRAVE::Vector _color, OpenRAVE::dReal _x, OpenRAVE::dReal _y, 
		OpenRAVE::dReal _z, OpenRAVE::dReal _theta, OpenRAVE::dReal _ex,
		OpenRAVE::dReal _ey, OpenRAVE::dReal _ez);

	double get_height() const { return z + ez; }
	OpenRAVE::Vector get_color() const { return color; }

	OpenRAVE::Transform get_transform() const;
	OpenRAVE::Transform get_inverse_transform() const;
	std::vector<OpenRAVE::AABB> get_parameter() const;
};

class Ground_box : public Box {
public:
	Ground_box(OpenRAVE::KinBodyPtr _kinbody);
};

class General_box : public Box {
public:
	General_box(OpenRAVE::KinBodyPtr _kinbody, OpenRAVE::dReal _x, OpenRAVE::dReal _y,
				OpenRAVE::dReal _z, OpenRAVE::dReal _theta, OpenRAVE::dReal _ex,
				OpenRAVE::dReal _ey, OpenRAVE::dReal _ez);
};

// class Trimesh_surface : public Structure {
// public:
// 	Trimesh_surface(std::vector<OpenRAVE::dReal> plane_parameters, );
// };

#endif
