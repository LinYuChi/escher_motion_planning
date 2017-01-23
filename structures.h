#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <openrave/plugin.h>
#include <string>
#include <vector>

class Structure {
	OpenRAVE::KinBodyPtr kinbody;
	int id;
public:
	Structure(int _id, OpenRAVE::KinBodyPtr _kinbody = boost::shared_ptr<OpenRAVE::KinBody>()) : id(_id), kinbody(_kinbody) {}
};

class Box : public Structure {
	OpenRAVE::Vector color;
	OpenRAVE::dReal x;
	OpenRAVE::dReal y;
	OpenRAVE::dReal z;
	OpenRAVE::dReal theta;
	OpenRAVE::dReal ex;
	OpenRAVE::dReal ey;
	OpenRAVE::dReal ez;
public:
	Box(int _id, OpenRAVE::dReal _z = 0);
	double get_height() const { return z + ez; }
	OpenRAVE::Transform get_transform() const;
	OpenRAVE::Transform get_inverse_transform() const;
};

// class Trimesh_surface : public Structure {
// public:
// 	Trimesh_surface(int _id, std::vector<OpenRAVE::dReal> plane_parameters, );
// };

#endif
