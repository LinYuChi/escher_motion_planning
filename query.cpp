#include "query.h"

#include <memory>
#include <cmath>
#include <cstdlib>
#include <cstdio>

using OpenRAVE::dReal; using OpenRAVE::RaveTransformMatrix;
using std::vector;
using std::unique_ptr;
using std::sqrt; using std::pow;
using std::min;
using std::rand;

// from utility
extern const dReal foot_height_c;
extern const dReal foot_width_c;
extern const dReal hand_height_c;
extern const dReal hand_width_c;

// returns overlapping circular planes in environment
vector<Contact_region> get_contact_regions(const Environment_handler & env_handler) {
	srand(time(NULL)); // initialize random seed (upgrade to c++11 rand?)
	dReal normal_contact_radius = 0.15;

	vector<Contact_region> ret_regions;

	for(const unique_ptr<Tri_mesh> & tri_mesh : env_handler.tri_meshes) {
		// do fragment checking

		RaveTransformMatrix<dReal> tri_tf = tri_mesh->get_transform();

		dReal boundary_clearance = sqrt(pow(hand_height_c / 2), + pow(hand_width_c / 2, 2)); // move euclidean distance functions to utility
		dReal density = min(tri_mesh->get_max_proj_x() - tri_mesh->get_min_proj_x() / 20.0,
							tri_mesh->get_max_proj_y() - tri_mesh->get_min_proj_y() / 20.0);

		vector<Vector> non_occupied_contact_samples = env_handler.sample_points(tri_mesh, density, boundary_clearance);

		while(non_occupied_contact_samples.size()) {
			int rand_sample_ind = rand() % non_occupied_contact_samples.size();
			Vector rand_contact = non_occupied_contact_samples[rand_sample_ind];

			Vector center = tri_tf * rand_contact;
			dReal r = center.z;
			// filter out based on radii

			ret_regions.push_back({center, tri_mesh->get_normal(), r});
		}
	}	
}