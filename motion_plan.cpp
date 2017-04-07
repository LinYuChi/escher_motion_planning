#include "motion_plan.h"

#include "gurobi_c++.h"
#include <iostream>

using std::vector;

using OpenRAVE::Vector;
using OpenRAVE::dReal;

int max_opt_iter_c = 100;

dReal motion_plan_bucket_size_c = .5; 

vector<Motion_plan_cluster> Motion_plan_library::query(const vector<Contact_region> & contact_regions,
											   const Vector & start, const Vector & goal) const {
	
	dReal motion_plan_xy_length = euclidean_distance_2d(start, goal);

	// round motion plan length down to nearest bucket size factor
	int motion_plan_bucket_key = int(motion_plan_xy_length / motion_plan_bucket_size_c) * motion_plan_bucket_size_c;

	auto it = partitioned_clusters.find(motion_plan_bucket_key);

	if(it == partitioned_clusters.end()) {
		return {};
	}

	for(const Motion_plan_cluster & cluster : it->second) {
		// "snap" cluster representative to environment

		const Motion_plan & cluster_rep = motion_plans[cluster.plan_indices[cluster.representative_index]];

		for(int i = 0; i < max_opt_iter_c; ++i) {
			// compute jacobian
			// J = ;
			for(const Contact_region & cr : contact_regions) {

			}

			// compute J+ (pseudo inverse)

			// compute change in contact poses

			// apply changes, move small step
		}
	}

}

void Motion_plan_library::learn(vector<Contact> contact_sequence) {
	assert(contact_sequence.size() > 3);

	Vector first_pose{contact_sequence.front().tf.x, contact_sequence.front().tf.y, 0};
	Vector second_pose{contact_sequence[1].tf.x, contact_sequence[1].tf.y, 0};
	Vector mp_pose = (first_pose + second_pose); mp_pose /= 2;

	Vector penultimate_pose{contact_sequence[contact_sequence.size() - 2].tf.x, contact_sequence[contact_sequence.size() - 2].tf.y, 0};
	Vector last_pose{contact_sequence.back().tf.x, contact_sequence.back().tf.y, 0};
	Vector mp_fin_pose = (penultimate_pose + last_pose); mp_fin_pose /= 2;
}
