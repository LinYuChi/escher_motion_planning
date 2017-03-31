#include "motion_plan.h"

using std::vector;

using OpenRAVE::Vector;
using OpenRAVE::dReal;

int num_jacobian_iter_c = 100;

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

		for(int i = 0; i < num_jacobian_iter_c; ++i) {
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
