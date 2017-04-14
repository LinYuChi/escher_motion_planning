#include "motion_plan.h"

#include "gurobi_c++.h"

#include <iostream>
#include <cmath>
#include <cassert>
#include <iostream>

using std::vector;

using OpenRAVE::Vector; using OpenRAVE::dReal;
using std::sin; using std::cos; using std::atan2;

int max_opt_iter_c = 100;

dReal motion_plan_bucket_size_c = .5; 

// can make this more efficient based on usage
// accounts for acyclic motion
vector<Contact> Motion_plan_library::transform_plan(const vector<Contact> & c_seq, dReal x_mp, dReal y_mp,
													dReal z_mp, dReal theta_mp, const vector<dReal> & s) {
	assert(s.size() == c_seq.size());
	
	vector<Contact> transformed_plan = c_seq;

	// first perform "stretches"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		// find "previous" footstep
		bool found_prev_pose = false;
		Contact &curr_pose = transformed_plan[i];
		Contact prev_pose;
		for(int j = i - 1; j >= 0; --j) {
			if(curr_pose.manip == transformed_plan[j].manip) {
				found_prev_pose = true;
				std::cout << " J: " << j << std::endl;
				prev_pose = transformed_plan[j];
				break;
			}
		}

		if(!found_prev_pose) continue; // cannot stretch pose 

		Vector step_vec{curr_pose.tf.x - prev_pose.tf.x, curr_pose.tf.y - prev_pose.tf.y, curr_pose.tf.z - prev_pose.tf.z};
		dReal step_len = sqrt(step_vec.lengthsqr3());
	 	dReal stretch_diff = s[i] - step_len;
		Vector step_diff =  (stretch_diff / step_len) * step_vec;
		std::cout << step_vec << " " << s[i] << " " << step_len << " " << step_diff << std::endl;
		
		// update all affected contact poses
		for(size_t k = i; k < transformed_plan.size(); ++k) {
			Contact &future_pose = transformed_plan[k];
			// std::cout << step_diff << std::endl;
			future_pose.tf.x += step_diff.x;
			future_pose.tf.y += step_diff.y;
			future_pose.tf.z += step_diff.z;
		}
	}

	// perform "rotations"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		Contact &curr_pose = transformed_plan[i];

		Vector pos{curr_pose.tf.x, curr_pose.tf.y, 0};
		dReal r = sqrt(pos.lengthsqr2());
		dReal curr_pose_phi = atan2(pos.y, pos.x);
		curr_pose.tf.x = r * cos(theta_mp + curr_pose_phi);
		curr_pose.tf.y = r * sin(theta_mp + curr_pose_phi);
	}

	// perform "translations"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		Contact &curr_pose = transformed_plan[i];

		curr_pose.tf.x += x_mp;
		curr_pose.tf.y += y_mp;
		curr_pose.tf.y += z_mp;
	}

	return transformed_plan;
}

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
