#include "motion_plan.h"

#include "gurobi_c++.h"

#include <iostream>
#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>

using std::vector;
using OpenRAVE::Vector; using OpenRAVE::dReal;
using std::sin; using std::cos; using std::atan2;
using std::numeric_limits;

int max_opt_iter_c = 100;
dReal attractive_range_c = 0.3;

dReal motion_plan_bucket_size_c = .5; 

bool is_valid_manip(Manip manip) {
	return manip == Manip::L_foot || manip == Manip::R_foot || manip == Manip::L_hand || manip == Manip::R_hand;
}

Manip get_alternate_manip(Manip manip) {
	assert(is_valid_manip(manip));

	if(manip == Manip::L_foot) return Manip::R_foot;
	if(manip == Manip::R_foot) return Manip::L_foot;
	if(manip == Manip::L_hand) return Manip::R_hand;
	if(manip == Manip::R_hand) return Manip::L_hand;
}

// i is index of current contact pose
const Contact & get_pivot_contact(const vector<Contact> c_seq, int i) {
	Contact curr_pose = c_seq[i];
	Manip pivot_manip = get_alternate_manip(curr_pose.manip);

	for(int j = i - 1; j >= 0; --j) {
		if(c_seq[j].manip == pivot_manip) {
			return c_seq[j];
		}
	}

	throw std::runtime_error{"Error finding pivot contact"};
}

// distance between a contact pose and a contact region
dReal contact_pose_to_region_distance(const Contact_region & cr, const Contact & pose) {
	return 0.;
}

// distance between a contact pose and a contact region
dReal contact_pose_to_region_orientation(const Contact_region & cr, const Contact & pose) {
	return 0.;
}


Vector get_attractive_vector(const vector<Contact_region> & contact_regions, Contact & contact) {
	assert(contact_regions.size());

	Vector attractive_vec;
	double nearest_contact_dist = numeric_limits<double>::max();
	for(const Contact_region & cr : contact_regions) {
		// distance between contact and contact region
	}

	return attractive_vec;
}

// can make this more efficient based on usage
// accounts for acyclic motion
vector<Contact> Motion_plan_library::transform_plan(const vector<Contact> & c_seq, dReal x_mp, dReal y_mp,
													dReal z_mp, dReal theta_mp, const vector<Vector> & s) {
	assert(s.size() == c_seq.size());
	
	vector<Contact> transformed_plan = c_seq;

	// first perform "stretches"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		// find "pivot" footstep
		bool found_pivot_pose = false;
		Contact &curr_pose = transformed_plan[i];

		try {
			const Contact & pivot_pose = get_pivot_contact(transformed_plan, i);

			Vector step_vec{curr_pose.tf.x - pivot_pose.tf.x, curr_pose.tf.y - pivot_pose.tf.y, curr_pose.tf.z - pivot_pose.tf.z};
			Vector step_diff =  s[i] - step_vec;

			// update all affected contact poses
			for(size_t k = i; k < transformed_plan.size(); ++k) {
				Contact &future_pose = transformed_plan[k];
				future_pose.tf.x += step_diff.x;
				future_pose.tf.y += step_diff.y;
				future_pose.tf.z += step_diff.z;
			}	
		} catch(...) {}
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
		// snap cluster representative to environment

		// use safer c++ map semantics
		const Motion_plan & cluster_rep = motion_plans[cluster.plan_indices[cluster.representative_index]];
		const vector<Contact> & local_c_seq = cluster_rep.contact_sequence;
		assert(local_c_seq.size() > 2);

		vector<Contact> global_c_seq = cluster_rep.contact_sequence;
		
		dReal x_mp = 0;
		dReal y_mp = 0;
		dReal z_mp = 0;
		dReal theta_mp = 0;

		vector<dReal> s_x(local_c_seq.size());
		vector<dReal> s_y(local_c_seq.size());
		vector<dReal> s_z(local_c_seq.size());

		// initialize delta stretch vectors
		// cannot stretch first contact pose
		s_x[0] = 0;
		s_y[0] = 0;
		s_z[0] = 0;
		for(int i = 1; i < local_c_seq.size(); ++i) {
			const Contact & curr_contact = local_c_seq[i];

			try {
				const Contact & pivot_contact = get_pivot_contact(local_c_seq, i);
				s_x[i] = curr_contact.tf.x - pivot_contact.tf.x;
				s_y[i] = curr_contact.tf.y - pivot_contact.tf.y;
				s_z[i] = curr_contact.tf.z - pivot_contact.tf.z;
			} catch(...) {}

		}

		for(int i = 0; i < max_opt_iter_c; ++i) {
			
			// compute partial derivatives for each contact pose
			vector<dReal> theta_mp_x_pd(global_c_seq.size(), 1);
			vector<dReal> theta_mp_y_pd(global_c_seq.size(), 1);

			for(int j = 0; j < global_c_seq.size(); ++j) {
				Contact global_contact = global_c_seq[j];

				dReal r = euclidean_distance_2d({x_mp, y_mp, 0}, {global_contact.tf.x, global_contact.tf.y, 0});
				dReal theta = atan2(global_contact.tf.y - y_mp, global_contact.tf.x - x_mp);

				// x = rcos(theta) ==> -rsin(theta)
				theta_mp_x_pd[j] = -r*sin(theta);

				// y = rsin(theta) ==> rcos(theta)
				theta_mp_x_pd[j] = -r*sin(theta);
			}

			// calculate attractive vectors for each contact pose
			vector<Vector> attractive_vecs(global_c_seq.size());
			for(int i = 0; i < global_c_seq.size(); ++i) {

				attractive_vecs[i] = get_attractive_vector(contact_regions, global_c_seq[i]);
			}

			// optimize motion plan variables

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
