#include "motion_plan.h"
#include "drawing.h"

#include "gurobi_c++.h"

#include <iostream>
#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>

using std::cout; using std::endl;
using std::vector;
using OpenRAVE::Vector; using OpenRAVE::dReal;
using std::sin; using std::cos; using std::atan2;
using std::numeric_limits; using std::max;

const int max_opt_iter_c = 100;
const dReal attractive_range_c = 0.3;
const dReal max_delta_size_c = 0.002;
const dReal min_delta_size_c = 0.0005;

const dReal motion_plan_bucket_size_c = .5; 

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

// TODO: distance between a contact pose and a contact region
// dReal contact_pose_to_region_distance(const Contact_region & cr, const Contact & pose) {
	// return 0.;
// }

// TODO: distance between a contact pose and a contact region
// dReal contact_pose_to_region_orientation(const Contact_region & cr, const Contact & pose) {
	// return 0.;
// }


Vector get_attractive_vector(const vector<Contact_region> & contact_regions, Contact & contact) {
	assert(contact_regions.size());

	Vector contact_pos {contact.tf.x, contact.tf.y, contact.tf.z};

	Vector attractive_vec;
	double nearest_contact_dist = numeric_limits<double>::max();
	for(const Contact_region & cr : contact_regions) {
		// distance between contact and contact region
		dReal xy_dist = max(0., euclidean_distance_2d(cr.position, contact_pos) - cr.radius);
		dReal contact_to_region_dist = sqrt(pow(xy_dist, 2) + pow(cr.position.z - contact.tf.z, 2));
		if(contact_to_region_dist < nearest_contact_dist) {
			nearest_contact_dist = contact_to_region_dist;

			if(!xy_dist) {
				attractive_vec = {0, 0, cr.position.z - contact_pos.z};
			} else {
				attractive_vec = cr.position - contact_pos;
			}
		}
	}

	return attractive_vec;
}

struct Mp_optimization_vars {
	dReal x_mp;
	dReal y_mp;
	dReal z_mp;
	dReal theta_mp;
	vector<dReal> s_x;
	vector<dReal> s_y;
	vector<dReal> s_z;
};

// can make this more efficient based on usage
// accounts for acyclic motion
vector<Contact> Motion_plan_library::transform_plan(const vector<Contact> & c_seq, const Mp_optimization_vars & mp_vars) const {
	assert(mp_vars.s_x.size() == c_seq.size());
	
	vector<Vector> s(c_seq.size());
	for(int i = 0; i < c_seq.size(); ++i) {
		s[i] = {mp_vars.s_x[i], mp_vars.s_y[i], mp_vars.s_z[i]};
	}

	vector<Contact> transformed_plan = c_seq;

	bool seen_left_foot = false;
	bool seen_right_foot = false;
	bool seen_left_hand = false;
	bool seen_right_hand = false;

	// first perform "stretches"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		// find "pivot" footstep
		const Contact &curr_pose = transformed_plan[i];

		// cannot stretch/compress first instance of a manipulator
		if(!seen_left_foot && curr_pose.manip == Manip::L_foot) {
			seen_left_foot = true;
			continue;
		} else if(!seen_right_foot && curr_pose.manip == Manip::R_foot) {
			seen_right_foot = true;
			continue;
		} else if(!seen_left_hand && curr_pose.manip == Manip::L_hand) {
			seen_left_hand = true;
			continue;
		} else if(!seen_right_hand && curr_pose.manip == Manip::R_hand) {
			seen_right_hand = true;
			continue;
		}

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
		curr_pose.tf.x = r * cos(mp_vars.theta_mp + curr_pose_phi);
		curr_pose.tf.y = r * sin(mp_vars.theta_mp + curr_pose_phi);
		curr_pose.tf.yaw = c_seq[i].tf.yaw + mp_vars.theta_mp;
	}

	// perform "translations"
	for(size_t i = 0; i < transformed_plan.size(); ++i) {
		Contact &curr_pose = transformed_plan[i];

		curr_pose.tf.x += mp_vars.x_mp;
		curr_pose.tf.y += mp_vars.y_mp;
		curr_pose.tf.z += mp_vars.z_mp;
	}

	return transformed_plan;
}

Mp_optimization_vars optimize_plan(const vector<Contact> & global_c_seq, const Mp_optimization_vars & mp_vars,
								   const vector<Vector> & attractive_vecs, const vector<dReal> & theta_mp_x_pd,
								   const vector<dReal> & theta_mp_y_pd, const Vector & start, const Vector & goal) {

	try {
	    // initialize gurobi
	    GRBEnv env = GRBEnv();
	    GRBModel model = GRBModel(env);
	    model.set("OutputFlag", "0"); // silence gurobi standard output

	    // add motion plan variables to model
	    GRBVar delta_x_mp = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS, "delta_x_mp"); // replace +-100 with inifinity
	    GRBVar delta_y_mp = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS, "delta_y_mp");
	    GRBVar delta_z_mp = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS, "delta_z_mp");
	    GRBVar delta_theta_mp = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS, "delta_theta_mp");
	    vector<GRBVar> delta_s_x(global_c_seq.size());
	    vector<GRBVar> delta_s_y(global_c_seq.size());
	    vector<GRBVar> delta_s_z(global_c_seq.size());

	    for(int i = 0; i < global_c_seq.size(); ++i) {

	    	if(i == 0 || i == 1) { // generalize to acyclic motion with hand placements
				continue;
	    	}

	    	delta_s_x[i] = model.addVar(.1 - mp_vars.s_x[i], .4 - mp_vars.s_x[i], 0.0, GRB_CONTINUOUS, "delta_s_x_of_" + i);
	    	if(global_c_seq[i].manip == Manip::L_foot) {
	    		delta_s_y[i] = model.addVar(0.2 - mp_vars.s_y[i], 0.4 - mp_vars.s_y[i], 0.0, GRB_CONTINUOUS, "delta_s_y_of_" + i);
	    	} else if(global_c_seq[i].manip == Manip::R_foot) {
	    		delta_s_y[i] = model.addVar(-0.4 - mp_vars.s_y[i], -0.2 - mp_vars.s_y[i], 0.0, GRB_CONTINUOUS, "delta_s_y_of_" + i);
	    	}
	    	delta_s_z[i] = model.addVar(-0.6 - mp_vars.s_z[i], 0.6 - mp_vars.s_z[i], 0.0, GRB_CONTINUOUS, "delta_s_z_of_" + i);
	    }

	    // start region constraints
	    model.addQConstr((mp_vars.x_mp + delta_x_mp - start.x)*(mp_vars.x_mp + delta_x_mp - start.x) +
	    				(mp_vars.y_mp + delta_y_mp - start.y)*(mp_vars.y_mp + delta_y_mp - start.y) - start[3]*start[3] <= 0);


	    // set objective
	    GRBQuadExpr obj;

	    // minimize distance to contact regions
	    GRBLinExpr final_cp_delta_x;
	    GRBLinExpr final_cp_delta_y;
	    GRBLinExpr final_cp_delta_z;
	    for(int i = 0; i < attractive_vecs.size(); ++i) {
	    	GRBLinExpr delta_x_i = 0;
	    	delta_x_i += delta_x_mp;
	    	delta_x_i += theta_mp_x_pd[i]*delta_theta_mp;
	    	for(int j = 2; j <= i; ++j) {
	    		delta_x_i += delta_s_x[j] * cos(mp_vars.theta_mp) - delta_s_y[j] * sin(mp_vars.theta_mp);
	    	}

	    	GRBLinExpr delta_y_i = 0;
	    	delta_y_i += delta_y_mp;
	    	delta_y_i += theta_mp_y_pd[i]*delta_theta_mp;
	    	for(int j = 2; j <= i; ++j) {
	    		delta_y_i += delta_s_x[j] * sin(mp_vars.theta_mp) + delta_s_y[j] * cos(mp_vars.theta_mp);
	    	}

	    	GRBLinExpr delta_z_i = 0;
	    	delta_z_i += delta_z_mp;
	    	for(int j = 2; j <= i; ++j) {
	    		delta_z_i += delta_s_z[j];
	    	}

	    	if(i == attractive_vecs.size() - 1) {
	    		final_cp_delta_x = delta_x_i;
	    		final_cp_delta_y = delta_y_i;
	    		final_cp_delta_z = delta_z_i;
	    	}
	    	obj += (delta_x_i - attractive_vecs[i].x)*(delta_x_i - attractive_vecs[i].x) + 
	    		   (delta_y_i - attractive_vecs[i].y)*(delta_y_i - attractive_vecs[i].y) + 
	    		   (delta_z_i - attractive_vecs[i].z)*(delta_z_i - attractive_vecs[i].z);
	    }

	    // goal region constraints if within goal region
	    if(euclidean_distance_2d({global_c_seq.back().tf.x, global_c_seq.back().tf.y, 0}, goal) < goal[3]) {
		    model.addQConstr((global_c_seq.back().tf.x + final_cp_delta_x - goal.x)*(global_c_seq.back().tf.x + final_cp_delta_x - goal.x) +
		    				(global_c_seq.back().tf.y + final_cp_delta_y - goal.y)*(global_c_seq.back().tf.y + final_cp_delta_y - goal.y) -
		    					goal[3]*goal[3] <= 0);
	    } else {
	    	// end goal constraint is represented as a task if not within region
	    	obj += 100*(global_c_seq.back().tf.x + final_cp_delta_x - goal.x)*(global_c_seq.back().tf.x + final_cp_delta_x - goal.x) +
		    				(global_c_seq.back().tf.y + final_cp_delta_y - goal.y)*(global_c_seq.back().tf.y + final_cp_delta_y - goal.y) -
		    					goal[3]*goal[3];
	    }

	    model.setObjective(obj);
	    model.optimize();

	    // data structure conversion
	    Mp_optimization_vars delta;
	    delta.x_mp = delta_x_mp.get(GRB_DoubleAttr_X);
	    delta.y_mp = delta_y_mp.get(GRB_DoubleAttr_X);
	    delta.z_mp = delta_z_mp.get(GRB_DoubleAttr_X);
	    delta.theta_mp = delta_theta_mp.get(GRB_DoubleAttr_X);
	    delta.s_x.resize(delta_s_x.size());
	    delta.s_y.resize(delta_s_y.size());
	    delta.s_z.resize(delta_s_z.size());
	    delta.s_x[0] = 0;
	    delta.s_y[0] = 0;
	    delta.s_z[0] = 0;
	    delta.s_x[1] = 0;
	    delta.s_y[1] = 0;
	    delta.s_z[1] = 0;
	    for(int i = 2; i < delta_s_x.size(); ++i) {
	    	delta.s_x[i] = delta_s_x[i].get(GRB_DoubleAttr_X);
	    	delta.s_y[i] = delta_s_y[i].get(GRB_DoubleAttr_X);
	    	delta.s_z[i] = delta_s_z[i].get(GRB_DoubleAttr_X);
	    }
	    model.reset();
	    return delta;
	} catch(GRBException e) {
		std::cout << "Gurobi error code: " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	}


}

// returns "delta size", an indication of how much change this iteration of the optimizer has suggested
// scales passed in optimization deltas passed by reference
dReal scale_deltas(Mp_optimization_vars & optimization_deltas) {
	dReal delta_size = pow(optimization_deltas.x_mp, 2) + pow(optimization_deltas.y_mp, 2) + pow(optimization_deltas.z_mp, 2) + pow(optimization_deltas.theta_mp, 2);
	dReal stretch_size = 0.;
	for(int i = 0; i < optimization_deltas.s_x.size(); ++i) {
		stretch_size += pow(optimization_deltas.s_x[i], 2) + pow(optimization_deltas.s_y[i], 2) + pow(optimization_deltas.s_z[i], 2);
	}

	stretch_size /= optimization_deltas.s_x.size();
	delta_size += stretch_size;

	if(delta_size > max_delta_size_c) { // maybe choose something less random, with weights on each variable :)
		dReal scale_factor = max_delta_size_c / delta_size;

		optimization_deltas.x_mp *= scale_factor;
		optimization_deltas.y_mp *= scale_factor;
		optimization_deltas.z_mp *= scale_factor;
		optimization_deltas.theta_mp *= scale_factor;

		for(dReal & s : optimization_deltas.s_x) {
			s *= scale_factor;
		}
		for(dReal & s : optimization_deltas.s_y) {
			s *= scale_factor;
		}
		for(dReal & s : optimization_deltas.s_z) {
			s *= scale_factor;
		}
	}

	return delta_size;
}

// should return motion plans in order of most likely to fit environment
// start/goal are packed as [x,y,z,radius]
void Motion_plan_library::query(Drawing_handler & dh, const vector<Contact_region> & contact_regions,
											   const Vector & start, const Vector & goal) const {
	
	// dReal motion_plan_xy_length = euclidean_distance_2d(start, goal);

	// round motion plan length down to nearest bucket size factor
	// int motion_plan_bucket_key = int(motion_plan_xy_length / motion_plan_bucket_size_c) * motion_plan_bucket_size_c;

	// auto it = partitioned_clusters.find(motion_plan_bucket_key);

	// if(it == partitioned_clusters.end()) {
	// 	return {};
	// }

	// for(const Motion_plan_cluster & cluster : it->second) {
		// snap cluster representative to environment

	// const Motion_plan & cluster_rep = motion_plans[cluster.plan_indices[cluster.representative_index]];

	const vector<Contact> & local_c_seq = cluster_rep.contact_sequence;
	assert(local_c_seq.size() > 1);

	vector<Contact> global_c_seq = cluster_rep.contact_sequence;
	
	Mp_optimization_vars mp_optim;
	mp_optim.x_mp = start.x;
	mp_optim.y_mp = start.y;
	mp_optim.z_mp = start.z;
	mp_optim.theta_mp = atan2(goal.y - start.y, goal.x - start.x);
	mp_optim.s_x.resize(local_c_seq.size());
	mp_optim.s_y.resize(local_c_seq.size());
	mp_optim.s_z.resize(local_c_seq.size());

	// initialize delta stretch vectors
	// cannot stretch first contact pose
	mp_optim.s_x[0] = 0;
	mp_optim.s_y[0] = 0;
	mp_optim.s_z[0] = 0;
	for(int i = 1; i < local_c_seq.size(); ++i) {
		const Contact & curr_contact = local_c_seq[i];

		try {
			const Contact & pivot_contact = get_pivot_contact(local_c_seq, i);
			mp_optim.s_x[i] = curr_contact.tf.x - pivot_contact.tf.x;
			mp_optim.s_y[i] = curr_contact.tf.y - pivot_contact.tf.y;
			mp_optim.s_z[i] = curr_contact.tf.z - pivot_contact.tf.z;
		} catch(...) {}

	}

	global_c_seq = transform_plan(local_c_seq, mp_optim);

	for(int d = 0; d < max_opt_iter_c; ++d) {

		dh.ClearHandler();
		for(int i = 0; i < global_c_seq.size(); ++i) {
			dh.DrawRegion({global_c_seq[i].tf.x, global_c_seq[i].tf.y, global_c_seq[i].tf.z + .01}, {0, 0, 1}, 0.05, 1);
		}

		if(!d) usleep(1000000);

		usleep(100000);
		
		// compute partial derivatives for each contact pose
		vector<dReal> theta_mp_x_pd(global_c_seq.size());
		vector<dReal> theta_mp_y_pd(global_c_seq.size());

		for(int i = 0; i < global_c_seq.size(); ++i) {
			Contact global_contact = global_c_seq[i];

			dReal r = euclidean_distance_2d({mp_optim.x_mp, mp_optim.y_mp, 0}, {global_contact.tf.x, global_contact.tf.y, 0});
			dReal theta = atan2(global_contact.tf.y - mp_optim.y_mp, global_contact.tf.x - mp_optim.x_mp);

			// x = rcos(theta), x' = -rsin(theta)
			theta_mp_x_pd[i] = -r*sin(theta);

			// y = rsin(theta), y' = rcos(theta)
			theta_mp_y_pd[i] = r*cos(theta);
		}

		// calculate attractive vectors for each contact pose
		vector<Vector> attractive_vecs(global_c_seq.size());
		for(int i = 0; i < global_c_seq.size(); ++i) {
			attractive_vecs[i] = get_attractive_vector(contact_regions, global_c_seq[i]);
		}

		// optimize motion plan variables
		Mp_optimization_vars optimization_deltas = optimize_plan(global_c_seq, mp_optim, attractive_vecs,
																 theta_mp_x_pd, theta_mp_y_pd, start, goal);

		if(scale_deltas(optimization_deltas) < min_delta_size_c) {
			break;
		}

		// apply changes, move small step
		mp_optim.x_mp += optimization_deltas.x_mp;
		mp_optim.y_mp += optimization_deltas.y_mp;
		mp_optim.z_mp += optimization_deltas.z_mp;
		mp_optim.theta_mp += optimization_deltas.theta_mp;
		for(int i = 0; i < local_c_seq.size(); ++i) {
			mp_optim.s_x[i] += optimization_deltas.s_x[i];
			mp_optim.s_y[i] += optimization_deltas.s_y[i];
			mp_optim.s_z[i] += optimization_deltas.s_z[i];
		}

		// calculate updated global contact sequence
		global_c_seq = transform_plan(local_c_seq, mp_optim);
	}
	dh.ClearHandler();
	for(int i = 0; i < global_c_seq.size(); ++i) {
		dh.DrawRegion({global_c_seq[i].tf.x, global_c_seq[i].tf.y, global_c_seq[i].tf.z + .01}, {0, 0, 1}, 0.05, 1);
	}

	usleep(1000000);
}

// INCOMPLETE
void Motion_plan_library::learn(vector<Contact> contact_sequence) {
	assert(contact_sequence.size() > 3);

	Vector first_pose{contact_sequence.front().tf.x, contact_sequence.front().tf.y, 0};
	Vector second_pose{contact_sequence[1].tf.x, contact_sequence[1].tf.y, 0};
	Vector mp_pose = (first_pose + second_pose); mp_pose /= 2;

	Vector penultimate_pose{contact_sequence[contact_sequence.size() - 2].tf.x, contact_sequence[contact_sequence.size() - 2].tf.y, 0};
	Vector last_pose{contact_sequence.back().tf.x, contact_sequence.back().tf.y, 0};
	Vector mp_fin_pose = (penultimate_pose + last_pose); mp_fin_pose /= 2;
}
