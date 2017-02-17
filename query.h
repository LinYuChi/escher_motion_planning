#ifndef QUERY_H
#define QUERY_H

#include "motion_plan.h"

class Environment_handler; // forward declaration

#include <vector>

// returns overlapping circular planes in environment
std::vector<Contact_region> get_contact_regions(const Environment_handler & env_handler);
// ^ maybe use set instead of vector

#endif 
