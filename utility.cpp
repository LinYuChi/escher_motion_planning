#include "Utility.h"

#include <cmath>

using OpenRAVE::dReal;
using std::vector;
using std::cos; using std::sin;

vector<dReal> get_SO3(dReal roll_in_deg, dReal pitch_in_deg, dReal yaw_in_deg) {
	double roll_in_rad = roll_in_deg * (M_PI / 180);
	double pitch_in_rad = pitch_in_deg * (M_PI / 180);
	double yaw_in_rad = yaw_in_deg * (M_PI / 180);

	double roll_x = cos(roll_in_rad);
	double roll_y = sin(roll_in_rad);

	double pitch_x = cos(pitch_in_rad);
	double pitch_y = sin(pitch_in_rad);

	double yaw_x = cos(yaw_in_rad);
	double yaw_y = sin(yaw_in_rad);

	// rpy -> SO3
	return {};
}
