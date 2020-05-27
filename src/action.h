#ifndef ACTION_H_
#define ACTION_H_

struct Action {
	int lane;
	double target_velocity;
	bool change_lane;
};
#endif // ACTION_H_