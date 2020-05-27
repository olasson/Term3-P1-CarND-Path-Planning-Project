#ifndef EGO_CAR_H
#define EGO_CAR_H_

struct EgoCar {
	// Cartesian
	double x;
	double y;

	// Frenet
	double s;
	double d;

	double yaw;

	double velocity;
};
#endif // EGO_CAR_H_