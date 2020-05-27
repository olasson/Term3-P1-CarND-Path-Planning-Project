#ifndef EGO_H
#define EGO_H_

struct EgoCar {
	// Cartesian
	double x;
	double y;

	// Frenet
	double s;
	double d;

	double yaw;

	double velocity;
}
#endif // EGO_H_