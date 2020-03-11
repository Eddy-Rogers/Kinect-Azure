#pragma once

struct point {
	float x;
	float y;
	float z;

	point() {
		this->x = 0.0;
		this->y = 0.0;
		this->z = 0.0;
	}

	point(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

class Joints {
	point kneeLocation;
	point ankleLocation;

	Joints() {
		this->kneeLocation = point();
		this->ankleLocation = point();
	}

	Joints(point kneeLocation, point ankleLocation) {
		this->kneeLocation = kneeLocation;
		this->ankleLocation = ankleLocation;
	}

	point getKneeLocation() {
		return this->kneeLocation;
	}

	point getAnkleLocation() {
		return this->ankleLocation;
	}
};