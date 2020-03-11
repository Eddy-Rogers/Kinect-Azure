#pragma once

struct JointLocation {

	float x;
	float y;
	float z;

	JointLocation() {
		this->x = 0.0;
		this->y = 0.0;
		this->z = 0.0;
	}

	JointLocation(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
};

class Joints {
	JointLocation kneeLocation;
	JointLocation ankleLocation;

	Joints() {
		kneeLocation = JointLocation();
		ankleLocation = JointLocation();
	}
};