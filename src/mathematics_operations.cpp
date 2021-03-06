#include <adis16350/mathematics_operations.h>

MathematicsOperations::MathematicsOperations(){

	this->wasFirstMeasurement = false;
	this->usingGyroPercentage = 0.98;
	this->usingAkcelPercentage = 0.02;
}

MathematicsOperations::MathematicsOperations(double usingGyroPercentage, double usingAkcelPercentage){

	this->wasFirstMeasurement = false;
	this->usingGyroPercentage = usingGyroPercentage;
	this->usingAkcelPercentage = usingAkcelPercentage;
}

void MathematicsOperations::setComplementaryFilterParams(double usingGyroPercentage, double usingAkcelPercentage){

	this->usingGyroPercentage = usingGyroPercentage;
	this->usingAkcelPercentage = usingAkcelPercentage;
}


geometry_msgs::Vector3 MathematicsOperations::computeOrientationFromGyro(sensor_msgs::Imu *imu){

	if (!wasFirstMeasurement){
		last_time = imu->header.stamp;
		wasFirstMeasurement = true;
	}

	double dt = (imu->header.stamp - last_time).toSec();
	last_time = imu->header.stamp;

	cartezianAngle.x += imu->angular_velocity.x * dt;
		// uhol[1] += gyro[1] * dt * M_PI/180;
		if (imu->linear_acceleration.z < 0)
		{
			cartezianAngle.z += PI2 - (imu->angular_velocity.z * dt);
		 	//akcel[0] *= -1;
			cartezianAngle.y += PI2 - (imu->angular_velocity.y * dt);
		 	imu->linear_acceleration.y *= -1;
		 }
		 else
		 {
		  	 cartezianAngle.z += imu->angular_velocity.z * dt;
		  	 cartezianAngle.y += imu->angular_velocity.y * dt;
		  }

		return cartezianAngle;
}

geometry_msgs::Vector3 MathematicsOperations::computeComplementary(sensor_msgs::Imu imu){

	if (imu.linear_acceleration.z == 0)
		return cartezianAngle;

	cartezianAngle.x = usingGyroPercentage * cartezianAngle.x + usingAkcelPercentage  * atan2((double)imu.linear_acceleration.y,(double)imu.linear_acceleration.z);
	cartezianAngle.y = usingGyroPercentage * cartezianAngle.y + usingAkcelPercentage  * atan2((double)imu.linear_acceleration.x,(double)imu.linear_acceleration.z);

	return cartezianAngle;
}


geometry_msgs::Quaternion MathematicsOperations::createQuaternion(){

	return tf::createQuaternionMsgFromRollPitchYaw(cartezianAngle.x, cartezianAngle.y, cartezianAngle.z);
}
geometry_msgs::Quaternion MathematicsOperations::createQuaternion(geometry_msgs::Vector3 angle){

	return tf::createQuaternionMsgFromRollPitchYaw(angle.x, angle.y,angle.z);
}

geometry_msgs::Quaternion MathematicsOperations::createQuaternion(double angle){

	return tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
}

geometry_msgs::Vector3 MathematicsOperations::getOrientation(){

	return cartezianAngle;
}

void MathematicsOperations::createCovarianceMatrix(sensor_msgs::Imu *imu){

	imu->linear_acceleration_covariance[0] = 0.005;
	imu->linear_acceleration_covariance[4] = 0.005;
	imu->linear_acceleration_covariance[8] = 100000;

	imu->angular_velocity_covariance[0] = 0.0001;
	imu->angular_velocity_covariance[4] = 0.0001;
	imu->angular_velocity_covariance[8] = 0.0005;

	imu->orientation_covariance[0] = 0.00001;
	imu->orientation_covariance[4] = 0.00001;
	imu->orientation_covariance[8] = 0.00005;
}

void MathematicsOperations::createCovarianceMatrix(sensor_msgs::Imu *imu, std::vector<double> pose, std::vector<double> angular_velocity, std::vector<double> linear_acceleration){

	imu->linear_acceleration_covariance[0] = linear_acceleration[0];
	imu->linear_acceleration_covariance[4] = linear_acceleration[1];
	imu->linear_acceleration_covariance[8] = linear_acceleration[2];

	imu->angular_velocity_covariance[0] = angular_velocity[0];
	imu->angular_velocity_covariance[4] = angular_velocity[1];
	imu->angular_velocity_covariance[8] = angular_velocity[2];

	imu->orientation_covariance[0] = pose[0];
	imu->orientation_covariance[4] = pose[1];
	imu->orientation_covariance[8] = pose[2];
}


