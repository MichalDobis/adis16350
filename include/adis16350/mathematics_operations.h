#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf/tf.h>

class MathematicsOperations {

public:
	const static double PI2 = 2 * M_PI;
	const static double PI_05 = M_PI / 180;
	MathematicsOperations();
	MathematicsOperations(double usingGyroPercentage, double usingAkcelPercentage);
	geometry_msgs::Vector3 computeOrientationFromGyro(sensor_msgs::Imu *imu);
	geometry_msgs::Vector3 computeComplementary(sensor_msgs::Imu imu);
	geometry_msgs::Vector3 getOrientation();

	geometry_msgs::Quaternion createQuaternion();
	geometry_msgs::Quaternion createQuaternion(geometry_msgs::Vector3 angle);
	void createCovarianceMatrix(sensor_msgs::Imu *imu);

	void setComplementaryFilterParams(double usingGyroPercentage, double usingAkcelPercentage);

private:
	ros::Time last_time;
	geometry_msgs::Vector3 cartezianAngle;
	bool wasFirstMeasurement;

	double usingGyroPercentage;
	double usingAkcelPercentage;
};
