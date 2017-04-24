#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf/tf.h>

class MathematicsOperations {

public:
	const static double PI2 = 2 * M_PI;
	MathematicsOperations();
	MathematicsOperations(double usingGyroPercentage, double usingAkcelPercentage);
	geometry_msgs::Vector3 computeOrientationFromGyro(sensor_msgs::Imu *imu);
	geometry_msgs::Vector3 computeComplementary(sensor_msgs::Imu imu);
	geometry_msgs::Vector3 getOrientation();

	geometry_msgs::Quaternion createQuaternion();
	geometry_msgs::Quaternion createQuaternion(geometry_msgs::Vector3 angle);
	static void createCovarianceMatrix(sensor_msgs::Imu *imu);
	static void createCovarianceMatrix(sensor_msgs::Imu *imu, std::vector<double> pose, std::vector<double> angular_velocity, std::vector<double> linear_acceleration);

	void setComplementaryFilterParams(double usingGyroPercentage, double usingAkcelPercentage);

private:
	ros::Time last_time;
	geometry_msgs::Vector3 cartezianAngle;
	bool wasFirstMeasurement;

	double usingGyroPercentage;
	double usingAkcelPercentage;
};
