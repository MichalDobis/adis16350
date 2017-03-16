#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
 

 tf::Transform imu_transform;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	 tf::Quaternion	q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	 
	 imu_transform.setRotation(q);
	 imu_transform.setOrigin( tf::Vector3(0,0,0));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("imu_data", 5, imu_callback); 
  sleep(1);
  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;
  tf::Quaternion q(0,0,0,1);
  imu_transform.setRotation(q);
  imu_transform.setOrigin(tf::Vector3(0,0,0));

  
  while(n.ok()){
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    broadcaster.sendTransform(
      tf::StampedTransform(imu_transform, ros::Time::now(),"base_link", "board"));

    broadcaster.sendTransform(
      tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0)), ros::Time::now(),"board", "imu"));

    r.sleep();
  }

  return 0;
}
