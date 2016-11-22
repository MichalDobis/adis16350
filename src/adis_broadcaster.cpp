#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
 

 tf::Transform imu_transform;
 ros::Time current_time;
double vx = 0;
double vy = 0;
double vz = 0;
double x = 0;
double y = 0;
double z = 0;
 
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static ros::Time last_time = ros::Time::now();
	 tf::Quaternion	q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	 
	 imu_transform.setRotation(q);
	 
	 current_time = ros::Time::now();
	 double dt = (current_time - last_time).toSec();
	 last_time = current_time;
	 
	 vx += msg->linear_acceleration.x * dt;
	 vy += msg->linear_acceleration.y * dt;
	 vz += msg->linear_acceleration.z * dt;
	 //ROS_INFO("vx %.2f vy %.2f vz %.2f",vx,vy,vz);
	 
	 x += vx * dt;
	 y += vy * dt;
	 z += vz * dt;
	// ROS_INFO("x %.2f y %.2f z %.2f",x,y,z);  
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
  imu_transform.setOrigin(tf::Vector3(x,y,z));
  
  while(n.ok()){
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //broadcaster laser na base_link
  
    broadcaster.sendTransform(
      tf::StampedTransform(imu_transform, ros::Time::now(),"base_link", "board"));
   // r.sleep();
     broadcaster.sendTransform(
      tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0)), ros::Time::now(),"board", "imu"));
    r.sleep();
    
    //listener a broadcaster base_link na odom
    /*
     tf::StampedTransform transform;
       try{
         listener.lookupTransform("odom", "base_link",  
                                  ros::Time(0), transform);
       }
       catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
       }
    
       broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(transform.getRotation(), transform.getOrigin()),
        ros::Time::now(),"odom", "base_link"));*/
  }
  return 0;
}
