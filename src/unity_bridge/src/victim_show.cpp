
//#include "victim_signal_gen.h"
#include <ros/ros.h>
//#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PolygonStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "victim_signal_node");
  ros::NodeHandle nh;

  ros::Publisher Victimcoord = nh.advertise<geometry_msgs::PolygonStamped>("victimCoord_continous_topic",10);
  geometry_msgs::PolygonStamped vic_msg;
  geometry_msgs::Point32 point;

  for(int i =0;i<8;i++){
    point.x = (rand()%(60-40))+40;
    point.y = (rand()%(60-40))+40;
    point.z = 55;
    vic_msg.polygon.points.push_back(point);
  }
  //vic_msg.header.frame_id = "victim";
  ros::Rate loop_rate(1);

  while(ros::ok()){
    Victimcoord.publish(vic_msg);
    int i =0;
    float coordinates[3] = {
      static_cast<float>(vic_msg.polygon.points[i].x),
      static_cast<float>(vic_msg.polygon.points[i].y),
      static_cast<float>(vic_msg.polygon.points[i].z)
    };
    
    ROS_INFO("victim(%d):%f,%f,%f",i,coordinates[0],coordinates[1],coordinates[2]);
    if(i<8) i++;
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}

