#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fix2nmea");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::spin();
  return 0;
}