#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>

class Transformer {
 public:
  Transformer();

 private:
  ros::Publisher nmea_pub_;
  ros::Subscriber navsatfix_sub_;

  void receiveNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &navsat_msg);
};

Transformer::Transformer() {
  ros::NodeHandle nh;
  navsatfix_sub_ = nh.subscribe("navsatfix", 1, &Transformer::receiveNavSatFix, this);
  ROS_INFO("Subscribing to NavSatFix topic %s", navsatfix_sub_.getTopic().c_str());
  nmea_pub_ = nh.advertise<nmea_msgs::Sentence>("nmea", 1);
  ROS_INFO("Advertising NMEA topic %s", nmea_pub_.getTopic().c_str());
}

void Transformer::receiveNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &navsat_msg) {
  ROS_INFO_ONCE("Received first NavSatFix message.");
  // Conversion inspired from: https://answers.ros.org/question/377844/converting-sensors_msgsnavsatfix-to-nmea_msgssentence-messages/
  nmea_msgs::Sentence nmea_msg;
  nmea_pub_.publish(nmea_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fix2nmea");

  auto transformer = Transformer(nh_private);

  ros::spin();
  return 0;
}