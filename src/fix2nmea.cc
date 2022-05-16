#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

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

  char buf[255]; // Buffer for GPGGA sentence

  // Time conversion
  auto time = navsat_msg->header.stamp.toBoost().time_of_day();
  long int deci_seconds = navsat_msg->header.stamp.nsec / 1e7;

  // Latitude conversion
  char lat_dir = navsat_msg->latitude < 0.0 ? 'S' : 'N';
  int8_t lat_degs = navsat_msg->latitude;
  double lat_mins = (navsat_msg->latitude - (double) lat_degs) * 60.0;

  // Longitude conversion
  char lon_dir = navsat_msg->longitude < 0.0 ? 'W' : 'E';
  int8_t lon_degs = navsat_msg->longitude;
  double lon_mins = (navsat_msg->longitude - (double) lon_degs) * 60.0;

  // Status conversion
  int8_t status = navsat_msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX ? 1 : 0;

  // Minimum number of satellites


  uint8_t
      len = sprintf(buf,
                    "$GPGGA,%02ld%02ld%02ld.%ld,%02d%018.15f,%c,%02d%018.15f,%c,%d",
                    time.hours(),
                    time.minutes(),
                    time.seconds(),
                    deci_seconds,
                    lat_degs,
                    lat_mins,
                    lat_dir,
                    lon_degs,
                    lon_mins,
                    lon_dir,
                    status);

  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header = navsat_msg->header;
  nmea_msg.sentence = buf;
  nmea_pub_.publish(nmea_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fix2nmea");

  Transformer transformer;

  ros::spin();
  return 0;
}