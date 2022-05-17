#include <limits>

#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <ublox_msgs/NavHPPOSLLH.h>

class Transformer {
 public:
  Transformer();

 private:
  ros::Publisher nmea_pub_;
  ros::Subscriber navsatfix_sub_;
  ros::Subscriber navhpposllh_sub_;

  void receiveNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &navsat_msg);
  void receiveNavHpPosLlh(const ublox_msgs::NavHPPOSLLH::ConstPtr &navsat_msg);
};

Transformer::Transformer() {
  ros::NodeHandle nh;
  navsatfix_sub_ = nh.subscribe("navsatfix", 1, &Transformer::receiveNavSatFix, this);
  ROS_INFO("Subscribing to NavSatFix topic %s", navsatfix_sub_.getTopic().c_str());
  navhpposllh_sub_ = nh.subscribe("navhpposllh", 1, &Transformer::receiveNavHpPosLlh, this);
  ROS_INFO("Subscribing to NavHPPOSLLH topic %s", navhpposllh_sub_.getTopic().c_str());
  nmea_pub_ = nh.advertise<nmea_msgs::Sentence>("nmea", 1);
  ROS_INFO("Advertising NMEA topic %s", nmea_pub_.getTopic().c_str());
}

void Transformer::receiveNavHpPosLlh(const ublox_msgs::NavHPPOSLLH::ConstPtr &navhp_msg) {
  ROS_INFO_ONCE("Received first NavHPPOSLLH message.");

  char buf[255];

  // Time conversion
  boost::posix_time::milliseconds ms(navhp_msg->iTOW);
  auto now = ros::Time::now();
  auto time = now.toBoost().time_of_day();
  long int deci_seconds = now.nsec / 1e7;

  // Lat conversion
  char lat_dir = navhp_msg->lat < 0 ? 'S' : 'N';
  int8_t lat_degs = navhp_msg->lat / 1e7;
  double lat_mins = (navhp_msg->lat - lat_degs * 1e7) / 1e7 * 60.0;

  // Lon conversion
  char lon_dir = navhp_msg->lon < 0 ? 'W' : 'E';
  int8_t lon_degs = navhp_msg->lon / 1e7;
  double lon_mins = (navhp_msg->lon - lon_degs * 1e7) / 1e7 * 60.0;

  // Status conversion
  int8_t status = (navhp_msg->invalid_llh & 0b1) ? 0 : 1;

  // horizontal accuracy conversion
  //double h_acc = (double) navhp_msg->hAcc / (double) std::numeric_limits<uint32_t>::max();
  double h_acc = 0.9;

  uint8_t
      len = sprintf(buf,
                    "$GPGGA,%02ld%02ld%02ld.%ld,%02d%08.5f,%c,%03d%08.5f,%c,%d,08,%.1f,%d.%d,M,%d.%d,M,,",
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
                    status,
                    h_acc,
                    navhp_msg->hMSL / 1000,
                    navhp_msg->hMSL % 1000,
                    (navhp_msg->height - navhp_msg->hMSL) / 1000,
                    std::abs((navhp_msg->height - navhp_msg->hMSL)) % 1000
  );
  // Calculate checksum of sentence and add it to the end of the sentence
  uint8_t checksum = 0;
  for (int i = 1; i < len; i++) {
    checksum ^= buf[i];
  }
  sprintf(&buf[len], "*%02X\r\n", checksum);

  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header.stamp = now;
  nmea_msg.header.frame_id = "gps";
  nmea_msg.sentence = buf;
  nmea_pub_.publish(nmea_msg);
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

  // Minimum number of satellites is service times 4.
  int num_satellites = 0;
  if ((navsat_msg->status.service & sensor_msgs::NavSatStatus::SERVICE_GPS) == sensor_msgs::NavSatStatus::SERVICE_GPS) {
    num_satellites += 4;
  }
  if ((navsat_msg->status.service & sensor_msgs::NavSatStatus::SERVICE_GLONASS)
      == sensor_msgs::NavSatStatus::SERVICE_GLONASS) {
    num_satellites += 4;
  }
  if ((navsat_msg->status.service & sensor_msgs::NavSatStatus::SERVICE_COMPASS)
      == sensor_msgs::NavSatStatus::SERVICE_COMPASS) {
    num_satellites += 4;
  }
  if ((navsat_msg->status.service & sensor_msgs::NavSatStatus::SERVICE_GALILEO)
      == sensor_msgs::NavSatStatus::SERVICE_GALILEO) {
    num_satellites += 4;
  }

  uint8_t
      len = sprintf(buf,
                    "$GPGGA,%02ld%02ld%02ld.%ld,%02d%018.15f,%c,%02d%018.15f,%c,%d,%d,0.9,",
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
                    status,
                    num_satellites);

  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header = navsat_msg->header;
  nmea_msg.sentence = buf;
  //nmea_pub_.publish(nmea_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fix2nmea");

  Transformer transformer;

  ros::spin();
  return 0;
}