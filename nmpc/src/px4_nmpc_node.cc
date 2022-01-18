#include "nmpc/px4_nmpc.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_controller");
  ros::NodeHandle nh;
  px4_ctrl::Px4Nmpc px4_controller(nh);

  ros::spin();
  return 0;
}