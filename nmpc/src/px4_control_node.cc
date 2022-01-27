#include "nmpc/px4_control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_controller");
  ros::NodeHandle nh;
  px4_ctrl::PX4Control px4_controller(nh, 20);

  ros::spin();

  px4_controller.publisher_worker_t.join();

  return 0;
}