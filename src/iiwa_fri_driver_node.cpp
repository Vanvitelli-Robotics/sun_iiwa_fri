#include "sun_iiwa_fri/IIWAFRINode.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "iiwa_fri_driver");

  ros::NodeHandle nh;

  sun::iiwa::fri::IIWAFRINode node(nh);

  node.connect();
  node.run();

  return 0;
}
