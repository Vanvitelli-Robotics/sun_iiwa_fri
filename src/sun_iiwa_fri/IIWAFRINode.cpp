#include "sun_iiwa_fri/IIWAFRINode.h"
#include "sun_iiwa_fri/check_realtime.h"

using namespace sun::iiwa::fri;

IIWAFRINode::IIWAFRINode(const ros::NodeHandle &nh, int udp_port,
                         unsigned int receiveTimeout)
    : fri_client_(nh), connection_(receiveTimeout), udp_port_(udp_port),
      app_(connection_, fri_client_) {}
IIWAFRINode::~IIWAFRINode() { disconnect(); }

void IIWAFRINode::connect() {
  // Connect client application to KUKA Sunrise controller.
  // Parameter NULL means: repeat to the address, which sends the data
  const char *remoteHost;
  if (remoteHost_.empty() || remoteHost_ == "") {
    remoteHost = NULL;
  } else {
    remoteHost = remoteHost_.c_str();
  }
  app_.connect(udp_port_, remoteHost);
}

void IIWAFRINode::disconnect() { app_.disconnect(); }

void IIWAFRINode::run() {

  sun::check_and_set_realtime();

  // repeatedly call the step routine to receive and process FRI packets
  bool success = true;
  while (ros::ok() && success) {
    success = app_.step();
  }
}
