#include "sun_iiwa_fri/IIWAFRINode.h"

using namespace sun::iiwa::fri;

IIWAFRINode::IIWAFRINode(const ros::NodeHandle &nh,
                         const ros::NodeHandle &nh_params, int udp_port,
                         unsigned int receiveTimeout)
    : fri_client_(nh, nh_params), connection_(receiveTimeout),
      udp_port_(udp_port), app_(connection_, fri_client_) {}
      
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

void IIWAFRINode::run_init() { fri_client_.init(); }

bool IIWAFRINode::step() { return app_.step(); }

void IIWAFRINode::run() {
  run_init();
  // repeatedly call the step routine to receive and process FRI packets
  bool success = true;
  while (ros::ok() && success) {
    success = app_.step();
  }
}
