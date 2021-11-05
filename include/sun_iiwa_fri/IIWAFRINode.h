#ifndef SUN_IIWA_FRI_NODE_H
#define SUN_IIWA_FRI_NODE_H

#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include "sun_iiwa_fri/IIWAFRIClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

namespace sun::iiwa::fri {

class IIWAFRINode {
protected:
  // create new client
  RosFriClient fri_client_;

  // create new udp connection
  KUKA::FRI::UdpConnection connection_;

  // pass connection and client to a new FRI client application
  KUKA::FRI::ClientApplication app_;

  // Parameter empty string means: repeat to the address, which sends the data
  std::string remoteHost_;
  int udp_port_;

  static const int DEFAULT_PORTID = 30200;

public:
  IIWAFRINode(const ros::NodeHandle &nh, int udp_port = DEFAULT_PORTID, unsigned int receiveTimeout = 10000);
  ~IIWAFRINode();

  void connect();

  void disconnect();

  void run();
};

} // namespace sun::iiwa::fri

#endif