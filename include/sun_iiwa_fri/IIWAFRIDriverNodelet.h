#ifndef SUN_IIWA_FRI_NODELET_H
#define SUN_IIWA_FRI_NODELET_H

#include "nodelet/nodelet.h"
#include "sun_iiwa_fri/IIWAFRINode.h"
#include <thread>

namespace sun::iiwa::fri {

class IIWAFRIDriverNodelet : public nodelet::Nodelet {

public:
  volatile bool running_;
  std::unique_ptr<std::thread> nodeletThread_;
  std::unique_ptr<sun::iiwa::fri::IIWAFRINode> driverNode_;

  ~IIWAFRIDriverNodelet() {
    if (running_) {
      running_ = false;
      nodeletThread_->join();
    }
  }

  virtual void onInit() override {

    driverNode_ = std::unique_ptr<sun::iiwa::fri::IIWAFRINode>(
        new sun::iiwa::fri::IIWAFRINode(getNodeHandle(),
                                        getPrivateNodeHandle()));

    // spawn device thread
    running_ = true;
    nodeletThread_ = std::unique_ptr<std::thread>(
        new std::thread(std::bind(&IIWAFRIDriverNodelet::threadCB, this)));
  }

  /** Nodelet device poll thread main function. */
  void threadCB() {
    driverNode_->connect();
    driverNode_->run_init();
    // repeatedly call the step routine to receive and process FRI packets
    bool success = true;
    while (ros::ok() && success && running_) {
      success = driverNode_->step();
    }
  }
};

} // namespace sun::iiwa::fri

#endif