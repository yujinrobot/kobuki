#include <iostream>
#include "../include/kobukibot_controller/controller_app.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_app");

  controller_app::ControllerApp controller;
  controller.init();
  controller.spin();

  return 0;
}

