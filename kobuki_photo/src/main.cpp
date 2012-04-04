#include <iostream>
#include "../include/kobukibot_photo/photo_app.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "photo_app");

  photo_app::PhotoApp photo;
  photo.init();
  photo.spin();

  return 0;
}

