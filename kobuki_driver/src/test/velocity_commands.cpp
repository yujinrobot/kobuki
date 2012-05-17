

#include <iostream>

double bias, radius, speed;

void velocityCommand(const double& vx, const double &wz) {
  if (wz == 0.0f) {
    radius = 0;
  } else if (vx == 0.0f && wz > 0.0f) {
    radius = 1;
  } else if (vx == 0.0f && wz < 0.0f) {
    radius = -1;
  } else {
    radius = (short)(vx * 1000.0f / wz);
  }
  if ( vx < 0 ) {
    speed = (short)(1000.0f * std::min(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  } else {
    speed = (short)(1000.0f * std::max(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  }
}
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  std::cout << "Hello Dude" << std::endl;

  bias = 0.23;
  double vx = -0.10;
  double wz = 0.4;
  const unsigned int steps = 20;
  for (unsigned int i = 0; i < steps; ++i ) {
    velocityCommand(vx,wz);
    std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
    vx += 0.2/static_cast<double>(steps);
  }
  vx = -0.10;
  wz = 0.4;
  for (unsigned int i = 0; i < steps; ++i ) {
    velocityCommand(vx,wz);
    std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
    vx += 0.2/static_cast<double>(steps);
  }
  return 0;
}
