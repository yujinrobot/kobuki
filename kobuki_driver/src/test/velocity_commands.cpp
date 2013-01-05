/**
 * @file /kobuki_driver/src/test/velocity_commands.cpp
 *
 * @brief Unit test for velocity command inputs.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdio>
#include <cmath>

double bias, radius, speed;

void velocityCommand(const double& vx, const double &wz) {
  const double epsilon = 0.0001;
  if ( std::abs(wz) < epsilon ) {
    radius = 0;
  } else if ( (std::abs(vx) < epsilon ) && ( wz > epsilon ) ) {
    radius = 1;
  } else if ((std::abs(vx) < epsilon ) && ( wz < -1*epsilon ) ) {
    radius = -1;
  } else {
    radius = (short)(vx * 1000.0f / wz);
  }
  if ( vx < 0.0 ) {
    speed = (short)(1000.0f * std::min(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  } else {
    speed = (short)(1000.0f * std::max(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  }
}

class Rb2Vw
{
public:
    int half_wheel_2_wheel;
    Rb2Vw( const int wheelToWheelInMM = 230 ) : half_wheel_2_wheel(wheelToWheelInMM/2) {}

    void getVW( const int leftWheelVelocity, const int rightWheelVelocity, int & linearSpeed, int & rotationalSpeedInDegree )
    {
        if( std::abs( static_cast<float>(leftWheelVelocity ))
            > std::abs( static_cast<float>(rightWheelVelocity) ) )
            linearSpeed = leftWheelVelocity;
        else
            linearSpeed = rightWheelVelocity;

        rotationalSpeedInDegree = ( rightWheelVelocity - leftWheelVelocity  ) * 180 / (half_wheel_2_wheel*2) / 3.14152;
    }

    void getSR( const int leftWheelVelocity, const int rightWheelVelocity, int & desiredSpeed, int & desiredRadius )
    {
        if( leftWheelVelocity == rightWheelVelocity )
        {
            desiredSpeed    = leftWheelVelocity;
            desiredRadius   = 0;
        }
        else if( std::abs(static_cast<float>(leftWheelVelocity)) == std::abs(static_cast<float>(rightWheelVelocity)) )
        {
            desiredSpeed = std::abs(static_cast<float>(leftWheelVelocity));
            if( leftWheelVelocity < rightWheelVelocity )    desiredRadius = +1;
            else                                            desiredRadius = -1;
        }
        else
        {
            desiredRadius   = -(leftWheelVelocity+rightWheelVelocity)*half_wheel_2_wheel/(leftWheelVelocity-rightWheelVelocity);
            if( std::abs(static_cast<float>(leftWheelVelocity)) < std::abs(static_cast<float>(rightWheelVelocity)) )
            {
                desiredSpeed    = rightWheelVelocity;
            }
            else
            {
                desiredSpeed = leftWheelVelocity;
                //desiredRadius = -desiredRadius;
            }
        }
    }

    void getWheelVelocity( const double & linearVelInMeter, const double & rotationalVelInRadian, double & leftWheelVel, double & rightWheelVel )
    {
        leftWheelVel  = linearVelInMeter - rotationalVelInRadian * static_cast<double>( half_wheel_2_wheel ) * 0.001;
        rightWheelVel = linearVelInMeter + rotationalVelInRadian * static_cast<double>( half_wheel_2_wheel ) * 0.001;
    }

    void update( const int desiredSpeed, const int desiredRadius, int & leftWheelVelocity, int & rightWheelVelocity )
    {
        bool change_direction(false);

        if( desiredSpeed == 0 && desiredRadius == 0 )
        {
            leftWheelVelocity = 0;
            rightWheelVelocity = 0;
            return;
        }

        if( desiredRadius == 0 )
        {
            leftWheelVelocity   = desiredSpeed;
            rightWheelVelocity  = desiredSpeed;
        }
        else if( desiredRadius == -1 || desiredRadius == +1 )
        {
            leftWheelVelocity = -desiredRadius * desiredSpeed;
            rightWheelVelocity = desiredRadius * desiredSpeed;
        }
        else
        {
            int radius( desiredRadius );
            if( radius < 0 )
            {
                radius = -radius;
                change_direction = true;
            }

            int left( radius - half_wheel_2_wheel );
            int right( radius + half_wheel_2_wheel );

            if( change_direction )
            {
                leftWheelVelocity   = (desiredSpeed*right)/right;
                rightWheelVelocity  = (desiredSpeed*left)/right;
            }
            else
            {
                leftWheelVelocity   = (desiredSpeed*left)/right;
                rightWheelVelocity  = (desiredSpeed*right)/right;
            }
        }
    }
};


int main(int argc, char **argv) {
  std::cout << "Hello Dude" << std::endl;

  bias = 0.23;
  double vx = -0.10;
  double wz = 0.15;

  Rb2Vw fw;
//      const unsigned int steps = 30;
//      for (unsigned int i = 0; i < steps; ++i ) {
//              velocityCommand(vx,wz);
//              std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
//              vx += 0.2/static_cast<double>(steps);
//      }
//      vx = -0.10;
//      wz = -0.15;
//      for ( i = 0; i < steps; ++i ) {
//              velocityCommand(vx,wz);
//              std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
//              vx += (0.2/static_cast<double>(steps));
//      }

  std::cout << "Reproduce the issue with slow motion command" << std::endl;
  vx = -0.03;
  wz = 0.4;
  velocityCommand(vx,wz);
  std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;

  int left_wheel_speed;
  int right_wheel_speed;
  fw.update( speed, radius, left_wheel_speed, right_wheel_speed );
  printf("fw will generate left right wheel speed as [%d mm/sec | %d mm/sec] \n", left_wheel_speed, right_wheel_speed );

  double left_wheel_speed_in_meter;
  double right_wheel_speed_in_meter;
  int desire_speed, desired_radius;
  fw.getWheelVelocity( vx, wz, left_wheel_speed_in_meter, right_wheel_speed_in_meter );
  printf("When you have vx(%f), wz(%f), robot have to generate left right wheel speed as %f, %f \n", vx, wz, left_wheel_speed_in_meter, right_wheel_speed_in_meter );
  fw.getSR( left_wheel_speed_in_meter*1000, right_wheel_speed_in_meter*1000, desire_speed, desired_radius );
  printf("My quick code result for speed and radius are %d, %d \n", desire_speed, desired_radius );

  fw.update( desire_speed, desired_radius, left_wheel_speed, right_wheel_speed );
  printf("fw will generate left right wheel speed as [%d mm/sec | %d mm/sec] \n", left_wheel_speed, right_wheel_speed );

  double err_l, err_r;
  for( vx=-0.3;vx<0.3; vx+=0.001 )
  {
          for( wz=-0.5;wz<0.5; wz += 0.1 )
          {
                  velocityCommand(vx,wz);
                  fw.update( speed, radius, left_wheel_speed, right_wheel_speed );
                  fw.getWheelVelocity( vx, wz, left_wheel_speed_in_meter, right_wheel_speed_in_meter );

                  err_l = left_wheel_speed - left_wheel_speed_in_meter*1000.0;
                  err_r = right_wheel_speed - right_wheel_speed_in_meter*1000.0;

                  if( fabs(err_l) > 2.5 || fabs(err_r) > 2.5 )
                  {
                          printf("something wrong [%f,%f] --> [%f,%f] [%d,%d][%f,%f]\n", vx, wz, speed, radius,
                                  left_wheel_speed, right_wheel_speed,
                                  left_wheel_speed_in_meter*1000.0, right_wheel_speed_in_meter*1000.0 );
                  }
          }
  }

  for( vx=-0.3;vx<0.3; vx+=0.001 )
  {
          for( wz=-0.5;wz<0.5; wz += 0.1 )
          {
                  fw.getWheelVelocity( vx, wz, left_wheel_speed_in_meter, right_wheel_speed_in_meter );
                  fw.getSR( left_wheel_speed_in_meter*1000, right_wheel_speed_in_meter*1000, desire_speed, desired_radius );
                  fw.update( desire_speed, desired_radius, left_wheel_speed, right_wheel_speed );


                  err_l = left_wheel_speed - left_wheel_speed_in_meter*1000.0;
                  err_r = right_wheel_speed - right_wheel_speed_in_meter*1000.0;

                  if( fabs(err_l) > 2.5 || fabs(err_r) > 2.5 )
                  {
                          printf("something wrong [%f,%f] --> [%f,%f] [%d,%d][%f,%f]\n", vx, wz, speed, radius,
                                  left_wheel_speed, right_wheel_speed,
                                  left_wheel_speed_in_meter*1000.0, right_wheel_speed_in_meter*1000.0 );
                  }
          }
  }

  return 0;
}



///*****************************************************************************
//** Main
//*****************************************************************************/
//
//int main(int argc, char **argv) {
//
//  std::cout << "Hello Dude" << std::endl;
//
//  bias = 0.23;
//  double vx = -0.10;
//  double wz = 0.4;
//  const unsigned int steps = 20;
//  for (unsigned int i = 0; i < steps; ++i ) {
//    velocityCommand(vx,wz);
//    std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
//    vx += 0.2/static_cast<double>(steps);
//  }
//  vx = -0.10;
//  wz = 0.4;
//  for (unsigned int i = 0; i < steps; ++i ) {
//    velocityCommand(vx,wz);
//    std::cout << "[" << vx << "," << wz << "] -> [" << speed << "," << radius << "]" << std::endl;
//    vx += 0.2/static_cast<double>(steps);
//  }
//  return 0;
//}
