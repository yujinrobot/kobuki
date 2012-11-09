#include <ros/ros.h>

#include <camera_calibration/pose_estimator.hpp>

/**
 * This class uses the poseEstimator to calculate the pose of a chess board relative to the camera.
 * As the chess board is placed over the kobuki, calling estimate two times and turning in between,
 * we can evaluate the accuracy of current robot's imu.
 *
 * We must provide a xml file that describes the chess board and the camera calibration parameters.
 */
class TestIMU
{
public:
  TestIMU() : cap(0) // Assume we use the first video input
  {
  }

  ~TestIMU()
  {
    delete pose_estimator;
    pose_estimator = NULL;
  }

  bool init(std::string& calib_file)
  {
    if (cap.isOpened() == false)
    {
      ROS_ERROR("Unable to open video input");
      return false;
    }

    try
    {
      cv::FileStorage fs(calib_file, cv::FileStorage::READ);
      if (fs.isOpened() == false)
      {
        ROS_ERROR("Unable to open camera calibration file: %s", calib_file.c_str());
        return false;
      }

      ROS_DEBUG("Reading camera calibration file: %s", calib_file.c_str());
      fs["board_Width"]             >> board_width;
      fs["board_Height"]            >> board_height;
      fs["square_Size"]             >> square_size;
      fs["Camera_Matrix"]           >> cameraMatrix;
      fs["Distortion_Coefficients"] >> distCoeffs;

//    ROS_DEBUG("Camera calibration parameters:");
//    ROS_DEBUG(" board_width :               %d", board_width);
//    ROS_DEBUG(" board_height :              %d", board_height);
//    ROS_DEBUG(" square_size :               %f", square_size);
//    ROS_DEBUG(" Camera_Matrix :             %s", cameraMatrix);
//    ROS_DEBUG(" Distortion_Coefficients :   %s", distCoeffs);

      // Set the properties of camera
      pose_estimator = new poseEstimator(board_width, board_height, square_size, cameraMatrix, distCoeffs);

      std::stringstream ss;
      ss << "Pose estimator created; camera calibration parameters are:"
         << "\n board_width :               " << board_width
         << "\n board_height :              " << board_height
         << "\n square_size :               " << square_size
         << "\n Camera_Matrix :             " << cameraMatrix
         << "\n Distortion_Coefficients :   " << distCoeffs;

      ROS_INFO("%s", ss.str().c_str());
          /*
      ROS_DEBUG_STREAM("Camera calibration parameters:\n"
                    << " board_width :               %d" << board_width
                    << " board_height :              %d" << board_height
                    << " square_size :               %f" << square_size
                    << " Camera_Matrix :             %s" << cameraMatrix
                    << " Distortion_Coefficients :   %s" << distCoeffs);*/
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("Unable to open camera calibration file: %s", calib_file.c_str());
      return false;
    }

    return true;
  }

  double getYaw()
  {
    double avg_yaw = 0.0;
    for (unsigned int i = 0; /* inf. loop */; i++)
    {
      // capture image
      cap >> img;

      // estimation of pose
      if (pose_estimator->estimate(img) == true)
      {
        double yaw = pose_estimator->getYawAngle();
        if (i > 10)  // skip some estimations
          avg_yaw += yaw;

        if (i > 30)  // averate over 20 stimations
        {
          avg_yaw /= 20.0;
          ROS_DEBUG("Pose estimated; yaw angle is %f", avg_yaw);
          return avg_yaw;
        }
      }
      else if (i > 10)
      {
        ROS_ERROR("Pose cannot be estimated. Please place the robot right under the camera");
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

      // wait for key
//      char key_code = waitKey( 10 );
  //    if( key_code == 'q' ) break;
  };

private:
  poseEstimator* pose_estimator;
  int board_width, board_height;
  double square_size;
  cv::Mat cameraMatrix, distCoeffs;

  cv::VideoCapture cap;
  cv::Mat img;
};
