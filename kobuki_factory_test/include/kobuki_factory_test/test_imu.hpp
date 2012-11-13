#include <numeric>
#include <functional>

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
  TestIMU() : pose_estimator(NULL)
  {
  }

  ~TestIMU()
  {
    if (pose_estimator != NULL)
    {
      delete pose_estimator;
      pose_estimator = NULL;
    }
  }

  bool init(std::string& calib_file, unsigned int device)
  {
    if (cap.open(device) == false)
    {
      ROS_ERROR("Unable to open video input device %d", device);
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

      // Set the properties of camera
      pose_estimator = new poseEstimator(board_width, board_height, square_size, cameraMatrix, distCoeffs);

      std::stringstream ss;
      ss << "Pose estimator created; camera calibration parameters are:"
         << "\n board_width :               " << board_width
         << "\n board_height :              " << board_height
         << "\n square_size :               " << square_size
         << "\n Camera_Matrix :\n"            << cameraMatrix
         << "\n Distortion_Coefficients :   " << distCoeffs;

      ROS_INFO("%s", ss.str().c_str());
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
    std::vector<double> yaw(10, std::numeric_limits<double>::quiet_NaN());

    for (unsigned int i = 0; /* inf. loop */; i++)
    {
      // capture image
      cap >> img;

      // estimation of pose
      if (pose_estimator->estimate(img) == true)
      {
        if (i < 5)  // skip some estimations
          continue;

        yaw[(i - 5)%yaw.size()] = pose_estimator->getYawAngle();

        if (i >= 5 + yaw.size())  // vector should be full
        {
          if (stdd(yaw) < 0.01)
          {
            double avg_yaw = mean(yaw);
            ROS_DEBUG("Pose estimated; yaw angle is %f (std. dev. is %f)", avg_yaw, stdd(yaw));
            return avg_yaw;
          }
          else
          {
            // Show a warning but anyway take the latest value
            ROS_WARN("Mean = %f; standard deviation = %f; near +pi/-pi?", mean(yaw), stdd(yaw));
            return pose_estimator->getYawAngle();
          }
        }
      }
      else if (i >= 5)
      {
        ROS_ERROR("Pose cannot be estimated. Please place the robot right under the camera");
        return std::numeric_limits<double>::quiet_NaN();
      }
    }
  };

private:
  poseEstimator* pose_estimator;
  int board_width, board_height;
  double square_size;
  cv::Mat cameraMatrix, distCoeffs;

  cv::VideoCapture cap;
  cv::Mat img;

  double mean(const std::vector<double>& v) {
    double t = std::accumulate(v.begin(), v.end(), 0.0);
    return t / v.size();
  }

  double stdd(const std::vector<double>& v) {
    double v_mean = mean(v);
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    return std::sqrt(sq_sum/v.size() - v_mean*v_mean);
  }
};
