// NOLINT(legal/copyright)

#include <stdexcept>

#include "gtest/gtest.h"
#include "aruco_ros/aruco_ros_utils.hpp"

namespace
{
sensor_msgs::msg::CameraInfo makeCameraInfo()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1280;
  info.height = 720;
  info.distortion_model = "plumb_bob";
  info.k = {{906.25, 0.25, 654.5, 0.0, 904.5, 367.25, 0.0, 0.0, 1.0}};
  info.p = {{900.0, 0.0, 640.0, -45.0, 0.0, 901.0, 360.0, 2.0, 0.0, 0.0, 1.0, 0.0}};
  info.d = {0.1, -0.02, 0.003, -0.004};
  return info;
}
}  // namespace

TEST(CameraInfoConversion, RawUsesKInRowMajorOrderAndZeroExtrinsics)
{
  const auto info = makeCameraInfo();
  const auto params = aruco_ros::rosCameraInfo2ArucoCamParams(info, false);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_FLOAT_EQ(params.CameraMatrix.at<float>(row, col), info.k[row * 3 + col]);
    }
  }
  EXPECT_EQ(params.CamSize, cv::Size(1280, 720));
  EXPECT_DOUBLE_EQ(cv::norm(params.ExtrinsicMatrix), 0.0);
}

TEST(CameraInfoConversion, PreservesFourPlumbBobCoefficients)
{
  const auto info = makeCameraInfo();
  const auto params = aruco_ros::rosCameraInfo2ArucoCamParams(info, false);
  ASSERT_EQ(params.Distorsion.total(), info.d.size());
  for (size_t i = 0; i < info.d.size(); ++i) {
    EXPECT_FLOAT_EQ(params.Distorsion.ptr<float>()[i], info.d[i]);
  }
}

TEST(CameraInfoConversion, PreservesAllFiveNonzeroPlumbBobCoefficients)
{
  auto info = makeCameraInfo();
  info.d.push_back(0.005);
  const auto params = aruco_ros::rosCameraInfo2ArucoCamParams(info, false);
  ASSERT_EQ(params.Distorsion.total(), info.d.size());
  for (size_t i = 0; i < info.d.size(); ++i) {
    EXPECT_FLOAT_EQ(params.Distorsion.ptr<float>()[i], info.d[i]);
  }
}

TEST(CameraInfoConversion, PreservesFiveZeroRealSenseCoefficients)
{
  auto info = makeCameraInfo();
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  const auto params = aruco_ros::rosCameraInfo2ArucoCamParams(info, false);
  ASSERT_EQ(params.Distorsion.total(), 5u);
  EXPECT_DOUBLE_EQ(cv::norm(params.Distorsion), 0.0);
}

TEST(CameraInfoConversion, RectifiedUsesPAndZeroDistortionEvenForFisheyeSource)
{
  auto info = makeCameraInfo();
  info.distortion_model = "equidistant";
  const auto params = aruco_ros::rosCameraInfo2ArucoCamParams(info, true);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_FLOAT_EQ(params.CameraMatrix.at<float>(row, col), info.p[row * 4 + col]);
    }
    EXPECT_DOUBLE_EQ(params.ExtrinsicMatrix.at<double>(0, row), info.p[row * 4 + 3]);
  }
  EXPECT_DOUBLE_EQ(cv::norm(params.Distorsion), 0.0);
}

TEST(CameraInfoConversion, RejectsRawFisheyeRatherThanTreatingItAsPlumbBob)
{
  auto info = makeCameraInfo();
  info.distortion_model = "equidistant";
  EXPECT_THROW(aruco_ros::rosCameraInfo2ArucoCamParams(info, false), std::invalid_argument);
}

TEST(CameraInfoConversion, RejectsRawRationalModelRatherThanTruncatingIt)
{
  auto info = makeCameraInfo();
  info.distortion_model = "rational_polynomial";
  info.d = {0.1, -0.02, 0.003, -0.004, 0.005, 0.006, -0.007, 0.008};
  EXPECT_THROW(aruco_ros::rosCameraInfo2ArucoCamParams(info, false), std::invalid_argument);
}

TEST(CameraInfoConversion, RejectsUnknownRawModel)
{
  auto info = makeCameraInfo();
  info.distortion_model = "unknown";
  EXPECT_THROW(aruco_ros::rosCameraInfo2ArucoCamParams(info, false), std::invalid_argument);
}

TEST(CameraInfoConversion, RejectsInvalidPlumbBobCoefficientCounts)
{
  for (const size_t count : {0u, 1u, 3u, 6u, 8u}) {
    SCOPED_TRACE(count);
    auto info = makeCameraInfo();
    info.d.assign(count, 0.01);
    EXPECT_THROW(aruco_ros::rosCameraInfo2ArucoCamParams(info, false), std::invalid_argument);
  }
}
