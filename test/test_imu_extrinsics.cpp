#include "source/imu_extrinsics.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace robosense
{
namespace lidar
{
namespace
{

constexpr double kTolerance = 1e-9;

DeviceInfo makeDeviceInfo(
  double qx, double qy, double qz, double qw,
  double x, double y, double z)
{
  DeviceInfo info;
  info.state = true;
  info.qx = static_cast<float>(qx);
  info.qy = static_cast<float>(qy);
  info.qz = static_cast<float>(qz);
  info.qw = static_cast<float>(qw);
  info.x = static_cast<float>(x);
  info.y = static_cast<float>(y);
  info.z = static_cast<float>(z);
  return info;
}

TEST(ImuExtrinsics, InvertsRawDifopImuFromLidar)
{
  const auto info = makeDeviceInfo(0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0);

  const auto tf = lidarToPublishedImuTransformFromDifop(info, false);

  EXPECT_NEAR(tf.rotation.x, 0.0, kTolerance);
  EXPECT_NEAR(tf.rotation.y, 0.0, kTolerance);
  EXPECT_NEAR(tf.rotation.z, 0.0, kTolerance);
  EXPECT_NEAR(tf.rotation.w, 1.0, kTolerance);
  EXPECT_NEAR(tf.translation.x, -1.0, kTolerance);
  EXPECT_NEAR(tf.translation.y, -2.0, kTolerance);
  EXPECT_NEAR(tf.translation.z, -3.0, kTolerance);
}

TEST(ImuExtrinsics, AppliesNedToFluBeforeInverting)
{
  const double half_sqrt = std::sqrt(0.5);
  const auto info = makeDeviceInfo(0.0, 0.0, half_sqrt, half_sqrt, 1.0, 2.0, 3.0);

  const auto tf = lidarToPublishedImuTransformFromDifop(info, true);

  EXPECT_NEAR(tf.translation.x, -2.0, kTolerance);
  EXPECT_NEAR(tf.translation.y, 1.0, kTolerance);
  EXPECT_NEAR(tf.translation.z, -3.0, kTolerance);
  EXPECT_NEAR(std::abs(tf.rotation.x), half_sqrt, kTolerance);
  EXPECT_NEAR(std::abs(tf.rotation.y), half_sqrt, kTolerance);
  EXPECT_NEAR(tf.rotation.z, 0.0, kTolerance);
  EXPECT_NEAR(tf.rotation.w, 0.0, kTolerance);
}

}  // namespace
}  // namespace lidar
}  // namespace robosense
