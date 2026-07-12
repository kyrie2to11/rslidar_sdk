/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved
*********************************************************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rs_driver/driver/driver_param.hpp>

namespace robosense
{
namespace lidar
{

struct ImuExtrinsicsQuaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct ImuExtrinsicsVector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImuExtrinsicsTransform
{
  ImuExtrinsicsVector3 translation;
  ImuExtrinsicsQuaternion rotation;
};

namespace detail
{

struct Matrix3
{
  double m[3][3];
};

inline ImuExtrinsicsQuaternion normalizedQuaternion(
  double x, double y, double z, double w)
{
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm <= 0.0)
  {
    return ImuExtrinsicsQuaternion{};
  }

  ImuExtrinsicsQuaternion q;
  q.x = x / norm;
  q.y = y / norm;
  q.z = z / norm;
  q.w = w / norm;
  return q;
}

inline Matrix3 quaternionToMatrix(const ImuExtrinsicsQuaternion& q)
{
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  return Matrix3{{
    {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)},
    {2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)},
    {2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}
  }};
}

inline Matrix3 transpose(const Matrix3& matrix)
{
  return Matrix3{{
    {matrix.m[0][0], matrix.m[1][0], matrix.m[2][0]},
    {matrix.m[0][1], matrix.m[1][1], matrix.m[2][1]},
    {matrix.m[0][2], matrix.m[1][2], matrix.m[2][2]}
  }};
}

inline Matrix3 applyNedToFluRows(const Matrix3& matrix)
{
  return Matrix3{{
    {matrix.m[0][0], matrix.m[0][1], matrix.m[0][2]},
    {-matrix.m[1][0], -matrix.m[1][1], -matrix.m[1][2]},
    {-matrix.m[2][0], -matrix.m[2][1], -matrix.m[2][2]}
  }};
}

inline ImuExtrinsicsVector3 multiply(
  const Matrix3& matrix, const ImuExtrinsicsVector3& vector)
{
  ImuExtrinsicsVector3 result;
  result.x = matrix.m[0][0] * vector.x + matrix.m[0][1] * vector.y + matrix.m[0][2] * vector.z;
  result.y = matrix.m[1][0] * vector.x + matrix.m[1][1] * vector.y + matrix.m[1][2] * vector.z;
  result.z = matrix.m[2][0] * vector.x + matrix.m[2][1] * vector.y + matrix.m[2][2] * vector.z;
  return result;
}

inline ImuExtrinsicsQuaternion matrixToQuaternion(const Matrix3& matrix)
{
  ImuExtrinsicsQuaternion q;
  const double trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];

  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (matrix.m[2][1] - matrix.m[1][2]) / s;
    q.y = (matrix.m[0][2] - matrix.m[2][0]) / s;
    q.z = (matrix.m[1][0] - matrix.m[0][1]) / s;
  }
  else if (matrix.m[0][0] > matrix.m[1][1] && matrix.m[0][0] > matrix.m[2][2])
  {
    const double s = std::sqrt(1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) * 2.0;
    q.w = (matrix.m[2][1] - matrix.m[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (matrix.m[0][1] + matrix.m[1][0]) / s;
    q.z = (matrix.m[0][2] + matrix.m[2][0]) / s;
  }
  else if (matrix.m[1][1] > matrix.m[2][2])
  {
    const double s = std::sqrt(1.0 + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) * 2.0;
    q.w = (matrix.m[0][2] - matrix.m[2][0]) / s;
    q.x = (matrix.m[0][1] + matrix.m[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (matrix.m[1][2] + matrix.m[2][1]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) * 2.0;
    q.w = (matrix.m[1][0] - matrix.m[0][1]) / s;
    q.x = (matrix.m[0][2] + matrix.m[2][0]) / s;
    q.y = (matrix.m[1][2] + matrix.m[2][1]) / s;
    q.z = 0.25 * s;
  }

  return normalizedQuaternion(q.x, q.y, q.z, q.w);
}

}  // namespace detail

inline ImuExtrinsicsTransform lidarToPublishedImuTransformFromDifop(
  const DeviceInfo& info, bool imu_ned_to_flu)
{
  const auto difop_q = detail::normalizedQuaternion(info.qx, info.qy, info.qz, info.qw);
  auto imu_from_lidar_rotation = detail::quaternionToMatrix(difop_q);
  ImuExtrinsicsVector3 imu_from_lidar_translation{info.x, info.y, info.z};

  if (imu_ned_to_flu)
  {
    imu_from_lidar_rotation = detail::applyNedToFluRows(imu_from_lidar_rotation);
    imu_from_lidar_translation.y = -imu_from_lidar_translation.y;
    imu_from_lidar_translation.z = -imu_from_lidar_translation.z;
  }

  const auto lidar_from_imu_rotation = detail::transpose(imu_from_lidar_rotation);
  const auto rotated_translation =
    detail::multiply(lidar_from_imu_rotation, imu_from_lidar_translation);

  ImuExtrinsicsTransform transform;
  transform.translation.x = -rotated_translation.x;
  transform.translation.y = -rotated_translation.y;
  transform.translation.z = -rotated_translation.z;
  transform.rotation = detail::matrixToQuaternion(lidar_from_imu_rotation);
  return transform;
}

}  // namespace lidar
}  // namespace robosense
