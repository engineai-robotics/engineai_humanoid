#pragma once

#include "Types.h"

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename T>
Eigen::Matrix<T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<T, 3, 1> &eulerAngles)
{
  const T z = eulerAngles(0);
  const T y = eulerAngles(1);
  const T x = eulerAngles(2);

  const T c1 = cos(z);
  const T c2 = cos(y);
  const T c3 = cos(x);
  const T s1 = sin(z);
  const T s2 = sin(y);
  const T s3 = sin(x);

  const T s2s3 = s2 * s3;
  const T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

// template <typename T>
// T square(T a)
// {
//   return a * a;
// }

template <typename T>
Eigen::Matrix<T, 3, 1> quatToZyx(const Eigen::Quaternion<T> &q)
{
  Eigen::Matrix<T, 3, 1> zyx;

  T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

template <typename T>
Eigen::Matrix<T, 3, 1> quatToXyz(const Eigen::Quaternion<T> &q)
{
  Eigen::Matrix<T, 3, 1> xyz;

  // Roll (X-axis rotation)
  T sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  T cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  xyz(0) = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (Y-axis rotation)
  T sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    xyz(1) = std::copysign(M_PI / 2, sinp);
  else
    xyz(1) = std::asin(sinp);

  // Yaw (Z-axis rotation)
  T siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  T cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  xyz(2) = std::atan2(siny_cosp, cosy_cosp);

  return xyz;
}
