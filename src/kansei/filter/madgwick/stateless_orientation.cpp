// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "kansei/filter/madgwick.hpp"

template<typename T>
static inline void crossProduct(
  T ax, T ay, T az,
  T bx, T by, T bz,
  T & rx, T & ry, T & rz)
{
  rx = ay * bz - az * by;
  ry = az * bx - ax * bz;
  rz = ax * by - ay * bx;
}

template<typename T>
static inline T normalizeVector(T & vx, T & vy, T & vz)
{
  T norm = sqrt(vx * vx + vy * vy + vz * vz);
  T inv = 1.0 / norm;
  vx *= inv;
  vy *= inv;
  vz *= inv;
  return norm;
}

namespace kansei
{

bool StatelessOrientation::compute_orientation(
  WorldFrame frame,
  geometry_msgs::msg::Vector3 A,
  geometry_msgs::msg::Vector3 E,
  geometry_msgs::msg::Quaternion & orientation)
{
  float Hx, Hy, Hz;
  float Mx, My, Mz;
  float normH;

  // A: pointing up
  float Ax = A.x, Ay = A.y, Az = A.z;

  // E: pointing down/north
  float Ex = E.x, Ey = E.y, Ez = E.z;

  // H: vector horizontal, pointing east
  // H = E x A
  crossProduct(Ex, Ey, Ez, Ax, Ay, Az, Hx, Hy, Hz);

  // normalize H
  normH = normalizeVector(Hx, Hy, Hz);
  if (normH < 1E-7) {
    // device is close to free fall (or in space?), or close to
    // magnetic north pole.
    // mag in T => Threshold 1E-7, typical values are  > 1E-5.
    return false;
  }

  // normalize A
  normalizeVector(Ax, Ay, Az);

  // M: vector horizontal, pointing north
  // M = A x H
  crossProduct(Ax, Ay, Az, Hx, Hy, Hz, Mx, My, Mz);

  // Create matrix for basis transformation
  tf2::Matrix3x3 R;
  switch (frame) {
    case NED:
      // vector space world W:
      // Basis: bwx (1,0,0) north, bwy (0,1,0) east, bwz (0,0,1) down
      // vector space local L:
      // Basis: M ,H, -A
      // W(1,0,0) => L(M)
      // W(0,1,0) => L(H)
      // W(0,0,1) => L(-A)

      // R: Transform Matrix local => world equals basis of L, because basis of W is I
      R[0][0] = Mx;
      R[0][1] = Hx;
      R[0][2] = -Ax;
      R[1][0] = My;
      R[1][1] = Hy;
      R[1][2] = -Ay;
      R[2][0] = Mz;
      R[2][1] = Hz;
      R[2][2] = -Az;
      break;

    case NWU:
      // vector space world W:
      // Basis: bwx (1,0,0) north, bwy (0,1,0) west, bwz (0,0,1) up
      // vector space local L:
      // Basis: M ,H, -A
      // W(1,0,0) => L(M)
      // W(0,1,0) => L(-H)
      // W(0,0,1) => L(A)

      // R: Transform Matrix local => world equals basis of L, because basis of W is I
      R[0][0] = Mx;
      R[0][1] = -Hx;
      R[0][2] = Ax;
      R[1][0] = My;
      R[1][1] = -Hy;
      R[1][2] = Ay;
      R[2][0] = Mz;
      R[2][1] = -Hz;
      R[2][2] = Az;
      break;

    default:
    case ENU:
      // vector space world W:
      // Basis: bwx (1,0,0) east, bwy (0,1,0) north, bwz (0,0,1) up
      // vector space local L:
      // Basis: H, M , A
      // W(1,0,0) => L(H)
      // W(0,1,0) => L(M)
      // W(0,0,1) => L(A)

      // R: Transform Matrix local => world equals basis of L, because basis of W is I
      R[0][0] = Hx;
      R[0][1] = Mx;
      R[0][2] = Ax;
      R[1][0] = Hy;
      R[1][1] = My;
      R[1][2] = Ay;
      R[2][0] = Hz;
      R[2][1] = Mz;
      R[2][2] = Az;
      break;
  }

  // Matrix.getRotation assumes vector rotation, but we're using
  // coordinate systems. Thus negate rotation angle (inverse).
  tf2::Quaternion q;
  R.getRotation(q);
  tf2::convert(q.inverse(), orientation);
  return true;
}

bool StatelessOrientation::compute_orientation(
  WorldFrame frame,
  geometry_msgs::msg::Vector3 A,
  geometry_msgs::msg::Quaternion & orientation)
{
  // This implementation could be optimized regarding speed.
  // magnetic Field E must not be parallel to A,
  // choose an arbitrary orthogonal vector
  geometry_msgs::msg::Vector3 E;
  if (fabs(A.x) > 0.1 || fabs(A.y) > 0.1) {
    E.x = A.y;
    E.y = A.x;
    E.z = 0.0;
  } else if (fabs(A.z) > 0.1) {
    E.x = 0.0;
    E.y = A.z;
    E.z = A.y;
  } else {
    // free fall
    return false;
  }

  return compute_orientation(frame, A, E, orientation);
}

}  // namespace kansei
