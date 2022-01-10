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

#ifndef KANSEI__MEASUREMENT__FILTER__MADGWICK__STATELESS_ORIENTATION_HPP_
#define KANSEI__MEASUREMENT__FILTER__MADGWICK__STATELESS_ORIENTATION_HPP_

#include "kansei/measurement/filter/madgwick/world_frame.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace kansei
{

namespace measurement
{

class StatelessOrientation
{
public:
  static bool compute_orientation(
    WorldFrame frame,
    geometry_msgs::msg::Vector3 acceleration,
    geometry_msgs::msg::Vector3 magneticField,
    geometry_msgs::msg::Quaternion & orientation);

  static bool compute_orientation(
    WorldFrame frame,
    geometry_msgs::msg::Vector3 acceleration,
    geometry_msgs::msg::Quaternion & orientation);
};

}  // namespace measurement

}  // namespace kansei

#endif  // KANSEI__MEASUREMENT__FILTER__MADGWICK__STATELESS_ORIENTATION_HPP_
