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

#ifndef KANSEI__LOCOMOTION__NODE__LOCOMOTION_DETERMINANT_HPP_
#define KANSEI__LOCOMOTION__NODE__LOCOMOTION_DETERMINANT_HPP_

#include <memory>
#include <string>

#include "kansei/locomotion/node/locomotion_status.hpp"
#include "kansei/locomotion/determinant/determinant_locomotion.hpp"
#include "keisan/keisan.hpp"

namespace kansei::locomotion
{

class LocomotionDeterminant
{
public:
  explicit LocomotionDeterminant(const DeterminantLocomotion & type);

  void load_config(const std::string & path);

  void update_locomotion_status(const keisan::Euler<double> & rpy);
  void update_locomotion_status(const keisan::Vector<3> & acc);

  bool is_fallen() const;
  FallenStatus get_fallen_status() const;

  DeterminantType get_determinant_type() const;

private:
  LocomotionStatus locomotion_status;

  DeterminantLocomotion data_type;

  // locomotion balance status
  float small_disturbance_comp;
  float large_disturbance_comp;
};

}  // namespace kansei::locomotion

#endif  // KANSEI__LOCOMOTION__NODE__LOCOMOTION_DETERMINANT_HPP_
