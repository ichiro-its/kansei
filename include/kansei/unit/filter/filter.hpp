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

#ifndef KANSEI__UNIT__FILTER__FILTER_HPP_
#define KANSEI__UNIT__FILTER__FILTER_HPP_

#include <memory>
#include <string>

#include "kansei/unit/filter/madgwick.hpp"
#include "kansei/unit/measurement_unit.hpp"
#include "keisan/keisan.hpp"

namespace kansei
{

class Filter : public MeasurementUnit
{
public:
  Filter();

  void update_rpy();

  void reset_orientation();
  void set_orientation_to(const keisan::Angle<double> & target_orientation);
  void set_orientation_raw_to(const keisan::Angle<double> & target_raw_orientation);

  void load_data(std::string path);

private:
  MadgwickFilter filter;
  bool is_initialized;

  keisan::Angle<double> yaw_raw;
  keisan::Vector<3> gy_raw_mux;

  keisan::Angle<double> orientation_compensation;
  keisan::Angle<double> raw_orientation_compensation;
};

}  // namespace kansei

#endif  // KANSEI__UNIT__FILTER__FILTER_HPP_
