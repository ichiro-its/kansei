// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "gtest/gtest.h"

#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"

class MeasurementTest : public testing::Test,
  public testing::WithParamInterface<kansei::measurement::MeasurementUnit>
{
protected:
  MeasurementTest()
  : measurement_unit(GetParam())
  {
  }

  kansei::measurement::MeasurementUnit measurement_unit;
};

TEST_P(MeasurementTest, ClassInitialization) {
  keisan::Euler<double> rpy = measurement_unit.get_orientation();

  ASSERT_EQ(0.0, rpy.roll.degree());
  ASSERT_EQ(0.0, rpy.pitch.degree());
  ASSERT_EQ(0.0, rpy.yaw.degree());
}

INSTANTIATE_TEST_CASE_P(
  ClassInitializationTestCase, MeasurementTest, testing::Values(
    kansei::measurement::Filter(), kansei::measurement::MPU("/dev/ttyUSB0")));
