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

#include "kansei/fallen/fallen.hpp"
#include "keisan/keisan.hpp"

class FallenTest : public testing::Test
{
protected:
  FallenTest()
  : fallen_determinant(kansei::fallen::FallenDeterminant("accelero"))
  {
  }

  kansei::fallen::FallenDeterminant fallen_determinant;
};

TEST_F(FallenTest, ClassInitialization) {
  ASSERT_EQ(kansei::fallen::FallenStatus::STANDUP, fallen_determinant.get_fallen_status());
}

struct fallen_status_args
{
  keisan::Vector<3> acc;
  kansei::fallen::FallenStatus status;
};

class FallenStatusTest : public FallenTest,
  public testing::WithParamInterface<fallen_status_args>
{
protected:
  FallenStatusTest()
  {
    fallen_determinant.update_fallen_status(GetParam().acc);
  }
};

TEST_P(FallenStatusTest, StatusUpdate) {
  ASSERT_TRUE(fallen_determinant.is_fallen());
  ASSERT_EQ(GetParam().status, fallen_determinant.get_fallen_status());
}

INSTANTIATE_TEST_CASE_P(
  StatusUpdateTestCase, FallenStatusTest, testing::Values(
    fallen_status_args {keisan::Vector<3>(510.0, 450.0, 0.0),
      kansei::fallen::FallenStatus::FORWARD},
    fallen_status_args {keisan::Vector<3>(510.0, 500.0, 0.0),
      kansei::fallen::FallenStatus::BACKWARD},
    fallen_status_args {keisan::Vector<3>(530.0, 470.0, 0.0),
      kansei::fallen::FallenStatus::RIGHT},
    fallen_status_args {keisan::Vector<3>(490.0, 470.0, 0.0),
      kansei::fallen::FallenStatus::LEFT}));
