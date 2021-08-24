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

#include <cmath>
#include <string>

#include "kansei/mpu.hpp"

#include "keisan/angle.hpp"

#include "./errno.h"
#include "./fcntl.h"
#include "./stdio.h"
#include "./string.h"
#include "./termios.h"
#include "./unistd.h"
#include "sys/stat.h"
#include "sys/ioctl.h"

#define MINIMUM_ERROR 0.001

using namespace keisan::literals;  // NOLINT

namespace kansei
{

MPU::MPU(const std::string & port_name)
: rpy(0_deg, 0_deg, 0_deg),
  angle_error(0, true), angle_compensation(0, true),
  calibrated_(false)
{
  set_port_name(port_name);
}

MPU::~MPU()
{
  if (socket_fd_ != -1) {
    close(socket_fd_);
  }
  socket_fd_ = -1;
}

bool MPU::connect()
{
  socket_fd_ = open(serial_name_.c_str(), O_RDWR | O_NOCTTY);
  if (socket_fd_ > 0) {
    int iFlags = TIOCM_DTR;
    ioctl(socket_fd_, TIOCMBIS, &iFlags);

    iFlags = TIOCM_DTR;
    ioctl(socket_fd_, TIOCMBIC, &iFlags);

    struct termios newtio;
    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ~ICANON;

    newtio.c_cc[VINTR] = 0;
    newtio.c_cc[VQUIT] = 0;
    newtio.c_cc[VERASE] = 0;
    newtio.c_cc[VKILL] = 0;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VSWTC] = 0;
    newtio.c_cc[VSTART] = 0;
    newtio.c_cc[VSTOP] = 0;
    newtio.c_cc[VSUSP] = 0;
    newtio.c_cc[VEOL] = 0;
    newtio.c_cc[VREPRINT] = 0;
    newtio.c_cc[VDISCARD] = 0;
    newtio.c_cc[VWERASE] = 0;
    newtio.c_cc[VLNEXT] = 0;
    newtio.c_cc[VEOL2] = 0;

    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);

    return true;
  } else {
    fprintf(stderr, "Can not connect MPU on %s\n", serial_name_.c_str());
    return false;
  }
}

void MPU::angle_update()
{
  unsigned char usart_data = 0;
  unsigned char usart_status = 0;
  unsigned char usart_buffer[4];

  float orientation_angle = 0;
  float pitch_angle = 0;
  float roll_angle = 0;
  int count = 0;

  while (true) {
    if (read(socket_fd_, &usart_data, 1) <= 0) {
      continue;
    }

    if (usart_status == 0 && usart_data == 'i') {
      usart_status++;
    } else if (usart_status == 1 && usart_data == 't') {
      usart_status++;
    } else if (usart_status == 2 && usart_data == 's') {
      usart_status++;
    } else if (usart_status == 3) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 4) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 5) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 6) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&orientation_angle, usart_buffer, 4);
    } else if (usart_status == 7 && usart_data == ':') {
      usart_status++;
    } else if (usart_status == 8) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 9) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 10) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 11) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&pitch_angle, usart_buffer, 4);
    } else if (usart_status == 12 && usart_data == ':') {
      usart_status++;
    } else if (usart_status == 13) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 14) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 15) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 16) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&roll_angle, usart_buffer, 4);

      if (!calibrated_) {
        if (fabs(rpy.yaw.degree() - orientation_angle) < MINIMUM_ERROR) {
          count++;
        }

        if (count > 50) {
          calibrated_ = true;
          reset();
        }
      }

      rpy = keisan::EulerAngles(
        keisan::make_degree<double>(roll_angle),
        keisan::make_degree<double>(pitch_angle),
        keisan::make_degree<double>(orientation_angle));

      break;
    } else {
      usart_status = 0;
    }
  }
}

void MPU::set_port_name(const std::string & port_name)
{
  serial_name_ = port_name;
}

double MPU::get_angle()
{
  auto angle = rpy.yaw + angle_error + angle_compensation;
  return angle.normalize().degree();
}

double MPU::get_pitch()
{
  return rpy.pitch.degree();
}

double MPU::get_roll()
{
  return rpy.roll.degree();
}

void MPU::reset()
{
  angle_error = -rpy.yaw;
  angle_compensation = 0_deg;
}

void MPU::set_compensation(double compensation)
{
  angle_compensation = keisan::make_degree<double>(compensation);
}

}  // namespace kansei
