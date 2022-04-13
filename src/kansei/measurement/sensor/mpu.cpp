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
#include <iostream>
#include <string>

#include "kansei/measurement/sensor/mpu.hpp"

#include "keisan/angle.hpp"

#include "errno.h"  // NOLINT
#include "fcntl.h"  // NOLINT
#include "stdio.h"  // NOLINT
#include "string.h"  // NOLINT
#include "termios.h"  // NOLINT
#include "unistd.h"  // NOLINT
#include "sys/stat.h"
#include "sys/ioctl.h"

using namespace keisan::literals;  // NOLINT

namespace kansei::measurement
{

MPU::MPU(const std::string & port_name)
: socket_fd(-1), oreintation_error(0_deg), oreintation_compensation(0_deg)
{
  set_port_name(port_name);
}

MPU::~MPU()
{
  if (socket_fd != -1) {
    close(socket_fd);
  }

  socket_fd = -1;
}

bool MPU::connect()
{
  socket_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
  if (socket_fd > 0) {
    int iFlags = TIOCM_DTR;
    ioctl(socket_fd, TIOCMBIS, &iFlags);

    iFlags = TIOCM_DTR;
    ioctl(socket_fd, TIOCMBIC, &iFlags);

    struct termios newtio;
    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ~ICANON;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VMIN] = 1;

    tcflush(socket_fd, TCIFLUSH);
    tcsetattr(socket_fd, TCSANOW, &newtio);

    return true;
  } else {
    fprintf(stderr, "Can not connect MPU on %s\n", port_name.c_str());
    return false;
  }
}

void MPU::update_rpy()
{
  unsigned char usart_data = 0;
  unsigned char usart_status = 0;
  unsigned char usart_buffer[4];

  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  int count = 0;

  while (true) {
    if (read(socket_fd, &usart_data, 1) <= 0) {
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

      memcpy(&yaw, usart_buffer, 4);
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

      memcpy(&pitch, usart_buffer, 4);
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

      memcpy(&roll, usart_buffer, 4);

      if (!is_calibrated) {
        // validate the value to minimum error about 0.001
        if (fabs(rpy.yaw.degree() - yaw) < 0.001) {
          count++;
        }

        if (count > 50) {
          is_calibrated = true;
          reset_orientation();
        }
      }

      rpy.roll = keisan::make_degree(roll);
      rpy.pitch = keisan::make_degree(pitch);
      rpy.yaw = keisan::make_degree(yaw) + oreintation_error + oreintation_compensation;

      break;
    } else {
      usart_status = 0;
    }
  }
}

void MPU::set_port_name(const std::string & port_name)
{
  this->port_name = port_name;
}

void MPU::reset_orientation()
{
  oreintation_error = -rpy.yaw;
  oreintation_compensation = 0_deg;
}

void MPU::set_orientation_to(const keisan::Angle<double> & target_orientation)
{
  oreintation_compensation = target_orientation;
}

}  // namespace kansei::measurement
