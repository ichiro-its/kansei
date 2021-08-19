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

#include <kansei/mpu.hpp>

#include <keisan/angle.hpp>

#include <cmath>
#include <string>

#include "./fcntl.h"
#include "./termios.h"
#include "./stdio.h"
#include "./unistd.h"
#include "./string.h"
#include "./errno.h"
#include "sys/stat.h"
#include "sys/ioctl.h"

#define MINIMUM_ERROR 0.001

namespace kansei
{

MPU::MPU()
: thread_handler_(true), angle_(0), angle_error_(0)
{
}

void * MPU::threadMethod(void * object)
{
  reinterpret_cast<MPU *>(object)->angleUpdate();
  return 0;
}

void MPU::setup()
{
  angle_ = 0;
  angle_error_ = 0;
  angle_compensation_ = 0;
  calibrated_ = false;

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
}

void MPU::angleUpdate()
{
  unsigned char usart_data = 0;
  unsigned char usart_status = 0;
  unsigned char usart_buffer[4];

  float orientation_angle = 0;
  float pitch_angle = 0;
  float roll_angle = 0;
  int count = 0;

  while (thread_handler_) {
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
        if (fabs(angle_ - orientation_angle) < MINIMUM_ERROR) {
          count++;
        }

        if (count > 50) {
          calibrated_ = true;
          reset();
        }
      }

      pitch_ = pitch_angle;
      roll_ = roll_angle;
      angle_ = orientation_angle;
    } else {
      usart_status = 0;
    }
  }
}

bool MPU::setPortName(const std::string & serial_name)
{
  serial_name_ = serial_name;
  socket_fd_ = open(serial_name_.c_str(), O_RDWR | O_NOCTTY);
  if (socket_fd_ > 0) {
    return true;
  } else {
    fprintf(stderr, "Can not connect MPU on %s\n", serial_name_);
    return false;
  }
}

bool MPU::isPortExist()
{
  struct stat st;
  if (stat("/dev/ttyUSB0", &st) != -1 && S_ISCHR(st.st_mode)) {
    return true;
  } else if (stat("/dev/ttyUSB1", &st) != -1 && S_ISCHR(st.st_mode)) {
    return true;
  }

  fprintf(stderr, "Cannot identify mpu: %d, %s\n", errno, strerror(errno));
  return false;
}

void MPU::start()
{
  setup();
  pthread_create(&thread_, NULL, &threadMethod, this);
}

double MPU::getAngle()
{
  double angle = angle_ + angle_error_ + angle_compensation_;
  return keisan::wrap_deg(angle);
}

double MPU::getPitch()
{
  return pitch_;
}

double MPU::getRoll()
{
  return roll_;
}

void MPU::reset()
{
  angle_error_ = -angle_;
  angle_compensation_ = 0.0;
}

void MPU::setCompensation(double compensation)
{
  angle_compensation_ = compensation;
}

}  // namespace kansei
