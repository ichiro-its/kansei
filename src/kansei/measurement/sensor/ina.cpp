// Copyright (c) 2025 Ichiro ITS
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

#include "kansei/measurement/sensor/ina.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "errno.h"  // NOLINT
#include "fcntl.h"  // NOLINT
#include "keisan/angle.hpp"
#include "stdio.h"   // NOLINT
#include "string.h"  // NOLINT
#include "sys/ioctl.h"
#include "sys/stat.h"
#include "termios.h"  // NOLINT
#include "unistd.h"   // NOLINT

using namespace keisan::literals;  // NOLINT

namespace kansei::measurement
{

INA::INA(const std::string & port_name)
: socket_fd(-1)
{
  set_port_name(port_name);
}

INA::~INA()
{
  if (socket_fd != -1) {
    close(socket_fd);
  }

  socket_fd = -1;
}

bool INA::connect()
{
  socket_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
  if (socket_fd > 0) {
    int iFlags = TIOCM_DTR;
    ioctl(socket_fd, TIOCMBIS, &iFlags);

    iFlags = TIOCM_DTR;
    ioctl(socket_fd, TIOCMBIC, &iFlags);

    struct termios newtio;
    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag &= ~ICANON;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VMIN] = 1;

    tcflush(socket_fd, TCIFLUSH);
    tcsetattr(socket_fd, TCSANOW, &newtio);

    pthread_create(&thread, NULL, &start, this);

    return true;
  } else {
    fprintf(stderr, "Can not connect INA on %s\n", port_name.c_str());
    return false;
  }
}

void INA::update_voltage()
{
  unsigned char usart_data = 0;
  unsigned char usart_status = 0;
  unsigned char usart_buffer[4];

  float pc_vol = 0;
  float pc_cur = 0;
  float servo_vol = 0;
  float servo_cur = 0;

  while (true) {
    if (read(socket_fd, &usart_data, 1) <= 0) {
      continue;
    }

    if (usart_status == 0 && usart_data == 'i') {
      usart_status++;
    } else if (usart_status == 1 && usart_data == 'c') {
      usart_status++;
    } else if (usart_status == 2 && usart_data == 'h') {
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

      memcpy(&pc_vol, usart_buffer, sizeof(pc_vol));
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

      memcpy(&pc_cur, usart_buffer, sizeof(pc_cur));
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

      memcpy(&servo_vol, usart_buffer, sizeof(servo_vol));
    } else if (usart_status == 17 && usart_data == ':') {
      usart_status++;
    } else if (usart_status == 18) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 19) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 20) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 21) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&servo_cur, usart_buffer, sizeof(servo_cur));

      pc_voltage = pc_vol;
      pc_current = pc_cur;
      servo_voltage = servo_vol;
      servo_current = servo_cur;
    } else {
      usart_status = 0;
    }
  }
}

void * INA::start(void * object)
{
  reinterpret_cast<INA *>(object)->update_voltage();

  return 0;
}

void INA::set_port_name(const std::string & port_name) { this->port_name = port_name; }

}  // namespace kansei::measurement
