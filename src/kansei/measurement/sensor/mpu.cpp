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

#include "kansei/measurement/sensor/mpu.hpp"

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

MPU::MPU(const std::string & port_name)
: socket_fd(-1), orientation_error(0_deg), orientation_compensation(0_deg), raw_orientation(0_deg)
{
  set_port_name(port_name);

  // BNO055 sends Euler angles directly, not ADC-encoded values
  for (int i = 0; i < 3; ++i) {
    filtered_gy[i] = 0.0;
    filtered_acc[i] = 0.0;
  }
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

    pthread_create(&thread, NULL, &start, this);

    return true;
  } else {
    fprintf(stderr, "Can not connect MPU on %s\n", port_name.c_str());
    return false;
  }
}

void MPU::update_rpy()
{
  // τ ≈ 20 ms at 100 Hz sensor rate, matches RL walking loop period
  // If the data is too noisy, increase the τ.
  // IF the RL feels sluggish, decrease the τ.
  constexpr double kAlpha = 0.4;

  unsigned char usart_data = 0;
  unsigned char usart_status = 0;
  unsigned char usart_buffer[4];

  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float start = 0;
  float stop = 0;
  int count = 0;

  float gravity_x = 0;
  float gravity_y = 0;
  float gravity_z = 0;

  float accel_x = 0;
  float accel_y = 0;
  float accel_z = 0;

  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;

  while (true) {
    if (need_update_led) {
      std::string l = std::to_string(led_mode);
      int s = write(socket_fd, l.c_str(), l.size());
      need_update_led = false;
    }

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

      memcpy(&roll, usart_buffer, 4);
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

      memcpy(&pitch, usart_buffer, 4);

      if (!calibrated) {
        // validate the value to minimum error about 0.001
        if (fabs(rpy.yaw.degree() - yaw) < 0.001) {
          count++;
        }

        if (count > 50) {
          calibrated = true;
          reset();
        }
      }

      raw_roll = keisan::make_degree(roll);
      rpy.roll = raw_roll + roll_error + roll_compensation;

      raw_pitch = keisan::make_degree(pitch);
      rpy.pitch = raw_pitch + pitch_error + pitch_compensation;

      raw_orientation = keisan::make_degree(yaw);
      rpy.yaw = raw_orientation + orientation_error + orientation_compensation;

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

      memcpy(&start, usart_buffer, 4);
    } else if (usart_status == 22 && usart_data == ':') {
      usart_status++;
    } else if (usart_status == 23) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 24) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 25) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 26) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&stop, usart_buffer, 4);

      start_button = start;
      stop_button = stop;
    } else if (usart_status == 27 && usart_data == ':') { // GRAVITY X
      usart_status++;
    } else if (usart_status == 28) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 29) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 30) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 31) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&gravity_x, usart_buffer, 4);
    } else if (usart_status == 32 && usart_data == ':') { // GRAVITY Y
      usart_status++;
    } else if (usart_status == 33) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 34) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 35) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 36) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&gravity_y, usart_buffer, 4);
    } else if (usart_status == 37 && usart_data == ':') { // GRAVITY Z
      usart_status++;
    } else if (usart_status == 38) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 39) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 40) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 41) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&gravity_z, usart_buffer, 4);

      gravity.x = gravity_x;
      gravity.y = gravity_y;
      gravity.z = gravity_z;
    } else if (usart_status == 42) {
      if (usart_data == ':') {
        usart_status++;
      } else if (usart_data == 'i') {
        usart_status = 1;
      } else {
        usart_status = 0;
      }
    } else if (usart_status == 43) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 44) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 45) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 46) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&accel_x, usart_buffer, 4);
    } else if (usart_status == 47 && usart_data == ':') { // ACCEL Y
      usart_status++;
    } else if (usart_status == 48) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 49) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 50) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 51) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&accel_y, usart_buffer, 4);
    } else if (usart_status == 52 && usart_data == ':') { // ACCEL Z
      usart_status++;
    } else if (usart_status == 53) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 54) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 55) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 56) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&accel_z, usart_buffer, 4);

      // BNO055: robot_x=-bno_x, robot_y=bno_y, robot_z=-bno_z
      filtered_acc[0] = kAlpha * (-accel_x) + (1.0 - kAlpha) * filtered_acc[0];
      filtered_acc[1] = kAlpha * accel_y    + (1.0 - kAlpha) * filtered_acc[1];
      filtered_acc[2] = kAlpha * (-accel_z) + (1.0 - kAlpha) * filtered_acc[2];
    } else if (usart_status == 57 && usart_data == ':') { // GYRO X
      usart_status++;
    } else if (usart_status == 58) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 59) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 60) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 61) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&gyro_x, usart_buffer, 4);
    } else if (usart_status == 62 && usart_data == ':') { // GYRO Y
      usart_status++;
    } else if (usart_status == 63) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 64) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 65) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 66) {
      usart_buffer[3] = usart_data;
      usart_status++;

      memcpy(&gyro_y, usart_buffer, 4);
    } else if (usart_status == 67 && usart_data == ':') { // GYRO Z
      usart_status++;
    } else if (usart_status == 68) {
      usart_buffer[0] = usart_data;
      usart_status++;
    } else if (usart_status == 69) {
      usart_buffer[1] = usart_data;
      usart_status++;
    } else if (usart_status == 70) {
      usart_buffer[2] = usart_data;
      usart_status++;
    } else if (usart_status == 71) {
      usart_buffer[3] = usart_data;
      usart_status = 0;

      memcpy(&gyro_z, usart_buffer, 4);

      // BNO055: robot_x=-bno_x, robot_y=bno_y, robot_z=-bno_z
      constexpr double DEG2RAD = M_PI / 180.0;
      filtered_gy[0] = kAlpha * (-gyro_x * DEG2RAD) + (1.0 - kAlpha) * filtered_gy[0];
      filtered_gy[1] = kAlpha * (gyro_y  * DEG2RAD) + (1.0 - kAlpha) * filtered_gy[1];
      filtered_gy[2] = kAlpha * (-gyro_z * DEG2RAD) + (1.0 - kAlpha) * filtered_gy[2];
    } else {
      usart_status = 0;
    }
  }
}

void * MPU::start(void * object)
{
  reinterpret_cast<MPU *>(object)->update_rpy();

  return 0;
}

void MPU::set_port_name(const std::string & port_name) { this->port_name = port_name; }

void MPU::reset()
{
  reset_pitch();
  reset_roll();
  reset_orientation();
}

void MPU::reset_pitch()
{
  pitch_error = -raw_pitch;
  pitch_compensation = 0_deg;
}

void MPU::reset_roll()
{
  roll_error = -raw_roll;
  roll_compensation = 0_deg;
}

void MPU::reset_orientation()
{
  orientation_error = -raw_orientation;
  orientation_compensation = 0_deg;
}

void MPU::set_orientation_to(const keisan::Angle<double> & target_orientation)
{
  orientation_error = -raw_orientation;
  orientation_compensation = target_orientation;
}

}  // namespace kansei::measurement
