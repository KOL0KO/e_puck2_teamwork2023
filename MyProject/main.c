//
// MIT License
//
// Copyright (c) 2023 Leander Stephen Desouza
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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <main.h>

// CAMERA
// #include "camera/camera.h"
// #include "camera/dcmi_camera.h"

// LEDS
#include <leds.h>
#include <spi_comm.h>
// PROXIMITY
#include <sensors/proximity.h>
// TOF
#include <sensors/VL53L0X/VL53L0X.h>
// BLUETOOTH
#include <epuck1x/uart/e_uart_char.h>
#include <serial_comm.h>
// MOTORS
#include <motors.h>
// SELECTOR
#include <selector.h>


// Macros
#define MAX_SPEED 750
#define P_THRESHOLD 200
#define T_THRESHOLD 250
#define BACKWARD_THRESHOLD 1000
#define FORWARD_THRESHOLD 100
#define LED_THRESHOLD 100
#define START_SELECTOR 0

// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
uint16_t tof_value = 0;      // tof sensor reading
double left_speed = 0.0;
double right_speed = 0.0;
int left_dir_counter = 0;
int right_dir_counter = 0;

uint8_t *img_buff_ptr;
int8_t cam_error = 0;
uint16_t r = 0, g = 0, b = 0;
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Proximity sensors initialization
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * Function run the initialization process
*/
void init() {
  halInit();
  chSysInit();
  mpu_init();

  // LED Initialization
  clear_leds();
  spi_comm_start();

  // Motor Initialization
  motors_init();

  // Selector Initialization
  serial_start();

  // Proximity Initialization
  messagebus_init(&bus, &bus_lock, &bus_condvar);
  proximity_start(0);
  calibrate_ir();

  // TOF Initialization
  VL53L0X_start();

  // Init camera.
  // if(cam_advanced_config(FORMAT_COLOR, 240, 160, 160, 120, SUBSAMPLING_X4, SUBSAMPLING_X4) < 0) {
  //   cam_error = -1;
  // }
  // dcmi_disable_double_buffering();
  // dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
  // if(dcmi_prepare() < 0) {
  //   cam_error = -2;
  // }
}



char *base64_encode(const unsigned char *data, size_t input_length) {
  size_t output_length = 4 * ((input_length + 2) / 3);
  char *encoded_data = (char *)malloc(output_length + 1);
  if (encoded_data == NULL) {
      return NULL;
  }

  for (size_t i = 0, j = 0; i < input_length;) {
      uint32_t octet_a = i < input_length ? data[i++] : 0;
      uint32_t octet_b = i < input_length ? data[i++] : 0;
      uint32_t octet_c = i < input_length ? data[i++] : 0;

      uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

      encoded_data[j++] = base64_chars[(triple >> 18) & 0x3F];
      encoded_data[j++] = base64_chars[(triple >> 12) & 0x3F];
      encoded_data[j++] = base64_chars[(triple >> 6) & 0x3F];
      encoded_data[j++] = base64_chars[triple & 0x3F];
  }

  while (output_length > 0 && encoded_data[output_length - 1] == '=') {
      output_length--;
  }

  encoded_data[output_length] = '\0';
  return encoded_data;
}


void send_bt_values() {

  // Read camera.
  // if(cam_error == 0) {
  //   spi_comm_suspend();
  //   dcmi_capture_start();
  //   wait_image_ready();
  //   img_buff_ptr = cam_get_last_image_ptr();

  //   // encode using base64
  //   char *base64_str = base64_encode(img_buff_ptr, 76800);
  //   int str_length = strlen(base64_str);

  //   // send image through bluetooth
  //   e_send_uart1_char(base64_str, str_length);



    // send entire image through bluetooth
    // e_send_uart1_char(img_buff_ptr, 76800);

    // // r = (int)img_buff_ptr[0]&0xF8;
    // // g = (int)(img_buff_ptr[0]&0x07)<<5 | (img_buff_ptr[1]&0xE0)>>3;
    // // b = (int)(img_buff_ptr[1]&0x1F)<<3;
    // // dcmi_reset_error();
    // // spi_comm_resume();



    // // send rgb values through bluetooth
    // char rgb_str[100];
    // int str_length;
    // str_length = sprintf(rgb_str, "%d,%d,%d\n", r, g, b);
    // e_send_uart1_char(rgb_str, str_length);


  // send calibrated proximity values through bluetooth
  char prox_str[100];
  int str_length;
  str_length = sprintf(prox_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    get_calibrated_prox(0),
    get_calibrated_prox(1),
    get_calibrated_prox(2),
    get_calibrated_prox(3),
    get_calibrated_prox(4),
    get_calibrated_prox(5),
    get_calibrated_prox(6),
    get_calibrated_prox(7),
    left_motor_get_desired_speed(),
    right_motor_get_desired_speed(),
    get_selector()
  );
  e_send_uart1_char(prox_str, str_length);    // obstacle avoidance

}


/*
 * Function to fill sensor values
*/
void fill_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_values[i] = get_calibrated_prox(i);
  }
  tof_value = VL53L0X_get_dist_mm();
}

/*
 * Function to fill proximity weights
*/
void fill_proximity_weights() {
  for (int i = 0; i < 8; i++) {
    if (proximity_values[i] > P_THRESHOLD) {
      proximity_weights[i] = 1.0;
    } else {
      proximity_weights[i] = 0.0;
    }
  }
}

/*
 * Function to stop the robot
*/
void stop() {
  left_motor_set_speed(0);
  right_motor_set_speed(0);
}

/*
 * Function to move the robot backwards
*/
void move_backward(float speed) {
  left_motor_set_speed(-speed);
  right_motor_set_speed(-speed);
}

/*
 * Function to move the robot forward
*/
void move_forward(float speed) {
  left_motor_set_speed(speed);
  right_motor_set_speed(speed);
}

/*
 * Function to turn the robot left
*/
void turn_left(float speed) {
  left_motor_set_speed(-speed);
  right_motor_set_speed(speed);
}

/*
 * Function to turn the robot right
*/
void turn_right(float speed) {
  left_motor_set_speed(speed);
  right_motor_set_speed(-speed);
}

/*
 * Function to get the last sensor input direction
*/
void get_last_sensor_input_direction() {
  // set counter to 0
  left_dir_counter = 0;
  right_dir_counter = 0;

  if (proximity_weights[4] || proximity_weights[5] ||
    proximity_weights[6] || proximity_weights[7]) {
    left_dir_counter++;
  } else {
    right_dir_counter++;
  }
}

/*
 * Function to glow LEDs
*/
void glow_leds() {
  if((get_calibrated_prox(0) > LED_THRESHOLD) || (get_calibrated_prox(7) > LED_THRESHOLD)) {
    e_set_led(0, 1);
  } else {
    e_set_led(0, 0);
  }

  if(get_calibrated_prox(1) > LED_THRESHOLD) {
    e_set_led(1, 1);
  } else {
    e_set_led(1, 0);
  }

  if(get_calibrated_prox(2) > LED_THRESHOLD) {
    e_set_led(2, 1);
  } else {
    e_set_led(2, 0);
  }

  if(get_calibrated_prox(3) > LED_THRESHOLD) {
    e_set_led(3, 1);
  } else {
    e_set_led(3, 0);
  }

  if((get_calibrated_prox(3) > LED_THRESHOLD) || (get_calibrated_prox(4) > LED_THRESHOLD)) {
    e_set_led(4, 1);
  } else {
    e_set_led(4, 0);
  }

  if(get_calibrated_prox(4) > LED_THRESHOLD) {
    e_set_led(5, 1);
  } else {
    e_set_led(5, 0);
  }

  if(get_calibrated_prox(5) > LED_THRESHOLD) {
    e_set_led(6, 1);
  } else {
    e_set_led(6, 0);
  }

  if(get_calibrated_prox(6) > LED_THRESHOLD) {
    e_set_led(7, 1);
  } else {
    e_set_led(7, 0);
  }
}

/*
 * Function to send a single image through bluetooth
*/
void send_camera_feed_bt() {
}


int main(void) {

  init();

  /* Infinite loop. */
  while (1) {

    glow_leds();

    if (get_selector() != START_SELECTOR) {
      stop();
      chThdSleepMilliseconds(100);
      continue;
    }

    fill_sensor_values();
    fill_proximity_weights();

    if (proximity_values[0] > BACKWARD_THRESHOLD || proximity_values[7] > BACKWARD_THRESHOLD ||
      proximity_values[1] > BACKWARD_THRESHOLD || proximity_values[6] > BACKWARD_THRESHOLD) {

      get_last_sensor_input_direction();
      move_backward(MAX_SPEED/2);

    } else {

        // if tof and no proximity, rotate left till object is detected
        if (tof_value > T_THRESHOLD &&
          !(proximity_weights[5] || proximity_weights[6] || proximity_weights[7] ||
            proximity_weights[0] || proximity_weights[1] || proximity_weights[2])) {

          if (left_dir_counter > right_dir_counter) {
            turn_left(MAX_SPEED/2);
            chThdSleepMilliseconds(50);
          } else {
            turn_right(MAX_SPEED/2);
            chThdSleepMilliseconds(50);
          }

        } else if (proximity_values[0] > FORWARD_THRESHOLD || proximity_values[7] > FORWARD_THRESHOLD && tof_value < T_THRESHOLD) {
            get_last_sensor_input_direction();
            stop();

        } else if (proximity_weights[5] || proximity_weights[6]) {
            get_last_sensor_input_direction();
            // rotate left
            turn_left(MAX_SPEED/2);
            chThdSleepMilliseconds(50);

        } else if (proximity_weights[1] || proximity_weights[2]) {
            get_last_sensor_input_direction();
            // rotate right
            turn_right(MAX_SPEED/2);
            chThdSleepMilliseconds(50);

        } else if (proximity_weights[7] && !proximity_weights[0] && tof_value > T_THRESHOLD) {
            get_last_sensor_input_direction();
            // move left
            turn_left(MAX_SPEED/2);
            chThdSleepMilliseconds(50);

        } else if (proximity_weights[0] && !proximity_weights[7] && tof_value > T_THRESHOLD) {
            get_last_sensor_input_direction();
            // move right
            turn_right(MAX_SPEED/2);
            chThdSleepMilliseconds(50);

        } else {
            get_last_sensor_input_direction();
            // move forward
            move_forward(MAX_SPEED/2);
        }
    }

    // set all sensor weights to 0
    for (int i = 0; i < 8; i++) {
      proximity_weights[i] = 0.0;
    }

    // slow down update rate
    chThdSleepMilliseconds(50);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
