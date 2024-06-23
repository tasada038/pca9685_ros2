// MIT License

// Copyright (c) 2024 Takumi Asada
// Copyright (c) 2019 Pushkal Katara

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// https://github.com/jetsonhacks/JHPWMDriver
// https://github.com/DiamondSheep/Servo_driver

/*------------------------------------------------------------------------------*/
/* Define MACRO */
/*------------------------------------------------------------------------------*/
#ifndef PCA9685_COMPONENT_HPP_
#define PCA9685_COMPONENT_HPP_
#define CHANNEL_SIZE 16

/*------------------------------------------------------------------------------*/
/* Include */
/*------------------------------------------------------------------------------*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pca9685_ros2/JHPWMPCA9685.h>

/*------------------------------------------------------------------------------*/
/* Class */
/*------------------------------------------------------------------------------*/
class Controller : public rclcpp::Node {
public:
	Controller();
	~Controller();

    // Calibrated for SG-90 servo (top of the pan-tilt) and MG-90S servo (bottom of the pan-tilt)
    int servoMin = 110; // 150 for the SG-90 on the top of the pan-tilt
    int servoMax = 490; // 336 for the SG-90 on the top of the pan-tilt
    int servoMiddle = 300;
    int bus = 1; // bus 1 (pin 3 and pin 5)
    int frequency = 50; // default 50 Hz

    float angle_limit = 45.0;
    float rviz_convert_40 = 0.698;
    float rviz_convert_45 = 0.785;
    float rviz_param[CHANNEL_SIZE+1] = {};  /* Convert to Joint msg to Rviz 2 param */
    float offset_param[CHANNEL_SIZE+1] = {};  /* Offset parameter of Servo */

    PCA9685 *controller;
    PCA9685 *controller_2;

    void init(PCA9685 *controller);
    void angle(PCA9685 *controller, int channel, float angle, float angle_offset);
    void dcdriver(PCA9685 *controller, int in_1, int in_2, int speed, bool direction);

private:
    // map degrees to the servo value
    int map(int degree);
    /* Callback Function */
    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void lumen_cb(const std_msgs::msg::Float32::SharedPtr msg);
    void led_cb(const std_msgs::msg::Float32::SharedPtr msg);
    void dcdriverA_cb(const std_msgs::msg::Float32::SharedPtr msg);
    void dcdriverB_cb(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lumen_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr led_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dcdriverA_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dcdriverB_sub_;

};

#endif /* PCA9685_COMPONENTR_HPP_ */
