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

/*------------------------------------------------------------------------------*/
/* Include */
/*------------------------------------------------------------------------------*/
#include "pca9685_ros2/pca9685_component.hpp"
#include <time.h>

/*------------------------------------------------------------------------------*/
/* Constructor */
/*------------------------------------------------------------------------------*/
Controller::Controller()
: Node("controller_node")
{
	RCLCPP_INFO(this->get_logger(), "Initialize");

	controller = new PCA9685(0x40, bus);
	controller_2 = new PCA9685(0x41, bus);
	controller->openPCA9685();
	controller_2->openPCA9685();
  RCLCPP_INFO(this->get_logger(), "PCA9685 Device Address 1: 0x%02X\n",controller->kI2CAddress);
  RCLCPP_INFO(this->get_logger(), "PCA9685 Device Address 2: 0x%02X\n",controller_2->kI2CAddress);
  this->init(controller);
	this->init(controller_2);
  controller->setPWMFrequency(frequency);
	controller_2->setPWMFrequency(frequency);
	sleep(1);
  RCLCPP_INFO(this->get_logger(), "Initialization completed!");

  /* Init parameter */
  memset(rviz_param, 0.0, sizeof(rviz_param));
  memset(offset_param, 0.0, sizeof(offset_param));

  rviz_param[0] = angle_limit/rviz_convert_45;
  /* lf leg-fins */
  rviz_param[1] = angle_limit/rviz_convert_40;
  rviz_param[2] = angle_limit/rviz_convert_40*-1;  /* sig of + Up, - down */
  rviz_param[3] = angle_limit/rviz_convert_45;
  rviz_param[4] = (2*angle_limit)/(2*rviz_convert_45);
  /* lh leg-fins */
  rviz_param[5] = angle_limit/rviz_convert_40*-1;
  rviz_param[6] = angle_limit/rviz_convert_40;
  rviz_param[7] = angle_limit/rviz_convert_45;
  rviz_param[8] = (2*angle_limit)/(2*rviz_convert_45)*-1;  /* sig of + Up, - down */
  /* rf leg-fins */
  rviz_param[9] = angle_limit/rviz_convert_40;
  rviz_param[10] = angle_limit/rviz_convert_40;
  rviz_param[11] = angle_limit/rviz_convert_45;
  rviz_param[12] = (2*angle_limit)/(2*rviz_convert_45);
  /* rh leg-fins */
  rviz_param[13] = angle_limit/rviz_convert_40*-1;
  rviz_param[14] = angle_limit/rviz_convert_40*-1;  /* sig of + Up, - down */
  rviz_param[15] = angle_limit/rviz_convert_45;
  rviz_param[16] = (2*angle_limit)/(2*rviz_convert_45)*-1;

  /* Create Subscription */
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Controller::joint_cb, this, std::placeholders::_1));
  lumen_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "lumen_node", 10, std::bind(&Controller::lumen_cb, this, std::placeholders::_1));
  led_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "led_node", 10, std::bind(&Controller::led_cb, this, std::placeholders::_1));
  dcdriverA_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "dcdriverA_node", 10, std::bind(&Controller::dcdriverA_cb, this, std::placeholders::_1));
  dcdriverB_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "dcdriverB_node", 10, std::bind(&Controller::dcdriverB_cb, this, std::placeholders::_1));
}

/*------------------------------------------------------------------------------*/
/* Destructor */
/*------------------------------------------------------------------------------*/
Controller::~Controller() {
	RCLCPP_INFO(this->get_logger(), "closePWM");
  this->angle(controller_2, 0, 0.0, offset_param[0]);
  this->angle(controller_2, 1, -40.0, 0.0);
  this->angle(controller_2, 2, -40.0, 0.0);
  for (size_t i = 1; i < CHANNEL_SIZE+1; ++i){
    this->angle(controller, i-1, 0.0, offset_param[i]);
  }
  this->init(controller);
	this->init(controller_2);
  controller->closePCA9685();
  controller_2->closePCA9685();
}

/* JointState msg callback */
void Controller::joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg){
  this->angle(controller_2, 0, msg->position[0]*rviz_param[0], offset_param[0]);
  for (size_t i = 1; i < CHANNEL_SIZE+1; ++i){
    if ((i-1)%4 == 0){  // ajust 4n+1 servo ID
      this->angle(controller, i-1, 2*msg->position[i]*rviz_param[i]-angle_limit, offset_param[i]);
    }
    else{
      this->angle(controller, i-1, msg->position[i]*rviz_param[i], offset_param[i]);
    }
  }
}

/* Lumen Light Float32 msg callback */
/* Test Publish /lumen_node topic below: */
/* ros2 topic pub /lumen_node std_msgs/Float32 'data: 50'*/
void Controller::lumen_cb(const std_msgs::msg::Float32::SharedPtr msg){
  this->angle(controller_2, 1, 0.8*msg->data-40.0, 0.0);
}

/* LED Float32 msg callback */
/* Test Publish /led_node topic below: */
/* ros2 topic pub /led_node std_msgs/Float32 'data: 50'*/
void Controller::led_cb(const std_msgs::msg::Float32::SharedPtr msg){
	this->angle(controller_2, 2, 0.8*msg->data-40.0, 0.0);
}

/* DC motor Driver A callback */
void Controller::dcdriverA_cb(const std_msgs::msg::Float32::SharedPtr msg){
	this->dcdriver(controller_2, 3, 4, 4095, true);
    sleep(10);

	this->dcdriver(controller_2, 3, 4, 0, true);
    sleep(2);

	this->dcdriver(controller_2, 3, 4, 4095, false);
    sleep(10);

	this->dcdriver(controller_2, 3, 4, 0, false);
    sleep(2);
}

/* DC motor Driver B callback */
void Controller::dcdriverB_cb(const std_msgs::msg::Float32::SharedPtr msg){
	this->dcdriver(controller_2, 5, 6, 4095, true);
    sleep(10);

	this->dcdriver(controller_2, 5, 6, 0, true);
    sleep(2);

	this->dcdriver(controller_2, 5, 6, 4095, false);
    sleep(10);

	this->dcdriver(controller_2, 5, 6, 0, false);
    sleep(2);
}


void Controller::init(PCA9685 *controller){
    controller->setAllPWM(0, 0);
    controller->reset();
	RCLCPP_INFO(this->get_logger(), "setAllPWM and reset DONE");
}

void Controller::angle(PCA9685 *controller, int channel, float angle, float angle_offset){
    int value = this->map((angle + angle_offset));
    controller->setPWM(channel, 0, value);
}

/*------------------------------------------------------------------------------*/
/**
 * @fn dcdriver
 * @brief DC Driver using PWM
 * @param in_1: Input Pin 1
 * @param in_2: Input Pin 2
 * @param speed: DC Driver Speed from 0 to 4095
 * @param direction: true is CW, false is CCW.
 */
/*------------------------------------------------------------------------------*/
void Controller::dcdriver(PCA9685 *controller, int in_1, int in_2, int speed, bool direction){
	if(direction) { //CW
		RCLCPP_INFO(this->get_logger(), "CW");
		controller->setPWM(in_1, 0, speed);
		controller->setPWM(in_2, 0, 0);
	}
	else { //CCW
		RCLCPP_INFO(this->get_logger(), "CCW");
		controller->setPWM(in_1, 0, 0);
		controller->setPWM(in_2, 0, speed);
	}
}

// private
int Controller::map (int degree) {
    if (degree >= -90 && degree <= 90)
        return ((degree+90) * (servoMax-servoMin)/180 + servoMin);
    else {
	printf("Out of angle range.\n");
	return servoMiddle;
    }
}
