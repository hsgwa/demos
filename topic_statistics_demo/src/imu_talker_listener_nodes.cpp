// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <random>
#include <string>

#include "topic_statistics_demo/imu_talker_listener_nodes.hpp"

using namespace std::chrono_literals;

ImuTalker::ImuTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("imu_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period),
  random_generator_(random_number_seed_()),
  random_distribution_(0.0, 1.0) {}

void ImuTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });
}

void ImuTalker::publish()
{
  sensor_msgs::msg::Imu msg;
  // use current time to measure real message age
  msg.header.stamp = this->now();

  RCLCPP_DEBUG(get_logger(), "Publishing header: %lu", msg.header.stamp.nanosec);
  publisher_->publish(msg);
}


ImuListener::ImuListener(
  const std::string & topic_name,
  const rclcpp::SubscriptionOptions & subscription_options)
: Node("imu_listener"),
  subscription_options_(subscription_options),
  topic_name_(topic_name) {}

void ImuListener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void ImuListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        auto now = this->now();
        auto stamp_time = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec,
                                       this->get_clock()->get_clock_type());
        auto message_age = now - stamp_time;
        auto message_age_ms = 1.0e-6*message_age.nanoseconds();

        RCLCPP_INFO(get_logger(), "Listener heard: %lu %lu. message_age_ms: %lf",
                    msg->header.stamp.sec, msg->header.stamp.nanosec, message_age_ms);

        rclcpp::sleep_for(100ms);
      },
      subscription_options_);
  }
}
