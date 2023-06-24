/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yusuke Mizoguchi
   Desc: TODO(GITHUB_NAME):
*/
#include <wrench2thrust/wrench2thrust.hpp>

using std::placeholders::_1;

namespace wrench2thrust_ns
{

wrench2thrust::wrench2thrust(const rclcpp::NodeOptions& options) : Node("wrench2thrust", options)
{
  RCLCPP_INFO(get_logger(), "Initializing wrench2thrust...");
  wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "wamv/wrench", 1, std::bind(&wrench2thrust::sub_callback, this, _1));
  //  left_thrust_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust/data", 10);
  //  timer_ = this->create_wall_timer(100ms, std::bind(&wrench2thrust::timer_callback, this));
}

void wrench2thrust::sub_callback(const geometry_msgs::msg::WrenchStamped &msg)
{
  RCLCPP_INFO(get_logger(),"%f",msg.wrench.force.x);
}

void wrench2thrust::timer_callback()
{
  auto msg_left_thrust = std_msgs::msg::Float64();
  msg_left_thrust.data = 0.40;
  left_thrust_pub->publish(msg_left_thrust);
  RCLCPP_INFO(get_logger(), "now publishing message");
}

}  // namespace wrench2thrust_ns
