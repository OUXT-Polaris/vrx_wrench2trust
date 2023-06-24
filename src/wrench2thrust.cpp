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

namespace wrench2thrust_ns {

    wrench2thrust::wrench2thrust(const rclcpp::NodeOptions &options) : Node("vrx_wrench2thrust", options) {
        RCLCPP_INFO(get_logger(), "Initializing wrench2thrust...");
        wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "wamv/wrench", 1, std::bind(&wrench2thrust::sub_callback, this, _1));
        left_prop_thrust_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        left_prop_pos_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 10);
        right_prop_thrust_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        right_prop_pos_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 10);
    }

    void wrench2thrust::sub_callback(const geometry_msgs::msg::WrenchStamped &msg) {
        double wamv_props_angle = atan2(msg.wrench.force.y, msg.wrench.force.x);
        double wamv_props_thrust = sqrt(pow(msg.wrench.force.x, 2) + pow(msg.wrench.force.y, 2));
        RCLCPP_INFO(get_logger(), "angle(rad)is %f,\t angle(deg) is %f,\t thrust is %f", wamv_props_angle,
                    wamv_props_angle / M_PI * 180.0, wamv_props_thrust);
        auto prop_angle_msg = std_msgs::msg::Float64();
        auto prop_thrust_msg = std_msgs::msg::Float64();
        double prop_angle = wamv_props_angle;
        prop_angle_msg.data = prop_angle;
        prop_thrust_msg.data = wamv_props_thrust / 2;
        RCLCPP_DEBUG(get_logger(), "prop ang is %f", prop_angle);
        left_prop_pos_pub->publish(prop_angle_msg);
        right_prop_pos_pub->publish(prop_angle_msg);
        left_prop_thrust_pub->publish(prop_thrust_msg);
        right_prop_thrust_pub->publish(prop_thrust_msg);
    }

    void wrench2thrust::timer_callback() {
        auto msg_left_thrust = std_msgs::msg::Float64();
        msg_left_thrust.data = 0.40;
        left_prop_thrust_pub->publish(msg_left_thrust);
        RCLCPP_INFO(get_logger(), "now publishing message");
    }

}  // namespace wrench2thrust_ns
