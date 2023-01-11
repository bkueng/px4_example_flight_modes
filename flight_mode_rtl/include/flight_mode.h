/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <px4_sdk/components/mode.h>
#include <px4_sdk/components/mode_executor.h>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cassert>

#include <Eigen/Core>

using namespace std::chrono_literals;
using namespace px4_sdk;

static const char *name = "Custom RTL";
static const char *node_name = "flight_mode_rtl";

class FlightModeTest : public ModeBase
{
public:
	FlightModeTest(rclcpp::Node &node)
		: ModeBase(node, Settings{name, true, ModeBase::ID_NAVIGATION_STATE_AUTO_RTL}, ModeRequirements::autonomous())
	{
		_vehicle_land_detected_sub = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
						     "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
		[this](px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
			_landed = msg->landed;
		});
		setSetpointUpdateRate(30.f);
	}

	virtual ~FlightModeTest() {}

	void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) override
	{
	}

	void onActivate() override
	{
		_activation_time = node().get_clock()->now();
		setpoints().configureSetpointsSync(SetpointSender::SetpointConfiguration{});
	}

	void onDeactivate() override {}

	void updateSetpoint() override
	{
		if (_landed) {
			completed(Result::Success);
			return;
		}

		Eigen::Vector3f velocity{0.f, 1.f, 5.f };
		setpoints().sendTrajectorySetpoint(velocity);
	}

private:
	rclcpp::Time _activation_time{};
	bool _landed{true};
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
};

class TestNode : public rclcpp::Node
{
public:
	TestNode() : Node(node_name)
	{
		// Enable debug output
		auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

		if (ret != RCUTILS_RET_OK) {
			RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
			rcutils_reset_error();
		}

		_mode = std::make_unique<FlightModeTest>(*this);

		if (!_mode->doRegister()) {
			throw std::runtime_error("Registration failed");
		}
	}
private:
	std::unique_ptr<FlightModeTest> _mode;
};

