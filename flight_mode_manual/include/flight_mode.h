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
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <cassert>

using namespace std::chrono_literals;
using namespace px4_sdk;

static const char *name = "My Manual Mode";
static const char *node_name = "flight_mode_manual";

static inline Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
{
	// YPR is ZYX axes
	return Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}

static inline Eigen::Quaterniond quaternion_from_euler(const double roll, const double pitch, const double yaw)
{
	return quaternion_from_euler(Eigen::Vector3d(roll, pitch, yaw));
}

static inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
	// YPR is ZYX axes
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

static inline void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
	const auto euler = quaternion_to_euler(q);
	roll = euler.x();
	pitch = euler.y();
	yaw = euler.z();
}


class FlightModeTest : public ModeBase
{
public:
	FlightModeTest(rclcpp::Node &node)
		: ModeBase(node, Settings{name}, ModeRequirements::manualControlledPosition())
	{
		_manual_control_setpoint_sub = node.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
						       "/fmu/out/manual_control_setpoint", rclcpp::QoS(1).best_effort(),
		[this, &node](px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
			_manual_control_setpoint = *msg;
			_last_manual_control_setpoint = node.get_clock()->now();
		});
		_vehicle_rates_setpoint_pub = node.create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
						      "/fmu/in/vehicle_rates_setpoint", 1);
		_vehicle_attitude_setpoint_pub = node.create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
				"/fmu/in/vehicle_attitude_setpoint", 1);

		setSetpointUpdateRate(60.f);
		_last_manual_control_setpoint = node.get_clock()->now();
	}

	virtual ~FlightModeTest() {}

	void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) override
	{
		rclcpp::Time now = node().get_clock()->now();

		if (now - _last_manual_control_setpoint > rclcpp::Duration::from_seconds(1) || !_manual_control_setpoint.valid) {
			// TODO: mode requirement for RC
			/* EVENT
			 */
			reporter.armingCheckFailureExt(events::ID("check_custom_mode_no_rc"),
						       events::Log::Error, "No manual control input");
		}
	}

	void onActivate() override
	{
		_activation_time = node().get_clock()->now();
		_last_update = _activation_time;

		_config.manual_enabled = false;
		_config.auto_enabled = false;
		_config.rates_enabled = true;
		_config.attitude_enabled = true;
		_config.acceleration_enabled = false;
		_config.velocity_enabled = false;
		_config.position_enabled = false;
		_config.altitude_enabled = false;
		setpoints().configureSetpointsSync(_config);
	}

	void onDeactivate() override {}

	void updateSetpoint() override
	{
		rclcpp::Time now = node().get_clock()->now();

		const float threshold = 0.9f;
		bool want_rates = fabs(_manual_control_setpoint.x) > threshold || fabsf(_manual_control_setpoint.y) > threshold;

		if (_config.attitude_enabled == want_rates) {
			_config.attitude_enabled = !want_rates;
			setpoints().configureSetpointsSync(_config);
		}

		float dt = (now - _last_update).seconds();

		float yaw_rate = _manual_control_setpoint.r * 120.f * M_PI / 180.f;

		if (want_rates) {
			px4_msgs::msg::VehicleRatesSetpoint sp{};
			sp.thrust_body[2] = -_manual_control_setpoint.z;
			sp.yaw = yaw_rate;
			sp.roll = _manual_control_setpoint.y * 500.f * M_PI / 180.f;
			sp.pitch = -_manual_control_setpoint.x * 500.f * M_PI / 180.f;
			sp.timestamp = node().get_clock()->now().nanoseconds() / 1000;
			_vehicle_rates_setpoint_pub->publish(sp);

		} else {
			_yaw += yaw_rate * dt;
			px4_msgs::msg::VehicleAttitudeSetpoint sp{};
			sp.thrust_body[2] = -_manual_control_setpoint.z;
			sp.yaw_sp_move_rate = yaw_rate;
			Eigen::Quaterniond qd = quaternion_from_euler(
							_manual_control_setpoint.y * 55.f * M_PI / 180.f,
							-_manual_control_setpoint.x * 55.f * M_PI / 180.f,
							_yaw
						);
			sp.q_d[0] = qd.w();
			sp.q_d[1] = qd.x();
			sp.q_d[2] = qd.y();
			sp.q_d[3] = qd.z();
			sp.timestamp = node().get_clock()->now().nanoseconds() / 1000;
			_vehicle_attitude_setpoint_pub->publish(sp);
		}

		_last_update = now;
	}

private:
	rclcpp::Time _activation_time{};
	rclcpp::Time _last_update{};
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_setpoint_sub;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr _vehicle_attitude_setpoint_pub;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _vehicle_rates_setpoint_pub;
	px4_msgs::msg::ManualControlSetpoint _manual_control_setpoint{};
	rclcpp::Time _last_manual_control_setpoint{};
	float _yaw{0.f};
	SetpointSender::SetpointConfiguration _config{};
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

