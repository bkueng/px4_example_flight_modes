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
#include <px4_sdk/components/wait_for_fmu.h>

#include <rclcpp/rclcpp.hpp>
#include <cassert>

#include <Eigen/Core>

using namespace std::chrono_literals;
using namespace px4_sdk;

static const char *name = "Autonomous Executor";
static const char *node_name = "flight_mode_test";

class FlightModeTest : public ModeBase
{
public:
	FlightModeTest(rclcpp::Node &node)
		: ModeBase(node, Settings{name, false}, ModeRequirements::autonomous())
	{
		setSetpointUpdateRate(30.f);
	}

	virtual ~FlightModeTest() {}

	void checkArmingAndRunConditions(HealthAndArmingCheckReporter &reporter) override
	{
		bool fail = (++_counter % 40) >= 20;

		if (fail && false) {
			/* EVENT
			 * @description Custom flight mode failure
			 */
			reporter.armingCheckFailureExt(events::ID("check_custom_mode_failure"),
						       events::Log::Error, "Custom check failed");
		}

	}

	void onActivate() override
	{
		_activation_time = node().get_clock()->now();
		setpoints().configureSetpointsSync(SetpointSender::SetpointConfiguration{});
	}

	void onDeactivate() override {}

	void updateSetpoint() override
	{
		rclcpp::Time now = node().get_clock()->now();

		if (now - _activation_time > rclcpp::Duration::from_seconds(5)) {
			completed(Result::Success);
			return;
		}

		float elapsed_s = (now - _activation_time).seconds();
		Eigen::Vector3f velocity{10.f, elapsed_s * 2.f, -2.f };
		setpoints().sendTrajectorySetpoint(velocity);
	}

private:
	int _counter{0};
	rclcpp::Time _activation_time{};
};

class ModeExecutorTest : public ModeExecutorBase
{
public:
	ModeExecutorTest(rclcpp::Node &node, ModeBase &owned_mode)
		: ModeExecutorBase(node, ModeExecutorBase::Settings{false}, owned_mode),
		  _node(node)
	{
	}

	enum class State {
		Reset,
		WaitForArming,
		Arming,
		TakingOff,
		MyMode,
		RTL,
		WaitUntilDisarmed,
		Shutdown,
	};

	void onActivate() override
	{
		runState(State::WaitForArming, Result::Success);
	}

	void onDeactivate(DeactivateReason reason) override
	{

	}

	void runState(State state, Result previous_result)
	{
		if (previous_result != Result::Success) {
			RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
//			assert(previous_result == Result::Success);
			return;
		}

		RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

		switch (state) {
		case State::Reset:
			break;

		case State::WaitForArming:
			waitReadyToArm([this](Result result) { runState(State::Arming, result); });
			break;

		case State::Arming:
			arm([this](Result result) { runState(State::TakingOff, result); });
			break;

		case State::TakingOff:
			takeoff([this](Result result) { runState(State::MyMode, result); });
			break;

		case State::MyMode:
			scheduleMode(ownedMode().id(), [this](Result result) { runState(State::RTL, result); });
			break;

		case State::RTL:
			rtl([this](Result result) { runState(State::WaitUntilDisarmed, result); });
			break;

		case State::WaitUntilDisarmed:
			waitUntilDisarmed([this](Result result) { runState(State::Shutdown, result); });
			break;

		case State::Shutdown:
//			rclcpp::shutdown();
			break;
		}
	}

private:
	rclcpp::Node &_node;
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

		if (!waitForFMU(*this)) {
			throw std::runtime_error("No message from FMU");
		}

		_mode = std::make_unique<FlightModeTest>(*this);
		_mode_executor = std::make_unique<ModeExecutorTest>(*this, *_mode.get());

		if (!_mode_executor->doRegister()) {
			throw std::runtime_error("Registration failed");
		}
	}
private:
	std::unique_ptr<FlightModeTest> _mode;
	std::unique_ptr<ModeExecutorTest> _mode_executor;
};

