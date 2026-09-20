/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file rate_control.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/rate_ctrl_terms.h>

class RateControl
{
public:
	enum class RollITransferMode : uint8_t {
		TowardZero = 0,
		PairPreserving = 1
	};

	enum class RollITransferLimitReason : uint8_t {
		None = 0,
		Invalid = 1,
		WrongDirection = 2,
		ZeroCrossing = 3,
		IntegratorLimit = 4,
		TotalEquivalentLimit = 5
	};

	struct RollITransferRequest {
		float requested_delta_raw;
		float transferred_i_raw;
		RollITransferMode mode;
		float headroom_release_ratio{0.f};
		float transferred_i_after_raw{0.f};
		bool validate_post_state{false};
	};

	struct RollILimits {
		float lower{0.f};
		float upper{0.f};
		float total_limit{0.f};
		bool valid{false};
	};

	struct RollITransferResult {
		float requested_delta_raw{0.f};
		float accepted_delta_raw{0.f};
		float residual_before_raw{0.f};
		float residual_after_raw{0.f};
		bool valid{false};
		bool limited_at_zero{false};
		bool limited_by_imax{false};
		RollITransferLimitReason reason{RollITransferLimitReason::None};
		uint32_t reset_epoch{0};
	};

	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control PID gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setPidGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/** Apply an explicit Roll integral state transfer in raw-I coordinates. */
	RollITransferResult applyRollITransfer(const RollITransferRequest &request);

	/** Compute residual Roll-I bounds for the transferred state and headroom ratio. */
	static RollILimits computeRollILimits(float imax_raw, float transferred_i_raw,
			float headroom_release_ratio);

	/** Set the transferred raw-I state and effective headroom used by natural Roll integration. */
	void setRollITransferContext(float transferred_i_raw, float headroom_release_ratio, bool enabled)
	{
		_roll_i_transfer_context_raw = transferred_i_raw;
		_roll_i_headroom_release_ratio = headroom_release_ratio;
		_roll_i_transfer_context_enabled = enabled;
	}

	void setRollITransferContext(float transferred_i_raw, bool enabled)
	{
		setRollITransferContext(transferred_i_raw, 0.f, enabled);
	}

	uint32_t rollIntegralResetEpoch() const { return _roll_i_reset_epoch; }
	float rollIntegralRaw() const { return _rate_int(0); }
	float rollIntegralLimit() const { return _lim_int(0); }
	void setRollIntegralLimit(float limit) { _lim_int(0) = limit; }

	/** Joint V4 commit, HR=0. Single owning controller thread only (not a mutex).
	 * Validates BOTH final states and expected snapshot before either write.
	 * No clipping, callbacks, or fallible operations after validation.
	 * Unused by the existing V3/flight path.
	 */
	bool commitRollMemoryPair(float &memory, float expected_memory, float expected_i,
			uint32_t expected_epoch, float expected_limit, float memory_post, float i_post, float memory_limit);

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
				 const matrix::Vector3<bool> &saturation_negative);

	/**
	 * Set individual saturation flags
	 * @param axis 0 roll, 1 pitch, 2 yaw
	 * @param is_saturated value to update the flag with
	 */
	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @param terms optional snapshot of terms used in this output, before updating the integral
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed,
				rate_ctrl_terms_s *terms = nullptr);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral()
	{
		_rate_int.zero();
		_roll_i_reset_epoch++;
		resetRollIntegralDiagnostics();
	}

	/**
	 * Set the integral term to 0 for specific axes
	 * @param  axis roll 0 / pitch 1 / yaw 2
	 * @see _rate_int
	 */
	void resetIntegral(size_t axis)
	{
		if (axis < 3) {
			_rate_int(axis) = 0.f;

			if (axis == 0) {
				_roll_i_reset_epoch++;
				resetRollIntegralDiagnostics();
			}
		}
	}

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	friend struct ResidualSlowFeedforwardMemoryTestAccess;
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);
	void resetRollIntegralDiagnostics();

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller
	float _roll_rate_error{0.f};
	float _roll_i_delta_raw{0.f};
	float _roll_i_delta_pre_imax{0.f};
	float _roll_i_delta_accepted{0.f};
	float _roll_i_shadow_no_imax{0.f};
	float _roll_i_raw_drive_accum{0.f};
	bool _roll_i_update_enabled{false};
	float _roll_i_transfer_context_raw{0.f};
	float _roll_i_headroom_release_ratio{0.f};
	bool _roll_i_transfer_context_enabled{false};
	uint32_t _roll_i_reset_epoch{0};

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
