/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "L1ControlMath.hpp"

#include <cmath>
#include <mathlib/mathlib.h>

namespace l1_control
{

static constexpr float MIN_CURVATURE_RADIUS = 0.5f;

float calculateCurvatureLateralAcceleration(const matrix::Vector2f &ground_vel,
		const matrix::Vector2f &unit_path_tangent, float track_error, float path_curvature)
{
	if (!std::isfinite(path_curvature) || fabsf(path_curvature) < FLT_EPSILON) {
		return 0.0f;
	}

	const float path_frame_curvature = path_curvature / math::max(1.0f - path_curvature * track_error,
					       fabsf(path_curvature) * MIN_CURVATURE_RADIUS);
	const float tangent_ground_speed = math::max(ground_vel.dot(unit_path_tangent), 0.0f);
	return tangent_ground_speed * tangent_ground_speed * path_frame_curvature;
}

float calculateLateralAcceleration(float ground_speed, float l1_distance, float k_l1, float eta,
				   const matrix::Vector2f &ground_vel, const matrix::Vector2f &unit_path_tangent,
				   float track_error, float path_curvature)
{
	const float l1_distance_limited = math::max(l1_distance, 0.1f);
	const float l1_lateral_acceleration = k_l1 * ground_speed * ground_speed / l1_distance_limited * sinf(eta);
	return l1_lateral_acceleration
	       + calculateCurvatureLateralAcceleration(ground_vel, unit_path_tangent, track_error, path_curvature);
}

} // namespace l1_control
