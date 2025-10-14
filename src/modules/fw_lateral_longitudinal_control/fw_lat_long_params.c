/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * L1 Period
 *
 * The L1 period is the characteristic time constant of the L1 guidance law.
 * Higher values result in a more damped response but slower convergence.
 *
 * @unit s
 * @min 10.0
 * @max 50.0
 * @decimal 1
 * @increment 1.0
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 25.0f);

/**
 * L1 Damping
 *
 * The L1 damping ratio controls the response characteristics of the L1 guidance law.
 * Higher values result in a more damped response.
 *
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_DAMPING, 0.75f);

/**
 * L1 Roll Limit
 *
 * Maximum roll angle for L1 guidance law.
 *
 * @unit deg
 * @min 10.0
 * @max 60.0
 * @decimal 1
 * @increment 5.0
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_ROLL_LIM, 30.0f);

/**
 * L1 Roll Slew Rate
 *
 * Maximum roll angle slew rate for L1 guidance law.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 5.0
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_ROLL_SLEW, 0.0f);

/**
 * Guidance Mode Selection
 *
 * Select the guidance algorithm for fixed-wing control:
 * 0: L1 Guidance (default, proven algorithm)
 * 1: PID Guidance (precise control)
 * 2: NPFG Guidance (advanced nonlinear guidance)
 *
 * @min 0
 * @max 2
 * @value 0 L1 Guidance
 * @value 1 PID Guidance
 * @value 2 NPFG Guidance
 * @group FW Lateral Control
 */
PARAM_DEFINE_INT32(FW_GUIDANCE_MODE, 0);

/**
 * PID Course Proportional Gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_CRS_KP, 2.0f);

/**
 * PID Course Integral Gain
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_CRS_KI, 0.1f);

/**
 * PID Course Derivative Gain
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_CRS_KD, 0.5f);

/**
 * PID Heading Proportional Gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_HDG_KP, 1.5f);

/**
 * PID Heading Integral Gain
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_HDG_KI, 0.05f);

/**
 * PID Heading Derivative Gain
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_HDG_KD, 0.3f);

/**
 * PID Mode Selection
 *
 * @min 0
 * @max 1
 * @value 0 Standard PID
 * @value 1 Advanced PID
 * @group FW Lateral Control
 */
PARAM_DEFINE_INT32(FW_PID_MODE, 0);
