/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file rpm_pid_params.c
 *
 * Parameters for flapping-wing RPM / frequency controller.
 */

/**
 * Minimum flapping frequency.
 *
 * This is the frequency (Hz) corresponding to throttle = 0.
 *
 * @unit Hz
 * @min 0.0
 * @max 10.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_F_MIN, 0.0f);

/**
 * Maximum flapping frequency.
 *
 * This is the frequency (Hz) corresponding to throttle = 1.
 *
 * @unit Hz
 * @min 0.1
 * @max 20.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_F_MAX, 5.0f);

/**
 * Gear ratio: motor revs per wing flap.
 *
 * Motor RPM / (flapping frequency in Hz * 60).
 *
 * @min 0.1
 * @max 100.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_RATIO, 7.5f);

/**
 * RPM controller P gain.
 *
 * Proportional gain from RPM error to normalized motor command.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_KP, 0.001f);

/**
 * RPM controller I gain.
 *
 * Integral gain from RPM error to normalized motor command.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_KI, 0.0f);

/**
 * RPM controller D gain.
 *
 * Derivative gain from RPM error to normalized motor command.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 5
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_KD, 0.0f);

/**
 * RPM controller integrator limit.
 *
 * Absolute limit for the integral term (RPM units).
 *
 * @min 0.0
 * @max 10000.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_I_MAX, 500.0f);

/**
 * Glide stop throttle threshold.
 *
 * Below this normalized throttle, glide stop logic may engage.
 *
 * @min 0.0
 * @max 0.2
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_GLIDE_THR, 0.03f);

/**
 * Glide stop hold command.
 *
 * Minimum motor command to hold while waiting for phase alignment.
 *
 * @min 0.0
 * @max 0.5
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_GLIDE_HOLD, 0.05f);

/**
 * Glide stop phase tolerance [deg].
 *
 * Stop when wing phase is within this tolerance of target (0 or 180 deg).
 *
 * @min 1.0
 * @max 20.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_GLIDE_TOL, 5.0f);

/**
 * Glide stop timeout [s].
 *
 * If phase is not reached within this time, stop motor anyway.
 *
 * @min 0.1
 * @max 5.0
 * @group Flapping Wing Control
 */
PARAM_DEFINE_FLOAT(FLAP_GLIDE_TO, 1.0f);

/**
 * Glide stop RC channel selection.
 *
 * 0: disabled, 1..18: selects RC function 1..18 (aux/switches) in manual_control_setpoint.
 *
 * @min 0
 * @max 6
 * @group Flapping Wing Control
 */
PARAM_DEFINE_INT32(FLAP_GLIDE_CH, 6);
