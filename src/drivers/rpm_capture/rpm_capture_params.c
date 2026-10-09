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
 * RPM capture enable
 *
 * Enables the RPM capture module to estimate RPM from pulses detected on a PWM pin configured as "RPM Input".
 *
 * @boolean
 * @group System
 * @reboot_required true
 */
PARAM_DEFINE_INT32(RPM_CAP_ENABLE, 0);

/**
 * Minimum accepted pulse interval
 *
 * Pulses arriving sooner than this interval after the last accepted pulse are treated as input glitches.
 * Set to 0 to disable pulse interval filtering.
 *
 * @unit us
 * @min 0
 * @max 1000000
 * @group System
 * @reboot_required true
 */
PARAM_DEFINE_INT32(RPM_CAP_MIN_US, 0);

/**
 * Voltage pulses per revolution
 *
 * Number of voltage pulses per one rotor revolution on the capturing pin.
 *
 * @group System
 * @min 1
 * @max 50
 * @reboot_required true
 */
PARAM_DEFINE_INT32(RPM_PULS_PER_REV, 1);
