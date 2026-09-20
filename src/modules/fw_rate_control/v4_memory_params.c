// SPDX-License-Identifier: BSD-3-Clause
#include <px4_platform_common/param.h>

/**
 * V4 memory EN prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 * @boolean
 */
PARAM_DEFINE_INT32(FW_V4_EN, 0);

/**
 * V4 memory BMAX prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_BMAX, 0.1f);

/**
 * V4 memory R prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_R, 0.05f);

/**
 * V4 memory TAU prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_TAU, 5.0f);

/**
 * V4 memory KB prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_KB, 0.1f);

/**
 * V4 memory SLEW prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_SLEW, 0.01f);

/**
 * V4 memory GWIN prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_GWIN, 3.0f);

/**
 * V4 memory GSTD prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_GSTD, 0.025f);

/**
 * V4 memory GSIGN prototype
 *
 * NOT FLIGHT VALIDATED. These defaults are not a tuning recommendation.
 * Enable requires disarmed acquisition. HR is internally zero.
 * R/BMAX/GSTD use raw I units; TAU/GWIN seconds; SLEW raw I per second.
 * KB is residual learning gain; GSIGN is same-sign fraction.
 *
 * @group FW V4 Prototype
 */
PARAM_DEFINE_FLOAT(FW_V4_GSIGN, 0.90f);

