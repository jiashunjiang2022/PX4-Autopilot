#include <px4_platform_common/param.h>

/**
 * Enable SITL airspeed injector
 *
 * 0: disabled
 * 1: enabled
 *
 * @min 0
 * @max 1
 * @group Airspeed Injector
 */
PARAM_DEFINE_INT32(ASPD_INJ_EN, 0);

/**
 * Injector base differential pressure
 *
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_BASE, 30.0f);

/**
 * Injector flap frequency for narrowband disturbance
 *
 * @unit Hz
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_FLAP_HZ, 5.0f);

/**
 * Injector narrowband disturbance amplitude
 *
 * @min 0.0
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_NB_AMP, 0.0f);

/**
 * Injector spike disturbance amplitude
 *
 * @min 0.0
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_SPK_AMP, 0.0f);

/**
 * Injector spike disturbance period
 *
 * @unit s
 * @min 0.1
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_SPK_PER, 2.0f);
