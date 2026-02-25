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
 * Injector publish instance for differential_pressure
 *
 * Use 1 to avoid interfering with primary airspeed chain on instance 0.
 *
 * @min 0
 * @max 3
 * @group Airspeed Injector
 */
PARAM_DEFINE_INT32(ASPD_INJ_INST, 1);

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
 * NOTE: PX4 parameter names are limited to 16 chars, so this uses ASPD_INJ_SPK_AMP
 * for requested SPIKE_AMP functionality.
 *
 * @min 0.0
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_SPK_AMP, 0.0f);

/**
 * Injector spike disturbance period
 *
 * NOTE: PX4 parameter names are limited to 16 chars, so this uses ASPD_INJ_SPK_PER
 * for requested SPIKE_PERIOD functionality. Set to 0 to disable spikes.
 *
 * @unit s
 * @min 0.0
 * @decimal 2
 * @group Airspeed Injector
 */
PARAM_DEFINE_FLOAT(ASPD_INJ_SPK_PER, 0.0f);
