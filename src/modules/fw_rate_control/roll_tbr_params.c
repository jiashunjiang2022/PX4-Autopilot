/**
 * Enable Mission Roll task-band residual
 *
 * Default off. Nonzero tuning defaults are offline/bench candidates only.
 * Requires actual aircraft bench and flight qualification before use.
 *
 * @boolean
 * @group Roll TBR
 */
PARAM_DEFINE_INT32(FLAP_TBR_EN, 0);

/**
 * Roll TBR LP1 cutoff (offline/bench candidate)
 *
 * @unit Hz
 * @min 0.01
 * @max 10.0
 * @decimal 4
 * @group Roll TBR
 */
PARAM_DEFINE_FLOAT(FLAP_TBR_FC, 0.625f);

/**
 * Roll TBR gain (offline/bench candidate)
 *
 * Normalized controller command per rad/s of filtered roll rate error.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 5
 * @group Roll TBR
 */
PARAM_DEFINE_FLOAT(FLAP_TBR_K, 0.01f);

/**
 * Roll TBR symmetric command cap (offline/bench candidate)
 *
 * @min 0.00001
 * @max 1.0
 * @decimal 5
 * @group Roll TBR
 */
PARAM_DEFINE_FLOAT(FLAP_TBR_MAX, 0.025f);

/**
 * Roll TBR slew limit (offline/bench candidate)
 *
 * Normalized controller command change per second. Applied after symmetric cap.
 *
 * @min 0.00001
 * @max 10.0
 * @decimal 9
 * @group Roll TBR
 */
PARAM_DEFINE_FLOAT(FLAP_TBR_SLEW, 0.035031594f);
