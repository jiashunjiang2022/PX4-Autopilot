/**
 * Enable Mission-only 1f Pitch FF shaping
 *
 * Default off. Internal 2f nuisance regression never enters applied shaping.
 * Requires bench and flight qualification before use.
 *
 * @boolean
 * @group Pitch FF Shaper
 */
PARAM_DEFINE_INT32(FLAP_PFF_EN, 0);

/**
 * Pitch FF 1f subtraction fraction
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Pitch FF Shaper
 */
PARAM_DEFINE_FLOAT(FLAP_PFF_ALPHA, 0.5f);

/**
 * Pitch FF adaptation time in wing cycles
 *
 * Winning reference is 1.0 cycle. Other values require qualification.
 *
 * @min 1.0
 * @max 8.0
 * @decimal 2
 * @group Pitch FF Shaper
 */
PARAM_DEFINE_FLOAT(FLAP_PFF_ADAPT, 1.0f);
