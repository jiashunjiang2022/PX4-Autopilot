# Selector traceability

`airspeed_selector_quality_status` records pre-quality source/device, quality
source instance/device/timestamp/age, final source/device/sample timestamp,
strict identity match, original validity, quality rejection, and final validity.

`trigger_reason` is independent from `fallback_outcome`. Trigger values separate
low quality, stale quality, invalid quality, original PX4 invalidity, blockage,
source-ID mismatch, timeout, and unavailable source. Fallback outcome separates
alternate physical, ground-minus-wind, synthetic, and unavailable results.

An identity mismatch clears the quality latch, records
`SOURCE_ID_MISMATCH`, and does not reject or rewrite the original PX4 result.
When quality rejection and blockage coexist, the quality trigger is retained and
`concurrent_blockage=true`; blockage does not overwrite the initiating cause.

The original PX4 validator and selection run first. Quality logic is active only
in FULL_PROPOSED and only with the strict single-Pitot identity proof. Helper
tests cover low/stale/invalid quality, hold/recovery, identity combinations,
alternate/ground/synthetic/unavailable fallback choices, and original validity.
