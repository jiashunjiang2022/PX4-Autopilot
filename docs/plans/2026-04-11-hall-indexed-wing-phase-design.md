# Hall-Indexed Wing Phase Design

Goal: Restore the previous Hall-indexed wing phase behavior so `wing_phase` uses the most recent Hall pulse as the mechanical `0 deg` reference and AS5600 counts provide continuous phase within that cycle.

Design:
- Keep `as5600` responsible for publishing `encoder_count` and `flap_frequency`.
- Restore a dedicated `wing_phase` module that subscribes to `encoder_count`, `flap_frequency`, and `hall_event`.
- Use `rpm_capture` to publish `hall_event` from the configured RPM input pulse stream.
- On each new Hall pulse, latch the current encoder total count as the zero reference.
- Compute wrapped phase from `encoder_total_count - zero_count` over one flap cycle `4096 * FLAP_RATIO`.
- Do not publish a valid absolute phase before the first Hall pulse has been observed.

Why this design:
- It matches the previous implementation pattern in this branch history.
- It keeps Hall zeroing logic out of the AS5600 driver.
- It preserves the existing `wing_phase` topic name used by logging and downstream consumers.
