# Actuator margin audit

ControlAllocator.cpp:400–444 consumes torque/thrust into six-axis setpoints,
allocates, applies auxiliary controls, slew limits, then clips. At 490–590,
servos receive normalized [-1,1] limits and configured trim/slew. Fixed-wing
effectiveness adds configured rotors and surfaces (ActuatorEffectivenessFixedWing.cpp),
then applies flaps and spoilers after allocation. ControlAllocation.cpp:54–61
clips trim/linearization; PseudoInverse.cpp:180–188 uses trim + mix*(request-control_trim).
PseudoInverse normalization and selected allocation method also affect the mix.
verify_flap_slow_configuration checks CA_METHOD 0 or 2, the three surface torque
columns (-.55,1,0), (.55,1,0), (0,0,1), zero surface trim, MAIN assignments,
and PWM_MAIN_REV=17; these are runtime checks, not proof of actual aircraft settings.
The ideal inverse cL=P/2-R/1.1, cR=P/2+R/1.1 does not include auxiliary
deflections or slew-state constraints. fw_rate_control lacks a synchronous
allocator snapshot proving those quantities. Delayed actuator_servos is not used.
Reversal remains downstream; this patch never reapplies reversal.
Next method: allocator-owned additional-roll interval using its actual mixing
matrix, auxiliary offsets, trims, bounds and slew state for the same request,
or a verified frozen configuration with auxiliary channels disabled and proven
normalization. No such guarantee is asserted in this implementation.
ACTUATOR_MARGIN_IMPLEMENTED=NO
READY_FOR_FAST_ON_FLIGHT=NO
