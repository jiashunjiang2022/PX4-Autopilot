# Accepted control architecture

Baseline 19a108f17666942d199fb3856be5e85bc9aa6f83. No frozen branch edits.
Source contract passes with the newly authorized separation: RateControl returns
P+D+FF+I_used (rate_control.cpp:183–198); Slow S is composed before scaling
(FixedwingRateControl.cpp:478–486). Compression observes that baseline at 492.
The new roll request is calculated after compression, with the saved pre-update
roll scale, and installed after baseline roll-to-yaw feedforward (537–540).
Clipping remains [-1,1] after trim. T_model is the exact I_post+S argument at
the original model update (635), cached with independent DELTA4 validity.
That frame first becomes eligible in the next Run. No scheduler is moved.
Live guard is the previous completed Run's I_post+S, refreshed at the same
diagnostic site each Run, with a 100000 us timeout. It can only reduce authority.
Model timeout is also 100000 us: two nominal fixed-phase 50000 us periods;
age > timeout or clock reversal fails zero. No logger observation drives age.
Control resets invalidate the control cache and require nine newly completed
finite frames for control eligibility without modifying the predictor history.
Disarm/regime/disable invalidation also prevents recycling an old target.
No model constants, native I, Slow S, allocator, or compression observer changes.
Disabled-from-startup equivalence means no arithmetic addition to baseline output;
it does not promise restoration of a counterfactual trajectory after enabling.
READY_FOR_FAST_ON_FLIGHT=NO
