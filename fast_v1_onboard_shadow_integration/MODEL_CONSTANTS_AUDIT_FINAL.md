# Model constants audit

The source constants in `FastV1ShadowModel.hpp` are the float32 conversions of
the frozen `model.json` mean, scale, coefficient, and intercept arrays.

ABS4_MODEL_CONSTANTS=VERIFIED
DELTA4_MODEL_CONSTANTS=VERIFIED
ABS4_SHA256=88d243911ac8d048099ba5bed058cbb89c43135aebc91b8c3725fc02be342bec
DELTA4_SHA256=134eb1a349925d9441c7d839f5880f70e8ffb67f3f07339cae62460d73d61181

The export verifier passed for both models. Reference-vector float32 replay
gave ABS4 max absolute error 1.12e-8 and DELTA4 max absolute error 9.31e-10
over 130 rows each.
