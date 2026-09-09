// AUTO-GENERATED - DO NOT EDIT
// SOURCE_ARTIFACT_SHA256=dfd86a2d170243a408de5a7fa96e18c4ab04f7560abad0d364700ca477e42675
// SOURCE_NORMALIZATION_SHA256=ebb52c5a575b9ab0ee4886ae2451627db6536fc33b325d4ada1468cfafa371ef
// FEATURE_MANIFEST_SHA256=9c8673f3ad8073b0797edd03f38e6ddb4f9ae2f91ec0b5bb88ca769163ea1293
// GENERATOR_SHA256=035bfcb3e3c3fe6d6cd1da29e4881d6feb5aac70ebc6a08dc25b31b024c1050b
#pragma once

#include <cstddef>

namespace flap_aug_shadow::generated::v2_roll
{
static constexpr size_t FeatureCount = 20;
static constexpr float Intercept = 6.38383281e-05f;
static constexpr float Mean[FeatureCount]{
	0.0655303672f, 0.272220343f, -0.0435887985f, -0.0208251383f, 0.0641032606f, -0.0123359719f,
	-0.0227560941f, -0.00981792342f, -0.0704178214f, -0.031203635f, 0.0235093106f, -0.102731466f,
	-0.0797001645f, 0.436015368f, 0.624467492f, -0.0269422177f, -0.0434168428f, -0.0598967336f,
	-0.00823731255f, -0.0351795293f
};

static constexpr float Std[FeatureCount]{
	0.223290861f, 0.116239794f, 0.484168231f, 0.309788704f, 0.189483091f, 0.205159068f,
	0.124570198f, 0.690840065f, 0.586007535f, 0.249842137f, 0.0990282968f, 0.268833041f,
	0.118774459f, 0.26243645f, 0.357346237f, 0.295088261f, 0.320741355f, 0.152585611f,
	0.263347208f, 0.160075903f
};

static constexpr float Coefficient[FeatureCount]{
	0.00141179783f, 0.000991950976f, 0.00670062006f, -0.00284786103f, -0.00447971467f, 0.0037220032f,
	0.00131461117f, 0.000818105997f, -3.9585284e-05f, 0.0041778828f, -0.00111467054f, 0.000517763023f,
	-0.000627339934f, -0.000967643107f, -0.00105085142f, -0.00114945706f, 0.000724057551f, 0.000159029179f,
	0.00108545134f, -0.000333040371f
};

static constexpr const char *FeatureNames[FeatureCount]{
	"roll_sp",
	"pitch_sp",
	"p_sp",
	"q_sp",
	"r_sp",
	"roll_error",
	"pitch_error",
	"p_error",
	"q_error",
	"r_error",
	"rollspeed_integ",
	"pitchspeed_integ",
	"yawspeed_integ",
	"roll_integrator_utilization",
	"pitch_integrator_utilization",
	"horizontal_tail_1_raw",
	"horizontal_tail_2_raw",
	"vertical_tail_raw",
	"u_tail_roll",
	"u_tail_pitch"
};

} // namespace flap_aug_shadow::generated::v2_roll
