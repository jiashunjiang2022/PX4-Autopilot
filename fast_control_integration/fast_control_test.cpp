#include "../src/modules/fw_rate_control/FastControl.hpp"
#include "../src/modules/fw_rate_control/FastV1ShadowModel.hpp"
#include <cassert>
#include <algorithm>
#include <cstdio>
#include <limits>

static void ready(FastControl &f, float t, float p, float live)
{
	for (unsigned i=1; i<=9; ++i) { f.complete({p,t,1000000,i,true},live,1000000,true); }
}
int main()
{
	FastControl::Config c; c.enabled=true;
	unsigned cases=0;
	for (float t : {0.f,.149999f,.15f,.165f,.18f,.180001f,-.149999f,-.15f,-.165f,-.18f,-.180001f}) {
		for (float p : {0.f,.001f,-.001f,.02f,-.02f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
			FastControl f; ready(f,t,p,t);
			auto d=f.step(c,1000001,.1f,true,false);
			float g=std::fabs(t)<=.15f ? 1.f : (std::fabs(t)>=.18f ? 0.f : (.18f-std::fabs(t))/.03f);
			float expected=std::isfinite(p) ? g*std::max(-.005f,std::min(.005f,p)) : 0.f;
			assert(std::isfinite(d.final)); assert(std::fabs(d.final-expected)<1e-7f);
			assert(std::fabs(t+d.final)<=.20f); ++cases;
		}
	}
	FastControl f; ready(f,0,.02f,0);
	auto d=f.step(c,1000001,.01f,true,false); assert(std::fabs(d.final-.0005f)<1e-8f);
	f.complete({},.18f,1000010,false); assert(f.step(c,1000020,.01f,true,false).final==0);
	ready(f,0,.02f,0); assert(f.step(c,1100001,.01f,true,false).final==0);
	f.complete({.02f,0,1100002,10,true},0,1100002,true);
	assert(f.step(c,1100003,.01f,true,false).final==0); // stale requires fresh warmup
	for (int mode=0;mode<5;++mode) {
		ready(f,0,.02f,0); auto cfg=c;
		if(mode==0) cfg.enabled=false;
		if(mode==3) cfg.on=cfg.off;
		if(mode==4) cfg.max=std::nextafter(FastControl::AUTHORITY_MAX, INFINITY);
		assert(f.step(cfg,1000001,.01f,mode!=1,mode==2).final==0);
	}
	// Production maps AUTO_MISSION to state_ok (checked by source isolation test).
	ready(f,0,.02f,0);
	assert(f.step(c,1000001,.01f,true,false).final>0);
	assert(f.step(c,1000002,.01f,false,false).final==0); // mission -> stabilized
	for (unsigned i=0;i<20;++i) {
		ready(f,0,.02f,0); // even a full warmup cannot authorize a non-mission mode
		auto blocked=f.step(c,1000003+i,.01f,false,false);
		assert(blocked.final==0 && blocked.state && !blocked.valid);
	}
	for (float slew : {0.f,-.01f}) {
		ready(f,0,.02f,0);
		assert(f.step(c,1000001,.01f,true,false).final>0);
		auto cfg=c; cfg.slew=slew;
		assert(f.step(cfg,1000002,.01f,true,false).final==0);
		assert(f.step(c,1000003,.01f,true,false).final==0); // invalid config cleared readiness
	}
	// Envelope cases exercise each parameter independently after real nonzero output.
	const float nan=std::numeric_limits<float>::quiet_NaN();
	const float inf=std::numeric_limits<float>::infinity();
	using Config=FastControl::Config;
	const auto check_config=[&](Config cfg, bool valid) {
		FastControl ctrl; ready(ctrl,0,.02f,0);
		assert(ctrl.step(c,1000001,.01f,true,false).final>0);
		auto result=ctrl.step(cfg,1000002,.01f,true,false);
		assert(result.config_invalid==!valid);
		assert(result.valid==valid);
		if (!valid) {
			assert(result.final==0);
			assert(ctrl.step(c,1000003,.01f,true,false).final==0);
		}
	};
	struct Bound { float Config::*field; float high; };
	for (auto b : {Bound{&Config::k,5.f}, Bound{&Config::max,.020f},
		Bound{&Config::on,.180f}, Bound{&Config::off,.200f}, Bound{&Config::slew,.200f}}) {
		Config cfg=c; check_config(cfg,true);
		cfg.off=.20f; cfg.*(b.field)=b.high; check_config(cfg,true);
		cfg.*(b.field)=std::nextafter(b.high,inf); check_config(cfg,false);
		for (float bad : {-1.f,nan,inf,-inf}) { cfg.*(b.field)=bad; check_config(cfg,false); }
	}
	for (float k : {1.f,1.5f,3.f,5.f}) {
		Config cfg=c; cfg.k=k; check_config(cfg,true);
		FastControl ctrl; ready(ctrl,0,.004f,0);
		auto result=ctrl.step(cfg,1000001,.1f,true,false);
		assert(std::fabs(result.raw-k*.004f)<1e-8f);
		assert(std::fabs(result.saturated-std::min(k*.004f,.005f))<1e-8f);
		assert(std::fabs(result.final)<=cfg.max);
	}
	for (float v : {.005f,.010f,.020f}) { Config cfg=c; cfg.max=v; check_config(cfg,true); }
	for (float v : {.15f,.17f,.18f}) { Config cfg=c; cfg.on=v; cfg.off=.20f; check_config(cfg,true); }
	for (float v : {.18f,.19f,.20f}) { Config cfg=c; cfg.off=v; check_config(cfg,true); }
	for (float v : {FastControl::SLEW_MIN,.05f,.10f,.20f}) { Config cfg=c; cfg.slew=v; check_config(cfg,true); }
	for (float v : {0.f,-.01f,std::nextafter(FastControl::SLEW_MIN,0.f)}) {
		Config cfg=c; cfg.slew=v; check_config(cfg,false);
	}
	{ Config cfg=c; cfg.on=cfg.off; check_config(cfg,false); cfg.off=.14f; check_config(cfg,false); }
	{ Config cfg=c; cfg.k=0; check_config(cfg,true); cfg.max=0; check_config(cfg,true); cfg.on=0; check_config(cfg,true); }
	// Synthetic model/live grid covers both signs, fade, projection and minimum gate.
	unsigned envelope_cases=0;
	for (float maximum : {.010f,.020f}) {
		for (auto thresholds : {std::pair<float,float>{.15f,.18f},{.17f,.19f},{.18f,.20f}}) {
			Config cfg=c; cfg.max=maximum; cfg.k=5; cfg.slew=.2f;
			cfg.on=thresholds.first; cfg.off=thresholds.second;
			const auto gate=[&](float t) {
				float a=std::fabs(t);
				return a<=cfg.on ? 1.f : (a>=cfg.off ? 0.f : (cfg.off-a)/(cfg.off-cfg.on));
			};
			for (float t : {0.f,cfg.on,std::nextafter(cfg.on,0.f),std::nextafter(cfg.on,inf),
				(cfg.on+cfg.off)/2,cfg.off,std::nextafter(cfg.off,0.f),std::nextafter(cfg.off,inf),.2f}) {
				for (float live : {0.f,cfg.on,(cfg.on+cfg.off)/2,cfg.off,.2f}) {
					for (float sign : {-1.f,1.f}) {
						for (float pred : {-.1f,.1f}) {
							FastControl ctrl; ready(ctrl,sign*t,pred,-sign*live);
							auto result=ctrl.step(cfg,1000001,.1f,true,false);
							const float g=std::min(gate(t),gate(live));
							assert(std::fabs(result.final)<=maximum+1e-8f);
							assert(std::fabs(sign*t+result.final)<=.20f+1e-7f);
							assert(std::fabs(-sign*live+result.final)<=.20f+1e-7f);
							assert(std::fabs(result.model_gate-gate(t))<1e-6f);
							assert(std::fabs(result.live_gate-gate(live))<1e-6f);
							assert(std::fabs(result.gated-std::copysign(maximum*g,pred))<1e-7f);
							if(g==0) assert(result.final==0);
							if(t==0 && live==0) assert(std::fabs(std::fabs(result.final)-maximum)<1e-7f);
							++envelope_cases;
						}
					}
				}
			}
		}
	}
	// A previously large actual must contract immediately when burden approaches .20.
	for (float sign : {-1.f,1.f}) {
		Config cfg=c; cfg.max=.02f; cfg.slew=.2f; cfg.on=.18f; cfg.off=.2f;
		FastControl ctrl; ready(ctrl,0,sign*.1f,0);
		assert(std::fabs(ctrl.step(cfg,1000001,.1f,true,false).final-sign*.02f)<1e-7f);
		ctrl.complete({sign*.1f,sign*.199f,1000002,10,true},sign*.198f,1000002,true);
		auto result=ctrl.step(cfg,1000003,.001f,true,false);
		assert(std::fabs(result.final)<=.001001f);
		assert(std::fabs(sign*.199f+result.final)<=.20f+1e-7f);
		assert(std::fabs(sign*.198f+result.final)<=.20f+1e-7f);
		ctrl.complete({sign*.1f,sign*.2f,1000004,11,true},0,1000004,true);
		assert(ctrl.step(cfg,1000005,.001f,true,false).final==0);
	}
	std::printf("PASS: research config bounds/nonfinite/relationships, large-K saturation, %u expanded authority/gate cases and safety contraction\n",envelope_cases);
	FastV1ShadowModel model; FastV1ShadowModel::Output out;
	for(unsigned i=0;i<9;++i) {
		assert(model.update(1000000+i*50000,.01f*i,.1f,.05f,out));
		assert(out.control_delta_valid==(i==8));
	}
	assert(out.control_model_t==.08f);
	assert(std::fabs(out.t_lag2-.06f)<1e-7f);
	const auto held=out;
	assert(!model.update(1400001,.5f,1.f,1.f,out));
	assert(out.control_model_t==held.control_model_t && out.v2c_delta4==held.v2c_delta4);
	model.resetFeatureHistory();
	assert(!model.update(1400002,0,0,0,out)); assert(!out.control_delta_valid);
	assert(model.update(1450000,0,0,0,out)); assert(!out.control_delta_valid);
	std::printf("PASS: %u boundary cases plus mission exit/non-mission persistence, zero/negative slew shutdown, live veto, stale rewarmup, state/reset/config, model history and held-frame checks\n",cases);
}
