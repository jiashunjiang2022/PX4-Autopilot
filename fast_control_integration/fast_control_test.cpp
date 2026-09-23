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
		if(mode==4) cfg.max=.01f;
		assert(f.step(cfg,1000001,.01f,mode!=1,mode==2).final==0);
	}
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
	std::printf("PASS: %u boundary cases plus slew, live veto, stale rewarmup, state/reset/config, model history and held-frame checks\n",cases);
}
