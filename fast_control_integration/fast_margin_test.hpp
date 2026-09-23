#include "../src/modules/fw_rate_control/FastActuatorMargin.hpp"
#include <cstring>

static void test_margin()
{
	using M = FastActuatorMargin;
	using C = FastControl;
	const float inf=std::numeric_limits<float>::infinity();
	const auto close=[](float a,float b) { assert(std::fabs(a-b)<3e-6f); };
	assert(M::configuration(true,0,0,0,0,1,0,0,0,0));
	for(int i=0;i<4;++i) {
		float x[4]{}; x[i]=.1f;
		assert(!M::configuration(true,x[0],x[1],x[2],x[3],1,0,0,0,0));
		C ctrl; ready(ctrl,0,.02f,0); C::Config cfg; cfg.enabled=true;
		bool supported=M::configuration(true,x[0],x[1],x[2],x[3],1,0,0,0,0);
		assert(ctrl.step(cfg,1000001,.1f,supported,false).final==0);
	}
	assert(!M::configuration(false,0,0,0,0,1,0,0,0,0));
	assert(!M::configuration(true,0,0,0,0,2,0,0,0,0));
	assert(!M::configuration(true,0,0,0,0,1,.1f,0,0,0));
	assert(!M::configuration(true,0,0,0,0,1,0,.1f,0,0));
	assert(!M::configuration(true,0,0,0,0,1,0,0,.1f,0));
	assert(!M::configuration(true,0,0,0,0,1,0,0,0,.1f));
	for(float p : {-1.f,-.8f,-.5f,0.f,.5f,.8f,1.f}) {
		for(float r : {-1.f,-.66f,-.55f,0.f,.55f,.6f,.66f,1.f}) {
			const auto m=M::compute(r,p,2.f);
			close(-.55f*M::tail0(r,p)+.55f*M::tail1(r,p),r);
			close(M::tail0(r,p)+M::tail1(r,p),p);
			assert(m.valid);
			if(m.baseline_feasible) {
				assert(m.fast_lower<=0 && m.fast_upper>=0);
				assert(M::postcheck(r+2.f*m.fast_lower,p));
				assert(M::postcheck(r+2.f*m.fast_upper,p));
			} else { assert(m.blocked && m.fast_lower==0 && m.fast_upper==0); }
		}
	}
	close(M::compute(0,0,1).roll_upper,1);
	for(float p : {-1.f,1.f}) close(M::compute(0,p,1).roll_upper,.55f);
	close(M::compute(.6f,.8f,1).roll_headroom_positive,.06f);
	close(M::compute(-.6f,-.8f,1).roll_headroom_negative,.06f);
	assert(!M::compute(.55f,1,1).blocked);
	assert(!M::compute(std::nextafter(.55f,0.f),1,1).blocked);
	assert(M::compute(std::nextafter(.55f,inf),1,1).blocked);
	for(float bad : {inf,-inf,std::numeric_limits<float>::quiet_NaN()}) {
		assert(!M::compute(bad,0,1).valid); assert(!M::compute(0,bad,1).valid);
		assert(!M::compute(0,0,bad).valid); assert(!M::postcheck(bad,0));
	}
	assert(!M::compute(0,0,0).valid); assert(!M::compute(0,0,-1).valid);
	assert(!M::postcheck(1.f,1.f));
	unsigned grid=0;
	C::Config cfg; cfg.enabled=true; cfg.max=.02f; cfg.on=.18f; cfg.off=.20f; cfg.slew=.2f;
	for(float r : {-.6f,0.f,.6f}) for(float p : {-1.f,-.8f,0.f,.8f,1.f})
		for(float t : {-.199f,-.18f,0.f,.18f,.199f}) for(float pred : {-.1f,.1f}) {
			C limited, original; ready(limited,t,pred,-t); ready(original,t,pred,-t);
			auto m=M::compute(r,p,1);
			auto d=limited.step(cfg,1000001,.1f,true,false,{m.valid,m.fast_lower,m.fast_upper});
			auto ref=original.step(cfg,1000001,.1f,true,false);
			assert(std::fabs(d.final)<=d.limit+1e-7f);
			assert(std::fabs(t+d.final)<=.2f+1e-7f && std::fabs(-t+d.final)<=.2f+1e-7f);
			if(m.blocked) assert(d.final==0);
			else {
				assert(M::postcheck(r+d.final,p));
				if(ref.final>=m.fast_lower && ref.final<=m.fast_upper) close(d.final,ref.final);
			}
			++grid;
		}
	for(float sign : {-1.f,1.f}) {
		C ctrl; ready(ctrl,0,sign*.1f,0);
		assert(std::fabs(ctrl.step(cfg,1000001,.1f,true,false).final)>.019f);
		auto tight=M::compute(sign*.5499f,1.f,1.f);
		cfg.slew=.000001f;
		auto d=ctrl.step(cfg,1000002,.001f,true,false,{true,tight.fast_lower,tight.fast_upper});
		assert(std::fabs(d.final)<.000101f && d.actuator_margin_limited);
		assert(ctrl.step(cfg,1000003,.001f,true,false,{true,0,0}).final==0);
		// Collapsed authority must NOT discard readiness.
		assert(ctrl.step(cfg,1000004,.001f,true,false).valid);
		cfg.slew=.2f;
	}
	// Production zero path skips replacement: bit-identical baseline for every fail-zero reason.
	for(int failure=0;failure<9;++failure) {
		C ctrl; ready(ctrl,0,.02f,0); auto c=cfg;
		if(failure==0) c.enabled=false;
		if(failure==1) c.k=6;
		if(failure==2) ctrl.complete({NAN,0,1000000,10,true},0,1000000,true);
		C::DynamicBounds b{true,-.02f,.02f};
		if(failure==3) b={true,0,0};
		if(failure==4) b.valid=false;
		const auto d=ctrl.step(c,failure==5 ? 1100001 : 1000001,.01f,
			failure!=6 && failure!=8,failure==7,b);
		assert(d.final==0);
		float baseline=-.1234567f, output=baseline;
		if(d.final>0 || d.final<0) output=baseline+d.final;
		assert(std::memcmp(&baseline,&output,sizeof(float))==0);
	}
	std::printf("PASS: actuator configuration/geometry/headroom, %u intersection cases, contraction/readiness and zero-path checks\n",grid);
}
