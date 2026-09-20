#include "components/counting_core/counting_core.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
using namespace esphome::counting_core;
struct Rig {
  Fusion f;
  uint32_t now{0};
  std::array<uint8_t,4> states{};
  std::vector<Result> results;
  void tick(uint32_t dt, std::array<uint8_t,4> s, uint8_t health=15, std::array<uint8_t,4> fresh={3,3,3,3}) {
    now += dt; states=s;
    auto r=f.update(now,health,states,fresh);
    if(r.decision!=Decision::NONE) results.push_back(r);
  }
  void arm() { tick(1,{}); tick(100,{}); }
  void all(uint8_t state, uint32_t dt=100) { tick(dt,{state,state,state,state}); }
  void finish() { all(0); all(0); }
};
int main() {
  { Rig r; r.arm(); r.all(1); r.all(3); r.all(2); assert(r.results.empty()); r.finish();
    assert(r.results.size()==1 && r.results[0].decision==Decision::OUT); r.all(0); assert(r.results.size()==1); }
  { Rig r; r.arm(); r.all(2); r.all(3); r.all(1); r.finish(); assert(r.results[0].decision==Decision::IN); }
  { Rig r; r.f.invert=true; r.arm(); r.all(1); r.all(3); r.all(2); r.finish(); assert(r.results[0].decision==Decision::IN); }
  { Rig r; r.arm(); r.all(1); r.all(3); r.all(2); r.all(2,10000); assert(r.results.empty());
    r.all(3); r.all(1); r.finish(); assert(r.results.size()==1 && r.results[0].decision==Decision::REJECTED); }
  { Rig r; r.arm(); r.all(1); r.all(3,10000); r.all(2); r.finish(); assert(r.results[0].decision==Decision::OUT); }
  { Rig r; r.arm(); r.all(3); r.all(2); r.finish(); assert(r.results[0].decision==Decision::REJECTED); }
  { Rig r; r.arm(); r.all(1); r.all(2); r.finish(); assert(r.results[0].decision==Decision::OUT); } // fast, no sampled overlap
  { Rig r; r.arm(); r.tick(100,{1,1,1,2}); r.all(3); r.tick(100,{2,2,2,1}); r.finish();
    assert(r.results[0].decision==Decision::REJECTED); } // one opposing complete track vetoes majority
  { Rig r; r.arm(); r.tick(100,{1,1,0,0}); r.tick(100,{2,2,0,0}); r.finish();
    assert(r.results[0].decision==Decision::UNSURE_OUT); }
  { Rig r; r.arm(); r.all(1); r.tick(100,{3,3,3,3},3); r.tick(100,{2,2,2,2},3); r.finish();
    for(auto x:r.results) assert(x.decision!=Decision::IN && x.decision!=Decision::OUT); }
  { Rig r; r.arm(); r.all(1); r.tick(100,{3,3,3,3},7); r.tick(100,{2,2,2,2},7); r.tick(100,{},7); r.tick(100,{},7);
    assert(r.results.back().decision==Decision::OUT); } // still three healthy complete tracks
  { Rig r; r.arm(); r.all(1); r.all(2); r.tick(100,{},15,{0,0,0,0}); r.tick(1000,{},15,{0,0,0,0});
    assert(r.results.empty()); r.tick(1,{}); assert(r.results[0].decision==Decision::OUT); }
  { Rig r; r.all(1); r.all(2); r.finish(); assert(r.results.empty()); } // boot while occupied
  { Rig r; r.now=UINT32_MAX-250; r.arm(); r.all(1); r.all(3); r.all(2); r.finish();
    assert(r.results[0].decision==Decision::OUT); }
  { Rig r; r.arm(); for(int i=0;i<1000;++i){r.all(1);r.all(3);r.all(2);r.finish();}
    assert(r.results.size()==1000); for(auto x:r.results) assert(x.decision==Decision::OUT); }
  { Rig r; r.arm(); r.all(1); r.all(2); r.all(0); r.all(1,20); r.all(2); r.finish();
    assert(r.results[0].decision==Decision::REJECTED); } // unseparated traffic is not guessed
  { Debounce d; assert(!d.update(true,1,25)); assert(!d.active);
    assert(!d.update(false,1000,25)); assert(!d.active);
    assert(!d.update(true,1100,25)); assert(d.update(true,1174,25) && d.active);
    assert(!d.update(false,1200,25)); assert(d.update(false,1274,25) && !d.active); }
  { Debounce d; assert(!d.update(true,UINT32_MAX-10,25)); assert(d.update(true,20,25)); }
  { Statistics s; for(int i=0;i<100;++i)s.add(4000); s.add(std::numeric_limits<float>::quiet_NaN());
    assert(s.count==100 && s.mean==4000 && s.deviation()==0); }
  { Rig r; r.f.agreement_ms=200; r.arm(); r.all(1); r.all(2);
    r.tick(100,{0,2,2,2}); r.tick(500,{}); r.tick(100,{});
    assert(r.results[0].decision==Decision::REJECTED); }
  { Rig r; r.arm(); r.all(1); r.tick(100,{3,3,3,3},7); r.tick(100,{2,2,2,1},15); r.finish();
    assert(r.results[0].decision==Decision::OUT && r.results[0].out_votes==3); } // recovered member cannot join this event
  { Rig r; r.f.required=4; r.arm(); r.all(1); r.tick(100,{2,2,2,2},7); r.finish();
    for(auto x:r.results) assert(x.decision!=Decision::IN && x.decision!=Decision::OUT); }
  { Fusion f; std::array<uint8_t,4> fresh{3,3,3,3};
    f.update(1,15,{},fresh);f.update(101,15,{},fresh);
    f.update(200,15,{1,1,1,1},fresh);f.update(300,15,{2,2,2,2},fresh);
    f.update(400,15,{},fresh);
    assert(f.update(500,15,{},fresh,true).decision==Decision::NONE);
    assert(f.update(600,15,{},fresh).decision==Decision::NONE);
    assert(f.update(700,15,{},fresh).decision==Decision::OUT); }
  // Exhaustively exercise all six-step paths: same-side returns never count,
  // and no output may occur while the doorway is occupied.
  for(unsigned code=0;code<729;++code) {
    Rig r; r.arm(); unsigned n=code; uint8_t first=0,last=0;
    for(int step=0;step<6;++step){uint8_t s=1+n%3;n/=3;if(!first&&s)first=s;if(s==1||s==2)last=s;r.all(s,30);
      assert(r.results.empty());}
    r.finish();
    if(first==last||first==3) for(auto x:r.results) assert(x.decision!=Decision::IN&&x.decision!=Decision::OUT);
  }
  std::cout << "Counting core: scenarios, 1000 passages, and 729 exhaustive paths passed\n";
}
