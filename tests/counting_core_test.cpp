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
  // Threshold crossing bounds use measurement time, never the later poll
  // which happens to confirm a debounced state.
  { Debounce d; d.update(false,100,25); d.update(true,140,25);
    assert(d.update(true,230,25));
    assert(d.crossing.valid && d.crossing.lower==100 && d.crossing.upper==140);
    d.update(false,250,25); d.update(true,270,25); // cancelled release
    d.update(false,300,25); assert(d.update(false,340,25));
    assert(d.crossing.lower==270 && d.crossing.upper==300); }
  { Debounce d; d.update(true,10,25); assert(d.update(true,50,25));
    assert(!d.crossing.valid); } // no preceding clear measurement
  { Debounce a,b;
    a.update(false,200,25,100);a.update(true,250,25,210);a.update(true,290,25,260);
    b.update(false,100,25,80);b.update(true,180,25,140);b.update(true,220,25,190);
    assert(crossing_order(a.crossing,b.crossing)==0); // late read cannot fabricate B-first
  }
  { assert(crossing_order({10,20,true},{20,30,true})==0);
    assert(crossing_order({10,20,true},{21,30,true})==1);
    assert(crossing_order({UINT32_MAX-30,UINT32_MAX-10,true},{10,30,true})==1); }
  auto timed_path=[](std::array<Crossing,2> rises, std::array<Crossing,2> falls) {
    Path p; p.observe(1,200,&rises); p.observe(3,300,&rises);
    p.observe(2,400,&falls); p.observe(0,500,&falls); return p;
  };
  const std::array<Crossing,2> clear_rise{{{100,140,true},{160,180,true}}};
  const std::array<Crossing,2> uncertain_rise{{{100,170,true},{110,180,true}}};
  const std::array<Crossing,2> clear_fall{{{310,340,true},{360,380,true}}};
  const std::array<Crossing,2> uncertain_fall{{{310,370,true},{320,380,true}}};
  assert(timed_path(uncertain_rise,uncertain_fall).direction(25)==0);
  assert(timed_path(clear_rise,uncertain_fall).direction(25)==1);
  assert(timed_path(uncertain_rise,clear_fall).direction(25)==1);
  assert(timed_path(clear_rise,{{clear_fall[1],clear_fall[0]}}).direction(25)==0);
  assert(timed_path({{clear_rise[1],clear_rise[0]}},uncertain_fall).direction(25)==0);
  // Sampling-phase ambiguity in the opposing track cannot veto two sound
  // tracks. A third genuinely contradictory direction must still veto them.
  for(bool strong_opposition:{false,true}) {
    Fusion f; f.required=2; f.required_health=3; f.require_timing_evidence=true;
    std::array<uint8_t,4> fresh{3,3,3,3};
    std::array<std::array<Crossing,2>,4> edges{};
    f.update(1,15,{},fresh,false,&edges); f.update(101,15,{},fresh,false,&edges);
    edges[0]=edges[1]=clear_rise;
    edges[2]=strong_opposition ? std::array<Crossing,2>{clear_rise[1],clear_rise[0]} : uncertain_rise;
    f.update(200,15,{1,1,2,0},fresh,false,&edges);
    f.update(300,15,{3,3,3,0},fresh,false,&edges);
    edges[0]=edges[1]=clear_fall;
    edges[2]=strong_opposition ? std::array<Crossing,2>{clear_fall[1],clear_fall[0]} : uncertain_fall;
    f.update(400,15,{2,2,1,0},fresh,false,&edges);
    f.update(500,15,{},fresh,false,&edges);
    const auto result=f.update(600,15,{},fresh,false,&edges);
    assert(result.decision==(strong_opposition?Decision::REJECTED:Decision::OUT));
    assert(result.uncertain_timing==(strong_opposition?0:1));
  }
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
  // Health requirement and direction agreement are independent settings.
  { Rig r;r.f.required=2;r.f.required_health=3;r.arm();
    r.tick(100,{1,1,0,0},7);r.tick(100,{2,2,0,0},7);r.tick(100,{},7);r.tick(100,{},7);
    assert(r.results.size()==1 && r.results[0].decision==Decision::OUT); }
  { Rig r;r.f.required=2;r.f.required_health=3;r.arm();
    r.tick(100,{1,1,0,0},3);r.tick(100,{2,2,0,0},3);r.tick(100,{},3);r.tick(100,{},3);
    assert(r.results.empty()); }
  { Rig r;r.f.required=2;r.f.required_health=3;r.arm();
    r.tick(100,{1,1,2,0});r.tick(100,{2,2,1,0});r.finish();
    assert(r.results[0].decision==Decision::REJECTED); }
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
