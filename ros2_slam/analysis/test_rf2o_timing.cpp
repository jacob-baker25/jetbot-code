#include "rf2o_laser_odometry/TimingDiagnostics.hpp"
#include <cassert>
#include <sstream>
#include <iostream>

int main() {
  rf2o::TimingDiagnostics d;
  d.scan(10.,10.03,100.);
  d.scan(10.1,10.15,100.4);
  assert(d.scan_sequence==2);
  assert(d.metrics[d.SCAN_GAP].maximum>.39);
  assert(d.metrics[d.SCAN_AGE].maximum>.049);
  d.record(d.ODOM_PUBLISH,.45,100.5);
  assert(d.metrics[d.ODOM_PUBLISH].slow==1);
  assert(d.events[1].scan_sequence==2);
  for (int i=0;i<100;++i) d.record(d.SPIN,.3,101.+i);
  assert(d.event_count==102);
  std::ostringstream out;
  d.write(out);
  const auto text=out.str();
  assert(text.find("\"event_count\":102")!=std::string::npos);
  assert(text.find("\"end_mono_s\":200")!=std::string::npos);
  assert(text.find("\"end_mono_s\":101,")==std::string::npos);
  d.enabled=false;
  d.record(d.SPIN,2.,201.);
  d.scan(20.,20.,201.);
  assert(d.event_count==102 && d.scan_sequence==2);
  d.enabled=true;
  { rf2o::TimingScope scope(d,d.CALLBACK); }
  assert(d.metrics[d.CALLBACK].count==1);
  std::cout << text;
}
