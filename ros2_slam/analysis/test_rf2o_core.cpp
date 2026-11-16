#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

sensor_msgs::msg::LaserScan room(double x,double y,double yaw,double time) {
  sensor_msgs::msg::LaserScan s;
  s.header.stamp.sec=int(time);
  s.header.stamp.nanosec=uint32_t(std::round((time-int(time))*1e9));
  s.angle_increment=float(2*M_PI/360);
  s.angle_min=-179.5f*float(M_PI)/180;
  s.angle_max=-s.angle_min;
  for(int i=0;i<360;++i) {
    double a=s.angle_min+i*s.angle_increment+yaw,dx=std::cos(a),dy=std::sin(a);
    double tx=std::abs(dx)<1e-10 ? 1e10 : ((dx>0?5.:-3.)-x)/dx;
    double ty=std::abs(dy)<1e-10 ? 1e10 : ((dy>0?3.:-2.)-y)/dy;
    s.ranges.push_back(float(std::min(tx,ty)));
  }
  return s;
}
bool step(rf2o::CLaserOdometry2D &o,const sensor_msgs::msg::LaserScan &s) {
  o.current_scan_time=s.header.stamp;
  return o.odometryCalculation(s);
}

void init(rf2o::CLaserOdometry2D &o, bool legacy=false) {
  geometry_msgs::msg::Pose pose; pose.orientation.w=1;
  o.suppress_small_motion=legacy;
  o.init(room(0,0,0,1),pose);
}
void require(bool ok,const char *why) {
  if (!ok) throw std::runtime_error(why);
}
int main() {
  rf2o::CLaserOdometry2D still; init(still);
  for(int i=1;i<=100;++i)
    require(step(still,room(0,0,0,1+i*.1)),"identical scans must remain valid");
  require(still.getPose().translation().norm()<1e-5,"stationary synthetic drift");
  require(std::abs(rf2o::getYaw(still.getPose().rotation()))<1e-5,"stationary yaw");

  for(int sign : {-1,1}) {
    rf2o::CLaserOdometry2D slow,legacy; init(slow); init(legacy,true);
    for(int i=1;i<=20;++i) {
      const auto scan=room(sign*i*.005,0,0,1+i*.1);
      require(step(slow,scan),"slow translation rejected");
      require(step(legacy,scan),"legacy comparison rejected");
    }
    std::cout << "slow_expected=" << sign*.1 << " actual=" << slow.getPose().translation().x()
              << " legacy=" << legacy.getPose().translation().x() << '\n';
    require(std::abs(slow.getPose().translation().x()-sign*.1)<.02,"small real motion lost");
    require(std::abs(legacy.getPose().translation().x())<.005,"legacy deadband control not reproduced");
    rf2o::CLaserOdometry2D turn; init(turn);
    for(int i=1;i<=10;++i)
      require(step(turn,room(0,0,sign*i*.01,1+i*.1)),"turn rejected");
    const double yaw=rf2o::getYaw(turn.getPose().rotation());
    std::cout << "turn_expected=" << sign*.1 << " actual=" << yaw << '\n';
    require(std::abs(yaw-sign*.1)<.02,"turn sign/magnitude");
  }

  rf2o::CLaserOdometry2D recovery,control; init(recovery); init(control);
  require(step(recovery,room(.02,0,0,1.1)),"first valid step");
  require(step(control,room(.02,0,0,1.1)),"control first step");
  const auto pose=recovery.getPose(); const auto prior=recovery.kai_loc_old_;
  const auto covariance=recovery.getIncrementCovariance(); const auto reference=recovery.range;
  const double stamp=recovery.last_odom_time.seconds();
  for(double time : {1.2,1.3}) {
    auto bad=room(.04,0,0,time);
    for(auto &range : bad.ranges) range=std::numeric_limits<float>::quiet_NaN();
    require(!step(recovery,bad),"missing scan must be rejected");
    require(recovery.last_odom_time.seconds()==stamp,"failure advanced measurement time");
    require(recovery.getPose().matrix().isApprox(pose.matrix()),"failure advanced pose");
    require(recovery.kai_loc_old_.isApprox(prior),"failure corrupted motion prior");
    require(recovery.getIncrementCovariance().isApprox(covariance),"failure corrupted covariance");
    for(size_t i=0;i<reference.size();++i)
      require(recovery.range[i].isApprox(reference[i]),"failure advanced reference scan");
  }
  const auto next=room(.06,0,.01,1.4);
  require(step(recovery,next),"failed to recover from short rejected interval");
  require(step(control,next),"control spanning interval failed");
  require(recovery.getPose().matrix().isApprox(control.getPose().matrix(),1e-5),
          "recovery lost motion over failed pair");
  require(recovery.rejected_estimates==2 && recovery.accepted_estimates==2,"validity counters");
  require(!step(recovery,next),"duplicate stamp accepted");
  auto short_scan=room(0,0,0,1.5); short_scan.ranges.resize(10);
  require(!step(recovery,short_scan),"short scan accepted");
  auto changed=room(0,0,0,1.5); changed.angle_increment*=2;
  require(!step(recovery,changed),"changed geometry accepted");
  require(!step(recovery,room(.07,0,0,2)),"unbounded reference gap accepted");
  require(std::string(recovery.failure_reason)=="reference_expired","wrong expiry reason");
  require(!step(recovery,room(.07,0,0,2.1)),"expired reference silently reset");

  rf2o::CLaserOdometry2D degenerate; init(degenerate);
  auto empty=room(0,0,0,1.1); for(auto &r:empty.ranges) r=std::numeric_limits<float>::infinity();
  require(!step(degenerate,empty),"all infinite returns accepted");
  require(step(degenerate,room(.02,0,0,1.2)),"invalid range recovery");
  degenerate.num_valid_range=6; degenerate.cols_i=8; degenerate.image_level=0;
  for(int i=1;i<7;++i) { degenerate.null(i)=0; degenerate.weights(i)=0; }
  require(!degenerate.solveSystemNonLinear(),"rank-zero matrix accepted");
  degenerate.cov_odo(0,0)=std::numeric_limits<float>::quiet_NaN();
  require(!degenerate.filterLevelSolution(),"nonfinite covariance accepted");
  std::cout << "RF2O_CORE_TEST_PASS identical, signed slow motion/turns, invalid inputs, rollback, expiry\n";
}
