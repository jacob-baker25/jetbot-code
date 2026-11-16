#pragma once

// Bounded, single-threaded diagnostics. No allocation or I/O in record().
// Durations include time descheduled; they are elapsed time, not CPU time.
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <ostream>

namespace rf2o {
class TimingDiagnostics {
public:
  enum Stage { SPIN, CALLBACK, COMPUTE, PROCESS, ODOM_PUBLISH, TF_PUBLISH,
               SLEEP, LOOP_GAP, SCAN_GAP, SCAN_AGE, STAGE_COUNT };
  struct Metric { uint64_t count=0, slow=0; double maximum=0; };
  struct Event {
    Stage stage=SPIN;
    double end_mono=0, seconds=0, scan_stamp=0;
    uint64_t scan_sequence=0;
  };
  static double now() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  std::array<Metric, STAGE_COUNT> metrics{};
  std::array<Event, 64> events{};
  uint64_t event_count=0, scan_sequence=0;
  double scan_stamp=0, previous_scan_receipt=0;
  bool enabled=true;

  static const char *name(Stage stage) {
    static const char *names[]={"spin", "scan_callback", "compute_pose_log",
      "process_total", "odom_publish", "tf_publish", "rate_sleep",
      "loop_gap", "scan_callback_gap", "scan_age_at_callback"};
    return names[stage];
  }
  void record(Stage stage, double seconds, double end) {
    if (!enabled) return;
    auto &m=metrics[stage];
    ++m.count;
    if (seconds>m.maximum) m.maximum=seconds;
    const double threshold=(stage==LOOP_GAP || stage==SLEEP || stage==SCAN_GAP) ? .2 : .1;
    if (seconds>threshold) {
      ++m.slow;
      Event e; e.stage=stage; e.seconds=seconds; e.end_mono=end;
      e.scan_stamp=scan_stamp; e.scan_sequence=scan_sequence;
      events[event_count % events.size()]=e;
      ++event_count;
    }
  }
  void scan(double stamp, double ros_now, double receipt) {
    if (!enabled) return;
    scan_stamp=stamp; ++scan_sequence;
    if (previous_scan_receipt!=0) record(SCAN_GAP, receipt-previous_scan_receipt, receipt);
    previous_scan_receipt=receipt;
    record(SCAN_AGE, ros_now-stamp, receipt);
  }
  void write(std::ostream &out) const {
    out << std::setprecision(17) << "RF2O_TIMING_SUMMARY {\"version\":1,\"scan_callbacks\":"
        << scan_sequence << ",\"event_count\":" << event_count << ",\"metrics\":{";
    for (int i=0;i<STAGE_COUNT;++i) {
      if (i) out << ',';
      out << '\"' << name(static_cast<Stage>(i)) << "\":{\"count\":" << metrics[i].count
          << ",\"slow_count\":" << metrics[i].slow << ",\"max_s\":" << metrics[i].maximum << '}';
    }
    out << "},\"events\":[";
    const uint64_t retained=event_count<events.size() ? event_count : events.size();
    for (uint64_t i=0;i<retained;++i) {
      if (i) out << ',';
      const auto &e=events[(event_count-retained+i)%events.size()];
      out << "{\"stage\":\"" << name(e.stage) << "\",\"end_mono_s\":" << e.end_mono
          << ",\"seconds\":" << e.seconds << ",\"scan_stamp_s\":" << e.scan_stamp
          << ",\"scan_sequence\":" << e.scan_sequence << '}';
    }
    out << "]}" << std::endl;
  }
};

// Nested stages intentionally overlap; do not add their durations together.
class TimingScope {
  TimingDiagnostics &data;
  TimingDiagnostics::Stage stage;
  double start;
public:
  TimingScope(TimingDiagnostics &d, TimingDiagnostics::Stage s):
      data(d), stage(s), start(d.enabled ? TimingDiagnostics::now() : 0) {}
  ~TimingScope() {
    if (data.enabled) {
      const double end=TimingDiagnostics::now();
      data.record(stage,end-start,end);
    }
  }
  TimingScope(const TimingScope &)=delete;
  TimingScope &operator=(const TimingScope &)=delete;
};
} // namespace rf2o
