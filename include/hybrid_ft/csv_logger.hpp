#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <boost/tuple/tuple.hpp>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace hybrid_ft {

template <bool Position> struct PositionLogColumns {
    static void header(std::ostream&) {}
    template <typename T> static void row(std::ostream&, const T&) {}
};
template <> struct PositionLogColumns<true> {
    static void header(std::ostream& out) {
        out << ",pdes_x,pdes_y,pdes_z,pcur_x,pcur_y,pcur_z";
    }
    template <typename T> static void row(std::ostream& out, const T& record) {
        for (int i = 0; i < 3; ++i) out << ',' << record.template get<7>()[i];
        for (int i = 0; i < 3; ++i) out << ',' << record.template get<8>()[i];
    }
};

template <bool Filtered> struct FilteredForceLogColumns {
    static void header(std::ostream&) {}
    template <typename T> static void row(std::ostream&, const T&) {}
};
template <> struct FilteredForceLogColumns<true> {
    static void header(std::ostream& out) { out << ",ffiltered_x,ffiltered_y,ffiltered_z"; }
    template <typename T> static void row(std::ostream& out, const T& record) {
        for (int i = 0; i < 3; ++i) out << ',' << record.template get<9>()[i];
    }
};

/**
 * Samples TupleGrouper output at the same rate as PeriodicDataLogger (periodMultiplier)
 * Writes base-frame references/feedback plus elapsed time, sensor force,
 * joint positions, and the exact tool-to-world rotation used for force conversion.
 */
template <typename RecordTuple, bool Position = false, bool Filtered = false>
class HybridBaseCsvLogger : public barrett::systems::System,
                            public barrett::systems::SingleInput<RecordTuple> {
 public:
  HybridBaseCsvLogger(barrett::systems::ExecutionManager* em,
                        const std::string& path,
                        size_t periodMultiplier = 1,
                        const std::string& sysName = "HybridBaseCsvLogger")
      : barrett::systems::System(sysName),
        barrett::systems::SingleInput<RecordTuple>(this),
        logging_(true),
        haveStartTime_(false),
        ecCount_(0),
        ecCountRollover_(periodMultiplier == 0 ? 1 : periodMultiplier) {
    out_.open(path.c_str());
    if (!out_) {
      throw std::runtime_error("HybridBaseCsvLogger: failed to open " + path);
    }
    out_ << std::fixed << std::setprecision(8);
    out_ << "vdes_x,vdes_y,vdes_z,vcur_x,vcur_y,vcur_z,"
            "fdes_x,fdes_y,fdes_z,fcur_x,fcur_y,fcur_z,"
            "time_s,fsensor_x,fsensor_y,fsensor_z";
    typedef typename boost::tuples::element<5, RecordTuple>::type joint_position_type;
    for (int i = 0; i < joint_position_type::RowsAtCompileTime; ++i) {
      out_ << ",q" << i + 1;
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        out_ << ",R_world_tool_" << row << col;
      }
    }
    PositionLogColumns<Position>::header(out_);
    FilteredForceLogColumns<Filtered>::header(out_);
    out_ << '\n';
    out_.flush();
    if (em != NULL) {
      em->startManaging(*this);
    }
  }

  virtual ~HybridBaseCsvLogger() {
    this->mandatoryCleanUp();
    closeFile();
  }

  void closeFile() {
    if (out_.is_open()) {
      out_.flush();
      out_.close();
    }
    logging_ = false;
  }

 protected:
  virtual bool inputsValid() {
    ecCount_ = (ecCount_ + 1) % ecCountRollover_;
    return logging_ && ecCount_ == 0 && this->input.valueDefined();
  }

  virtual void operate() {
    const auto now = std::chrono::steady_clock::now();
    if (!haveStartTime_) {
      startTime_ = now;
      haveStartTime_ = true;
    }
    const RecordTuple& rec = this->input.getValue();
    writeVec3(out_, rec.template get<0>());
    out_ << ',';
    writeVec3(out_, rec.template get<1>());
    out_ << ',';
    writeVec3(out_, rec.template get<2>());
    out_ << ',';
    writeVec3(out_, rec.template get<3>());
    out_ << ',' << std::chrono::duration<double>(now - startTime_).count() << ',';
    writeVec3(out_, rec.template get<4>());
    const auto& positions = rec.template get<5>();
    for (int i = 0; i < positions.size(); ++i) out_ << ',' << positions[i];
    const auto& rotation = rec.template get<6>();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) out_ << ',' << rotation(row, col);
    }
    PositionLogColumns<Position>::row(out_, rec);
    FilteredForceLogColumns<Filtered>::row(out_, rec);
    out_ << '\n';
  }

  virtual void invalidateOutputs() {}

 private:
  template <typename Vec3>
  static void writeVec3(std::ofstream& o, const Vec3& v) {
    o << v[0] << ',' << v[1] << ',' << v[2];
  }

  std::ofstream out_;
  bool logging_;
  bool haveStartTime_;
  std::chrono::steady_clock::time_point startTime_;
  size_t ecCount_;
  size_t ecCountRollover_;

  DISALLOW_COPY_AND_ASSIGN(HybridBaseCsvLogger);
};

}  // namespace hybrid_ft
