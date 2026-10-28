#pragma once

#include <libconfig.h++>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hybrid_control_ft {
struct ControllerConfig {
    std::array<double, 3> kv, kd, kf, ki, motionAxes, desiredForce, forceLimit;
    std::array<double, 7> target, holdKp, holdKd;
    double integralLimit, peakVelocity, velocityPeriod, forceFeedforward;
    int velocityAxis, logPeriod;
    bool useRobotHoldGains;
    std::string logDirectory;

    template <size_t N>
    static std::array<double, N> vector(const libconfig::Config& c,
                                       const char* name, bool nonnegative = false) {
        const auto& setting = c.lookup(name);
        if (!(setting.isArray() || setting.isList()) || setting.getLength() != N)
            throw std::runtime_error(std::string(name) + ": wrong vector length");
        std::array<double, N> result;
        for (size_t i = 0; i < N; ++i) {
            result[i] = static_cast<double>(setting[static_cast<int>(i)]);
            if (!std::isfinite(result[i]) || (nonnegative && result[i] < 0))
                throw std::runtime_error(std::string(name) + ": invalid value");
        }
        return result;
    }

    static ControllerConfig load(const std::string& path) {
        libconfig::Config c;
        c.setAutoConvert(true);
        c.readFile(path.c_str());
        ControllerConfig r;
        r.kv = vector<3>(c, "velocity_kp", true);
        r.kd = vector<3>(c, "velocity_kd", true);
        r.kf = vector<3>(c, "force_kp", true);
        r.ki = vector<3>(c, "force_ki", true);
        r.motionAxes = vector<3>(c, "motion_axes", true);
        for (double v : r.motionAxes)
            if (v != 0 && v != 1) throw std::runtime_error("motion_axes entries must be 0 or 1");
        r.desiredForce = vector<3>(c, "desired_force");
        r.forceLimit = vector<3>(c, "force_feedback_limit", true);
        r.target = vector<7>(c, "target_position");
        r.useRobotHoldGains = c.lookup("use_robot_hold_gains");
        r.holdKp = vector<7>(c, "hold_kp", true);
        r.holdKd = vector<7>(c, "hold_kd", true);
        r.integralLimit = c.lookup("force_integral_limit");
        r.forceFeedforward = c.lookup("force_feedforward_gain");
        r.peakVelocity = c.lookup("velocity_peak");
        r.velocityPeriod = c.lookup("velocity_period");
        r.velocityAxis = c.lookup("velocity_axis");
        r.logPeriod = c.lookup("log_period_multiplier");
        r.logDirectory = static_cast<const char*>(c.lookup("log_directory"));
        if (!std::isfinite(r.integralLimit) || r.integralLimit <= 0 ||
            !std::isfinite(r.forceFeedforward) || r.forceFeedforward < 0 ||
            !std::isfinite(r.peakVelocity) || r.peakVelocity < 0 ||
            !std::isfinite(r.velocityPeriod) || r.velocityPeriod <= 0 ||
            r.velocityAxis < 0 || r.velocityAxis > 2 || r.logPeriod < 1 || r.logDirectory.empty())
            throw std::runtime_error("Invalid scalar tuning setting");
        return r;
    }
};
}  // namespace hybrid_control_ft
