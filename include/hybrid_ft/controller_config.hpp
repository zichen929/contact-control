#pragma once

#include <libconfig.h++>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hybrid_ft {
struct ControllerConfig {
    std::array<double, 3> kv, kd, kf, ki, motionAxes, desiredForce, forceLimit;
    std::array<double, 7> target, holdKp, holdKd;
    std::array<double, 3> positionKp, positionKd;
    double positionDistance = 0.1, positionDuration = 6.0;
    std::string velocityPattern = "sine";
    double velocityDistance = 0.10, velocityDuration = 6.0;
    std::string positionPattern = "move";
    double positionAmplitude = 0.05, positionPeriod = 6.0;
    double forceFilterCutoffHz = 0.0;
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

    static ControllerConfig load(const std::string& path, bool positionControl = false) {
        libconfig::Config c;
        c.setAutoConvert(true);
        c.readFile(path.c_str());
        ControllerConfig r{};
        if (positionControl) {
            r.positionKp = vector<3>(c, "position_kp", true);
            r.positionKd = vector<3>(c, "position_kd", true);
            r.positionDistance = c.lookup("position_distance");
            r.positionDuration = c.lookup("position_duration");
            if (c.exists("position_pattern"))
                r.positionPattern = static_cast<const char*>(c.lookup("position_pattern"));
            if (r.positionPattern != "move" && r.positionPattern != "sine")
                throw std::runtime_error("position_pattern must be move or sine");
            if (r.positionPattern == "sine") {
                r.positionAmplitude = c.lookup("position_amplitude");
                r.positionPeriod = c.lookup("position_period");
                if (!std::isfinite(r.positionAmplitude) || r.positionAmplitude < 0 ||
                    !std::isfinite(r.positionPeriod) || r.positionPeriod <= 0)
                    throw std::runtime_error("Invalid sine position amplitude/period");
            }
            if (!std::isfinite(r.positionDistance) || !std::isfinite(r.positionDuration) || r.positionDuration <= 0)
                throw std::runtime_error("Invalid position trajectory setting");
            r.kv.fill(0); r.kd.fill(0);
        } else {
            r.kv = vector<3>(c, "velocity_kp", true);
            r.kd = vector<3>(c, "velocity_kd", true);
            if (c.exists("velocity_pattern"))
                r.velocityPattern = static_cast<const char*>(c.lookup("velocity_pattern"));
            if (r.velocityPattern != "sine" && r.velocityPattern != "move")
                throw std::runtime_error("velocity_pattern must be sine or move");
            if (r.velocityPattern == "move") {
                r.velocityDistance = c.lookup("velocity_distance");
                r.velocityDuration = c.lookup("velocity_duration");
                if (!std::isfinite(r.velocityDistance) || !std::isfinite(r.velocityDuration) || r.velocityDuration <= 0)
                    throw std::runtime_error("Invalid velocity move distance/duration");
            }
        }
        r.kf = vector<3>(c, "force_kp", true);
        r.ki = vector<3>(c, "force_ki", true);
        r.motionAxes = vector<3>(c, "motion_axes", true);
        for (double v : r.motionAxes)
            if (v != 0 && v != 1) throw std::runtime_error("motion_axes entries must be 0 or 1");
        r.desiredForce = vector<3>(c, "desired_force");
        if (c.exists("force_filter_cutoff_hz"))
            r.forceFilterCutoffHz = c.lookup("force_filter_cutoff_hz");
        if (!std::isfinite(r.forceFilterCutoffHz) || r.forceFilterCutoffHz < 0)
            throw std::runtime_error("force_filter_cutoff_hz must be finite and nonnegative");
        r.forceLimit = vector<3>(c, "force_feedback_limit", true);
        r.target = vector<7>(c, "target_position");
        r.useRobotHoldGains = c.lookup("use_robot_hold_gains");
        r.holdKp = vector<7>(c, "hold_kp", true);
        r.holdKd = vector<7>(c, "hold_kd", true);
        r.integralLimit = c.lookup("force_integral_limit");
        r.forceFeedforward = c.lookup("force_feedforward_gain");
        r.peakVelocity = (positionControl || r.velocityPattern == "move") ? 0.0 : double(c.lookup("velocity_peak"));
        r.velocityPeriod = (positionControl || r.velocityPattern == "move") ? 1.0 : double(c.lookup("velocity_period"));
        r.velocityAxis = positionControl ? 0 : int(c.lookup("velocity_axis"));
        if (positionControl && r.motionAxes != std::array<double, 3>{{1, 0, 0}})
            throw std::runtime_error("Position controller requires motion_axes = [1, 0, 0]");
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
}  // namespace hybrid_ft
