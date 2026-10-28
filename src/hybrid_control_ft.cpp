#include "hybrid_control_ft/barrett_ft_force.hpp"
#include "hybrid_control_ft/sensor_force_to_base.hpp"
#include "hybrid_control_ft/jacobian.hpp"
#include "hybrid_control_ft/references.hpp"
#include "hybrid_control_ft/force_clamp.hpp"
#include "hybrid_control_ft/base_frame_controller.hpp"
#include "hybrid_control_ft/joint_torque.hpp"
#include "hybrid_control_ft/csv_logger.hpp"

#include <barrett/systems/tuple_grouper.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/detail/ca_macro.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

/** Ensures data dir exists; path hybrid_base_ft_datalog_NNNN.csv (NNNN = 1 + max existing .csv). */
std::string nextHybridBaseLogPath(const std::string& directory)
{
    const char* dataDir = directory.c_str();
    const char* prefix = "hybrid_base_ft_datalog_";
    mkdir(dataDir, 0755);

    int maxN = 0;
    DIR* d = opendir(dataDir);
    if (d) {
        const size_t prefixLen = std::strlen(prefix);
        while (struct dirent* e = readdir(d)) {
            const char* name = e->d_name;
            if (std::strncmp(name, prefix, prefixLen) != 0) {
                continue;
            }
            char* endp = NULL;
            long v = std::strtol(name + prefixLen, &endp, 10);
            if (endp == name + prefixLen) {
                continue;
            }
            if (std::strcmp(endp, ".csv") != 0) {
                continue;
            }
            if (v > maxN) {
                maxN = static_cast<int>(v);
            }
        }
        closedir(d);
    }

    const int nextN = maxN + 1;
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s/%s%04d.csv", dataDir, prefix, nextN);
    return std::string(buf);
}

}  // namespace

template<size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam)
{
    using namespace hybrid_control_ft;
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef typename barrett::math::Vector<3>::type base_vector_force_type;
    typedef typename barrett::math::Vector<3>::type base_vector_velocity_type;

    const char* configOverride = std::getenv("HYBRID_CONTROL_FT_CONFIG");
    const std::string tuningPath = configOverride && configOverride[0]
        ? configOverride : HYBRID_CONTROL_FT_DEFAULT_CONFIG;
    ControllerConfig tuning;
    try {
        tuning = ControllerConfig::load(tuningPath);
    } catch (const libconfig::ParseException& e) {
        std::cerr << "Tuning config " << tuningPath << ":" << e.getLine()
                  << ": " << e.getError() << std::endl;
        return 1;
    } catch (const libconfig::SettingException& e) {
        std::cerr << "Invalid/missing tuning setting in " << tuningPath
                  << ": " << e.getPath() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Cannot load tuning config " << tuningPath << ": " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Tuning config: " << tuningPath << std::endl;

    if (DOF < 4) {
        std::cerr << "Planar control requires joints 2 and 4." << std::endl;
        return 1;
    }

    if (!pm.foundForceTorqueSensor() || pm.getForceTorqueSensor() == NULL) {
        std::cerr << "Barrett F/T sensor not detected." << std::endl;
        return 1;
    }
    BarrettFTForce<DOF> measuredForce(*pm.getForceTorqueSensor());
    SensorForceToBase<DOF> sensorForceToBase;
    barrett::systems::connect(measuredForce.output, sensorForceToBase.input);
    barrett::systems::connect(wam.kinematicsBase.kinOutput, sensorForceToBase.kinInput);

    ToolJacobianOutput<DOF> toolJacobian(wam, pm.getExecutionManager());
    HybridForceVelocityControl<DOF> hybridControl(pm.getExecutionManager(), tuning);
    v_type holdKp = wam.jpController.getKp(), holdKd = wam.jpController.getKd();
    if (!tuning.useRobotHoldGains) {
        for (size_t i = 0; i < DOF; ++i) {
            holdKp[i] = tuning.holdKp[i]; holdKd[i] = tuning.holdKd[i];
        }
    }
    BaseControlToJointTorque<DOF> controlToJoint(
        pm.getExecutionManager(), holdKp, holdKd);
    barrett::systems::connect(wam.jpOutput, controlToJoint.jointPositionIn);
    barrett::systems::connect(wam.jvOutput, controlToJoint.jointVelocityIn);
    // Both signals use base axes. Negate force only for robot-on-environment
    // controller feedback and CSV logging; printed sensor/base forces retain
    // the environment-on-robot convention.
    barrett::systems::Gain<cv_type, double, base_vector_velocity_type> baseVelocity(1.0);
    barrett::systems::Gain<cf_type, double, base_vector_force_type> baseForce(-1.0);
    barrett::systems::connect(wam.toolVelocity.output, baseVelocity.input);
    barrett::systems::connect(sensorForceToBase.output, baseForce.input);

    // All references, feedback, and controller output use the base frame.
    base_vector_force_type desiredForce;
    for (int i = 0; i < 3; ++i) desiredForce[i] = tuning.desiredForce[i];

    BaseVelocitySinusoid<base_vector_velocity_type> desiredVelocityTraj(
        pm.getExecutionManager(), tuning.velocityAxis, tuning.peakVelocity, tuning.velocityPeriod,
        "DesiredVelTraj");

    base_vector_force_type force_limit;
    for (int i = 0; i < 3; ++i) force_limit[i] = tuning.forceLimit[i];
    ForceClamp<DOF> forceClamp(pm.getExecutionManager(), force_limit);

    ConstantOutput<base_vector_force_type> desiredForceSource(pm.getExecutionManager(), desiredForce, "DesiredForce");

    barrett::systems::connect(desiredVelocityTraj.output, hybridControl.desiredVelocityIn);
    barrett::systems::connect(desiredForceSource.output, hybridControl.desiredForceIn);
    barrett::systems::connect(baseVelocity.output, hybridControl.currentVelocityIn);
    barrett::systems::connect(baseForce.output, forceClamp.input);
    barrett::systems::connect(forceClamp.output, hybridControl.currentForceIn);


    // Cartesian torques drive joints 2/4; all other joints hold their activation pose.
    barrett::systems::connect(hybridControl.controlOutput, controlToJoint.controlBaseIn);
    barrett::systems::connect(toolJacobian.output, controlToJoint.jacobianIn);
    
    barrett::systems::PrintToStream<cf_type> printSensorForce(pm.getExecutionManager(), "sensorForce (environment on robot): ");
    barrett::systems::PrintToStream<cf_type> printCartesianForce(pm.getExecutionManager(), "baseForce (environment on robot): ");
    barrett::systems::PrintToStream<jt_type> printJointCommand(pm.getExecutionManager(), "jointCommand: ");
    barrett::systems::connect(measuredForce.output, printSensorForce.input);
    barrett::systems::connect(sensorForceToBase.output, printCartesianForce.input);
    barrett::systems::connect(controlToJoint.jointTorqueOut, printJointCommand.input);

    jp_type target;
    for (size_t i = 0; i < DOF; ++i) target[i] = tuning.target[i];

    //datalog
    typedef barrett::systems::TupleGrouper<
    base_vector_velocity_type,
    base_vector_velocity_type,
    base_vector_force_type,
    base_vector_force_type,
    cf_type,
    jp_type,
    typename SensorForceToBase<DOF>::rotation_type> hybrid_base_log_grouper_t;
    hybrid_base_log_grouper_t hybridBaseLogGroup("HybridBaseLogGroup");
    pm.getExecutionManager()->startManaging(hybridBaseLogGroup);
    typedef typename hybrid_base_log_grouper_t::tuple_type hybrid_base_log_record_t;

    barrett::systems::connect(desiredVelocityTraj.output, hybridBaseLogGroup.template getInput<0>());
    barrett::systems::connect(baseVelocity.output, hybridBaseLogGroup.template getInput<1>());
    barrett::systems::connect(desiredForceSource.output, hybridBaseLogGroup.template getInput<2>());
    barrett::systems::connect(baseForce.output, hybridBaseLogGroup.template getInput<3>());
    barrett::systems::connect(measuredForce.output, hybridBaseLogGroup.template getInput<4>());
    barrett::systems::connect(wam.jpOutput, hybridBaseLogGroup.template getInput<5>());
    barrett::systems::connect(sensorForceToBase.rotationOutput, hybridBaseLogGroup.template getInput<6>());

    const size_t kHybridLogPeriodMult = tuning.logPeriod;

    HybridBaseCsvLogger<hybrid_base_log_record_t>* hybridBaseLogger = NULL;

    bool contactActive = false;

    std::cout << "\nCommands:\n";
    std::cout << "  t : move to target and zero F/T sensor\n";
    std::cout << "  c : start contact controller\n";
    std::cout << "  x : stop contact controller\n";
    std::cout << "  ENTER : move home\n";
    std::cout << "  q : quit\n";
    std::cout << "  (datalog directory: " << tuning.logDirectory << ")\n";

    barrett::systems::ExecutionManager* const em = pm.getExecutionManager();

    auto stopHybridDatalog = [&]() {
        if (hybridBaseLogger == NULL) {
            return;
        }
        barrett::systems::disconnect(hybridBaseLogger->input);
        hybridBaseLogger->closeFile();
        delete hybridBaseLogger;
        hybridBaseLogger = NULL;
    };

    auto startHybridDatalog = [&]() {
        const std::string logPath = nextHybridBaseLogPath(tuning.logDirectory);
        hybridBaseLogger = new HybridBaseCsvLogger<hybrid_base_log_record_t>(
            em, logPath, kHybridLogPeriodMult, "HybridBaseCsvLogger");
        barrett::systems::forceConnect(hybridBaseLogGroup.output, hybridBaseLogger->input);
        std::cout << "CSV datalog (base frame) -> " << logPath << std::endl;
    };

    wam.gravityCompensate();

    while (true) {
        std::cout << "\nEnter command: ";

        if (std::cin.peek() == '\n') {
            std::cin.get();

            if (contactActive) {
                barrett::systems::disconnect(wam.input);
                hybridControl.resetState();
                stopHybridDatalog();
                contactActive = false;
            }


            std::cout << "Returning home..." << std::endl;
            wam.moveHome();
            std::cout << "At home." << std::endl;

            continue;
        }

        

        char cmd;
        std::cin >> cmd;

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


        if (cmd == 'q') {
            std::cout << "Exiting..." << std::endl;

            if (contactActive) {
                barrett::systems::disconnect(wam.input);
                stopHybridDatalog();
                contactActive = false;
            }

            break;
        }

        switch (cmd) {

            case 't':
                if (contactActive) {
                    barrett::systems::disconnect(wam.input);
                    hybridControl.resetState();
                    stopHybridDatalog();
                    contactActive = false;
                }


                std::cout << "Moving to target..." << std::endl;
                wam.moveTo(target, true);
                std::cout << "At target. Zeroing F/T sensor..." << std::endl;
                {
                    BARRETT_SCOPED_LOCK(em->getMutex());
                    pm.getForceTorqueSensor()->tare();
                }
                std::cout << "F/T sensor zeroed." << std::endl;
                break;


            case 'c':
                if (!contactActive) {
                    std::cout << "Starting contact controller..." << std::endl;

                    hybridControl.resetState();
                    desiredVelocityTraj.resetPhase();

                    startHybridDatalog();


                    {
                        BARRETT_SCOPED_LOCK(em->getMutex());
                        controlToJoint.setHoldPosition(wam.getJointPositions());
                        wam.trackReferenceSignal(controlToJoint.jointTorqueOut);
                    }

                    contactActive = true;
                } else {
                    std::cout << "Contact controller already active." << std::endl;
                }
                break;

            case 'x':
                if (contactActive) {
                    std::cout << "Stopping contact controller..." << std::endl;

                    barrett::systems::disconnect(wam.input);
                    hybridControl.resetState();
                    stopHybridDatalog();


                    contactActive = false;
                } else {
                    std::cout << "Contact controller not active." << std::endl;
                }
                break;

            default:
                std::cout << "Unknown command." << std::endl;
                break;
        }
    }

    return 0;
}
