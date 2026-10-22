 /**
 * \file application.c
 * \author William Campbell, Justin Fong, Vincent Crocher
 * \version 0.3
 * \date 2021-12-18
 * \copyright Copyright (c) 2020 - 2021
 *
 * \brief  Application interface of CORC. Based on CANopenSocket.
 *
 */
#include "application.h"

#include <string>

//Select state machine to use for this application (can be set in cmake)
#ifndef STATE_MACHINE_TYPE
#error "State Machine Type not defined"
#endif

std::unique_ptr<STATE_MACHINE_TYPE> stateMachine;
static std::string canInterfaceName;

void app_setCANInterface(const char *interfaceName) {
    canInterfaceName = interfaceName ? interfaceName : "";
}

const char *app_getCANInterface() {
    return canInterfaceName.c_str();
}

/******************** RUNS BEFORE CO_init() ********************/
void app_communicationReset(int argc, char *argv[]) {
#if ROS > 0
    stateMachine = std::make_unique<STATE_MACHINE_TYPE>(argc, argv);
#else
    // A robot that cannot be configured (e.g. its parameter file fails to load) throws from its
    // constructor. Catching here unwinds the partly built state machine, so its destructors run
    // (restoring the terminal, closing devices), before exiting. No drive has been started yet.
    try {
        stateMachine = std::make_unique<STATE_MACHINE_TYPE>();
    } catch (const std::exception &e) {
        spdlog::critical("Startup aborted: {}", e.what());
        std::exit(EXIT_FAILURE);
    }
#endif
    stateMachine->configureMasterPDOs();
}

/******************** Runs at the Start of rt_control_thread********************/
void app_programStart(void) {
    spdlog::info("CORC Start application");

#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
#endif  // NOROBOT
    spdlog::info("Application thread running at {}Hz.", (int)(1000./(float)controlLoopPeriodInms));
    stateMachine->init();
    stateMachine->activate();
}

/******************** Runs in low priority thread ********************/
void app_programAsync(uint16_t timer1msDiffy) {
}

/******************** Runs in rt_control_thread ********************/
void app_programControlLoop(void) {
    std::chrono::steady_clock::time_point _t0 = std::chrono::steady_clock::now();

    //StateMachine execution
    if (stateMachine->running()) {
        stateMachine->update();
    }

    //Warn if time overflow (this is the effective used time, normally lower than the allocated time period).
    //Real stalls (> 10 ms) are reported at once; marginal overruns are summed and reported at most
    //every 2 s, since printing every one from this thread would itself cause more of them.
    static int overruns = 0;
    static double worstMs = 0.0;
    static std::chrono::steady_clock::time_point lastReport = _t0;
    const auto t1 = std::chrono::steady_clock::now();
    double dt = (std::chrono::duration_cast<std::chrono::microseconds>(t1 - _t0).count()) / 1e6;
    if(dt>controlLoopPeriodInms/1000.) {
        if(dt*1000. > 10.0) {
            spdlog::warn("Applicaton thread time overflow: {:.1f}ms (>{}ms) !", dt*1000., controlLoopPeriodInms);
        } else {
            overruns++;
            worstMs = std::max(worstMs, dt*1000.);
        }
    }
    if(overruns > 0 && t1 - lastReport > std::chrono::seconds(2)) {
        spdlog::warn("Applicaton thread time overflow: {} cycle(s) over {}ms in the last 2 s (worst {:.2f}ms)",
                     overruns, controlLoopPeriodInms, worstMs);
        overruns = 0;
        worstMs = 0.0;
        lastReport = t1;
    } else if(overruns == 0) {
        lastReport = t1;
    }
}

/******************** Runs at the End of rt_control_thread********************/
void app_programEnd(void) {
    stateMachine->end();
    stateMachine.reset(); //Explicit delete of the state machine to answer deletion on time
    spdlog::info("CORC End application");
}
