#include "StateMachineMulti.h"


MultiStateMachine::MultiStateMachine(): _lastToState(""), _running(false){
}

void MultiStateMachine::setRobot(std::unique_ptr<Robot> r) {
    spdlog::debug("MultiStateMachine::setRobot() -> addRobot()");
    // For the multi-robot state machine, setRobot will append to the robot vector.
    _robots.push_back(std::move(r));
}

bool MultiStateMachine::configureMasterPDOs() {
    spdlog::debug("MultiStateMachine::configureMasterPDOs()");
    if(_robots.empty()) {
        return false;
    }
    bool all_ok = true;
    for(auto &rptr : _robots) {
        if(rptr) {
            all_ok = all_ok && rptr->configureMasterPDOs();
        }
        else {
            all_ok = false;
        }
    }
    return all_ok;
}

void MultiStateMachine::setInitState(std::string state_name) {
    _currentState = state_name;
    if(_states.count(_currentState)<1) {
        spdlog::error("Requested initial state {} does not exists. Left to default.", _currentState);
    }
}

void MultiStateMachine::addState(std::string state_name, std::shared_ptr<State> s_ptr) {
   spdlog::debug("MultiStateMachine::addState({})", state_name);
   _states[state_name]=s_ptr;
   if(_states.size()==1) {
      _currentState=state_name;
   }
}

void MultiStateMachine::addTransition(std::string from, TransitionCb_t t_cb, std::string to) {
    spdlog::debug("MultiStateMachine::addTransition({} -> {})", from, to);
    if(_states.count(from)>0 && _states.count(to)>0) {
        _transitions[from].push_back(Transition_t(t_cb, to));
        _lastToState = to;
    }
    else {
        spdlog::error("State {} or {} do not exist. Cannot create requested transition.", from, to);
    }
}

void MultiStateMachine::addTransitionFromLast(TransitionCb_t t_cb, std::string to) {
    if(_lastToState.empty()) {
        spdlog::error("No last state transition registerd. Cannot create requested transition.");
        return;
    }
    else {
        spdlog::debug("MultiStateMachine::addTransitionFromLast({} -> {})", _lastToState, to);
        if(_states.count(_lastToState)>0 && _states.count(to)>0) {
            _transitions[_lastToState].push_back(Transition_t(t_cb, to));
            _lastToState = to;
        }
        else {
            spdlog::error("State {} or {} do not exist. Cannot create requested transition.", _lastToState, to);
        }
    }
}

void MultiStateMachine::addTransitionFromAny(TransitionCb_t t_cb, std::string to) {
    spdlog::debug("MultiStateMachine::addTransition(ANY -> {})", to);
    if(_states.count(to)>0) {
        for(const auto& [key, s]: _states) {
            if(key!=to) {
                _transitions[key].push_back(Transition_t(t_cb, to));
            }
        }
        _lastToState = to;
    }
    else {
        spdlog::error("State {} do not exist. Cannot create requested transitions.", to);
    }
}

void MultiStateMachine::activate() {
    spdlog::debug("MultiStateMachine::activate()");
    if(_states.count(_currentState)>0) {
        _running = true;
        _time_init = std::chrono::steady_clock::now();
        _time_running = 0;
        if(logHelper.isInitialised()) {
            logHelper.startLogger();
        }
        _states[_currentState]->doEntry();
    }
    else {
        spdlog::critical("StateMachine activation state ({}) does not exist. Exiting...", _currentState);
        std::raise(SIGTERM); //Clean exit
    }
}

void MultiStateMachine::update() {
    spdlog::trace("MultiStateMachine::update()");

    _time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - _time_init).count()) / 1e6;

    hwStateUpdate();

    bool transitioned = false;
    for (auto& tr : _transitions[_currentState]) {
        if(tr.first(*this)) {
            _states[_currentState]->doExit();
            _currentState=tr.second;
            _states[_currentState]->doEntry();
            transitioned=true;
            break;
        }
    }

    if(!transitioned) {
        _states[_currentState]->doDuring();
    }

    if(logHelper.isStarted() && logHelper.isInitialised())
        logHelper.recordLogData();
}

void MultiStateMachine::hwStateUpdate() {
    spdlog::trace("MultiStateMachine::hwStateUpdate()");
    for(auto &rptr : _robots) {
        if(rptr) rptr->updateRobot();
    }
}

void MultiStateMachine::end() {
    if(running()) {
        if(logHelper.isInitialised())
            logHelper.endLog();
        state()->doExit();
        for(auto &rptr : _robots) {
            if(rptr) rptr->disable();
        }
    }
    _running=false;
}
