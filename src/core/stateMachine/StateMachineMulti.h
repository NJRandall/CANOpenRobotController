/**
 * \file StateMachineMulti.h
 * Copied from StateMachine.h and renamed for multi-robot use.
 */
#ifndef MULTISTATEMACHINE_H
#define MULTISTATEMACHINE_H

#include <csignal> //For raise()
#include "State.h"
#include "Robot.h"
#include "LogHelper.h"

class MultiStateMachine; //Forward declaration for following type definitions

typedef std::function<bool(MultiStateMachine &)> TransitionCb_t; //!< Type of Transition function callbacks to register using AddTransition()
typedef std::pair<TransitionCb_t, std::string> Transition_t;


/**
 * \brief A generic (abstract class) state machine, managing States and Transitions between them as well as a Robot object.
 * Copied from `StateMachine` for multi-robot development. Modify this copy for your multi-robot implementation.
 */
class MultiStateMachine {
   public:
    MultiStateMachine();
    virtual ~MultiStateMachine(){};

    // Non-copyable: ownership of Robots is unique (std::unique_ptr)
    MultiStateMachine(const MultiStateMachine&) = delete;
    MultiStateMachine& operator=(const MultiStateMachine&) = delete;
    // Allow move semantics
    MultiStateMachine(MultiStateMachine&&) = default;
    MultiStateMachine& operator=(MultiStateMachine&&) = default;

    void setInitState(std::string state_name);
    void activate();
    virtual void update();
    virtual void init() = 0;
    virtual void end();
    virtual bool configureMasterPDOs();
    void setRobot(std::unique_ptr<Robot> r);
    void addState(std::string state_name, std::shared_ptr<State> s_ptr);
    void addTransition(std::string from, TransitionCb_t t_cb, std::string to);
    void addTransitionFromLast(TransitionCb_t t_cb, std::string to);
    void addTransitionFromAny(TransitionCb_t t_cb, std::string to);

    std::shared_ptr<State> state(std::string state_name) { return _states[state_name]; }
    template <typename S>
    std::shared_ptr<S> state(std::string state_name) { return std::static_pointer_cast<S>(_states[state_name]); }
    std::shared_ptr<State> state() { return _states[_currentState]; }

    // Safe robot accessors
    Robot* robotAt(size_t idx) { return (idx < _robots.size()) ? _robots[idx].get() : nullptr; }
    const Robot* robotAt(size_t idx) const { return (idx < _robots.size()) ? _robots[idx].get() : nullptr; }
    size_t robotCount() const { return _robots.size(); }
    Robot* robot(size_t idx) { return _robots[idx].get(); }

    bool running() { return _running; }
    double & runningTime() { return _time_running; }

   protected:
    virtual void hwStateUpdate();

    std::vector<std::unique_ptr<Robot>> _robots;    //!< Vector of robots managed by the multi state machine

    LogHelper logHelper;

   private:
    std::string _currentState;
    std::map<std::string, std::shared_ptr<State>> _states;
    std::map<std::string, std::vector<Transition_t>> _transitions;
    std::string _lastToState;

    bool _running;
    std::chrono::steady_clock::time_point _time_init;
    double _time_running=0;
};

#endif  //MULTISTATEMACHINE_H
