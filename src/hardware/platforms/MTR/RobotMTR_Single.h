/**
 * Thin wrapper for RobotMTR used for multi-robot testing (e.g. left/right arm).
 * Forwards construction to RobotMTR with an arbitrary set of drive node ids,
 * so a single "arm" instance can grow from one drive to several without
 * changing how MTRobotMachine registers it.
 */
#ifndef ROBOTMTR_SINGLE_H
#define ROBOTMTR_SINGLE_H

#include "RobotMTR.h"

class RobotMTRSingle : public RobotMTR {
   public:
    RobotMTRSingle(const std::string &robot_name = "RobotMTRSingle",
                   const std::string &yaml_config_file = "",
                   std::vector<int> drive_node_ids = {1})
        : RobotMTR(robot_name, yaml_config_file, drive_node_ids) {}

    ~RobotMTRSingle() {}
};

#endif // ROBOTMTR_SINGLE_H
