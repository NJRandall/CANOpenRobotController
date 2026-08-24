/**
 * Thin single-drive wrapper for RobotMTR used for multi-robot testing.
 * This class forwards construction to RobotMTR with a single drive node id.
 */
#ifndef ROBOTMTR_SINGLE_H
#define ROBOTMTR_SINGLE_H

#include "RobotMTR.h"

class RobotMTRSingle : public RobotMTR {
   public:
    RobotMTRSingle(const std::string &robot_name = "RobotMTRSingle",
                   const std::string &yaml_config_file = "",
                   int drive_node_id = 1)
        : RobotMTR(robot_name, yaml_config_file, std::vector<int>{drive_node_id}) {}

    ~RobotMTRSingle() {}
};

#endif // ROBOTMTR_SINGLE_H
