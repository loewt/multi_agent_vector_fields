/*
   Class for circular field agents
*/
#pragma once

#include <multi_agent_vector_fields/CfAgent.hpp>

namespace ghostplanner::cfplanner
{

    // Agent avoids obstacle in direction of its current velocity (see Ataka RAL
    // paper 2018)
    class VelocityHeuristicCfAgent : public CfAgent
    {
      public:
        VelocityHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos, const double detect_shell_rad,
                                 const double agent_mass, const double radius, const double velocity_max, const double approach_dist,
                                 const int num_obstacles, const std::vector<Obstacle> obstacles, Eigen::Quaterniond current_orientation_,
                                 Eigen::Quaterniond goal_orientation_)
          : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius, velocity_max, approach_dist, num_obstacles, obstacles,
                    current_orientation_, goal_orientation_) {};
        Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                      const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
        Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                const std::vector<Obstacle> &obstacles, const int obstacle_id) const override;
        Type getAgentType() override
        {
            return VEL_HEURISTIC;
        };
        std::unique_ptr<CfAgent> makeCopy() override
        {
            return std::unique_ptr<CfAgent>(new VelocityHeuristicCfAgent(*this));
        };
    };

}  // namespace ghostplanner::cfplanner
