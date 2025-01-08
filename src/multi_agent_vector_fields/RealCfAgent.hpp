/*
   Class for circular field agents
*/
#pragma once

#include <multi_agent_vector_fields/CfAgent.hpp>

namespace ghostplanner::cfplanner
{

    class RealCfAgent : public CfAgent
    {
      public:
        RealCfAgent(const sackmesser::Interface::Ptr &interface, const std::string &name, const int id, const Eigen::Vector3d agent_pos,
                    const Eigen::Vector3d goal_pos, const int num_obstacles, const std::vector<Obstacle> obstacles,
                    const Eigen::Quaterniond &initial_orientation, const Eigen::Quaterniond &goal_orientation)
          : CfAgent(interface, name, id, agent_pos, goal_pos, num_obstacles, obstacles, initial_orientation, goal_orientation) {};

        RealCfAgent() = default;

        Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                      const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                      const std::vector<Eigen::Vector3d> field_rotation_vecs, const CfAgent &agent) const;

        Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                const std::vector<Obstacle> &obstacles, const int obstacle_id, const CfAgent &agent) const;

        void circForce(const std::vector<Obstacle> &obstacles, const CfAgent &agent);

        void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const CfAgent &agent,
                       const double delta_t, const int steps = 1);

        void setPosition(Eigen::Vector3d position) override;

        bool isRealAgent() override
        {
            return true;
        };

        Type getAgentType() override
        {
            return REAL_AGENT;
        }

        std::vector<bool> getKnownObstacles()
        {
            return known_obstacles_;
        };
    };

}  // namespace ghostplanner::cfplanner
