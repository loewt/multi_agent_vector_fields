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
        RealCfAgent(const int id, const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos, const double detect_shell_rad,
                    const double agent_mass, const double radius, const double velocity_max, const double approach_dist, const int num_obstacles,
                    Eigen::Quaterniond current_orientation_, Eigen::Quaterniond goal_orientation_)
          : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius, velocity_max, approach_dist, num_obstacles,
                    std::vector<Obstacle>(), current_orientation_, goal_orientation_) {};
        RealCfAgent() = default;

        Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                      const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                      const std::vector<Eigen::Vector3d> field_rotation_vecs, const CfAgent &agent) const;
        Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                const std::vector<Obstacle> &obstacles, const int obstacle_id, const CfAgent &agent) const;
        void circForce(const std::vector<Obstacle> &obstacles, const double k_circ, const CfAgent &agent);
        void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const CfAgent &agent,
                       const double k_attr, const double k_circ, const double k_repel, const double k_damp, const double k_manip,
                       const double delta_t, const int steps = 1);
        void setPosition(Eigen::Vector3d position) override;
        bool isRealAgent() override
        {
            return true;
        };
        Type getAgentType() override
        {
            return REAL_AGENT;
        };
        std::unique_ptr<CfAgent> makeCopy() override
        {
            return std::unique_ptr<CfAgent>(new RealCfAgent(*this));
        };
        std::vector<bool> getKnownObstacles()
        {
            return known_obstacles_;
        };
    };

}  // namespace ghostplanner::cfplanner
