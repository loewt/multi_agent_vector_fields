/*
   Class for circular field agents
*/
#pragma once

#include <multi_agent_vector_fields/CfAgent.hpp>

namespace ghostplanner::cfplanner
{

    // Agent avoids obstacle in a random direction
    class RandomCfAgent : public CfAgent
    {
      protected:
        std::vector<Eigen::Vector3d> random_vecs_;

      public:
        RandomCfAgent(const sackmesser::Interface::Ptr &interface, const std::string &name, const int id, const Eigen::Vector3d agent_pos,
                      const Eigen::Vector3d goal_pos, const int num_obstacles, const std::vector<Obstacle> obstacles,
                      const Eigen::Quaterniond &initial_orientation, const Eigen::Quaterniond &goal_orientation)
          : CfAgent(interface, name, id, agent_pos, goal_pos, num_obstacles, obstacles, initial_orientation, goal_orientation)
        {
            // For recreating the simulation with the same behavior
            // saveRandomVecToFile(num_obstacles);
            for (size_t i = 0; i < num_obstacles; i++)
            {
                Eigen::Vector3d random_vec = makeRandomVector();
                random_vec.normalize();
                random_vecs_.push_back(random_vec);
            }
            Eigen::Vector3d random_vec;
        }

        Eigen::Vector3d makeRandomVector() const;
        void saveRandomVecToFile(const int num_obstacles);
        Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                      const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
        Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                const std::vector<Obstacle> &obstacles, const int obstacle_id) const override;
        Type getAgentType() override
        {
            return RANDOM_AGENT;
        };
    };

}  // namespace ghostplanner::cfplanner
