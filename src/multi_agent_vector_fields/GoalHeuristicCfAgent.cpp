#include <multi_agent_vector_fields/GoalHeuristicCfAgent.hpp>

namespace ghostplanner::cfplanner
{

    Eigen::Vector3d GoalHeuristicCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
                                                        const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                                        const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        Eigen::Vector3d goal_vec{ goal_pos - agent_pos };
        Eigen::Vector3d cfagent_to_obs{ obstacles[obstacle_id].getPosition() - agent_pos };
        cfagent_to_obs.normalize();
        Eigen::Vector3d current{ goal_vec - cfagent_to_obs * (cfagent_to_obs.dot(goal_vec)) };
        if (current.norm() < 1e-10)
        {
            current << 0.0, 0.0, 1.0;
            // current = makeRandomVector();
        }
        current.normalize();
        return current;
    }

    Eigen::Vector3d GoalHeuristicCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                                  const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        return { 0.0, 0.0, 1.0 };
    }

}  // namespace ghostplanner::cfplanner