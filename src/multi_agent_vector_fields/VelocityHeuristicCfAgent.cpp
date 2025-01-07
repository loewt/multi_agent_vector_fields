#include <multi_agent_vector_fields/VelocityHeuristicCfAgent.hpp>

namespace ghostplanner::cfplanner
{

    Eigen::Vector3d VelocityHeuristicCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
                                                            const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
                                                            const int obstacle_id, const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        Eigen::Vector3d normalized_vel = agent_vel.normalized();
        Eigen::Vector3d normalized_obs_to_agent{ (obstacles[obstacle_id].getPosition() - agent_pos).normalized() };
        Eigen::Vector3d current{ normalized_vel - (normalized_obs_to_agent * normalized_vel.dot(normalized_obs_to_agent)) };
        if (current.norm() < 1e-10)
        {
            current << 0.0, 0.0, 1.0;
            // current = makeRandomVector();
        }
        current.normalize();
        return current;
    }

    Eigen::Vector3d VelocityHeuristicCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                                      const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        return { 0.0, 0.0, 1.0 };
    }

}  // namespace ghostplanner::cfplanner