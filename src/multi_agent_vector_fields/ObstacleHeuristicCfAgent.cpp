#include <multi_agent_vector_fields/ObstacleHeuristicCfAgent.hpp>

namespace ghostplanner::cfplanner
{

    Eigen::Vector3d ObstacleHeuristicCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
                                                            const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
                                                            const int obstacle_id, const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        Eigen::Vector3d cfagent_to_obs{ obstacles[obstacle_id].getPosition() - agent_pos };
        cfagent_to_obs.normalize();
        Eigen::Vector3d current{ cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id)) };
        current.normalize();
        return current;
    }

    Eigen::Vector3d ObstacleHeuristicCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                                      const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        if (obstacles.size() < 2)
        {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        double min_dist_obs = 100.0;
        int closest_obstacle_it = 0;
        for (int i = 0; i < obstacles.size() - 1; i++)
        {
            if (i != obstacle_id)
            {
                double dist_obs{ (obstacles[obstacle_id].getPosition() - obstacles[i].getPosition()).norm() };
                if (min_dist_obs > dist_obs)
                {
                    min_dist_obs = dist_obs;
                    closest_obstacle_it = i;
                }
            }
        }
        // Vector from active obstacle to the obstacle which is closest to the
        // active obstacle
        Eigen::Vector3d obstacle_vec = obstacles[closest_obstacle_it].getPosition() - obstacles[obstacle_id].getPosition();
        Eigen::Vector3d cfagent_to_obs{ obstacles[obstacle_id].getPosition() - agent_pos };
        cfagent_to_obs.normalize();
        // Current vector is perpendicular to obstacle surface normal and shows in
        // opposite direction of obstacle_vec
        Eigen::Vector3d current{ (cfagent_to_obs * obstacle_vec.dot(cfagent_to_obs)) - obstacle_vec };
        Eigen::Vector3d rot_vec{ current.cross(cfagent_to_obs) };
        rot_vec.normalize();
        return rot_vec;
    }

}  // namespace ghostplanner::cfplanner