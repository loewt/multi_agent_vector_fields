#include <multi_agent_vector_fields/HadHeuristicCfAgent.hpp>

namespace ghostplanner::cfplanner
{

    Eigen::Vector3d HadHeuristicCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
                                                       const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                                       const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        Eigen::Vector3d cfagent_to_obs{ obstacles[obstacle_id].getPosition() - agent_pos };
        cfagent_to_obs.normalize();
        Eigen::Vector3d current = cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id));
        current.normalize();
        return current;
    }

    Eigen::Vector3d HadHeuristicCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                                 const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        Eigen::Vector3d obs_pos = obstacles.at(obstacle_id).getPosition();
        Eigen::Vector3d goal_vec{ goal_pos - agent_pos };
        Eigen::Vector3d rob_obs_vec{ obs_pos - agent_pos };
        Eigen::Vector3d d = agent_pos + goal_vec * (rob_obs_vec.dot(goal_vec) / pow(goal_vec.norm(), 2)) - obs_pos;
        Eigen::Vector3d rot_vec = d.cross(goal_vec) / (d.cross(goal_vec)).norm();
        return rot_vec;
    }

}  // namespace ghostplanner::cfplanner