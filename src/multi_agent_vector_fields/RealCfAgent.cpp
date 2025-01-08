#include <multi_agent_vector_fields/RealCfAgent.hpp>

namespace ghostplanner::cfplanner
{
    void RealCfAgent::setPosition(Eigen::Vector3d position)
    {
        trajectory_.push_back(gafro::Translator<double>::exp(position));
    }

    void RealCfAgent::circForce(const std::vector<Obstacle> &obstacles, const double k_circ, const CfAgent &agent)
    {
        Eigen::Vector3d goal_vec{ goal_pose_.getTranslator().toTranslationVector() - getLatestPosition() };
        for (int i = 0; i < obstacles.size() - 1; i++)
        {
            Eigen::Vector3d robot_obstacle_vec{ obstacles.at(i).getPosition() - getLatestPosition() };
            Eigen::Vector3d rel_vel{ vel_ - obstacles.at(i).getVelocity() };
            if (robot_obstacle_vec.normalized().dot(goal_vec.normalized()) < -0.01 && robot_obstacle_vec.dot(rel_vel) < -0.01)
            {
                continue;
            }
            double dist_obs{ (getLatestPosition() - obstacles.at(i).getPosition()).norm() - (rad_ + obstacles.at(i).getRadius()) };
            dist_obs = std::max(dist_obs, 1e-5);
            Eigen::Vector3d curr_force{ 0.0, 0.0, 0.0 };
            Eigen::Vector3d current;
            if (dist_obs < detect_shell_rad_)
            {
                if (!known_obstacles_.at(i))
                {
                    field_rotation_vecs_.at(i) = calculateRotationVector(getLatestPosition(), getGoalPosition(), obstacles, i, agent);
                    known_obstacles_.at(i) = true;
                }
                double vel_norm = rel_vel.norm();
                if (vel_norm != 0)
                {
                    Eigen::Vector3d normalized_vel = rel_vel / vel_norm;
                    current = currentVector(getLatestPosition(), rel_vel, getGoalPosition(), obstacles, i, field_rotation_vecs_, agent);
                    curr_force = (k_circ / pow(dist_obs, 2)) * normalized_vel.cross(current.cross(normalized_vel));
                }
            }
            force_ += curr_force;
        }
    }

    void RealCfAgent::cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const CfAgent &agent,
                                const double k_attr, const double k_circ, const double k_repel, const double k_damp, const double k_manip,
                                const double delta_t, const int steps)
    {
        for (size_t i = 0; i < steps; ++i)
        {
            resetForce();
            double k_goal_scale = 1.0;
            if (!(getDistFromGoal() < approach_dist_ || (vel_.norm() < 0.5 * vel_max_ && (getLatestPosition() - init_pos_).norm() < 0.2)))
            {
                circForce(obstacles, k_circ, agent);
                if (force_.norm() > 1e-5)
                {
                    k_goal_scale = attractorForceScaling(obstacles);
                }
            }
            repelForce(obstacles, k_repel);
            attractorForce(k_attr, k_damp, k_goal_scale);

            updatePositionAndVelocity(delta_t);
        }
    }

    Eigen::Vector3d RealCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                               const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                               const std::vector<Eigen::Vector3d> field_rotation_vecs, const CfAgent &agent) const
    {
        Eigen::Vector3d current = agent.currentVector(agent_pos, agent_vel, goal_pos, obstacles, obstacle_id, field_rotation_vecs);
        return current;
    }

    Eigen::Vector3d RealCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                         const std::vector<Obstacle> &obstacles, const int obstacle_id, const CfAgent &agent) const
    {
        Eigen::Vector3d rot_vec = agent.calculateRotationVector(agent_pos, goal_pos, obstacles, obstacle_id);
        return rot_vec;
    }

}  // namespace ghostplanner::cfplanner