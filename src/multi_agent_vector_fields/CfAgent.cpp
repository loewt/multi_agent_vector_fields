#include <chrono>
#include <multi_agent_vector_fields/CfAgent.hpp>

namespace ghostplanner::cfplanner
{
    CfAgent::CfAgent(const int id, const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos, const double detect_shell_rad,
                     const double agent_mass, const double radius, const double velocity_max, const double approach_dist, const int num_obstacles,
                     const std::vector<Obstacle> obstacles, const Eigen::Quaterniond &initial_orientation, const Eigen::Quaterniond &goal_orientation)
      : current_pose_(gafro::Rotor<double>::fromQuaternion(initial_orientation)),  //
        goal_pose_(goal_pos, goal_orientation),                                    //
        id_{ id },                                                                 //
        trajectory_{ gafro::Translator<double>::exp(agent_pos) },                  //
        vel_{ 0.01, 0.0, 0.0 },                                                    //
        acc_{ 0.0, 0.0, 0.0 },                                                     //
        init_pos_{ 0.0, 0.0, 0.0 },                                                //
        detect_shell_rad_{ detect_shell_rad },                                     //
        min_obs_dist_{ detect_shell_rad },                                         //
        force_{ 0.0, 0.0, 0.0 },                                                   //
        angular_force_{ 0.0, 0.0, 0.0 },                                           //
        mass_{ agent_mass },                                                       //
        rad_{ radius },                                                            //
        vel_max_{ velocity_max },                                                  //
        approach_dist_{ approach_dist },                                           //
        run_prediction_{ false },                                                  //
        running_{ false },                                                         //
        finished_{ false },                                                        //
        obstacles_{ obstacles },                                                   //
        angular_velocity_{ Eigen::Vector3d::Zero() }                               // 初始化角速度
    {
        Eigen::Vector3d default_rot_vec{ 0.0, 0.0, 1.0 };
        for (size_t i = 0; i < num_obstacles; i++)
        {
            field_rotation_vecs_.push_back(default_rot_vec);
            known_obstacles_.push_back(false);
        }
    };

    Eigen::Vector3d CfAgent::getFirstPosition() const
    {
        assert(trajectory_.size() != 0);
        return trajectory_[0].getTranslator().toTranslationVector();
    }

    Eigen::Vector3d CfAgent::getPosition(int id) const
    {
        assert(trajectory_.size() >= id);
        return trajectory_[id].getTranslator().toTranslationVector();
    }

    Eigen::Vector3d CfAgent::getLatestPosition() const
    {
        assert(trajectory_.size() != 0);
        return trajectory_.back().getTranslator().toTranslationVector();
    }

    double CfAgent::getPathLength() const
    {
        double path_length = 0;
        for (size_t i = 0; i < trajectory_.size() - 1; i++)
        {
            path_length +=
              (trajectory_.at(i + 1).getTranslator().toTranslationVector() - trajectory_.at(i).getTranslator().toTranslationVector()).norm();
        }
        return path_length;
    }

    void CfAgent::setInitalPosition(Eigen::Vector3d position)
    {
        init_pos_ = position;
        setPosition(position);
    }

    void CfAgent::setPosition(Eigen::Vector3d position)
    {
        trajectory_.clear();
        trajectory_.push_back(gafro::Translator<double>::exp(position));
    }

    void CfAgent::reset()
    {
        Eigen::Vector3d one_step_pos{ getFirstPosition() };
        trajectory_.clear();
        trajectory_.push_back(gafro::Translator<double>::exp(one_step_pos));
    }

    void CfAgent::setVelocity(const Eigen::Vector3d &velocity)
    {
        double velocity_norm = velocity.norm();
        if (velocity_norm > vel_max_)
        {
            vel_ = (vel_max_ / velocity_norm) * velocity;
        }
        else
        {
            vel_ = velocity;
        }
    }

    void CfAgent::setObstacles(const std::vector<Obstacle> &obstacles, const std::vector<bool> real_known_obstacles)
    {
        for (int i = 0; i < obstacles.size(); ++i)
        {
            obstacles_.at(i).setPosition(obstacles.at(i).getPosition());
            obstacles_.at(i).setVelocity(obstacles.at(i).getVelocity());
            known_obstacles_.at(i) = real_known_obstacles.at(i);
        }
    }

    void CfAgent::circForce(const std::vector<Obstacle> &obstacles, const double k_circ)
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
            double dist_obs{ robot_obstacle_vec.norm() - (rad_ + obstacles.at(i).getRadius()) };
            dist_obs = std::max(dist_obs, 1e-5);
            if (dist_obs < min_obs_dist_)
            {
                min_obs_dist_ = dist_obs;
            }
            Eigen::Vector3d curr_force{ 0.0, 0.0, 0.0 };
            Eigen::Vector3d current;
            if (dist_obs < detect_shell_rad_)
            {
                if (!known_obstacles_.at(i))
                {
                    field_rotation_vecs_.at(i) = calculateRotationVector(getLatestPosition(), getGoalPosition(), obstacles, i);
                    known_obstacles_.at(i) = true;
                }
                double vel_norm = rel_vel.norm();
                if (vel_norm != 0)
                {
                    Eigen::Vector3d normalized_vel = rel_vel / vel_norm;
                    current = currentVector(getLatestPosition(), rel_vel, getGoalPosition(), obstacles, i, field_rotation_vecs_);
                    curr_force = (k_circ / pow(dist_obs, 2)) * normalized_vel.cross(current.cross(normalized_vel));
                }
            }
            force_ += curr_force;
        }
    }

    double CfAgent::evalObstacleDistance(const std::vector<Obstacle> &obstacles) const
    {
        double min_dist_obs = detect_shell_rad_;
        for (const Obstacle &obstacle : obstacles)
        {
            double dist_obs{ (getLatestPosition() - obstacle.getPosition()).norm() - (rad_ + obstacle.getRadius()) };
            if (min_dist_obs > dist_obs)
            {
                min_dist_obs = dist_obs;
            }
        }
        return min_dist_obs;
    }

    void CfAgent::repelForce(const std::vector<Obstacle> &obstacles, const double k_repel)
    {
        Eigen::Vector3d goal_vec{ goal_pose_.getTranslator().toTranslationVector() - getLatestPosition() };

        Eigen::Vector3d total_repel_force{ 0.0, 0.0, 0.0 };
        Eigen::Vector3d robot_obstacle_vec{ obstacles.back().getPosition() - getLatestPosition() };
        Eigen::Vector3d rel_vel{ vel_ - obstacles.back().getVelocity() };
        Eigen::Vector3d dist_vec = -robot_obstacle_vec;
        double dist_obs{ dist_vec.norm() - (rad_ + obstacles.back().getRadius()) };
        dist_obs = std::max(dist_obs, 1e-5);
        Eigen::Vector3d repel_force{ 0.0, 0.0, 0.0 };
        if (dist_obs < detect_shell_rad_)
        {
            Eigen::Vector3d obs_to_robot = getLatestPosition() - obstacles.back().getPosition();
            obs_to_robot.normalize();
            repel_force = k_repel * obs_to_robot * (1.0 / dist_obs - 1.0 / detect_shell_rad_) / (dist_obs * dist_obs);
        }
        total_repel_force += repel_force;
        force_ += total_repel_force;
    }

    void CfAgent::attractorForce(const double k_attr, const double k_damp, const double k_goal_scale)
    {
        if (k_attr == 0.0)
        {
            return;
        }
        // --- rotational part ---
        Eigen::Vector3d goal_vec{ goal_pose_.getTranslator().toTranslationVector() - getLatestPosition() };
        Eigen::Vector3d vel_des = k_attr / k_damp * goal_vec;
        double scale_lim = std::min(1.0, vel_max_ / vel_des.norm());
        vel_des *= scale_lim;
        force_ += k_goal_scale * k_damp * (vel_des - vel_);

        // --- rotational part ---
        Eigen::Quaterniond error_quaternion = goal_pose_.getRotor().quaternion() * current_pose_.getRotor().quaternion().conjugate();
        Eigen::Vector3d error_vector = error_quaternion.vec();  //

        Eigen::Vector3d desired_angular_velocity = (k_attr / k_damp) * error_vector;
        double max_angular_velocity = 1.0;
        double scaling_factor = std::min(1.0, max_angular_velocity / desired_angular_velocity.norm());
        desired_angular_velocity *= scaling_factor;  // one by one like the paper

        angular_force_ = k_damp * (desired_angular_velocity - angular_velocity_);
    }

    double CfAgent::attractorForceScaling(const std::vector<Obstacle> &obstacles)
    {
        double w1;
        double w2;
        int id_closest_obstacle;
        bool no_close_obs = true;
        double closest_obs_dist = detect_shell_rad_;
        for (int i = 0; i < obstacles.size() - 1; i++)
        {
            double dist_obs{ (getLatestPosition() - obstacles.at(i).getPosition()).norm() - (rad_ + obstacles.at(i).getRadius()) };
            dist_obs = std::max(dist_obs, 1e-5);
            if (dist_obs < closest_obs_dist)
            {
                no_close_obs = false;
                closest_obs_dist = dist_obs;
                id_closest_obstacle = i;
            }
        }
        if (no_close_obs)
        {
            return 1;
        }
        Eigen::Vector3d goal_vec{ goal_pose_.getTranslator().toTranslationVector() - getLatestPosition() };
        if (goal_vec.dot(vel_) <= 0.0 && vel_.norm() < vel_max_ - 0.1 * vel_max_ && goal_vec.norm() > 0.15)
        {
            return 0.0;
        }
        w1 = 1 - std::exp(-std::sqrt(closest_obs_dist) / detect_shell_rad_);
        Eigen::Vector3d robot_obstacle_vec{ obstacles.at(id_closest_obstacle).getPosition() - getLatestPosition() };
        w2 = 1 - (goal_vec.dot(robot_obstacle_vec) / (goal_vec.norm() * robot_obstacle_vec.norm()));
        w2 = w2 * w2;
        return w1 * w2;
    }

    Eigen::Vector3d CfAgent::bodyForce(const std::vector<Obstacle> &obstacles, const double k_repel)
    {
        resetForce();
        repelForce(obstacles, k_repel);
        return force_;
    }

    void CfAgent::updatePositionAndVelocity(const double delta_t)
    {
        // linear pary
        Eigen::Vector3d robot_acc = force_ / mass_;
        double acc_norm = robot_acc.norm();
        if (acc_norm > 13.0)
        {
            robot_acc *= 13.0 / acc_norm;
        }
        Eigen::Vector3d new_pos = getLatestPosition() + 0.5 * robot_acc * delta_t * delta_t + vel_ * delta_t;
        vel_ += robot_acc * delta_t;
        double vel_norm = vel_.norm();
        if (vel_norm > vel_max_)
        {
            vel_ *= vel_max_ / vel_norm;
        }
        trajectory_.push_back(gafro::Translator<double>::exp(new_pos));

        // Angular part (update orientation)

        // ROS_INFO("angular_force_: x=%.2f, y=%.2f, z=%.2f, magnitude=%.2f", angular_force_.x(), angular_force_.y(), angular_force_.z(),
        // angular_force_.norm());
        Eigen::Vector3d angular_acc = angular_force_ / mass_;
        angular_velocity_ += angular_acc * delta_t;

        Eigen::Quaterniond delta_q;
        double angle = angular_velocity_.norm() * delta_t;
        // ROS_INFO("Angle %10f", angle);
        if (angle > 1e-6)
        {
            Eigen::Vector3d axis = angular_velocity_.normalized();
            delta_q = Eigen::AngleAxisd(angle, axis);
            current_pose_ =
              gafro::Motor<double>(gafro::Rotor<double>::fromQuaternion((current_pose_.getRotor().quaternion() * delta_q).normalized()));

            // ROS_INFO("Current orientation: [w=%.2f, x=%.2f, y=%.2f, z=%.2f]",
            //     current_orientation_.w(), current_orientation_.x(),
            //     current_orientation_.y(), current_orientation_.z());
        }
    }

    void CfAgent::updateAngularVelocity(const Eigen::Vector3d &new_angular_velocity)
    {
        angular_velocity_ = new_angular_velocity;
    }

    void CfAgent::updateOrientation(const double delta_t)
    {
        // (0, w_x, w_y, w_z)
        Eigen::Quaterniond omega(0, angular_velocity_.x(), angular_velocity_.y(), angular_velocity_.z());

        Eigen::Quaterniond current_orientation = current_pose_.getRotor().quaternion();
        // 0.5 * delta_t * (current_orientation_ * omega)
        Eigen::Quaterniond delta_q = Eigen::Quaterniond(0.5 * delta_t * (current_pose_.getRotor().quaternion() * omega).coeffs());

        current_orientation.coeffs() += delta_q.coeffs();
        current_orientation.normalize();  // kkep that

        current_pose_ = gafro::Motor<double>(gafro::Rotor<double>::fromQuaternion(current_orientation));
    }

    void CfAgent::predictObstacles(const double delta_t)
    {
        for (auto &&obstacle : obstacles_)
        {
            Eigen::Vector3d new_pos = obstacle.getPosition() + obstacle.getVelocity() * delta_t;
            obstacle.setPosition(new_pos);
        }
    }

    void CfAgent::cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const double k_attr,
                            const double k_circ, const double k_repel, const double k_damp, const double k_manip, const double delta_t,
                            const int steps)
    {
        for (size_t i = 0; i < steps; ++i)
        {
            resetForce();
            double k_goal_scale = 1.0;
            if (!(getDistFromGoal() < approach_dist_ || (vel_.norm() < 0.5 * vel_max_ && (getLatestPosition() - init_pos_).norm() < 0.2)))
            {
                circForce(obstacles, k_circ);
                if (force_.norm() > 1e-5)
                {
                    k_goal_scale = attractorForceScaling(obstacles);
                }
            }
            repelForce(obstacles, k_repel);
            attractorForce(k_attr, k_damp, k_goal_scale);
            // ROS_INFO("Agent force: [%.2f, %.2f, %.2f]", force_.x(), force_.y(), force_.z());
            // ROS_INFO("Agent velocity: [%.2f, %.2f, %.2f]", vel_.x(), vel_.y(), vel_.z());
            updatePositionAndVelocity(delta_t);
        }
    }

    void CfAgent::cfPrediction(const std::vector<Eigen::Vector3d> &manip_map, const double k_attr, const double k_circ, const double k_repel,
                               const double k_damp, const double k_manip, const double delta_t, const size_t max_prediction_steps)
    {
        while (!finished_)
        {
            std::chrono::steady_clock::time_point begin_prediction = std::chrono::steady_clock::now();
            while (run_prediction_ && getDistFromGoal() > 0.1 && trajectory_.size() < max_prediction_steps)
            {
                running_ = true;
                resetForce();
                double k_goal_scale = 1.0;
                if (!(getDistFromGoal() < approach_dist_ || (vel_.norm() < 0.5 * vel_max_ && (getLatestPosition() - init_pos_).norm() < 0.2)))
                {
                    circForce(obstacles_, k_circ);
                    if (force_.norm() > 1e-5)
                    {
                        k_goal_scale = attractorForceScaling(obstacles_);
                    }
                }
                repelForce(obstacles_, k_repel);
                attractorForce(k_attr, k_damp, k_goal_scale);
                updatePositionAndVelocity(delta_t);
                predictObstacles(delta_t);
            }
            auto end_prediction = std::chrono::steady_clock::now();
            if (running_)
            {
                prediction_time_ = (end_prediction - begin_prediction).count();
            }

            running_ = false;
        }
    }

    Eigen::Vector3d CfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                           const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                           const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d CfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                     const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d CfAgent::getInitialPosition() const
    {
        return init_pos_;
    }

    std::vector<gafro::Motor<double>> CfAgent::getPath() const
    {
        return trajectory_;
    }

    Eigen::Vector3d CfAgent::getGoalPosition() const
    {
        return goal_pose_.getTranslator().toTranslationVector();
    }

    void CfAgent::setOrientation(const Eigen::Quaterniond &orientation)
    {
        current_pose_ = gafro::Motor<double>(gafro::Rotor<double>::fromQuaternion(orientation));
    }

    void CfAgent::resetForce()
    {
        force_ << 0.0, 0.0, 0.0;
    }

    Eigen::Vector3d CfAgent::getForce()
    {
        return force_;
    }

    Eigen::Vector3d CfAgent::getVelocity()
    {
        return vel_;
    }

    Eigen::Vector3d CfAgent::getAngularVelocity() const
    {
        return angular_velocity_;
    }
    double CfAgent::getDistFromGoal() const
    {
        return (goal_pose_.getTranslator().toTranslationVector() - this->getLatestPosition()).norm();
    }

    int CfAgent::getNumPredictionSteps() const
    {
        return trajectory_.size();
    }

    int CfAgent::getAgentID() const
    {
        return id_;
    }

    bool CfAgent::getRunningStatus()
    {
        return running_;
    }

    double CfAgent::getMinObsDist()
    {
        return min_obs_dist_;
    }

    double CfAgent::getPredictionTime()
    {
        return prediction_time_;
    }

    bool CfAgent::hasReachedGoal()
    {
        return getDistFromGoal() < 0.100001;
    }

    void CfAgent::startPrediction()
    {
        run_prediction_ = true;
    }

    void CfAgent::stopPrediction()
    {
        run_prediction_ = false;
    }

    void CfAgent::shutdownAgent()
    {
        finished_ = true;
    }

    void CfAgent::resetMinObsDist()
    {
        min_obs_dist_ = detect_shell_rad_;
    }

    void CfAgent::setGoal(const Eigen::Vector3d &goal_position)
    {
        goal_pose_ = gafro::Translator<double>::exp(goal_position) * goal_pose_.getRotor();
    }

    bool CfAgent::isRealAgent()
    {
        return false;
    }

    CfAgent::Type CfAgent::getAgentType()
    {
        return UNDEFINED;
    }

}  // namespace ghostplanner::cfplanner