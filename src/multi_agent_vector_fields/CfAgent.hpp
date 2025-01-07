/*
   Class for circular field agents
*/
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <multi_agent_vector_fields/CopyableAtomic.hpp>
#include <multi_agent_vector_fields/Obstacle.hpp>
#include <vector>

namespace ghostplanner::cfplanner
{

    class CfAgent
    {
      protected:
        int id_;
        std::vector<Eigen::Vector3d> pos_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d acc_;
        Eigen::Vector3d init_pos_;
        Eigen::Vector3d g_pos_;
        Eigen::Vector3d force_;
        Eigen::Vector3d angular_force_;

        Eigen::Quaterniond current_orientation_;
        Eigen::Quaterniond goal_orientation_;
        Eigen::Vector3d angular_velocity_;
        double detect_shell_rad_;
        double mass_;
        double rad_;
        double vel_max_;
        double min_obs_dist_;
        double approach_dist_;
        CopyableAtomic<bool> run_prediction_;
        CopyableAtomic<bool> running_;
        CopyableAtomic<bool> finished_;
        std::vector<Obstacle> obstacles_;
        std::vector<bool> known_obstacles_;
        std::vector<Eigen::Vector3d> field_rotation_vecs_;
        double prediction_time_;
        bool reached_goal_;

      public:
        enum Type
        {
            REAL_AGENT,
            GOAL_HEURISTIC,
            OBSTACLE_HEURISTIC,
            GOAL_OBSTACLE_HEURISTIC,
            VEL_HEURISTIC,
            RANDOM_AGENT,
            HAD_HEURISTIC,
            UNDEFINED
        };
        CfAgent(const int id, const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos, const double detect_shell_rad, const double agent_mass,
                const double radius, const double velocity_max, const double approach_dist, const int num_obstacles,
                const std::vector<Obstacle> obstacles, const Eigen::Quaterniond &initial_orientation, const Eigen::Quaterniond &goal_orientation)
          : id_{ id }, pos_{ agent_pos }, vel_{ 0.01, 0.0, 0.0 }, acc_{ 0.0, 0.0, 0.0 }, init_pos_{ 0.0, 0.0, 0.0 }, g_pos_{ goal_pos },
            detect_shell_rad_{ detect_shell_rad }, min_obs_dist_{ detect_shell_rad }, force_{ 0.0, 0.0, 0.0 }, angular_force_{ 0.0, 0.0, 0.0 },
            mass_{ agent_mass }, rad_{ radius }, vel_max_{ velocity_max }, approach_dist_{ approach_dist }, run_prediction_{ false },
            running_{ false }, finished_{ false }, reached_goal_{ false }, obstacles_{ obstacles }, current_orientation_{ initial_orientation },
            goal_orientation_{ goal_orientation }, angular_velocity_{ Eigen::Vector3d::Zero() }  // 初始化角速度
        {
            // ROS_INFO("CfAgent initialized:");
            // ROS_INFO("  current_orientation_: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]", current_orientation_.w(), current_orientation_.x(),
            //          current_orientation_.y(), current_orientation_.z());
            // ROS_INFO("  goal_orientation_!!!!: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]", goal_orientation_.w(), goal_orientation_.x(),
            //          goal_orientation_.y(), goal_orientation_.z());
            Eigen::Vector3d default_rot_vec{ 0.0, 0.0, 1.0 };
            for (size_t i = 0; i < num_obstacles; i++)
            {
                field_rotation_vecs_.push_back(default_rot_vec);
                known_obstacles_.push_back(false);
            }
        };
        CfAgent() = default;
        virtual ~CfAgent() = default;
        CfAgent(const CfAgent &) = default;
        CfAgent(CfAgent &&) noexcept = default;
        CfAgent &operator=(const CfAgent &) = default;
        CfAgent &operator=(CfAgent &&) noexcept = default;
        Eigen::Vector3d getFirstPosition() const;
        Eigen::Vector3d getPosition(int id) const;
        Eigen::Vector3d getLatestPosition() const;
        Eigen::Vector3d getInitialPosition() const
        {
            return init_pos_;
        };
        std::vector<Eigen::Vector3d> getPath() const
        {
            return pos_;
        };
        double getPathLength() const;
        Eigen::Vector3d getGoalPosition() const
        {
            return g_pos_;
        };
        virtual void setPosition(Eigen::Vector3d position);
        void setInitalPosition(Eigen::Vector3d position);

        void setOrientation(const Eigen::Quaterniond &orientation)
        {
            current_orientation_ = orientation;
        }
        Eigen::Quaterniond getOrientation() const
        {
            return current_orientation_;
        }

        void resetForce()
        {
            force_ << 0.0, 0.0, 0.0;
        };
        Eigen::Vector3d getForce()
        {
            return force_;
        };
        Eigen::Vector3d getVelocity()
        {
            return vel_;
        };
        Eigen::Vector3d getAngularVelocity() const
        {
            return angular_velocity_;
        }
        double getDistFromGoal() const
        {
            return (g_pos_ - this->getLatestPosition()).norm();
        };
        int getNumPredictionSteps() const
        {
            return pos_.size();
        };
        int getAgentID() const
        {
            return id_;
        };
        bool getRunningStatus()
        {
            return running_;
        };
        double getMinObsDist()
        {
            return min_obs_dist_;
        };
        double getPredictionTime()
        {
            return prediction_time_;
        };
        bool getReachedGoal()
        {
            return reached_goal_;
        };
        void startPrediction()
        {
            run_prediction_ = true;
        };
        void stopPrediction()
        {
            run_prediction_ = false;
        };
        void shutdownAgent()
        {
            finished_ = true;
        };
        void resetMinObsDist()
        {
            min_obs_dist_ = detect_shell_rad_;
        };
        void setVelocity(const Eigen::Vector3d &velocity);
        void setObstacles(const std::vector<Obstacle> &obstacles, const std::vector<bool> real_known_obstacles);
        void setGoal(const Eigen::Vector3d &goal)
        {
            g_pos_ = goal;
        };
        virtual void circForce(const std::vector<Obstacle> &obstacles, const double k_circ);
        void repelForce(const std::vector<Obstacle> &obstacles, const double k_repel);
        void attractorForce(const double k_attr, const double k_damp, const double k_goal_scale);
        double attractorForceScaling(const std::vector<Obstacle> &obstacles);
        Eigen::Vector3d bodyForce(const std::vector<Obstacle> &obstacles, const double k_repel);
        void updatePositionAndVelocity(const double delta_t);
        void updateAngularVelocity(const Eigen::Vector3d &new_angular_velocity);
        void updateOrientation(const double delta_t);
        void predictObstacles(const double delta_t);
        virtual void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const double k_attr,
                               const double k_circ, const double k_repel, const double k_damp, const double k_manip, const double delta_t,
                               const int steps = 1);
        void cfPrediction(const std::vector<Eigen::Vector3d> &manip_map, const double k_attr, const double k_circ, const double k_repel,
                          const double k_damp, const double k_manip, const double delta_t, const size_t max_prediction_steps);
        void reset();

        virtual Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                              const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                              const std::vector<Eigen::Vector3d> field_rotation_vecs) const;

        virtual Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                        const std::vector<Obstacle> &obstacles, const int obstacle_id) const;

        double evalObstacleDistance(const std::vector<Obstacle> &obstacles) const;
        virtual bool isRealAgent()
        {
            return false;
        };
        virtual std::unique_ptr<CfAgent> makeCopy()
        {
            return std::unique_ptr<CfAgent>(new CfAgent(*this));
        };
        virtual Type getAgentType()
        {
            return UNDEFINED;
        };
    };
}  // namespace ghostplanner::cfplanner
