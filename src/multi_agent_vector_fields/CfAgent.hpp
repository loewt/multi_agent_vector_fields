/*
   Class for circular field agents
*/
#pragma once

#include <Eigen/Dense>
#include <gafro/gafro.hpp>
#include <multi_agent_vector_fields/CopyableAtomic.hpp>
#include <multi_agent_vector_fields/Obstacle.hpp>
#include <sackmesser/Configuration.hpp>
#include <sackmesser/Interface.hpp>
#include <vector>

namespace ghostplanner::cfplanner
{

    class CfAgent
    {
      protected:
        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            double detect_shell_radius;
            double mass;
            double radius;
            double max_velocity;
            double approach_distance;

            double k_circular_force;
            double k_repel_force;
            double k_attractor_force;
            double k_damping;
        } config_;

      protected:
        gafro::Motor<double> current_pose_;

        gafro::Motor<double> goal_pose_;

        double min_obstacle_distance_;

        int id_;
        std::vector<gafro::Motor<double>> trajectory_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d acc_;
        Eigen::Vector3d init_pos_;
        Eigen::Vector3d force_;
        Eigen::Vector3d angular_force_;

        Eigen::Vector3d angular_velocity_;
        CopyableAtomic<bool> run_prediction_;
        CopyableAtomic<bool> running_;
        CopyableAtomic<bool> finished_;
        std::vector<Obstacle> obstacles_;
        std::vector<bool> known_obstacles_;
        std::vector<Eigen::Vector3d> field_rotation_vecs_;
        double prediction_time_;

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

        CfAgent(const sackmesser::Interface::Ptr &interface, const std::string &name, const int id, const Eigen::Vector3d agent_pos,
                const Eigen::Vector3d goal_pos, const int num_obstacles, const std::vector<Obstacle> obstacles,
                const Eigen::Quaterniond &initial_orientation, const Eigen::Quaterniond &goal_orientation);

        CfAgent() = default;

        virtual ~CfAgent() = default;

        CfAgent(const CfAgent &) = default;

        CfAgent(CfAgent &&) noexcept = default;

        CfAgent &operator=(const CfAgent &) = default;

        CfAgent &operator=(CfAgent &&) noexcept = default;

        Eigen::Vector3d getFirstPosition() const;

        Eigen::Vector3d getPosition(int id) const;

        Eigen::Vector3d getLatestPosition() const;

        double getPathLength() const;

        virtual void setPosition(Eigen::Vector3d position);

        void setInitalPosition(Eigen::Vector3d position);

        Eigen::Vector3d getInitialPosition() const;

        std::vector<gafro::Motor<double>> getPath() const;

        Eigen::Vector3d getGoalPosition() const;

        void setOrientation(const Eigen::Quaterniond &orientation);

        void resetForce();

        Eigen::Vector3d getForce();

        Eigen::Vector3d getVelocity();

        Eigen::Vector3d getAngularVelocity() const;

        double getDistFromGoal() const;

        int getNumPredictionSteps() const;

        int getAgentID() const;

        bool getRunningStatus();

        double getMinObsDist();

        double getPredictionTime();

        bool hasReachedGoal();

        void startPrediction();

        void stopPrediction();

        void shutdownAgent();

        void resetMinObsDist();

        void setGoal(const Eigen::Vector3d &goal);

        virtual bool isRealAgent();

        virtual Type getAgentType();

        void setVelocity(const Eigen::Vector3d &velocity);

        void setObstacles(const std::vector<Obstacle> &obstacles, const std::vector<bool> real_known_obstacles);

        virtual void circForce(const std::vector<Obstacle> &obstacles);

        void repelForce(const std::vector<Obstacle> &obstacles);

        void attractorForce(const double &k_goal_scale);

        double attractorForceScaling(const std::vector<Obstacle> &obstacles);

        Eigen::Vector3d bodyForce(const std::vector<Obstacle> &obstacles);

        void updatePositionAndVelocity(const double delta_t);

        void updateAngularVelocity(const Eigen::Vector3d &new_angular_velocity);

        void updateOrientation(const double delta_t);

        void predictObstacles(const double delta_t);

        virtual void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map, const std::vector<Obstacle> &obstacles, const double delta_t,
                               const int steps = 1);

        void cfPrediction(const std::vector<Eigen::Vector3d> &manip_map, const double delta_t, const size_t max_prediction_steps);
        void reset();

        virtual Eigen::Vector3d currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                              const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                              const std::vector<Eigen::Vector3d> field_rotation_vecs) const;

        virtual Eigen::Vector3d calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                        const std::vector<Obstacle> &obstacles, const int obstacle_id) const;

        double evalObstacleDistance(const std::vector<Obstacle> &obstacles) const;
    };
}  // namespace ghostplanner::cfplanner
