/*
   Class for circular field agent manager
*/
#pragma once

#include <pthread.h>

#include <Eigen/Dense>
#include <multi_agent_vector_fields/CfAgent.hpp>
#include <multi_agent_vector_fields/Obstacle.hpp>
#include <multi_agent_vector_fields/RealCfAgent.hpp>
#include <thread>
#include <vector>

namespace ghostplanner
{
    namespace cfplanner
    {
        class CfManager
        {
            RealCfAgent real_ee_agent_;

            std::shared_ptr<CfAgent> best_agent_;

            std::vector<std::shared_ptr<CfAgent>> ee_agents_;

            std::vector<std::shared_ptr<CfAgent>> force_agents_;

            std::vector<double> k_a_ee_;

            std::vector<double> k_c_ee_;

            std::vector<double> k_r_ee_;

            std::vector<double> k_d_ee_;

            std::vector<double> k_manip_;

            std::vector<double> k_r_force_;

            std::vector<Eigen::Vector3d> manip_map_;

            bool run_prediction_;

            Eigen::Vector3d init_pos_;

            Eigen::Vector3d goal_pos_;

            std::vector<std::thread> prediction_threads_;

            double approach_dist_;

          public:
            CfManager(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos, const double delta_t, const std::vector<Obstacle> &obstacles,
                      const std::vector<double> &k_a_ee, const std::vector<double> &k_c_ee, const std::vector<double> &k_r_ee,
                      const std::vector<double> &k_d_ee, const std::vector<double> &k_manip, const std::vector<double> &k_r_force,
                      const Eigen::Quaterniond &start_orientation, const Eigen::Quaterniond &goal_orientation, const double velocity_max = 0.5,
                      const double approach_dist = 0.25, const double detect_shell_rad = 0.8, const size_t max_prediction_steps = 1500,
                      const size_t prediction_freq_multiple = 1, const double agent_mass = 1.0, const double radius = 0.01);

            CfManager() = default;

            ~CfManager();

            CfManager(const CfManager &) = default;

            CfManager(CfManager &&) noexcept = default;

            CfManager &operator=(const CfManager &) = default;

            CfManager &operator=(CfManager &&) noexcept = default;

            void startPrediction();

            void stopPrediction();

            void shutdownAllAgents();

            void joinPredictionThreads();

            std::vector<std::vector<gafro::Motor<double>>> getPredictedPaths();

            std::vector<double> getPredictedPathLengths();

            std::vector<double> getPredictionTimes();

            std::vector<bool> getAgentSuccess();

            int getBestAgentType();

            Eigen::Vector3d getNextPosition();

            Eigen::Vector3d getInitialPosition();

            Eigen::Vector3d getNextVelocity();

            Eigen::Vector3d getNextAngularVelocity() const;

            Eigen::Vector3d getEEForce();

            Eigen::Vector3d getGoalPosition() const;

            int getNumPredictionSteps(int agent_id);

            int getRealNumPredictionSteps();

            double getDistFromGoal() const;

            std::vector<gafro::Motor<double>> getPlannedTrajectory() const;

            void init(const Eigen::Vector3d goal_pos, const double delta_t, const std::vector<Obstacle> &obstacles, const std::vector<double> &k_a_ee,
                      const std::vector<double> &k_c_ee, const std::vector<double> &k_r_ee, const std::vector<double> &k_d_ee,
                      const std::vector<double> &k_manip, const std::vector<double> &k_r_force, const double velocity_max = 0.5,
                      const double approach_dist = 0.25, const double detect_shell_rad = 0.8,
                      const Eigen::Quaterniond &start_orientation = Eigen::Quaterniond::Identity(),
                      const Eigen::Quaterniond &goal_orientation = Eigen::Quaterniond::Identity(), const size_t max_prediction_steps = 1500,
                      const size_t prediction_freq_multiple = 1, const double agent_mass = 1.0, const double radius = 0.05);

            std::vector<Eigen::Vector3d> getLinkForce(const std::vector<Eigen::Vector3d> &link_positions, const std::vector<Obstacle> &obstacles);

            Eigen::Vector3d getRealEEAgentPosition() const;

            void resetEEAgents(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity, const std::vector<Obstacle> &obstacles);

            void setInitialEEPositions(const Eigen::Vector3d &position);

            void setInitialPosition(const Eigen::Vector3d &position);

            void moveRealEEAgent(const std::vector<Obstacle> &obstacles, const double delta_t, const int steps, const int agent_id);

            void moveAgent(const std::vector<Obstacle> &obstacles, const double delta_t, const int steps, const int id);

            void moveAgents(const std::vector<Obstacle> &obstacles, const double delta_t, const int steps = 1);

            void moveAgentsPar(const std::vector<Obstacle> &obstacles, const double delta_t, const int steps = 1);

            double evaluatePath(const std::vector<Obstacle> &obstacles);

            // CfAgent evaluateAgents(
            int evaluateAgents(const std::vector<Obstacle> &obstacles, const double k_goal_dist, const double k_path_len, const double k_safe_dist,
                               const double k_workspace, const Eigen::Matrix<double, 6, 1> des_ws_limits);

            RealCfAgent &getRealEEAgent();
        };
    }  // namespace cfplanner
}  // namespace ghostplanner
