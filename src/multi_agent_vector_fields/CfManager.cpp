#include <omp.h>

#include <cassert>
#include <iostream>
#include <limits>
#include <multi_agent_vector_fields/CfAgent.hpp>
#include <multi_agent_vector_fields/CfManager.hpp>
#include <multi_agent_vector_fields/GoalHeuristicCfAgent.hpp>
#include <multi_agent_vector_fields/GoalObstacleHeuristicCfAgent.hpp>
#include <multi_agent_vector_fields/HadHeuristicCfAgent.hpp>
#include <multi_agent_vector_fields/ObstacleHeuristicCfAgent.hpp>
#include <multi_agent_vector_fields/RandomCfAgent.hpp>
#include <multi_agent_vector_fields/RealCfAgent.hpp>
#include <multi_agent_vector_fields/VelocityHeuristicCfAgent.hpp>
#include <sackmesser/Configurations.hpp>

using Eigen::Vector3d;
using std::vector;

namespace ghostplanner
{
    namespace cfplanner
    {
        CfManager::CfManager(const sackmesser::Interface::Ptr &interface, const std::string &name, const Eigen::Vector3d agent_pos,
                             const Eigen::Vector3d goal_pos, const double delta_t, const std::vector<Obstacle> &obstacles,
                             const Eigen::Quaterniond &start_orientation, const Eigen::Quaterniond &goal_orientation,
                             const size_t max_prediction_steps, const size_t prediction_freq_multiple)
          : goal_pos_(goal_pos), run_prediction_{ false }
        {
            config_ = interface->getConfigurations()->load<Configuration>(name + "/");

            joinPredictionThreads();
            ee_agents_.clear();
            goal_pos_ = goal_pos;
            run_prediction_ = false;
            int obstacle_size = obstacles.size();

            real_ee_agent_ =
              RealCfAgent(interface, name + "/real_agent", 0, init_pos_, goal_pos, obstacle_size, obstacles, start_orientation, goal_orientation);

            ee_agents_.push_back(std::make_shared<HadHeuristicCfAgent>(interface, name + "/agent_1", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                       obstacle_size, obstacles, start_orientation, goal_orientation));
            ee_agents_.push_back(std::make_shared<GoalHeuristicCfAgent>(interface, name + "/agent_2", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                        obstacle_size, obstacles, start_orientation, goal_orientation));
            ee_agents_.push_back(std::make_shared<ObstacleHeuristicCfAgent>(interface, name + "/agent_3", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                            obstacle_size, obstacles, start_orientation, goal_orientation));
            ee_agents_.push_back(std::make_shared<GoalObstacleHeuristicCfAgent>(interface, name + "/agent_4", ee_agents_.size() + 1, init_pos_,
                                                                                goal_pos, obstacle_size, obstacles, start_orientation,
                                                                                goal_orientation));
            ee_agents_.push_back(std::make_shared<VelocityHeuristicCfAgent>(interface, name + "/agent_5", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                            obstacle_size, obstacles, start_orientation, goal_orientation));
            ee_agents_.push_back(std::make_shared<RandomCfAgent>(interface, name + "/agent_6", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                 obstacle_size, obstacles, start_orientation, goal_orientation));
            ee_agents_.push_back(std::make_shared<RandomCfAgent>(interface, name + "/agent_7", ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                 obstacle_size, obstacles, start_orientation, goal_orientation));

            Vector3d zeros{ 0.0, 0.0, 0.0 };

            for (size_t i = 0; i < 7; ++i)
            {
                force_agents_.push_back(std::make_shared<GoalHeuristicCfAgent>(interface, name + "/force_agent_" + std::to_string(i + 1),
                                                                               force_agents_.size() + ee_agents_.size() + 1, init_pos_, goal_pos,
                                                                               obstacle_size, obstacles, start_orientation, goal_orientation));
            }

            int dim_size = 60;
            manip_map_ = std::vector<Vector3d>(dim_size * dim_size * dim_size);

            for (size_t i = 0; i < ee_agents_.size(); i++)
            {
                prediction_threads_.push_back(std::thread(&CfAgent::cfPrediction, std::ref(*ee_agents_[i]), std::ref(manip_map_),
                                                          prediction_freq_multiple * delta_t, max_prediction_steps));
            }
        }

        CfManager::~CfManager()
        {
            joinPredictionThreads();
        }

        void CfManager::startPrediction()
        {
            for (auto &ee_agent : ee_agents_)
            {
                ee_agent->startPrediction();
            }
        }

        void CfManager::stopPrediction()
        {
            for (auto &ee_agent : ee_agents_)
            {
                ee_agent->stopPrediction();
            }
            // Wait for all agents to finish their prediction loop
            bool agent_running = true;
            while (agent_running)
            {
                agent_running = false;
                for (auto &agent : ee_agents_)
                {
                    if (agent->getRunningStatus())
                    {
                        agent_running = true;
                    }
                }
            }
        }

        void CfManager::shutdownAllAgents()
        {
            for (auto &ee_agent : ee_agents_)
            {
                ee_agent->shutdownAgent();
            }
        }

        void CfManager::joinPredictionThreads()
        {
            // Stop predictions
            for (auto &ee_agent : ee_agents_)
            {
                ee_agent->stopPrediction();
                ee_agent->shutdownAgent();
            }
            // Join and delete prediction threads
            for (int i = prediction_threads_.size() - 1; i >= 0; i--)
            {
                if (prediction_threads_.at(i).joinable())
                {
                    // It is only possible to get the thread handle before it is joined
                    pthread_t thread_handle = prediction_threads_.at(i).native_handle();
                    // Merges thread back into program
                    prediction_threads_.at(i).join();
                    // This is needed to really free OS memory
                    pthread_cancel(thread_handle);
                    // Calling the destructor apparently only works if the thread was
                    // not joined before and the it would terminate all threads? See
                    prediction_threads_.pop_back();
                }
                else
                {
                    std::cout << "not joinable" << std::endl; /* code */
                }
            }
        }

        std::vector<Vector3d> CfManager::getLinkForce(const std::vector<Eigen::Vector3d> &link_positions, const std::vector<Obstacle> &obstacles)
        {
            std::vector<Vector3d> forces;
            for (size_t i = 0; i < force_agents_.size(); ++i)
            {
                force_agents_[i]->setPosition(link_positions[i]);
                Vector3d force = force_agents_[i]->bodyForce(obstacles);
                forces.push_back(force);
            }
            return forces;
        }

        std::vector<std::vector<gafro::Motor<double>>> CfManager::getPredictedPaths()
        {
            std::vector<std::vector<gafro::Motor<double>>> predicted_paths;
            for (size_t i = 0; i < ee_agents_.size(); i++)
            {
                predicted_paths.push_back(ee_agents_[i]->getPath());
            }
            return predicted_paths;
        }

        std::vector<double> CfManager::getPredictedPathLengths()
        {
            std::vector<double> path_lengths;
            for (size_t i = 0; i < ee_agents_.size(); i++)
            {
                path_lengths.push_back(ee_agents_[i]->getPathLength());
            }
            return path_lengths;
        }

        std::vector<double> CfManager::getPredictionTimes()
        {
            std::vector<double> pred_time;
            for (size_t i = 0; i < ee_agents_.size(); i++)
            {
                pred_time.push_back(ee_agents_[i]->getPredictionTime());
            }
            return pred_time;
        }

        std::vector<bool> CfManager::getAgentSuccess()
        {
            std::vector<bool> success;
            for (size_t i = 0; i < ee_agents_.size(); i++)
            {
                success.push_back(ee_agents_[i]->hasReachedGoal());
            }
            return success;
        }

        void CfManager::resetEEAgents(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity, const std::vector<Obstacle> &obstacles)
        {
            for (auto &agent : ee_agents_)
            {
                agent->setPosition(position);
                agent->setVelocity(velocity);
                agent->setObstacles(obstacles, real_ee_agent_.getKnownObstacles());
                agent->resetMinObsDist();
            }
        }

        void CfManager::setInitialPosition(const Eigen::Vector3d &position)
        {
            init_pos_ = position;
            setInitialEEPositions(position);
        }

        void CfManager::setInitialEEPositions(const Eigen::Vector3d &position)
        {
            real_ee_agent_.setInitalPosition(position);
            for (auto &agent : ee_agents_)
            {
                agent->setInitalPosition(position);
            }
        }

        void CfManager::moveRealEEAgent(const vector<Obstacle> &obstacles, const double delta_t, const int steps, const int agent_id)
        {
            real_ee_agent_.cfPlanner(manip_map_, obstacles, *best_agent_, delta_t, steps);
        }

        void CfManager::moveAgent(const vector<Obstacle> &obstacles, const double delta_t, const int steps, const int id)
        {
            while (run_prediction_ && ee_agents_[id]->getDistFromGoal() > 0.05)
            {
                ee_agents_[id]->cfPlanner(manip_map_, obstacles, delta_t, steps);
            }
        }

        void CfManager::moveAgents(const vector<Obstacle> &obstacles, const double delta_t, const int steps)
        {
            for (size_t i = 0; i < ee_agents_.size(); ++i)
            {
                ee_agents_[i]->cfPlanner(manip_map_, obstacles, delta_t, steps);
            }
        }

        void CfManager::moveAgentsPar(const vector<Obstacle> &obstacles, const double delta_t, const int steps)
        {
#pragma omp parallel for
            for (auto it = ee_agents_.begin(); it < ee_agents_.end(); it++)
            {
                int idx = it - ee_agents_.begin();
                (*it)->cfPlanner(manip_map_, obstacles, delta_t, steps);
            }
        }

        Eigen::Vector3d CfManager::getRealEEAgentPosition() const
        {
            return real_ee_agent_.getLatestPosition();
        }

        int CfManager::evaluateAgents(const vector<Obstacle> &obstacles, const double k_goal_dist, const double k_path_len, const double k_safe_dist,
                                      const double k_workspace, const Eigen::Matrix<double, 6, 1> des_ws_limits)
        {
            assert(ee_agents_.size() != 0);
            std::vector<double> costs;
            for (auto &&agent : ee_agents_)
            {
                double cost = 0;
                for (const gafro::Motor<double> &pose : agent->getPath())
                {
                    Eigen::Vector3d position = pose.getTranslator().toTranslationVector();

                    if (position.x() > des_ws_limits(0))
                    {
                        cost += std::pow(std::abs(position.x() - des_ws_limits(0)) * k_workspace, 2);
                    }
                    else if (position.x() < des_ws_limits(1))
                    {
                        cost += std::pow(std::abs(position.x() - des_ws_limits(1)) * k_workspace, 2);
                    }
                    if (position.y() > des_ws_limits(2))
                    {
                        cost += std::pow(std::abs(position.y() - des_ws_limits(2)) * k_workspace, 2);
                    }
                    else if (position.y() < des_ws_limits(3))
                    {
                        cost += std::pow(std::abs(position.y() - des_ws_limits(3)) * k_workspace, 2);
                    }
                    if (position.z() > des_ws_limits(4))
                    {
                        cost += std::pow(std::abs(position.z() - des_ws_limits(4)) * k_workspace, 2);
                    }
                    else if (position.z() < des_ws_limits(5))
                    {
                        cost += std::pow(std::abs(position.z() - des_ws_limits(5)) * k_workspace, 2);
                    }
                }
                double goal_dist = agent->getDistFromGoal();
                if (goal_dist > config_.approach_distance)
                {
                    cost += goal_dist * k_goal_dist;
                }
                cost += agent->getPathLength() * k_path_len;
                cost += k_safe_dist / agent->getMinObsDist();
                if (agent->getMinObsDist() < 2e-5)
                {
                    cost += 10000.0;
                }
                costs.push_back(cost);
            }
            int min_cost_idx = 0;
            double min_cost = std::numeric_limits<double>::max();
            for (size_t idx = 0; idx < ee_agents_.size(); ++idx)
            {
                if (costs[idx] < min_cost)
                {
                    min_cost = costs[idx];
                    min_cost_idx = idx;
                }
            }
            if (best_agent_)
            {
                if (costs[min_cost_idx] < 0.9 * costs[best_agent_->getAgentID() - 1])
                {
                    best_agent_ = ee_agents_[min_cost_idx];
                }
                else
                {
                    min_cost_idx = best_agent_->getAgentID() - 1;
                }
            }
            else
            {
                best_agent_ = ee_agents_[min_cost_idx];
            }

            return min_cost_idx;
        }

        int CfManager::getBestAgentType()
        {
            return best_agent_->getAgentType();
        }

        Eigen::Vector3d CfManager::getNextPosition()
        {
            return real_ee_agent_.getLatestPosition();
        }

        Eigen::Vector3d CfManager::getInitialPosition()
        {
            return init_pos_;
        }

        Eigen::Vector3d CfManager::getNextVelocity()
        {
            return real_ee_agent_.getVelocity();
        }

        Eigen::Vector3d CfManager::getNextAngularVelocity() const
        {
            return real_ee_agent_.getAngularVelocity();
        }
        Eigen::Vector3d CfManager::getEEForce()
        {
            return real_ee_agent_.getForce();
        }

        Eigen::Vector3d CfManager::getGoalPosition() const
        {
            return goal_pos_;
        }

        int CfManager::getNumPredictionSteps(int agent_id)
        {
            return ee_agents_[agent_id]->getNumPredictionSteps();
        }

        int CfManager::getRealNumPredictionSteps()
        {
            return real_ee_agent_.getNumPredictionSteps();
        }

        double CfManager::getDistFromGoal() const
        {
            return (goal_pos_ - real_ee_agent_.getLatestPosition()).norm();
        }

        std::vector<gafro::Motor<double>> CfManager::getPlannedTrajectory() const
        {
            return real_ee_agent_.getPath();
        }

        RealCfAgent &CfManager::getRealEEAgent()
        {
            return real_ee_agent_;
        }

        bool CfManager::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
        {
            return server->loadParameter(ns + "approach_distance", &approach_distance, true);
        }

    }  // namespace cfplanner
}  // namespace ghostplanner
