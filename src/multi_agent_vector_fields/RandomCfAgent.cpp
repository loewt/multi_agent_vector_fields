#include <fstream>
#include <multi_agent_vector_fields/RandomCfAgent.hpp>
#include <random>

namespace ghostplanner::cfplanner
{

    Eigen::Vector3d RandomCfAgent::currentVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel, const Eigen::Vector3d goal_pos,
                                                 const std::vector<Obstacle> &obstacles, const int obstacle_id,
                                                 const std::vector<Eigen::Vector3d> field_rotation_vecs) const
    {
        Eigen::Vector3d cfagent_to_obs{ obstacles[obstacle_id].getPosition() - agent_pos };
        cfagent_to_obs.normalize();
        Eigen::Vector3d current = cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id));
        current.normalize();
        return current;
    }

    Eigen::Vector3d RandomCfAgent::makeRandomVector() const
    {
        Eigen::Vector3d ret;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);
        ret << dis(gen), dis(gen), dis(gen);
        return ret;
    }

    Eigen::Vector3d RandomCfAgent::calculateRotationVector(const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
                                                           const std::vector<Obstacle> &obstacles, const int obstacle_id) const
    {
        Eigen::Vector3d goal_vec{ goal_pos - agent_pos };
        goal_vec.normalize();
        Eigen::Vector3d rot_vec = goal_vec.cross(random_vecs_.at(obstacle_id));
        return rot_vec;
    }

    void RandomCfAgent::saveRandomVecToFile(const int num_obstacles)
    {
        std::ofstream outfile("multi_agent_vector_fields/config/last_random_vectors.txt", std::ios_base::app);
        outfile << "if(getAgentID()== " << getAgentID() << "){" << std::endl;
        for (size_t i = 0; i < num_obstacles; i++)
        {
            Eigen::Vector3d random_vec = makeRandomVector();
            random_vec.normalize();
            random_vecs_.push_back(random_vec);
            outfile << "random_vec << " << random_vec.x() << ", " << random_vec.y() << ", " << random_vec.z() << ";" << std::endl;
            outfile << "random_vecs_.push_back(random_vec);" << std::endl;
        }
        outfile << "}" << std::endl << std::endl;
        outfile.close();
    }

}  // namespace ghostplanner::cfplanner