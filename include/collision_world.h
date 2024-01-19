#pragma once

#include <hpp/fcl/collision.h>

#include <articulated_model.h>

class CollisionWorld {
public:
  CollisionWorld(std::vector<ArticulatedModelPtr> &articulated_models,
                 std::vector<hpp::fcl::CollisionObjectPtr_t> &static_models);

  bool check_self_collision(const std::vector<double> &joint_angles, size_t model_idx);
  bool check_self_collision(const Eigen::VectorXd &joint_angles, size_t model_idx);
  bool check_environment_collision(const std::vector<double> &joint_angles, size_t model_idx);
  bool check_environment_collision(const Eigen::VectorXd &joint_angles, size_t model_idx);

private:
  std::vector<ArticulatedModelPtr> m_articulated_models;
  std::vector<hpp::fcl::CollisionObjectPtr_t> m_static_models;
};