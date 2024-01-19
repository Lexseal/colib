#include <collision_world.h>

CollisionWorld::CollisionWorld(std::vector<ArticulatedModelPtr> &articulated_models,
                               std::vector<hpp::fcl::CollisionObjectPtr_t> &static_models)
  : m_articulated_models(articulated_models), m_static_models(static_models) {} 

bool CollisionWorld::check_self_collision(const std::vector<double> &joint_angles, size_t model_idx) {
  return false;
}

bool CollisionWorld::check_self_collision(const Eigen::VectorXd &joint_angles, size_t model_idx) {
  return false;
}

bool CollisionWorld::check_environment_collision(const std::vector<double> &joint_angles, size_t model_idx) {
  return false;
}

bool CollisionWorld::check_environment_collision(const Eigen::VectorXd &joint_angles, size_t model_idx) {
  return false;
}
