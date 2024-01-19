#include <articulated_model.h>

ArticulatedModel::ArticulatedModel(const std::string &urdf_file_path,
                                   const std::string &srdf_file_path,
                                   const Eigen_Transform3d &base_frame)
  : m_urdf_file_path(urdf_file_path)
  , m_srdf_file_path(srdf_file_path)
  , m_base_frame(base_frame) {
  pinocchio::urdf::buildModel(m_urdf_file_path, m_model);
  m_data = pinocchio::Data(m_model);
  calc_collision_model();
  calc_joint_names();
  calc_link_names();
  calc_joint_types();
  calc_joint_limits();
}

void ArticulatedModel::calc_collision_model() {
  std::string package_dir = "./";
  if (m_urdf_file_path.find("/") != std::string::npos) {
    package_dir = m_urdf_file_path.substr(0, m_urdf_file_path.rfind("/"));
  }
  if (m_srdf_file_path.empty()) {
    m_srdf_file_path = m_urdf_file_path.substr(0, m_urdf_file_path.rfind(".urdf")) + ".srdf";
  }
  // std::cout << package_dir << std::endl;
  pinocchio::urdf::buildGeom(m_model, m_urdf_file_path, pinocchio::COLLISION, m_geometry_model, package_dir);
  m_geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(m_model, m_geometry_model, m_srdf_file_path);
  m_geometry_data = pinocchio::GeometryData(m_geometry_model);
}

std::vector<Eigen_Transform3d> ArticulatedModel::calculate_link_poses(const Eigen::VectorXd &joint_angles,
                                                                    bool wrt_world) {
  assert(joint_angles.size() == m_joint_names.size());
  std::vector<Eigen_Transform3d> link_poses;
  pinocchio::forwardKinematics(m_model, m_data, joint_angles);
  
  for (const std::string &link_name : m_link_names) {
    size_t link_id = m_model.getFrameId(link_name);
    size_t joint_id = m_model.frames[link_id].parent;
    pinocchio::SE3 link_wrt_joint = m_model.frames[link_id].placement;
    pinocchio::SE3 joint_wrt_base = m_data.oMi[joint_id];
    pinocchio::SE3 link_wrt_base = joint_wrt_base * link_wrt_joint;
    Eigen_Transform3d link_pose_eigen = pinocchio_to_eigen_transform(link_wrt_base);
    if (wrt_world) link_pose_eigen = m_base_frame * link_pose_eigen;
    link_poses.push_back(link_pose_eigen);
  }
  
  return link_poses;
}

std::vector<Eigen_Transform3d> ArticulatedModel::calculate_link_poses(const std::vector<double> &joint_angles,
                                                                    bool wrt_world) {
  Eigen::VectorXd joint_angles_eigen = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
  return calculate_link_poses(joint_angles_eigen, wrt_world);
}

void ArticulatedModel::calc_joint_names() {
  for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
    m_joint_names.push_back(m_model.names[i]);
  }
}

void ArticulatedModel::calc_link_names() {
  for (size_t i = 0; i < m_model.frames.size(); i++) {
    if (m_model.frames[i].type == pinocchio::BODY) {
      m_link_names.push_back(m_model.frames[i].name);
    }
  }
}

void ArticulatedModel::calc_joint_types() {
  for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
    size_t idx_of_interest = m_model.joints[i].classname().size();
    switch (m_model.joints[i].shortname()[idx_of_interest]) {
      case 'R':
        if (m_model.joints[i].shortname()[idx_of_interest+1] == 'U') {
          m_joint_types.push_back(CONTINUOUS);
        } else {
          m_joint_types.push_back(REVOLUTE);
        } 
        break;
      case 'P':
        m_joint_types.push_back(PRISMATIC);
        break;
      default:
        m_joint_types.push_back(UNKNOWN);
        break;
    }
  }
}

void ArticulatedModel::calc_joint_limits() {
  for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
    m_joint_limits.emplace_back(m_model.lowerPositionLimit[i-1], m_model.upperPositionLimit[i-1]);
  }
}

void ArticulatedModel::calc_collision_objects() {
  for (const std::string &link_name : m_link_names) {
    // first get all the collision geometries associated with each link

  }
}

Eigen_Transform3d ArticulatedModel::pinocchio_to_eigen_transform(const pinocchio::SE3 &pinocchio_transform) {
  Eigen_Transform3d eigen_transform;
  eigen_transform.matrix() = pinocchio_transform.toHomogeneousMatrix();
  return eigen_transform;
}

int main() {
  ArticulatedModel model("/home/xinsonglin/colib/panda/panda.urdf");
  std::cout << "joint names: " << std::endl;
  for (const auto& name : model.get_joint_names()) {
    std::cout << name << std::endl;
  }
  std::cout << "link names: " << std::endl;
  for (const auto& name : model.get_link_names()) {
    std::cout << name << std::endl;
  }
  std::cout << "joint types: " << std::endl;
  for (const auto& type : model.get_joint_types()) {
    std::cout << type << std::endl;
  }
  std::cout << "joint limits: " << std::endl;
  for (const auto& limit : model.get_joint_limits()) {
    std::cout << limit.first << ", " << limit.second << std::endl;
  }
  std::cout << "link poses: " << std::endl;
  std::vector<double> joint_angles = {0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0};
  std::vector<Eigen_Transform3d> link_poses = model.calculate_link_poses(joint_angles);
  auto &link_names = model.get_link_names();
  for (int i = 0; i < link_names.size(); i++) {
    std::cout << link_names[i] << std::endl;
    std::cout << link_poses[i].matrix() << std::endl;
  }
}

