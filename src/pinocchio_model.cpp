#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

enum JointType {
  REVOLUTE,
  PRISMATIC,
  CONTINUOUS,
  UNKNOWN
};

using Eigen_Transform3d = Eigen::Transform<double, 3, Eigen::Isometry>;

class PinocchioModel {
public:
  PinocchioModel(const std::string &urdf_file_path) : m_urdf_file_path(urdf_file_path) {
    pinocchio::urdf::buildModel(m_urdf_file_path, m_model);
    m_data = pinocchio::Data(m_model);
    calc_joint_names();
    calc_link_names();
    calc_joint_types();
    calc_joint_limits();
  }

  const std::vector<std::string>& get_joint_names() const { return m_joint_names; }

  const std::vector<JointType>& get_joint_types() const { return m_joint_types; }

  const std::vector<std::string>& get_link_names() const { return m_link_names; }

  const std::vector<std::pair<double, double>> get_joint_limits() const { return m_joint_limits; }


  std::vector<Eigen_Transform3d> calculate_link_poses(const Eigen::VectorXd &joint_angles) {
    assert(joint_angles.size() == m_joint_names.size());
    std::vector<Eigen_Transform3d> link_poses;
    pinocchio::forwardKinematics(m_model, m_data, joint_angles);
    
    for (size_t i = 0; i < m_link_names.size(); i++) {
      const std::string &link_name = m_link_names[i];
      pinocchio::SE3 link_pose = m_data.oMi[m_model.getFrameId(link_name)];
      link_poses.push_back(pinocchio_to_eigen_transform(link_pose));
    }
    
    return link_poses;
  }

  std::vector<Eigen_Transform3d> calculate_link_poses(const std::vector<double> &joint_angles) {
    Eigen::VectorXd joint_angles_eigen = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    return calculate_link_poses(joint_angles_eigen);
  }

  const std::string& get_urdf_file_path() const { return m_urdf_file_path; }

  pinocchio::Model& get_model() { return m_model; }

private:
  std::string m_urdf_file_path;
  pinocchio::Model m_model;
  pinocchio::Data m_data;
  std::vector<std::string> m_joint_names;
  std::vector<std::string> m_link_names;
  std::vector<JointType> m_joint_types;
  std::vector<std::pair<double, double>> m_joint_limits;

  void calc_joint_names() {
    for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
      m_joint_names.push_back(m_model.names[i]);
    }
  }

  void calc_link_names() {
    for (size_t i = 0; i < m_model.frames.size(); i++) {
      if (m_model.frames[i].type == pinocchio::BODY) {
        m_link_names.push_back(m_model.frames[i].name);
      }
    }
  }

  void calc_joint_types() {
    for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
      size_t idx_of_interest = m_model.joints[i].classname().size();
      std::cout << m_model.joints[i].shortname() << " " << idx_of_interest << std::endl;
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

  void calc_joint_limits() {
    for (int i = 1; i < m_model.njoints; ++i) {  // starting from 1 to skip the universe joint
      m_joint_limits.emplace_back(m_model.lowerPositionLimit[i-1], m_model.upperPositionLimit[i-1]);
    }
  }

  Eigen_Transform3d pinocchio_to_eigen_transform(const pinocchio::SE3 &pinocchio_transform) {
    Eigen_Transform3d eigen_transform;
    eigen_transform.matrix() = pinocchio_transform.toHomogeneousMatrix();
    return eigen_transform;
  }
};

int main() {
  PinocchioModel model("/home/xinsonglin/colib/panda/panda.urdf");
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
  std::vector<double> joint_angles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  std::vector<Eigen_Transform3d> link_poses = model.calculate_link_poses(joint_angles);
  for (const auto& pose : link_poses) {
    std::cout << pose.matrix() << std::endl;
  }
}

