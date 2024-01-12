#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>

enum JointType {
  REVOLUTE,
  PRISMATIC,
  CONTINUOUS,
  UNKNOWN
};

class PinocchioModel {
public:
  PinocchioModel(const std::string& urdf_file_path) : m_urdf_file_path(urdf_file_path) {
    pinocchio::urdf::buildModel(m_urdf_file_path, m_model);
    calc_joint_names();
    calc_link_names();
    calc_joint_types();
    calc_joint_limits();
  }

  const std::vector<std::string>& get_joint_names() const { return m_joint_names; }

  const std::vector<JointType>& get_joint_types() const { return m_joint_types; }

  const std::vector<std::string>& get_link_names() const { return m_link_names; }

  int get_num_non_fixed_joints() const { return m_joint_limits.size(); }
  
  const std::vector<std::pair<double, double>> get_joint_limits() const { return m_joint_limits; }

  const std::string& get_urdf_file_path() const { return m_urdf_file_path; }

  pinocchio::Model& get_model() { return m_model; }

private:
  std::string m_urdf_file_path;
  pinocchio::Model m_model;
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
}

