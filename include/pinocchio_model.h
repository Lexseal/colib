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
  PinocchioModel(const std::string &urdf_file_path, const Eigen_Transform3d &base_frame=Eigen_Transform3d::Identity());

  const std::vector<std::string>& get_joint_names() const { return m_joint_names; }

  const std::vector<JointType>& get_joint_types() const { return m_joint_types; }

  const std::vector<std::string>& get_link_names() const { return m_link_names; }

  const std::vector<std::pair<double, double>> get_joint_limits() const { return m_joint_limits; }

  std::vector<Eigen_Transform3d> calculate_link_poses(const Eigen::VectorXd &joint_angles, bool wrt_world=false);
    
  std::vector<Eigen_Transform3d> calculate_link_poses(const std::vector<double> &joint_angles, bool wrt_world=false);

  const std::string& get_urdf_file_path() const { return m_urdf_file_path; }

  pinocchio::Model& get_model() { return m_model; }

private:
  std::string m_urdf_file_path;
  pinocchio::Model m_model;
  pinocchio::Data m_data;
  Eigen_Transform3d m_base_frame;
  std::vector<std::string> m_joint_names;
  std::vector<std::string> m_link_names;
  std::vector<JointType> m_joint_types;
  std::vector<std::pair<double, double>> m_joint_limits;

  void calc_joint_names();

  void calc_link_names();

  void calc_joint_types();

  void calc_joint_limits();

  Eigen_Transform3d pinocchio_to_eigen_transform(const pinocchio::SE3 &pinocchio_transform);
};
