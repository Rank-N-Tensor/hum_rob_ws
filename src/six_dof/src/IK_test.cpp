#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <cmath>
#include <iostream>
#include <string>

using namespace pinocchio;

double rad2deg(double radians) { return radians * (180.0 / M_PI); }

int main() {

  // mesh_dir
  const std::string mesh_dir = "src/six_dof/meshes";

  // urdf file name
  const std::string urdf_file = "src/six_dof/urdf/6dof_from_hip.urdf";

  // loading urdf
  Model model;
  model = pinocchio::urdf::buildModel(urdf_file, model);
  GeometryModel visual_model;
  visual_model = pinocchio::urdf::buildGeom(model, urdf_file, VISUAL,
                                            visual_model, mesh_dir);
  GeometryModel collision_model;
  collision_model = pinocchio::urdf::buildGeom(model, urdf_file, COLLISION,
                                               collision_model, mesh_dir);

  std::cout << "model name: " << model.name << std::endl;

  // Geometry data
  Data data(model);

  collision_model.addAllCollisionPairs();
  GeometryData collision_data(collision_model);

  GeometryData visual_data(visual_model);

  // neutral_config
  Eigen::VectorXd q = pinocchio::neutral(model);

  const int JOINT_ID = 8; // 8 is right foot, 4 is left foot

  const Eigen::Matrix3d R_des{
      {1.0, 0.0, 0.0}, {0.0, 0.000796327, -1.0}, {0.0, 1.0, 0.000796327}};

  const pinocchio::SE3 oMdes(R_des, Eigen::Vector3d(0.0235, -0.04, -0.260));

  const double eps = 1e-3;
  const int IT_MAX = 20000;
  const double DT = 1e-1;
  const double damp = 1e-1;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);

  // LOOP
  for (int i = 0;; i++) {
    std::cout<<i<<std::endl;
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector(); // in joint frame
    if (err.norm() < eps) {
      success = true;
      
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, JOINT_ID,
                                    J); // J in joint frame
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);

    

    // // collision modeling
     updateGeometryPlacements(model, data, collision_model, collision_data, q);
     bool is_collision =
         computeCollisions(model, data, collision_model, collision_data, q,true);
    //std::cout << "collision status= " << is_collision << std::endl;

    std::cout<<"normalized error is: "<<err.norm()<<std::endl;

    if (success) {
      std::cout << "Convergence achieved!" << std::endl;
      // convert to deg
      for (int i = 0; i < q.size(); i++) {
        q[i] = rad2deg(q[i]);
      }
      std::cout << "\n result in deg is:" << q.transpose() << "\n";
      break;
    } 
    else{
      std::cout<<"no convergence"<<std::endl;
    }
  }
  
}
