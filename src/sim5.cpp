#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Eigen>
#include <ostream>
#include <cstdlib>
#include <ios>
#include <streambuf>
#include "sim5vars.h"

using namespace Eigen;
using namespace std;


int main() {

class Robot {
      public:
      RowVectorXd genTraj (int T, RowVectorXd cs_orien0, RowVectorXd cs_orienf) { 
  traj_r = RowVectorXd::LinSpaced(T, cs_orien0[0], cs_orienf[0]); 
  traj_p = RowVectorXd::LinSpaced(T, cs_orien0[1], cs_orienf[1]); 
  traj_y = RowVectorXd::LinSpaced(T, cs_orien0[2], cs_orienf[2]); 
  
  return trajq;
  };

  RowVectorXd invKin (RowVector3d cs_orien) {
    
    double theta_A1 = atan2(
      cos(q_cs_des[0]) * sin(q_cs_des[2]) + cos(q_cs_des[2]) * sin(q_cs_des[1]) * sin(q_cs_des[0]),
      cos(q_cs_des[0]) * cos(q_cs_des[2]) * sin(q_cs_des[1]) - sin(q_cs_des[0]) * sin(q_cs_des[2])
    );

    double theta_A2 = acos(cos(q_cs_des[1])*cos(q_cs_des[2]));
    double B1_num = (cos(q_cs_des[2])*cos(beta)*cos(q_cs_des[0]) + sin(q_cs_des[2])*(sin(beta)*cos(q_cs_des[1])-cos(beta)*sin(q_cs_des[1])*sin(q_cs_des[0])));
    double theta_B1 = -atan2(
      B1_num, 
      cos(q_cs_des[0])*sin(q_cs_des[1])*sin(q_cs_des[2]) + cos(q_cs_des[2])* sin(q_cs_des[0])
    );
    double theta_B2 = acos(cos(q_cs_des[2])*cos(q_cs_des[0])*sin(beta) - sin(q_cs_des[2])*(cos(beta)*cos(q_cs_des[1]) + sin(beta)*sin(q_cs_des[1])*sin(q_cs_des[0])));
   
    mp_A = {theta_A1, theta_A2};
    mp_B = {theta_B1, theta_B2};
    mp_AB = {mp_A[0], mp_A[1], mp_B[0], mp_B[1]};

    cout << mp_A << endl;
    cout << mp_B << endl;
    cout << mp_AB << endl;

    return mp_AB;
  };
};


  Robot robot;
  robot.genTraj(T, cs_orien0, cs_orienf);
  cout << traj_r << endl;
  cout << traj_p << endl;
  cout << traj_y << endl;
  //cout << cs_orien0 << endl;
  //cout << cs_orienf << endl;
  

  for (int i=0; i<T; i++) {
      RowVector3d cs_traj = {traj_r[i],traj_p[i],traj_y[i]};
      robot.invKin(cs_traj);
      
  };
  

   
return 0;

}; 