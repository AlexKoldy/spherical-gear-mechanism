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


#define PI 3.14159

using namespace Eigen;
using namespace std;


const int T = 20;
const double beta = PI/2;


RowVector3d cs_orien0 = {0,0,0};
RowVector3d cs_orienf = {PI/2,PI,PI/4};
RowVector3d cs_orien;
RowVector3d q_cs_des = {cs_orien[0], cs_orien[1], cs_orien[2]};
RowVectorXd traj_r;
RowVectorXd traj_p;
RowVectorXd traj_y;
RowVectorXd trajq;
// trajq = {traj_r[i], traj_p[i], traj_y[i]};
// trajq = {traj_r, traj_p, traj_y};
VectorXd cs_traj_r;
VectorXd cs_traj_p;
VectorXd cs_traj_y;
RowVector2d mp_A;
RowVector2d mp_B;
RowVector4d mp_AB;
RowVectorXd mp_log; 



